import time
import threading
import math
import numpy as np
from collections import deque
from concurrent.futures import ThreadPoolExecutor, Future
from typing import Optional, Dict, List
from dataclasses import dataclass, field
from enum import Enum

import rclpy
from rclpy.node import Node
from arm_interfaces.msg import FrankaRIM
from teleop_interfaces.msg import Inverse3State


class DelayCompensationMethod(Enum):
    ZOH = "ZOH"
    ZOH_PHI = "ZOHPhi"
    DELAY_RIM = "DelayRIM"


@dataclass
class HapticState:
    """Haptic device state at a specific timestamp"""

    timestamp: float
    position: np.ndarray
    velocity: np.ndarray


@dataclass
class DelayedRIMPacket:
    """Represents a delayed RIM packet with timing information"""

    rim_msg: FrankaRIM
    arrival_time: float
    delay_seconds: float
    method: DelayCompensationMethod
    packet_id: int
    packet_creation_time: float = 0.0  # When the original robot state was captured


@dataclass
class ReducedModelState:
    """State of the reduced impedance model"""

    stiffness: float = 3000.0
    damping: float = 2.0
    hl: float = 0.001  # Integration timestep

    # RIM state (robot position/velocity)
    rim_position: np.ndarray = field(default_factory=lambda: np.zeros((3, 1)))
    rim_velocity: np.ndarray = field(default_factory=lambda: np.zeros((3, 1)))

    # Constraint deviation
    phi_position: np.ndarray = field(default_factory=lambda: np.zeros((3, 1)))
    phi_velocity: np.ndarray = field(default_factory=lambda: np.zeros((3, 1)))

    # Force output
    interface_force: np.ndarray = field(default_factory=lambda: np.zeros((3, 1)))

    # DelayRIM integration variables
    effective_mass: np.ndarray = field(default_factory=lambda: np.eye(3))
    effective_force: np.ndarray = field(default_factory=lambda: np.zeros((3, 1)))
    mass_factor: np.ndarray = field(default_factory=lambda: np.eye(3))
    inverse_augmented_mass_matrix: np.ndarray = field(default_factory=lambda: np.eye(3))


@dataclass
class PerformanceStats:
    """Performance monitoring statistics"""

    total_packets: int = 0
    avg_computation_time: float = 0.0  # in milliseconds
    max_computation_time: float = 0.0  # in milliseconds
    dropped_packets: int = 0
    active_threads: int = 0
    queue_length: int = 0


class DelayRIM:
    """Thread-based DelayRIM computation manager"""

    def __init__(self, max_workers: int = 8, max_haptic_history: int = 1000):
        self.max_workers = max_workers
        self.max_haptic_history = max_haptic_history

        # Threading infrastructure
        self.executor = ThreadPoolExecutor(max_workers=max_workers)
        self.lock = threading.RLock()

        # State management
        self.haptic_history: deque[HapticState] = deque(maxlen=max_haptic_history)
        self.active_computations: Dict[int, Future] = {}
        self.packet_counter = 0

        # Performance monitoring
        self.stats = PerformanceStats()
        self._computation_times: deque[float] = deque(maxlen=100)

        # Current state
        self.current_haptic_state: Optional[HapticState] = None
        self.latest_result: Optional[np.ndarray] = None

    def add_haptic_state(self, inverse3_msg: Inverse3State) -> None:
        """Add new haptic state to history"""
        timestamp = time.perf_counter()

        # Extract position and velocity (coordinate transform as needed)
        position = np.array(
            [inverse3_msg.pose.position.y, -inverse3_msg.pose.position.x, inverse3_msg.pose.position.z]
        ).reshape((3, 1))
        position[0] += 0.4253  # Offset as in current implementation

        velocity = np.array(
            [inverse3_msg.twist.linear.y, -inverse3_msg.twist.linear.x, inverse3_msg.twist.linear.z]
        ).reshape((3, 1))

        haptic_state = HapticState(timestamp, position, velocity)

        with self.lock:
            self.haptic_history.append(haptic_state)
            self.current_haptic_state = haptic_state

    def submit_rim_packet(self, node: Node, rim_msg: FrankaRIM, method: DelayCompensationMethod) -> int:
        """Submit a delayed RIM packet for processing"""
        packet_id = self.packet_counter
        self.packet_counter += 1

        arrival_time = time.perf_counter()

        # Calculate delay from message timestamps
        msg_timestamp = rclpy.time.Time.from_msg(rim_msg.header.stamp)
        current_time = node.get_clock().now()
        delay_duration = current_time.nanoseconds - msg_timestamp.nanoseconds
        delay_seconds = delay_duration / 1e9

        # Convert ROS timestamp to perf_counter equivalent for delay tracking
        packet_creation_time = arrival_time - delay_seconds

        # Calculate integration steps needed
        integration_steps = math.ceil(delay_seconds / 0.001)  # 1ms timesteps TODO: From attribute

        node.get_logger().debug(f"Packet {packet_id}: delay={delay_seconds:.3f}s, steps={integration_steps}")

        # Check if we have enough haptic history
        with self.lock:
            if len(self.haptic_history) == 0:
                node.get_logger().warn(f"No haptic history available for packet {packet_id}")
                self.stats.dropped_packets += 1
                return packet_id

            oldest_history_age = time.perf_counter() - self.haptic_history[0].timestamp
            if delay_seconds > oldest_history_age:
                node.get_logger().warn(
                    f"Packet {packet_id}: delay {delay_seconds:.3f}s > available history {oldest_history_age:.3f}s, dropping packet"
                )
                self.stats.dropped_packets += 1
                return packet_id

        packet = DelayedRIMPacket(
            rim_msg=rim_msg,
            arrival_time=arrival_time,
            delay_seconds=delay_seconds,
            method=method,
            packet_id=packet_id,
            packet_creation_time=packet_creation_time,
        )

        # Submit to thread pool
        future = self.executor.submit(self._process_delayed_packet, packet, node.get_logger())

        with self.lock:
            self.active_computations[packet_id] = future
            self.stats.total_packets += 1

        return packet_id

    def _process_delayed_packet(self, packet: DelayedRIMPacket, node_logger) -> Optional[np.ndarray]:
        """Process a delayed RIM packet (runs in worker thread)"""
        start_time = time.perf_counter()

        try:
            # Get haptic state history for catch-up
            haptic_states = self._get_haptic_history_for_catchup(packet.arrival_time, packet.delay_seconds)

            if not haptic_states:
                node_logger.warn(f"No haptic states for catch-up, packet {packet.packet_id}")
                return None

            # Perform DelayRIM computation
            if packet.method == DelayCompensationMethod.DELAY_RIM:
                result = self._compute_delay_rim(packet, haptic_states, node_logger)
            elif packet.method == DelayCompensationMethod.ZOH:
                result = self._compute_zoh(packet, haptic_states)
            else:  # ZOH_PHI
                result = self._compute_zoh_phi(packet, haptic_states)

            # # Calculate total delay from robot state capture to force computation
            # force_computation_time = time.perf_counter()
            # total_delay_ms = (force_computation_time - packet.packet_creation_time) * 1000
            # computation_time_ms = (force_computation_time - start_time) * 1000

            # node_logger.info(
            #     f"Packet {packet.packet_id} [{packet.method.value}]: "
            #     f"Total delay={total_delay_ms:.1f}ms "
            #     f"(network={packet.delay_seconds * 1000:.1f}ms + "
            #     f"computation={computation_time_ms:.1f}ms)"
            # )

            return result

        except Exception as e:
            node_logger.error(f"Error processing DelayRIM packet {packet.packet_id}: {e}")
            return None

    def _get_haptic_history_for_catchup(self, arrival_time: float, delay_seconds: float) -> List[HapticState]:
        """Get haptic state history needed for catch-up integration"""
        start_time = arrival_time - delay_seconds

        with self.lock:
            # Get states from start_time to current time
            relevant_states = [state for state in self.haptic_history if state.timestamp >= start_time]

        relevant_states = sorted(relevant_states, key=lambda s: s.timestamp)

        # If relevant state is empty -> use latest state
        if len(relevant_states) == 0:
            relevant_states = [self.haptic_history[-1]]

        return relevant_states

    def _compute_delay_rim(self, packet: DelayedRIMPacket, haptic_states: List[HapticState], node_logger) -> np.ndarray:
        """Compute DelayRIM force with catch-up integration (oneStep method)"""
        if not haptic_states:
            return np.zeros((3, 1))

        # Initialize reduced model from RIM packet
        reduced_model = ReducedModelState()
        reduced_model.rim_position = np.array(packet.rim_msg.rim_position[:3]).reshape((3, 1))
        reduced_model.rim_velocity = np.array(packet.rim_msg.rim_velocity[:3]).reshape((3, 1))
        reduced_model.stiffness = packet.rim_msg.interface_stiffness
        reduced_model.damping = packet.rim_msg.interface_damping

        # Extract effective parameters from RIM message
        effective_mass_flat = packet.rim_msg.effective_mass
        if len(effective_mass_flat) == 9:  # 3x3 matrix
            reduced_model.effective_mass = np.array(effective_mass_flat).reshape((3, 3))
        elif len(effective_mass_flat) == 1:  # scalar
            reduced_model.effective_mass = np.eye(3) * effective_mass_flat[0]
        else:
            node_logger.warn(f"Unexpected effective_mass size: {len(effective_mass_flat)}")
            reduced_model.effective_mass = np.eye(3)

        reduced_model.effective_force = np.array(packet.rim_msg.effective_force[:3]).reshape((3, 1))

        # Compute DelayRIM integration matrices (from oneStep method)
        hl = reduced_model.hl
        Id3 = np.eye(3)
        augmented_mass_matrix = (
            reduced_model.effective_mass + hl * (reduced_model.damping + hl * reduced_model.stiffness) * Id3
        )
        reduced_model.inverse_augmented_mass_matrix = np.linalg.inv(augmented_mass_matrix)
        reduced_model.mass_factor = reduced_model.inverse_augmented_mass_matrix @ reduced_model.effective_mass

        # Catch-up integration through haptic state history
        prev_haptic_state = haptic_states[0]
        haptic_position = prev_haptic_state.position
        haptic_velocity = prev_haptic_state.velocity

        # Integrate through catch-up period using oneStep method
        for i in range(1, len(haptic_states)):
            current_haptic = haptic_states[i]
            haptic_acceleration = (current_haptic.velocity - haptic_velocity) / hl

            # One-step DelayRIM integration (from local_processes.py oneStep)
            self._one_step_delay_rim(reduced_model, haptic_position, haptic_velocity, haptic_acceleration)

            haptic_position = current_haptic.position
            haptic_velocity = current_haptic.velocity

        # Compute final interface force (from getForce method)
        reduced_model.phi_position = reduced_model.rim_position - haptic_position
        reduced_model.phi_velocity = reduced_model.rim_velocity - haptic_velocity

        interface_force = (
            -reduced_model.stiffness * reduced_model.phi_position
            - (hl * reduced_model.stiffness + reduced_model.damping) * reduced_model.phi_velocity
        )

        return interface_force

    def _one_step_delay_rim(
        self, reduced_model: ReducedModelState, haptic_pos: np.ndarray, haptic_vel: np.ndarray, haptic_acc: np.ndarray
    ) -> None:
        """Perform one integration step of DelayRIM (from local_processes.py oneStep)"""

        # Update constraint deviation
        reduced_model.phi_position = reduced_model.rim_position - haptic_pos
        reduced_model.phi_velocity = reduced_model.rim_velocity - haptic_vel

        # Force terms from oneStep method
        regular_force_terms = (
            reduced_model.effective_mass @ haptic_acc / reduced_model.hl
            + reduced_model.effective_force
            - reduced_model.stiffness * reduced_model.phi_position
            + (reduced_model.damping + reduced_model.hl * reduced_model.stiffness) * haptic_vel
        )

        # Update reduced model state (from oneStep method)
        reduced_model.rim_velocity = (
            reduced_model.mass_factor @ reduced_model.rim_velocity
            + reduced_model.hl * reduced_model.inverse_augmented_mass_matrix @ regular_force_terms
        )
        reduced_model.rim_position = reduced_model.rim_position + reduced_model.hl * reduced_model.rim_velocity

    def _compute_zoh(self, packet: DelayedRIMPacket, haptic_states: List[HapticState]) -> np.ndarray:
        """Compute Zero-Order Hold force"""
        if not haptic_states:
            return np.zeros((3, 1))

        # Use first (oldest) haptic state for ZOH
        old_haptic = haptic_states[0]

        rim_position = np.array(packet.rim_msg.rim_position[:3]).reshape((3, 1))
        rim_velocity = np.array(packet.rim_msg.rim_velocity[:3]).reshape((3, 1))

        phi_position = rim_position - old_haptic.position
        phi_velocity = rim_velocity - old_haptic.velocity  # Use old velocity for ZOH

        interface_force = (
            -packet.rim_msg.interface_stiffness * phi_position - packet.rim_msg.interface_damping * phi_velocity
        )

        return interface_force

    def _compute_zoh_phi(self, packet: DelayedRIMPacket, haptic_states: List[HapticState]) -> np.ndarray:
        """Compute ZOH with current haptic state (ZOHPhi)"""
        if not haptic_states:
            return np.zeros((3, 1))

        # Use latest haptic state for ZOHPhi
        current_haptic = haptic_states[-1]

        rim_position = np.array(packet.rim_msg.rim_position[:3]).reshape((3, 1))
        rim_velocity = np.array(packet.rim_msg.rim_velocity[:3]).reshape((3, 1))

        phi_position = rim_position - current_haptic.position
        phi_velocity = rim_velocity - current_haptic.velocity

        interface_force = (
            -packet.rim_msg.interface_stiffness * phi_position - packet.rim_msg.interface_damping * phi_velocity
        )

        return interface_force

    def get_latest_result(self) -> Optional[np.ndarray]:
        """Get the most recent DelayRIM computation result"""
        ready_results = []
        completed_ids = []
        force_render_time = time.perf_counter()

        with self.lock:
            for packet_id, future in self.active_computations.items():
                if future.done():
                    try:
                        result = future.result()
                        if result is not None:
                            ready_results.append((packet_id, result))
                        completed_ids.append(packet_id)

                    except Exception as e:
                        print(f"Error retrieving result for packet {packet_id}: {e}")
                        completed_ids.append(packet_id)

            # Clean up completed computations
            for packet_id in completed_ids:
                del self.active_computations[packet_id]

            self.stats.active_threads = len(self.active_computations)

        # Return most recent result (highest packet_id) and log render timing
        if ready_results:
            ready_results.sort(key=lambda x: x[0])  # Sort by packet_id
            latest_packet_id, latest_result = ready_results[-1]

            # Log when force is actually rendered to user
            # print(f"FORCE RENDERED - Packet {latest_packet_id} rendered at {force_render_time:.6f}")

            self.latest_result = latest_result
            return latest_result

        return self.latest_result  # Return last known result

    def _update_performance_stats(self, computation_time: float) -> None:
        """Update performance monitoring statistics"""
        with self.lock:
            self._computation_times.append(computation_time)

            if self._computation_times:
                self.stats.avg_computation_time = np.mean(self._computation_times)
                self.stats.max_computation_time = max(self.stats.max_computation_time, computation_time)

    def get_performance_stats(self) -> PerformanceStats:
        """Get current performance statistics"""
        with self.lock:
            self.stats.queue_length = len(self.haptic_history)
            return self.stats

    def shutdown(self) -> None:
        """Shutdown the DelayRIM manager"""
        self.executor.shutdown(wait=True)
