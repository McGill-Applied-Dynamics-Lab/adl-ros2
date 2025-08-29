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

from copy import copy


class DelayCompensationMethod(Enum):
    ZOH = "zoh"
    ZOH_PHI = "zoh_phi"
    DELAY_RIM = "delay_rim"


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
    packet_creation_time: float = 0.0  # When the original robot state was captured (sec)


@dataclass
class ReducedModelState:
    """State of the reduced impedance model"""

    stiffness: float = 3000.0
    damping: float = 2.0
    hl: float = 0.001  # Integration timestep

    # RIM state (robot position/velocity)
    position: np.ndarray = field(default_factory=lambda: np.zeros((3, 1)))
    velocity: np.ndarray = field(default_factory=lambda: np.zeros((3, 1)))

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

    A_inv: np.ndarray = field(default_factory=lambda: np.eye(3))


@dataclass
class PerformanceStats:
    """Performance monitoring statistics"""

    total_packets: int = 0
    avg_computation_time: float = 0.0  # in milliseconds
    max_computation_time: float = 0.0  # in milliseconds
    avg_total_delay: float = 0.0  # in milliseconds
    dropped_packets: int = 0
    packet_queue: int = 0
    queue_length: int = 0


class DelayRIM:
    """Thread-based DelayRIM computation manager"""

    def __init__(self, node: Node, interface_dim: int = 1, max_workers: int = 8, max_haptic_history: int = 1000):
        self.max_workers = max_workers
        self.max_haptic_history = max_haptic_history

        self._interface_dim = interface_dim  # Dimension of the interface ('m')

        # Threading infrastructure
        self.executor = ThreadPoolExecutor(max_workers=max_workers)
        self.lock = threading.RLock()
        self._node: Node = node

        # State management
        self._last_update_time = time.time()
        self._loop_count: int = 0
        self._actual_periods = deque(maxlen=100)  # Store last 100 loop times

        self.haptic_history: deque[HapticState] = deque(maxlen=max_haptic_history)
        self.active_computations: Dict[int, Future] = {}
        self.packet_counter = 0

        # Performance monitoring
        self.stats = PerformanceStats()
        self._computation_times: deque[float] = deque(maxlen=100)
        self._total_delay_times: deque[float] = deque(maxlen=100)

        # Current state
        self.current_haptic_state: Optional[HapticState] = None
        self.latest_forces: Optional[np.ndarray] = None  # Most recent computed interface forces

        # Persistent state for continuous 1kHz stepping
        self.rim_state: Optional[ReducedModelState] = None  # Latest computed RIM state

    def add_haptic_state(self, i3_position: np.array, i3_velocity: np.array) -> None:
        """
        Add new haptic state to history.

        Only add the state in along the interfaces.

        For the 1D systems, this corresponds to the y-axis of the haptic device.

        """
        timestamp = self._node.get_clock().now().nanoseconds / 1e9

        # Extract position and velocity (coordinate transform as needed)
        position = np.array(
            [
                i3_position[0],
                # i3_position[1],
                # i3_position[2]
            ]
        ).reshape((self._interface_dim,))

        # position[0] += 0.4253  # For robot arm
        # position = position * 5  # For simple mass

        velocity = np.array(
            [
                i3_velocity[0],
                # i3_velocity[1],
                # i3_velocity[2]
            ]
        ).reshape((self._interface_dim,))

        haptic_state = HapticState(timestamp, position, velocity)

        with self.lock:
            self.haptic_history.append(haptic_state)
            self.current_haptic_state = haptic_state

    def _create_rim_state(self, rim_msg: FrankaRIM) -> ReducedModelState:
        """Create and configure a ReducedModelState from a FrankaRIM message"""
        reduced_model = ReducedModelState()

        # Update model parameters from RIM message
        reduced_model.stiffness = rim_msg.interface_stiffness
        reduced_model.damping = rim_msg.interface_damping
        reduced_model.hl = 0.001  # 1ms timestep

        # -- Update RIM state from message
        # If 1st message, initialize
        if self.rim_state is None:
            reduced_model.position = np.array(rim_msg.rim_position).reshape((self._interface_dim, 1))
            reduced_model.velocity = np.array(rim_msg.rim_velocity).reshape((self._interface_dim, 1))

        else:
            # State from last rim estimate
            reduced_model.position = self.rim_state.position.copy()
            reduced_model.velocity = self.rim_state.velocity.copy()

            # # State from msg
            # reduced_model.position = np.array(rim_msg.rim_position).reshape((self._interface_dim, 1))
            # reduced_model.velocity = np.array(rim_msg.rim_velocity).reshape((self._interface_dim, 1))

        # Update effective parameters
        effective_mass_flat = rim_msg.effective_mass
        reduced_model.effective_mass = np.array(effective_mass_flat).reshape((self._interface_dim, self._interface_dim))
        reduced_model.effective_force = np.array(rim_msg.effective_force).reshape((self._interface_dim, 1))

        # Update integration matrices
        hl = reduced_model.hl
        Id_m = np.eye(self._interface_dim)

        # Compute augmented mass matrix and its inverse
        augmented_mass_matrix = (
            reduced_model.effective_mass + hl * (reduced_model.damping + hl * reduced_model.stiffness) * Id_m
        )
        reduced_model.inverse_augmented_mass_matrix = np.linalg.inv(augmented_mass_matrix)
        reduced_model.mass_factor = reduced_model.inverse_augmented_mass_matrix @ reduced_model.effective_mass

        # Compute A_inv matrix
        A = Id_m + reduced_model.damping * reduced_model.effective_mass
        reduced_model.A_inv = np.linalg.inv(A)

        return reduced_model

    def submit_rim_packet(self, rim_msg: FrankaRIM, method: DelayCompensationMethod) -> int:
        """Submit a delayed RIM packet for processing"""
        packet_id = self.packet_counter
        self.packet_counter += 1

        # Arrival time in ROS time (seconds)
        arrival_time = self._node.get_clock().now().nanoseconds / 1e9

        # Calculate delay from message timestamps (ROS time)
        msg_timestamp = rclpy.time.Time.from_msg(rim_msg.header.stamp)
        current_time = self._node.get_clock().now()
        delay_duration = current_time.nanoseconds - msg_timestamp.nanoseconds
        delay_seconds = delay_duration / 1e9

        # node.get_logger().info(f"Received RIM packet {packet_id} with delay: {delay_seconds * 1000:.3f}ms")

        packet_creation_time = arrival_time - delay_seconds

        # Calculate integration steps needed
        integration_steps = math.ceil(delay_seconds / 0.001)  # 1ms timesteps TODO: From attribute

        self._node.get_logger().debug(
            f"Packet {packet_id}: delay={delay_seconds * 1000:.3f}ms, steps={integration_steps}",
            throttle_duration_sec=0.1,
        )

        # Check if we have enough haptic history
        with self.lock:
            if len(self.haptic_history) == 0:
                self._node.get_logger().warn(f"No haptic history available for packet {packet_id}")
                self.stats.dropped_packets += 1
                return packet_id

            now_sec = self._node.get_clock().now().nanoseconds / 1e9
            oldest_history_age = now_sec - self.haptic_history[0].timestamp
            if delay_seconds > oldest_history_age:
                self._node.get_logger().warn(
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
        future = self.executor.submit(self._process_delayed_packet, packet)

        with self.lock:
            self.active_computations[packet_id] = future
            self.stats.total_packets += 1

        return packet_id

    def _process_delayed_packet(self, packet: DelayedRIMPacket) -> Optional[np.ndarray]:
        """Process a delayed RIM packet (runs in worker thread)"""
        proc_start_time = self._node.get_clock().now().nanoseconds / 1e9  # Time when proc starts

        try:
            # Get haptic state history for catch-up
            haptic_states = self._get_haptic_history_for_catchup(packet.arrival_time, packet.delay_seconds)

            if not haptic_states:
                self._node.get_logger().warn(f"No haptic states for catch-up, packet {packet.packet_id}")
                return None

            # Init RIM state
            rim = self._create_rim_state(packet.rim_msg)

            # --- Perform DelayRIM computation
            if packet.method == DelayCompensationMethod.DELAY_RIM:
                rim = self._catchup_delay_rim(rim, rim_delay_seconds=packet.delay_seconds, haptic_states=haptic_states)

            elif packet.method == DelayCompensationMethod.ZOH:
                rim = self._catchup_zoh(rim, haptic_states)

            elif packet.method == DelayCompensationMethod.ZOH_PHI:
                rim = self._catchup_zoh_phi(rim, haptic_states)

            else:
                raise ValueError(f"Unknown DelayCompensationMethod: {packet.method}")

            # rendered_forces = Nonecomputation_time_ms

            # Calculate total delay from robot state capture to force computation
            proc_end_time = self._node.get_clock().now().nanoseconds / 1e9
            total_delay_ms = (proc_end_time - packet.packet_creation_time) * 1000
            computation_time_ms = (proc_end_time - proc_start_time) * 1000

            # node_logger.info(
            #     f"Packet {packet.packet_id} [{packet.method.value}]: "
            #     f"Total delay={total_delay_ms:.1f}ms "
            #     f"(network={packet.delay_seconds * 1000:.1f}ms + "
            #     f"computation={computation_time_ms:.1f}ms)",
            #     throttle_duration_sec=1.0,
            # )

            self._update_performance_stats(computation_time_ms, total_delay=total_delay_ms)

            return rim

        except Exception as e:
            self._node.get_logger().error(f"Error processing DelayRIM packet {packet.packet_id}: {e}")
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

    def _interpolate_haptic_state(self, haptic_states: List[HapticState], target_time: float, hl: float):
        """
        Interpolate haptic position, velocity, and acceleration at target_time
        """
        # Find the two haptic states that bracket the target time
        before_state = None
        after_state = None

        for i, state in enumerate(haptic_states):
            if state.timestamp <= target_time:
                before_state = state
            else:
                after_state = state
                break

        # If we only have one state or target_time is outside our data range
        if before_state is None:
            # Use first available state
            state = haptic_states[0]
            position = state.position.reshape((self._interface_dim, 1))
            velocity = state.velocity.reshape((self._interface_dim, 1))
            acceleration = np.zeros((self._interface_dim, 1))
            return position, velocity, acceleration

        if after_state is None:
            # Use last available state
            state = haptic_states[-1]
            position = state.position.reshape((self._interface_dim, 1))
            velocity = state.velocity.reshape((self._interface_dim, 1))
            acceleration = np.zeros((self._interface_dim, 1))
            return position, velocity, acceleration

        # Linear interpolation between before_state and after_state
        dt = after_state.timestamp - before_state.timestamp
        if dt <= 0:
            # States have same timestamp, use before_state
            position = before_state.position.reshape((self._interface_dim, 1))
            velocity = before_state.velocity.reshape((self._interface_dim, 1))
            acceleration = np.zeros((self._interface_dim, 1))
            return position, velocity, acceleration

        # Interpolation factor (0 = before_state, 1 = after_state)
        alpha = (target_time - before_state.timestamp) / dt
        alpha = np.clip(alpha, 0.0, 1.0)

        # Interpolate position and velocity
        position = (1 - alpha) * before_state.position + alpha * after_state.position
        velocity = (1 - alpha) * before_state.velocity + alpha * after_state.velocity

        # Compute acceleration from velocity difference
        acceleration = (after_state.velocity - before_state.velocity) / hl

        position = position.reshape((self._interface_dim, 1))
        velocity = velocity.reshape((self._interface_dim, 1))
        acceleration = acceleration.reshape((self._interface_dim, 1))

        return position, velocity, acceleration

    ###
    # Force Computations
    ###
    def get_interface_force(self) -> Optional[np.ndarray]:
        """Get the most recent interface forces computed using the selected delay compensation method."""
        ready_results = []
        completed_ids = []
        loop_time = time.perf_counter()

        if self._loop_count > 0:
            actual_period = loop_time - self._last_update_time
            self._actual_periods.append(actual_period)
            hl = actual_period

        with self.lock:
            for packet_id, future in self.active_computations.items():
                if future.done():
                    try:
                        rim_state = future.result()  # Return of `_process_delayed_packet`
                        if rim_state is not None:
                            ready_results.append((packet_id, rim_state))

                        completed_ids.append(packet_id)

                    except Exception as e:
                        self._node.get_logger().error(f"Error retrieving result for packet {packet_id}: {e}")
                        completed_ids.append(packet_id)

            # Clean up completed computations
            for packet_id in completed_ids:
                del self.active_computations[packet_id]

        if ready_results:
            # Return most recent result (highest packet_id) and log render timing
            ready_results.sort(key=lambda x: x[0])  # Sort by packet_id
            latest_packet_id, latest_rim = ready_results[-1]
            # latest_forces = latest_rim.interface_force

            # Log when force is actually rendered to user
            # print(f"FORCE RENDERED - Packet {latest_packet_id} rendered at {force_render_time:.6f}")

            self.rim_state = copy(latest_rim)

            self.latest_forces = self.rim_state.interface_force

        elif self.rim_state is not None:
            # No results available, step local rim model
            # self.latest_forces = np.zeros((self._interface_dim, 1))
            # self.node.get_logger().info(
            #     "No new DelayRIM results available, stepping local rim.", throttle_duration_sec=0.2
            # )

            latest_haptic_state = self.haptic_history[-1]
            haptic_position = latest_haptic_state.position.reshape((self._interface_dim, 1))
            haptic_velocity = latest_haptic_state.velocity.reshape((self._interface_dim, 1))

            self._step_rim(self.rim_state, haptic_position, haptic_velocity, haptic_acc=0.0, hl=hl)

            self._compute_interface_force(self.rim_state, haptic_position, haptic_velocity)

            self.latest_forces = self.rim_state.interface_force

        else:
            # No persistent state yet
            self.latest_forces = np.zeros((self._interface_dim, 1))
            self._node.get_logger().warn("No persistent RIM state available.", throttle_duration_sec=2.0)

        # Monitor timing
        if self._loop_count % 1000 == 0 and self._loop_count > 0:
            dt = actual_period * 1000
            self._node.get_logger().info(f"[DelayRIM Algo] Period: {dt:.4f}ms")

        self._last_update_time = loop_time
        self._loop_count += 1

        return self.latest_forces  # Return last known result

    def _compute_interface_force(
        self, rim: ReducedModelState, haptic_position: np.ndarray, haptic_velocity: np.ndarray
    ) -> None:
        """Compute the interface force based on the current reduced model state"""
        rim.phi_position = rim.position - haptic_position
        rim.phi_velocity = rim.velocity - haptic_velocity
        hl = rim.hl

        rim.interface_force = rim.stiffness * rim.phi_position + (hl * rim.stiffness + rim.damping) * rim.phi_velocity
        # rim.interface_force = -rim.damping * rim.phi_velocity

        # rim.interface_force = self._node._ctrl_force

    def _catchup_delay_rim(
        self, rim: ReducedModelState, rim_delay_seconds: float, haptic_states: List[HapticState]
    ) -> np.ndarray:
        """Compute DelayRIM force with catch-up integration"""
        if not haptic_states:
            self._node.get_logger().warn("No haptic states available for DelayRIM computation")
            return np.zeros((3, 1))

        # Calculate the exact number of integration steps needed for the delay
        # delay_seconds = packet.delay_seconds
        hl = rim.hl
        num_integration_steps = int(math.ceil(rim_delay_seconds / hl))  # e.g., 100ms / 1ms = 100 steps

        # node_logger.info(f"DelayRIM integration: {delay_seconds * 1000:.3f}ms -> {num_integration_steps} steps")

        # Get initial haptic state (oldest available)
        if not haptic_states:
            return np.zeros((self._interface_dim, 1))

        initial_haptic = haptic_states[0]
        haptic_position = initial_haptic.position.reshape((self._interface_dim, 1))
        haptic_velocity = initial_haptic.velocity.reshape((self._interface_dim, 1))

        # Interpolate haptic data over the delay period for integration
        for step in range(num_integration_steps):
            # Calculate current time within the delay period
            current_time_in_delay = step * hl

            # Interpolate haptic state at this time step
            haptic_position, haptic_velocity, haptic_acceleration = self._interpolate_haptic_state(
                haptic_states, initial_haptic.timestamp + current_time_in_delay, hl
            )

            # One-step DelayRIM integration (from local_processes.py oneStep)
            self._step_rim(rim, haptic_position, haptic_velocity, haptic_acceleration)

        # Compute final interface force (from getForce method)
        self._compute_interface_force(rim, haptic_position, haptic_velocity)

        # # Compute final interface force (from getForce method)
        # reduced_model.phi_position = reduced_model.rim_position - haptic_position
        # reduced_model.phi_velocity = reduced_model.rim_velocity - haptic_velocity

        # interface_force = (
        #     -reduced_model.stiffness * reduced_model.phi_position
        #     - (hl * reduced_model.stiffness + reduced_model.damping) * reduced_model.phi_velocity
        # )

        return rim

    def _step_rim(
        self,
        reduced_model: ReducedModelState,
        haptic_pos: np.ndarray,
        haptic_vel: np.ndarray,
        haptic_acc: np.ndarray,
        hl: float | None = None,
    ) -> None:
        """Perform one integration step of the RIM State.

        Integration method: TODO. Semi-implicit

        *original code from local_processes.py oneStep*
        """

        # Update constraint deviation
        reduced_model.phi_position = reduced_model.position - haptic_pos
        reduced_model.phi_velocity = reduced_model.velocity - haptic_vel

        # #! Code from Joe
        # # Force terms from oneStep method
        # regular_force_terms = (
        #     reduced_model.effective_mass @ haptic_acc / reduced_model.hl
        #     + reduced_model.effective_force
        #     - reduced_model.stiffness * reduced_model.phi_position
        #     + (reduced_model.damping + reduced_model.hl * reduced_model.stiffness) * haptic_vel
        # )

        # # Update reduced model state (from oneStep method)
        # reduced_model.rim_velocity = (
        #     reduced_model.mass_factor @ reduced_model.rim_velocity
        #     + reduced_model.hl * reduced_model.inverse_augmented_mass_matrix @ regular_force_terms
        # )
        # reduced_model.rim_position = reduced_model.rim_position + reduced_model.hl * reduced_model.rim_velocity

        # #! V2
        # forces_term = reduced_model.effective_force - reduced_model.stiffness * reduced_model.phi_position
        # rim_velocity_p = (
        #     reduced_model.A_inv @ reduced_model.rim_velocity
        #     + reduced_model.A_inv * (1 / reduced_model.hl) * reduced_model.effective_mass * forces_term
        # )
        # reduced_model.rim_velocity = rim_velocity_p

        # rim_position_p = reduced_model.rim_position + reduced_model.hl * reduced_model.rim_velocity
        # reduced_model.rim_position = rim_position_p

        #! V3
        if hl is None:
            hl = reduced_model.hl

        Di = reduced_model.damping
        Ki = reduced_model.stiffness

        # wi = reduced_model.phi_velocity

        regular_force_terms = (
            reduced_model.effective_force - Ki * reduced_model.phi_position + (Di + hl * Ki) * haptic_vel
        )
        # regular_force_terms = -reduced_model.effective_force

        # regular_force_terms = self._node._ctrl_force[0].reshape(1, 1)  # TODO: DEBUG

        rim_velocity_p = (
            reduced_model.mass_factor @ reduced_model.velocity
            + hl * reduced_model.inverse_augmented_mass_matrix @ regular_force_terms
        )
        reduced_model.velocity = rim_velocity_p

        rim_position_p = reduced_model.position + reduced_model.hl * reduced_model.velocity
        reduced_model.position = rim_position_p

    def _catchup_zoh(self, rim: ReducedModelState, haptic_states: List[HapticState]) -> np.ndarray:
        """
        Compute interface force using Zero-Order Hold.

        Returns:
            np.ndarray (m,): Interface force computed using ZOH method
        """
        if not haptic_states:
            return np.zeros((3, 1))

        # Use first (oldest) haptic state for ZOH
        old_haptic = haptic_states[0]  # (m,)

        # rim_msg: FrankaRIM = packet.rim_msg
        # m = rim_msg.m

        # rim_position = np.array(rim_msg.rim_position).reshape((m,))
        # rim_velocity = np.array(rim_msg.rim_velocity).reshape((m,))

        # phi_position = rim_position - old_haptic.position  # (m,)
        # phi_velocity = rim_velocity - old_haptic.velocity  # (m,)

        # interface_force = (
        #     -packet.rim_msg.interface_stiffness * phi_position - packet.rim_msg.interface_damping * phi_velocity
        # )

        self._compute_interface_force(rim, old_haptic.position, old_haptic.velocity)

        return rim

    def _catchup_zoh_phi(self, rim: ReducedModelState, haptic_states: List[HapticState]) -> np.ndarray:
        """Compute ZOH with current haptic state (ZOHPhi)"""
        if not haptic_states:
            return np.zeros((3, 1))

        # Use latest haptic state for ZOHPhi
        current_haptic = haptic_states[-1]

        # rim_position = np.array(packet.rim_msg.rim_position[:3]).reshape((3, 1))
        # rim_velocity = np.array(packet.rim_msg.rim_velocity[:3]).reshape((3, 1))

        # phi_position = rim_position - current_haptic.position
        # phi_velocity = rim_velocity - current_haptic.velocity

        # interface_force = (
        #     -packet.rim_msg.interface_stiffness * phi_position - packet.rim_msg.interface_damping * phi_velocity
        # )

        self._compute_interface_force(rim, current_haptic.position, current_haptic.velocity)

        return rim

    ###
    # Utilities
    ###
    def _update_performance_stats(self, computation_time: float, total_delay: float) -> None:
        """Update performance monitoring statistics.

        Attributes:
            computation_time (float): Time taken to compute the forces in milliseconds.
            total_delay (float): Total delay from packet creation to force result in milliseconds.

        """
        with self.lock:
            self._computation_times.append(computation_time)
            self._total_delay_times.append(total_delay)

            self.stats.max_computation_time = max(self.stats.max_computation_time, computation_time)

    def get_performance_stats(self) -> PerformanceStats:
        """Get current performance statistics"""
        with self.lock:
            if self._computation_times:
                self.stats.avg_computation_time = np.mean(self._computation_times)
                # self.stats.max_computation_time = max(self.stats.max_computation_time)

                self.stats.avg_total_delay = np.mean(self._total_delay_times)

            self.stats.queue_length = len(self.haptic_history)
            self.stats.packet_queue = len(self.active_computations)

            return self.stats

    def shutdown(self) -> None:
        """Shutdown the DelayRIM manager"""
        self.executor.shutdown(wait=True)
