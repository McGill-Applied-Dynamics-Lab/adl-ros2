"""
RIM Stepper Node - Simple validation node for Reduced Interface Model (RIM)

This node subscribes to FrankaRIM messages, integrates the RIM dynamics forward in time,
and publishes the resulting RIM state for validation against the actual robot state.
"""

import rclpy
from rclpy.node import Node
from arm_interfaces.msg import FrankaRIM
from geometry_msgs.msg import PoseStamped, TwistStamped, Quaternion
import numpy as np
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Optional

from adg_ros2_utils.debug_utils import wait_for_debugger

NODE_NAME = "rim_stepper_node"


@dataclass
class RIMState:
    """State of the reduced interface model"""

    # RIM parameters
    stiffness: float = 3000.0
    damping: float = 100.0
    interface_dim: int = 1  # Dimension of the interface ('m')

    # RIM state (position and velocity)
    position: np.ndarray = field(default_factory=lambda: np.zeros((1, 1)))
    velocity: np.ndarray = field(default_factory=lambda: np.zeros((1, 1)))

    # Effective dynamics parameters
    effective_mass: np.ndarray = field(default_factory=lambda: np.eye(1))
    effective_force: np.ndarray = field(default_factory=lambda: np.zeros((1, 1)))

    # Timestamp
    timestamp: float = 0.0


class RIMStepperNode(Node):
    def __init__(self):
        super().__init__("rim_stepper_node")
        self.get_logger().info("Initializing RIMStepperNode")

        # --- Parameters ---
        # Topic names
        self.declare_parameter("rim_topic", "/fr3_rim")
        self.declare_parameter("output_pose_topic", "/rim/pose")
        self.declare_parameter("output_twist_topic", "/rim/twist")

        # Control parameters
        self.declare_parameter("update_freq", 1000.0)  # Default: 1 KHz
        self.declare_parameter("rim_axis", "x")  # Axis for RIM ('x', 'y', 'z')

        # Drift compensation parameters
        self.declare_parameter("complementary_filter_alpha", 0.02)  # 0=trust integration, 1=trust measurement

        # Logging
        self.declare_parameter("log.enabled", False)
        self.declare_parameter("log.period", 1.0)

        # --- Get parameters ---
        self._update_freq = self.get_parameter("update_freq").get_parameter_value().double_value
        self._update_period = 1.0 / self._update_freq
        self._dt = self._update_period  # Integration timestep

        self._rim_topic_name = self.get_parameter("rim_topic").get_parameter_value().string_value
        self._output_pose_topic = self.get_parameter("output_pose_topic").get_parameter_value().string_value
        self._output_twist_topic = self.get_parameter("output_twist_topic").get_parameter_value().string_value

        self._rim_axis_str = self.get_parameter("rim_axis").get_parameter_value().string_value.lower()

        self._complementary_filter_alpha = (
            self.get_parameter("complementary_filter_alpha").get_parameter_value().double_value
        )

        self._log_enabled = self.get_parameter("log.enabled").get_parameter_value().bool_value
        self._log_period = self.get_parameter("log.period").get_parameter_value().double_value

        # --- State variables ---
        self.is_initialized = False
        self._rim_state: Optional[RIMState] = None
        self._last_rim_msg: Optional[FrankaRIM] = None

        self._workspace_centre: Optional[np.ndarray] = None

        # Coordinate transformations
        x_val = 0
        y_val = 0
        z_val = 0
        if "x" in self._rim_axis_str:
            x_val = 1
        elif "y" in self._rim_axis_str:
            y_val = 1
        elif "z" in self._rim_axis_str:
            z_val = 1
        else:
            raise ValueError(f"Invalid rim_axis parameter: {self._rim_axis_str}. Must be 'x', 'y', or 'z'.")

        self._T_rim_robot = np.array([[x_val, y_val, z_val]])  # Transformation from RIM to robot frame (1, 3)

        # Logging and timing
        self._loop_count = 0
        self._last_timer_time = None
        self._actual_periods = deque(maxlen=100)

        # Check for sim time
        use_sim_time = self.get_parameter("use_sim_time").get_parameter_value().bool_value

        if use_sim_time:
            self.get_logger().info("Simulation time detected - using wall timers")
            clock = rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.STEADY_TIME)
        else:
            self.get_logger().info("Using standard ROS timers for real-time operation")
            clock = None

        # --- Subscribers and Publishers ---
        self._rim_sub = self.create_subscription(FrankaRIM, self._rim_topic_name, self._rim_callback, 10)

        self._pose_pub = self.create_publisher(PoseStamped, self._output_pose_topic, 10)
        self._twist_pub = self.create_publisher(TwistStamped, self._output_twist_topic, 10)

        # --- Timer ---
        self._update_timer = self.create_timer(self._update_period, self._update, clock=clock)

        # Log initialization
        self.get_logger().info(
            f"\n--- RIM Stepper Node Initialized ---\n"
            f"Parameters:\n"
            f"  - Update frequency: {self._update_freq:.1f} Hz ({self._update_period * 1000:.3f} ms)\n"
            f"  - Active axis: '{self._rim_axis_str}'\n"
            f"  - Complementary filter alpha: {self._complementary_filter_alpha:.3f}\n"
            f"  - Use sim time: {use_sim_time}\n"
            f"Topics:\n"
            f"  - RIM input: {self._rim_topic_name}\n"
            f"  - Pose output: {self._output_pose_topic}\n"
            f"  - Twist output: {self._output_twist_topic}\n"
            f"Logging:\n"
            f"  - Enabled: {self._log_enabled}\n"
            f"  - Period: {self._log_period}s\n"
        )

    # --- Callbacks ---

    def _rim_callback(self, msg: FrankaRIM):
        """Callback for RIM messages - update parameters and initialize if needed"""
        self._last_rim_msg = msg

        # Initialize on first message
        if not self.is_initialized:
            self._initialize_rim_state(msg)
            return

        # Update RIM parameters from message
        self._update_rim_parameters(msg)

    def _initialize_rim_state(self, msg: FrankaRIM):
        """Initialize RIM state from the first received message"""
        m = msg.m  # Interface dimension

        self._rim_state = RIMState(
            stiffness=msg.interface_stiffness,
            damping=msg.interface_damping,
            interface_dim=m,
            position=np.array(msg.rim_position).reshape((m, 1)),
            velocity=np.array(msg.rim_velocity).reshape((m, 1)),
            effective_mass=np.array(msg.effective_mass).reshape((m, m)),
            effective_force=np.array(msg.effective_force).reshape((m, 1)),
            timestamp=self.get_clock().now().nanoseconds / 1e9,
        )

        # Initialize workspace center from first position
        rim_ws = self._T_rim_robot.T @ self._rim_state.position
        self._workspace_centre = rim_ws.reshape(
            3,
        )

        self.is_initialized = True
        self.get_logger().info(
            f"RIM state initialized:\n"
            f"  - Dimension: {m}\n"
            f"  - Position: {self._rim_state.position.flatten()}\n"
            f"  - Velocity: {self._rim_state.velocity.flatten()}\n"
            f"  - Stiffness: {self._rim_state.stiffness}\n"
            f"  - Damping: {self._rim_state.damping}\n"
            f"  - Workspace center: {self._workspace_centre}"
        )

    def _update_rim_parameters(self, msg: FrankaRIM):
        """Update RIM parameters from incoming message and apply complementary filter to state"""
        if self._rim_state is None:
            return

        m = msg.m
        self._rim_state.stiffness = msg.interface_stiffness
        self._rim_state.damping = msg.interface_damping
        self._rim_state.effective_mass = np.array(msg.effective_mass).reshape((m, m))
        self._rim_state.effective_force = np.array(msg.effective_force).reshape((m, 1))

        # Complementary filter: blend integrated state with measured state to prevent drift
        # alpha = 0: fully trust integration (no correction)
        # alpha = 1: fully trust measurement (no integration)
        alpha = self._complementary_filter_alpha

        measured_pos = np.array(msg.rim_position).reshape((m, 1))
        measured_vel = np.array(msg.rim_velocity).reshape((m, 1))

        self._rim_state.position = (1 - alpha) * self._rim_state.position + alpha * measured_pos
        self._rim_state.velocity = (1 - alpha) * self._rim_state.velocity + alpha * measured_vel

    def _update(self):
        """Main update loop - step RIM forward in time and publish state"""
        # Track timing
        loop_start_time = time.perf_counter()
        if self._loop_count > 0 and self._last_timer_time is not None:
            actual_period = loop_start_time - self._last_timer_time
            self._actual_periods.append(actual_period)

        self._last_timer_time = loop_start_time

        # Check if initialized
        if not self.is_initialized or self._rim_state is None:
            self.get_logger().debug("Waiting for RIM initialization...", throttle_duration_sec=2.0)
            return

        # Step the RIM forward in time
        self._step_rim()

        # Publish the current RIM state
        self._publish_rim_state()

        # Logging
        if self._log_enabled and self._loop_count % int(self._log_period * self._update_freq) == 0:
            self._log_info()

        self._loop_count += 1

    def _step_rim(self):
        """
        Step the RIM dynamics forward by one timestep using forward Euler integration.

        Integration: v_{k+1} = v_k + dt * M^{-1} * F
                     p_{k+1} = p_k + dt * v_{k+1}
        """
        # Update timestamp
        self._rim_state.timestamp = self.get_clock().now().nanoseconds / 1e9

        hl = self._update_period
        wi = self._rim_state.velocity

        # Compute acceleration: a = M^{-1} * F
        # Use @ for proper matrix multiplication
        acceleration = np.linalg.inv(self._rim_state.effective_mass) @ self._rim_state.effective_force

        # Update velocity: v_{k+1} = v_k + dt * a
        wi_p = wi + hl * acceleration

        # Update position: p_{k+1} = p_k + dt * v_{k+1}
        position = self._rim_state.position
        position_p = position + hl * wi_p

        self._rim_state.position = position_p
        self._rim_state.velocity = wi_p

    def _publish_rim_state(self):
        """Publish the current RIM state as PoseStamped and TwistStamped"""
        if self._rim_state is None:
            return

        timestamp = self.get_clock().now().to_msg()

        # --- Publish Pose ---
        pose_msg = PoseStamped()
        pose_msg.header.stamp = timestamp
        pose_msg.header.frame_id = "world"

        # Transform RIM position to world frame
        rim_ws = self._T_rim_robot.T @ self._rim_state.position  # (3, 1)

        # Add offset for non-active axes
        if self._workspace_centre is not None:
            ws_offset = (np.eye(3) - np.diag(self._T_rim_robot[0][:])) @ self._workspace_centre.reshape(3, 1)
            rim_world_position = rim_ws + ws_offset
        else:
            rim_world_position = rim_ws

        rim_world_position = rim_world_position.reshape(
            3,
        )
        pose_msg.pose.position.x = float(rim_world_position[0])
        pose_msg.pose.position.y = float(rim_world_position[1])
        pose_msg.pose.position.z = float(rim_world_position[2])
        pose_msg.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        self._pose_pub.publish(pose_msg)

        # --- Publish Twist ---
        twist_msg = TwistStamped()
        twist_msg.header.stamp = timestamp
        twist_msg.header.frame_id = "world"

        # Transform RIM velocity to world frame
        rim_velocity_ws = self._T_rim_robot.T @ self._rim_state.velocity  # (3, 1)
        rim_velocity_ws = rim_velocity_ws.reshape(
            3,
        )

        twist_msg.twist.linear.x = float(rim_velocity_ws[0])
        twist_msg.twist.linear.y = float(rim_velocity_ws[1])
        twist_msg.twist.linear.z = float(rim_velocity_ws[2])

        self._twist_pub.publish(twist_msg)

    def _log_info(self):
        """Log diagnostic information"""
        if not self._actual_periods:
            return

        avg_period_ms = np.mean(self._actual_periods) * 1000
        target_period_ms = self._update_period * 1000

        if self._rim_state is not None:
            self.get_logger().info(
                f"RIM Stepper - "
                f"Loop: {self._loop_count} | "
                f"Avg period: {avg_period_ms:.2f}ms (target: {target_period_ms:.2f}ms) | "
                f"Position: {self._rim_state.position.flatten()} | "
                f"Velocity: {self._rim_state.velocity.flatten()}"
            )


def main(args=None):
    wait_for_debugger(NODE_NAME)

    rclpy.init(args=args)
    node = RIMStepperNode()

    try:
        node.get_logger().info("RIMStepperNode launched, end with CTRL-C")
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt occurred, shutting down.\n")

    finally:
        if "node" in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
