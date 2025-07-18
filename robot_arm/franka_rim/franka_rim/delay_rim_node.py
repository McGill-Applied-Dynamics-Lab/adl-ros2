import rclpy
from rclpy.node import Node
from arm_interfaces.msg import FrankaRIM, Teleop
from teleop_interfaces.msg import Inverse3State
from geometry_msgs.msg import WrenchStamped, Twist, Point, Quaternion
import numpy as np
import time
from collections import deque
from typing import Optional

from franka_rim.delay_rim import DelayRIM, DelayCompensationMethod


class DelayRIMNode(Node):
    def __init__(self):
        super().__init__("delay_rim_node")
        self.get_logger().info("Initializing DelayRIMNode")

        # Parameters
        self.declare_parameter("rim_topic", "/rim_msg_delayed")  #  'fr3_rim_delayed'
        self.declare_parameter("cmd_topic", "/simple_system/cmd")  # '/teleop/ee_cmd_no_delay
        self.declare_parameter("control_period", 0.001)  # 1kHz control rate
        self.declare_parameter("delay_compensation_method", "DelayRIM")  # 'DelayRIM', 'ZOH', or 'ZOHPhi
        self.declare_parameter("interface_stiffness", 3000.0)
        self.declare_parameter("interface_damping", 100.0)
        self.declare_parameter("force_scaling", 0.02)
        self.declare_parameter("max_workers", 1)  # Threading parameter

        # Get parameters
        self.control_period = self.get_parameter("control_period").get_parameter_value().double_value
        method_str = self.get_parameter("delay_compensation_method").get_parameter_value().string_value
        self._interface_stiffness = self.get_parameter("interface_stiffness").get_parameter_value().double_value
        self._interface_damping = self.get_parameter("interface_damping").get_parameter_value().double_value
        self._force_scaling = self.get_parameter("force_scaling").get_parameter_value().double_value
        max_workers = self.get_parameter("max_workers").get_parameter_value().integer_value

        rim_topic = self.get_parameter("rim_topic").get_parameter_value().string_value
        cmd_topic = self.get_parameter("cmd_topic").get_parameter_value().string_value

        # Validate delay compensation method
        try:
            self._delay_method = DelayCompensationMethod(method_str)
        except ValueError:
            self.get_logger().warn(f"Invalid delay compensation method: {method_str}, using DelayRIM")
            self._delay_method = DelayCompensationMethod.DELAY_RIM

        interface_dim = 1  # Dimension of the interface (denoted as 'm')

        # Initialize DelayRIM manager
        self.delay_rim_manager = DelayRIM(self, interface_dim=interface_dim, max_workers=max_workers)

        # State variables
        self._last_rim_msg = None
        self._last_inverse3_msg = None
        self._interface_forces = np.zeros((3, 1))

        # Persistent state for continuous 1kHz stepping
        # self._persistent_reduced_model: Optional[ReducedModelState] = None
        # self._persistent_state_initialized = False
        # self._last_delayrim_result_id = -1  # Track which DelayRIM result we last processed
        self._persistent_initialized = False

        # Frequency monitoring
        self._control_loop_times = deque(maxlen=100)  # Store last 100 loop times
        self._control_loop_count = 0
        self._last_control_time = None
        self._desired_frequency = 1.0 / self.control_period

        # Subscribers
        self._rim_sub = self.create_subscription(FrankaRIM, rim_topic, self._rim_callback, 10)
        self._inverse3_sub = self.create_subscription(Inverse3State, "/inverse3/state", self._inverse3_callback, 10)

        # Publishers
        self._force_pub = self.create_publisher(WrenchStamped, "/inverse3/wrench_des", 10)
        self._teleop_pub = self.create_publisher(Teleop, cmd_topic, 10)

        # Control timer
        self._control_timer = self.create_timer(self.control_period, self._control_timer_callback)

        # Performance monitoring timer
        self._stats_timer = self.create_timer(2.0, self._log_performance_stats)

        self.get_logger().info(
            f"DelayRIMNode started with method={self._delay_method.value}, "
            f"control_period={self.control_period}s, max_workers={max_workers}, "
            f"RIM topic: {rim_topic}, Command topic: {cmd_topic}, "
            f"Expected frequency: {self._desired_frequency:.1f}Hz"
        )

    def _rim_callback(self, msg: FrankaRIM):
        """Callback for delayed RIM messages - submit for processing"""
        self._last_rim_msg = msg

        # Submit packet for delayed processing
        packet_id = self.delay_rim_manager.submit_rim_packet(self, msg, self._delay_method)

        self.get_logger().debug(f"Submitted RIM packet {packet_id} for DelayRIM processing")

    def _inverse3_callback(self, msg: Inverse3State):
        """Callback for Inverse3 state - add to haptic history"""
        self._last_inverse3_msg = msg
        self.delay_rim_manager.add_haptic_state(msg)

    def _compute_interface_forces(self) -> np.ndarray:
        """Get the latest DelayRIM computation result or step persistent model"""
        if self._last_rim_msg is None or self._last_inverse3_msg is None:
            self.get_logger().debug("No RIM or Inverse3 state available, returning zero force")
            return np.zeros((3,))

        interface_forces = self.delay_rim_manager.get_latest_forces()

        if interface_forces is not None:
            self._interface_forces = interface_forces
            return interface_forces

        else:
            # Use fallback if no result available
            self.get_logger().debug("No DelayRIM result available, using fallback")
            self._interface_forces = np.zeros((3, 1))  # Reset to zero if no forces available
            return self._interface_forces

        # # Get current haptic state
        # haptic_position, haptic_velocity = self._get_current_haptic_state()

        # # Get latest DelayRIM result or step persistent model
        # interface_forces = self.delay_rim_manager.get_latest_forces_or_step_persistent(haptic_position, haptic_velocity)

        # # Update fallback force
        # if interface_forces is not None:
        #     self._fallback_force = interface_forces
        #     return interface_forces.flatten()
        # else:
        #     return self._fallback_force.flatten()

    def _get_current_haptic_state(self) -> tuple[np.ndarray, np.ndarray]:
        """Get current haptic position and velocity"""
        if self._last_inverse3_msg is None:
            return np.zeros((1, 1)), np.zeros((1, 1))

        # Extract haptic position (1D interface - y-axis)
        haptic_position = (
            np.array([self._last_inverse3_msg.pose.position.y]).reshape((1, 1)) * 5
        )  # Apply scaling for simple system

        # Extract haptic velocity
        haptic_velocity = np.array([self._last_inverse3_msg.twist.linear.y]).reshape((1, 1))

        return haptic_position, haptic_velocity

    def _publish_force(self, force):
        """Publish computed force to haptic device"""
        force_msg = WrenchStamped()
        force_msg.header.stamp = self.get_clock().now().to_msg()
        force_msg.header.frame_id = "haptic_device"

        # Apply force scaling and coordinate transform
        rendered_forces = -self._force_scaling * force
        force_msg.wrench.force.y = float(rendered_forces[0])
        force_msg.wrench.force.x = 0.0
        force_msg.wrench.force.z = 0.0

        # self.get_logger().info(
        #     f"I3 force: {force_msg.wrench.force.y:.3f}N",
        #     throttle_duration_sec=0.1,
        # )

        self._force_pub.publish(force_msg)

    def _publish_teleop_command(self):
        """
        Publish teleoperation command to the robot based on haptic input.
        For now, this is a placeholder.
        """
        if self._last_inverse3_msg is None:
            return

        teleop_msg = Teleop()
        teleop_msg.header.stamp = self.get_clock().now().to_msg()
        teleop_msg.control_mode = Teleop.CONTROL_MODE_POSITION

        # Extract haptic position for teleop command
        haptic_position = np.array(
            [
                self._last_inverse3_msg.pose.position.y,
                -self._last_inverse3_msg.pose.position.x,
                self._last_inverse3_msg.pose.position.z,
            ]
        )
        haptic_position[0] += 0.4253

        ee_pos_des_msg = Point()
        ee_pos_des_msg.x = haptic_position[0]
        ee_pos_des_msg.y = 0.0
        ee_pos_des_msg.z = 0.025

        ee_quat_des = Quaternion()
        ee_quat_des.x = 1.0
        ee_quat_des.y = 0.0
        ee_quat_des.z = 0.0
        ee_quat_des.w = 0.0

        teleop_msg.ee_des.position = ee_pos_des_msg
        teleop_msg.ee_des.orientation = ee_quat_des
        teleop_msg.ee_vel_des = Twist()

        self._teleop_pub.publish(teleop_msg)

    def _control_timer_callback(self):
        """Main control loop at 1kHz with frequency monitoring"""
        current_time = time.perf_counter()

        # Monitor frequency
        if self._last_control_time is not None:
            loop_period = current_time - self._last_control_time
            self._control_loop_times.append(loop_period)

            # Log frequency issues
            actual_frequency = 1.0 / loop_period if loop_period > 0 else 0.0
            frequency_error = abs(actual_frequency - self._desired_frequency) / self._desired_frequency

            # if frequency_error > 0.1:  # More than 10% error
            #     self.get_logger().warn(
            #         f"Control loop frequency deviation: {actual_frequency:.1f}Hz "
            #         f"(expected {self._desired_frequency:.1f}Hz, error: {frequency_error * 100:.1f}%)",
            #         throttle_duration_sec=1.0,
            #     )

            # self.get_logger().info(f"Control loop frequency: {actual_frequency:.1f}Hz")

        self._last_control_time = current_time
        self._control_loop_count += 1

        # Check if we have haptic data
        if self._last_inverse3_msg is None:
            self.get_logger().debug("No Inverse3 state available", throttle_duration_sec=2.0)
            self._publish_force(np.zeros((3, 1)))
            return

        # Get latest DelayRIM result
        interface_forces = self._compute_interface_forces()

        # Publish force and teleop commands
        self._publish_force(interface_forces)
        self._publish_teleop_command()

    def _log_performance_stats(self):
        """Log performance statistics including frequency monitoring"""
        stats = self.delay_rim_manager.get_performance_stats()

        # Calculate frequency statistics
        if len(self._control_loop_times) > 1:
            periods = np.array(self._control_loop_times)
            avg_period = np.mean(periods)
            std_period = np.std(periods)
            min_period = np.min(periods)
            max_period = np.max(periods)

            avg_frequency = 1.0 / avg_period if avg_period > 0 else 0.0
            min_frequency = 1.0 / max_period if max_period > 0 else 0.0
            max_frequency = 1.0 / min_period if min_period > 0 else 0.0

            frequency_jitter = std_period / avg_period * 100 if avg_period > 0 else 0.0

            self.get_logger().info(
                f"DelayRIM Stats - "
                # f"Total packets: {stats.total_packets}, "
                f"Avg comp time: {stats.avg_computation_time:.1f}ms, "
                f"Max comp time: {stats.max_computation_time:.1f}ms, "
                f"Avg total delay: {stats.avg_total_delay:.1f}ms, "
                f"Queue: {stats.packet_queue}, "
                # f"Queue length: {stats.queue_length}, "
                # f"Dropped: {stats.dropped_packets}, "
            )

            # self.get_logger().info(
            #     f"Control Loop Stats - "
            #     # f"Total calls: {self._control_loop_count}, "
            #     f"Avg freq: {avg_frequency:.1f}Hz "
            #     f"(target: {self._desired_frequency:.1f}Hz), "
            #     f"Freq range: {min_frequency:.1f}-{max_frequency:.1f}Hz, "
            #     # f"Jitter: {frequency_jitter:.1f}%"
            # )
        else:
            self.get_logger().info(
                f"DelayRIM Stats - "
                f"Total packets: {stats.total_packets}, "
                f"Avg comp time: {stats.avg_computation_time:.1f}ms, "
                f"Max comp time: {stats.max_computation_time:.1f}ms, "
                f"Active threads: {stats.packet_queue}, "
                # f"Queue length: {stats.queue_length}, "
                # f"Dropped: {stats.dropped_packets}, "
                # f"Control loops: {self._control_loop_count} (freq monitoring starting...)"
            )

    def destroy_node(self):
        """Clean shutdown"""
        self.delay_rim_manager.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DelayRIMNode()

    try:
        node.get_logger().info(f"DelayRIMNode launched, end with CTRL-C")
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info(f"KeyboardInterrupt occurred, shutting down.\n")

    finally:
        # Cleanup
        if "node" in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
