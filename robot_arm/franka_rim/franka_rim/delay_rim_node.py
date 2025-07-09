import rclpy
from rclpy.node import Node
from arm_interfaces.msg import FrankaRIM, Teleop
from teleop_interfaces.msg import Inverse3State
from geometry_msgs.msg import WrenchStamped, Twist, Point, Quaternion
import numpy as np
import time
from collections import deque

from franka_rim.delay_rim import DelayRIM, DelayCompensationMethod


class DelayRIMNode(Node):
    def __init__(self):
        super().__init__("delay_rim_node")
        self.get_logger().info("Initializing DelayRIMNode")

        # Parameters
        self.declare_parameter("rim_topic", "/fr3_rim_delayed")  # Now using delayed topic
        self.declare_parameter("cmd_topic", "/teleop/ee_cmd_no_delay")
        self.declare_parameter("control_period", 0.001)  # 1kHz control rate
        self.declare_parameter("delay_compensation_method", "ZOH")  # 'DelayRIM', 'ZOH', or 'ZOHPhi
        self.declare_parameter("interface_stiffness", 3000.0)
        self.declare_parameter("interface_damping", 2.0)
        self.declare_parameter("force_scaling", 0.02)
        self.declare_parameter("max_workers", 8)  # Threading parameter

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

        # Initialize DelayRIM manager
        self.delay_rim_manager = DelayRIM(max_workers=max_workers)

        # State variables
        self._last_rim_msg = None
        self._last_inverse3_msg = None
        self._fallback_force = np.zeros((3, 1))

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
        self._stats_timer = self.create_timer(5.0, self._log_performance_stats)

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

    def _compute_rendered_force(self) -> np.ndarray:
        """Get the latest DelayRIM computation result"""
        if self._last_rim_msg is None or self._last_inverse3_msg is None:
            self.get_logger().debug("No RIM or Inverse3 state available, returning zero force")
            return np.zeros((3, 1))

        # Update phi (constraint deviation)
        rim_position = np.array(self._last_rim_msg.rim_position).reshape((3, 1))
        rim_velocity = np.array(self._last_rim_msg.rim_velocity).reshape((3, 1))

        _haptic_position = np.array(
            [
                self._last_inverse3_msg.pose.position.y,
                -self._last_inverse3_msg.pose.position.x,
                self._last_inverse3_msg.pose.position.z,
            ]
        ).reshape((3, 1))
        _haptic_position[0] += 0.4253

        # Extract linear velocity from twist
        _haptic_velocity = np.array(
            [
                self._last_inverse3_msg.twist.linear.y,
                -self._last_inverse3_msg.twist.linear.x,
                self._last_inverse3_msg.twist.linear.z,
            ]
        ).reshape((3, 1))

        phi_position = rim_position[0] - _haptic_position[0]
        phi_velocity = rim_velocity[0] - _haptic_velocity[0]

        forces = -2000 * phi_position - 2 * phi_velocity

        return forces

        #! Thread algo - Get latest result from DelayRIM manager
        result = self.delay_rim_manager.get_latest_result()

        if result is not None:
            self._fallback_force = result
            return result

        else:
            # Use fallback if no result available
            self.get_logger().debug("No DelayRIM result available, using fallback")
            return self._fallback_force

    def _publish_force(self, force):
        """Publish computed force to haptic device"""
        force_msg = WrenchStamped()
        force_msg.header.stamp = self.get_clock().now().to_msg()
        force_msg.header.frame_id = "haptic_device"

        # Apply force scaling and coordinate transform
        scaled_force = -self._force_scaling * force
        force_msg.wrench.force.y = float(scaled_force[0])
        force_msg.wrench.force.x = 0.0
        force_msg.wrench.force.z = 0.0

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

            if frequency_error > 0.1:  # More than 10% error
                self.get_logger().warn(
                    f"Control loop frequency deviation: {actual_frequency:.1f}Hz "
                    f"(expected {self._desired_frequency:.1f}Hz, error: {frequency_error * 100:.1f}%)",
                    throttle_duration_sec=1.0,
                )

            # self.get_logger().info(f"Control loop frequency: {actual_frequency:.1f}Hz")

        self._last_control_time = current_time
        self._control_loop_count += 1

        # Check if we have haptic data
        if self._last_inverse3_msg is None:
            self.get_logger().debug("No Inverse3 state available", throttle_duration_sec=2.0)
            self._publish_force(np.zeros((3, 1)))
            return

        # Get latest DelayRIM result
        rendered_force = self._compute_rendered_force()

        # Publish force and teleop commands
        self._publish_force(rendered_force)
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
                f"DelayRIM Stats - Total packets: {stats.total_packets}, "
                f"Avg comp time: {stats.avg_computation_time:.1f}ms, "
                f"Max comp time: {stats.max_computation_time:.1f}ms, "
                f"Active threads: {stats.active_threads}, "
                f"Queue length: {stats.queue_length}, "
                f"Dropped: {stats.dropped_packets}"
            )

            self.get_logger().info(
                f"Control Loop Stats - Total calls: {self._control_loop_count}, "
                f"Avg freq: {avg_frequency:.1f}Hz "
                f"(target: {self._desired_frequency:.1f}Hz), "
                f"Freq range: {min_frequency:.1f}-{max_frequency:.1f}Hz, "
                f"Jitter: {frequency_jitter:.1f}%"
            )
        else:
            self.get_logger().info(
                f"DelayRIM Stats - Total packets: {stats.total_packets}, "
                f"Avg comp time: {stats.avg_computation_time:.1f}ms, "
                f"Max comp time: {stats.max_computation_time:.1f}ms, "
                f"Active threads: {stats.active_threads}, "
                f"Queue length: {stats.queue_length}, "
                f"Dropped: {stats.dropped_packets}, "
                f"Control loops: {self._control_loop_count} (freq monitoring starting...)"
            )

    def destroy_node(self):
        """Clean shutdown"""
        self.delay_rim_manager.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DelayRIMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
