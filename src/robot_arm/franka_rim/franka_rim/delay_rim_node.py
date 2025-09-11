import rclpy
from rclpy.node import Node
from arm_interfaces.msg import FrankaRIM, Teleop
from teleop_interfaces.msg import Inverse3State
from geometry_msgs.msg import WrenchStamped, Twist, Point, Quaternion, PointStamped, PoseStamped, TwistStamped, Pose
import numpy as np
import time
from collections import deque
from typing import Optional

from franka_rim.delay_rim import DelayRIM, DelayCompensationMethod

from rclpy.callback_groups import ReentrantCallbackGroup


from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
import statistics

from adg_ros2_utils.debug_utils import wait_for_debugger
from adg_ros2_utils.msg_utils import np2ros

NODE_NAME = "delay_rim_node"

MONITOR_TIMING = True


class DelayRIMNode(Node):
    def __init__(self):
        super().__init__("delay_rim_node")
        self.get_logger().info("Initializing DelayRIMNode")

        # --- Parameters
        # Topic names
        self.declare_parameter("rim_topic", "/fr3_rim_delayed")  #  'fr3_rim_delayed', 'rim_msg_delayed'
        self.declare_parameter("haptic_pose_topic_name", "/haptic_pose")
        self.declare_parameter("haptic_twist_topic_name", "/haptic_twist")

        self.declare_parameter("i3_wrench_topic_name", "/i3/wrench")
        self.declare_parameter("i3_pose_topic_name", "/i3/pose")
        self.declare_parameter("i3_twist_topic_name", "/i3/twist")

        self.declare_parameter("interface_force_topic_name", "/fr3/interface_force")

        # Parameters
        self.declare_parameter("update_freq", 1000.0)  # Default: 1 KHz
        self.declare_parameter("delay_compensation_method", "delay_rim")  # 'delay_rim', 'zoh', or 'zoh_phi'
        # self.declare_parameter("interface_stiffness", 3000.0)
        # self.declare_parameter("interface_damping", 100.0)
        self.declare_parameter("force_scaling", 0.02)
        self.declare_parameter("max_workers", 1)  # Threading parameter
        self.declare_parameter("i3_position_scale", 1.0)  # Mapping between I3 and end-effector position

        # --- Get parameters
        self._update_freq = self.get_parameter("update_freq").get_parameter_value().double_value
        self._update_period = 1.0 / self._update_freq

        self._force_scaling = self.get_parameter("force_scaling").get_parameter_value().double_value
        self._max_workers = self.get_parameter("max_workers").get_parameter_value().integer_value
        self._i3_position_scale = self.get_parameter("i3_position_scale").get_parameter_value().double_value

        # Topic names
        self._rim_topic_name = self.get_parameter("rim_topic").get_parameter_value().string_value
        self._i3_pose_topic_name = self.get_parameter("i3_pose_topic_name").get_parameter_value().string_value
        self._i3_twist_topic_name = self.get_parameter("i3_twist_topic_name").get_parameter_value().string_value
        self._interface_force_topic_name = (
            self.get_parameter("interface_force_topic_name").get_parameter_value().string_value
        )

        self._haptic_pose_topic_name = self.get_parameter("haptic_pose_topic_name").get_parameter_value().string_value
        self._haptic_twist_topic_name = self.get_parameter("haptic_twist_topic_name").get_parameter_value().string_value
        self._i3_wrench_topic_name = self.get_parameter("i3_wrench_topic_name").get_parameter_value().string_value

        # Validate delay compensation method
        delay_comp_method_str = self.get_parameter("delay_compensation_method").get_parameter_value().string_value
        try:
            self._delay_method = DelayCompensationMethod(delay_comp_method_str.lower())

        except ValueError:
            self.get_logger().warn(f"Invalid delay compensation method: {delay_comp_method_str}, using DelayRIM")
            self._delay_method = DelayCompensationMethod.DELAY_RIM

        interface_dim = 1  # Dimension of the interface (denoted as 'm')

        # Initialize DelayRIM manager
        self.delay_rim_manager = DelayRIM(self, interface_dim=interface_dim, max_workers=self._max_workers)

        # --- State variables
        self.is_initialized = False

        self._last_rim_msg = None
        self._last_i3_pose_msg: PoseStamped = None
        self._last_i3_twist_msg: TwistStamped = None

        self._i3_ws_position = np.zeros(3)  # Current I3 position in robot workspace
        self._i3_ws_velocity = np.zeros(3)  # Current I3 velocity in robot workspace

        self._robot_pose = None  # np.zeros(3)  # Current robot end-effector position

        self.rim_state = None  # Estimated RIM state from DelayRIM computation

        self._interface_force = np.zeros((3, 1))  # Between RIM and haptic device

        # Real interface force from controller (for debugging)
        self._real_interface_force = np.zeros((3,))

        # self._desired_ee_position: np.ndarray | None = None  # Desired end-effector position from teleop commands

        self._workspace_centre = np.array([0.40, 0.0, 0.4854])
        # self._workspace_position = np.array([0.3085, 0.0, 0.4854])  # Home position
        # self._workspace_position = np.array([0.4253, 0.0, 0.00])  # Cube position

        self._T_i3_robot = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])  # Swap x and y, invert i3_xs

        # --- Frequency monitoring
        self._actual_periods = deque(maxlen=100)  # Store last 100 loop times
        self._loop_count = 0
        self._last_timer_time = None

        # Control timers - automatically use wall timer when sim_time is enabled to bypass /clock quantization
        use_sim_time = self.get_parameter("use_sim_time").get_parameter_value().bool_value

        if use_sim_time:
            self.get_logger().info("Simulation time detected - using wall timers to bypass /clock quantization (~25ms)")
            clock = rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.STEADY_TIME)
        else:
            self.get_logger().info("Using standard ROS timers for real-time operation")
            clock = None

        # --- Timers
        self._control_timer = self.create_timer(
            self._update_period, self._update, clock=clock, callback_group=ReentrantCallbackGroup()
        )
        self._haptic_state_timer = self.create_timer(
            self._update_period, self._pub_haptic_state, clock=clock, callback_group=ReentrantCallbackGroup()
        )
        self._add_haptic_state_cmd_timer = self.create_timer(
            self._update_period,
            self._add_haptic_state,
            clock=clock,
            callback_group=ReentrantCallbackGroup(),
        )

        # Add sim time diagnostic
        self.get_logger().info(f"Using simulation time: {use_sim_time}")

        # Performance monitoring timer
        # TODO: Uncomment for stats logging
        # self._stats_timer = self.create_timer(2.0, self._log_performance_stats)

        self._init_subscribers_and_publishers()

        self.get_logger().info(
            f"DelayRIMNode started with method={self._delay_method.value}, "
            f"control_period={self._update_period}s, max_workers={self._max_workers}, "
            f"RIM topic: {self._rim_topic_name}, Command topic: {self._haptic_pose_topic_name}, "
            f"Expected frequency: {self._update_freq:.1f}Hz"
        )

    def _init_subscribers_and_publishers(self):
        """Initialize subscribers and publishers"""
        #! Subscribers
        self.get_logger().info("Initializing subscribers...")

        # Subscribers
        self._rim_sub = self.create_subscription(FrankaRIM, self._rim_topic_name, self._rim_callback, 10)

        self._i3_pose_sub = self.create_subscription(
            PoseStamped, self._i3_pose_topic_name, self._inverse3_pose_callback, 10
        )
        self._i3_twist_sub = self.create_subscription(
            TwistStamped, self._i3_twist_topic_name, self._inverse3_twist_callback, 10
        )

        self._interface_force_sub = self.create_subscription(
            WrenchStamped,
            self._interface_force_topic_name,
            self._interface_force_callback,
            10,
        )

        self._robot_pose_sub = self.create_subscription(
            PoseStamped,
            "/fr3/current_pose",
            self._robot_pose_callback,
            10,
        )

        #! Publishers
        # Publishers
        self._rendered_force_pub = self.create_publisher(WrenchStamped, self._i3_wrench_topic_name, 10)

        self._haptic_pose_pub = self.create_publisher(PoseStamped, self._haptic_pose_topic_name, 10)
        self._haptic_twist_pub = self.create_publisher(TwistStamped, self._haptic_twist_topic_name, 10)

        self._interface_force_pub = self.create_publisher(WrenchStamped, "/rim/interface_force", 10)
        self._rim_state_pose_pub = self.create_publisher(PoseStamped, "/rim/pose", 10)
        self._rim_state_twist_pub = self.create_publisher(TwistStamped, "/rim/twist", 10)

    # ------------------------------------
    # --- Subscribers and Publishers -----
    # ------------------------------------
    # Subscriber callbacks
    def _rim_callback(self, msg: FrankaRIM):
        """Callback for delayed RIM messages - submit for processing"""
        self._last_rim_msg = msg

        # Submit packet for delayed processing
        packet_id = self.delay_rim_manager.submit_rim_packet(msg, self._delay_method)

        self.get_logger().debug(f"Submitted RIM packet {packet_id} for DelayRIM processing")

    def _inverse3_pose_callback(self, msg: Inverse3State):
        """Callback for Inverse3 state - add to haptic history

        Process the I3 message: changes the coordinate system to match the robot's and add the workspace offset.
        """
        if not self.is_initialized:
            return

        self._last_i3_pose_msg = msg

        i3_position = np.array(
            [
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
            ]
        )

        # Apply workspace offset, scaling and transform to robot frame
        active_axes = np.array([[1, 0, 0], [0, 0, 0], [0, 0, 0]])  # Only use x axis for yz plane teleop
        T_i3_rim = self._i3_position_scale * active_axes @ self._T_i3_robot
        self.i3_ws_position = T_i3_rim @ i3_position + self._workspace_centre

    def _inverse3_twist_callback(self, msg: TwistStamped):
        """Callback for Inverse3 twist - add to haptic history"""
        self._last_i3_twist_msg = msg

        i3_velocity = np.array(
            [
                msg.twist.linear.x,
                msg.twist.linear.y,
                msg.twist.linear.z,
            ]
        )

        self._i3_ws_velocity = self._i3_position_scale * self._T_i3_robot @ i3_velocity

    def _interface_force_callback(self, msg: WrenchStamped):
        """Callback for the interfaction force from the ros2 *haptic controller*."""
        force = np.array(
            [
                msg.wrench.force.x,
                msg.wrench.force.y,
                msg.wrench.force.z,
            ]
        )
        self._real_interface_force = force

    def _robot_pose_callback(self, msg: PoseStamped):
        """Callback for the current robot end-effector pose"""
        self._robot_pose = np.array(
            [
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
            ]
        )

    # Publisher methods
    def _pub_interface_force(self):
        """To Publish the computed interface force between RIM and haptic device.

        Publishes to:
        - ``/rim/interface``: Interface force (WrenchStamped)
        - ``/i3/wrench``: Rendered force to haptic device (WrenchStamped)

        """
        interface_force = self._interface_force.reshape(
            -1,
        )
        rendered_force = -self._force_scaling * interface_force
        # rendered_force = self._T_i3_robot.T @ rendered_force  # Transform to I3 frame TODO

        #! Publish interface force
        interface_force_msg = WrenchStamped()
        interface_force_msg.header.stamp = self.get_clock().now().to_msg()
        interface_force_msg.header.frame_id = "haptic_device"

        # Apply force scaling and coordinate transform
        interface_force_msg.wrench.force.x = float(interface_force[0])
        interface_force_msg.wrench.force.y = 0.0
        interface_force_msg.wrench.force.z = 0.0
        self._interface_force_pub.publish(interface_force_msg)

        #! Publish rendered force
        force_msg = WrenchStamped()
        force_msg.header.stamp = self.get_clock().now().to_msg()
        force_msg.header.frame_id = "haptic_device"

        # Apply force scaling and coordinate transform
        force_msg.wrench.force.y = float(rendered_force[0])
        force_msg.wrench.force.x = 0.0
        force_msg.wrench.force.z = 0.0

        self._rendered_force_pub.publish(force_msg)

    def _publish_teleop_command(self):
        """
        Publish teleoperation command to the robot based on haptic input.
        For now, this is a placeholder.
        """
        if self._last_i3_pose_msg is None or self.rim_state is None:
            return

        robot_pose_msg = PoseStamped()
        robot_pose_msg.header.stamp = self.get_clock().now().to_msg()
        # robot_pose_msg.control_mode = Teleop.CONTROL_MODE_POSITION

        #! Old - Send the haptic position as the desired end-effector position
        # Extract haptic position for teleop command
        # haptic_position = np.array(
        #     [
        #         self._last_inverse3_msg.pose.position.y,
        #         -self._last_inverse3_msg.pose.position.x,
        #         self._last_inverse3_msg.pose.position.z,
        #     ]
        # )

        # Lateral commands
        ee_pose_des_msg = Pose()

        ee_pose_des_msg.position.x = self._desired_ee_position[0]
        ee_pose_des_msg.position.y = self._desired_ee_position[1]
        ee_pose_des_msg.position.z = self._desired_ee_position[2]
        robot_pose_msg.pose = ee_pose_des_msg

        # Orientation - Pointing downwards
        ee_quat_des = Quaternion()
        ee_quat_des.x = 1.0
        ee_quat_des.y = 0.0
        ee_quat_des.z = 0.0
        ee_quat_des.w = 0.0

        robot_pose_msg.pose.orientation = ee_quat_des

        # # Vertical commands
        # haptic_position[0] += 0.025

        # ee_pos_des_msg = Point()
        # ee_pos_des_msg.x = 0.4253
        # ee_pos_des_msg.y = 0.0
        # ee_pos_des_msg.z = haptic_position[0]

        # # Send rim position as desired end-effector position
        # rim_position = self.rim_state.position

        # ee_pos_des_msg = Point()
        # ee_pos_des_msg.x = float(rim_position[0])
        # ee_pos_des_msg.y = 0.0
        # ee_pos_des_msg.z = 0.025

        self._teleop_pub.publish(robot_pose_msg)

    def _publish_rim_state(self):
        """
        Publish the estimated RIM state position.

        Args:
            rim_state (ReducedModelState): The estimated RIM state from DelayRIM computation
        """
        if self.rim_state is None:
            return

        rim_state_pose_msg = PoseStamped()
        rim_state_pose_msg.header.stamp = self.get_clock().now().to_msg()
        rim_state_pose_msg.header.frame_id = "world"

        # Extract position from RIM state (convert from (m,1) array to individual coordinates)
        rim_world_x = self.rim_state.position.flatten()[0]
        rim_world_position = np.array([rim_world_x, self._workspace_centre[1], self._workspace_centre[2]])

        rim_state_pose_msg.pose.position.x = float(rim_world_position[0])
        rim_state_pose_msg.pose.position.y = float(rim_world_position[1])
        rim_state_pose_msg.pose.position.z = float(rim_world_position[2])

        self._rim_state_pose_pub.publish(rim_state_pose_msg)

        # --- Twist
        rim_state_twist_msg = TwistStamped()
        rim_state_twist_msg.header.stamp = self.get_clock().now().to_msg()
        rim_state_twist_msg.header.frame_id = "world"

        rim_twist = self.rim_state.velocity.flatten()
        rim_state_twist_msg.twist.linear.x = float(rim_twist[0])

        self._rim_state_twist_pub.publish(rim_state_twist_msg)

    def _pub_haptic_state(self):
        """Callback to publish the haptic state. Will be read by the *haptic controller*.

        Publishes at `update_freq` (default 1kHz) to `haptic_pose_topic_name` and `haptic_twist_topic_name` (Default:
        "/haptic_pose" and "/haptic_twist").
        """
        if self._last_i3_pose_msg is None or self._last_i3_twist_msg is None:
            self.get_logger().warn("No Inverse3 state available to publish", throttle_duration_sec=2.0)
            return

        # I3 Pose message
        i3_pose_msg = PoseStamped()
        i3_pose_msg.header.stamp = self.get_clock().now().to_msg()
        i3_pose_msg.header.frame_id = "haptic_device"
        i3_pose_msg.pose.position = np2ros(self.i3_ws_position, "Point")

        i3_pose_msg.pose.orientation = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)  # Downward orientation

        # I3 Twist message
        i3_twist_msg = TwistStamped()
        i3_twist_msg.header.stamp = self.get_clock().now().to_msg()
        i3_twist_msg.header.frame_id = "haptic_device"
        i3_twist_msg.twist.linear = np2ros(self._i3_ws_velocity, "Vector3")

        # Publish command
        # self._publish_teleop_command()
        self._haptic_pose_pub.publish(i3_pose_msg)
        self._haptic_twist_pub.publish(i3_twist_msg)

    # Timer callbacks
    def _add_haptic_state(self):
        """Callback to add haptic state to DelayRIM at control frequency."""
        if self._last_i3_pose_msg is None or self._last_i3_twist_msg is None:
            self.get_logger().debug("No Inverse3 state available for haptic state update", throttle_duration_sec=2.0)
            return

        self.delay_rim_manager.add_haptic_state(self.i3_ws_position, self._i3_ws_velocity)

    # ----------------------------
    # --- DelayRIM computation ---
    # ----------------------------
    def _update(self):
        """Main control loop at 1kHz with frequency monitoring"""
        if not self.is_initialized:
            if self._robot_pose is None:
                self.get_logger().warn("Waiting for initial robot pose...", throttle_duration_sec=2.0)
                return
            else:
                self.get_logger().info("Received initial robot pose, initializing DelayRIM...")
                self.is_initialized = True

                self._workspace_centre = self._robot_pose.copy()

                self.get_logger().info("DelayRIM initialized.")

        loop_start_time = time.perf_counter()

        # -- Track timing
        if self._loop_count > 0:
            actual_period = loop_start_time - self._last_timer_time
            self._actual_periods.append(actual_period)

        self._last_timer_time = loop_start_time

        # Check if we have haptic data
        if self._last_i3_pose_msg is None or self._last_i3_twist_msg is None:
            self.get_logger().debug("No Inverse3 state available", throttle_duration_sec=2.0)
            self._interface_force = np.zeros((3, 1))
            self._pub_interface_force()
            return

        # Get latest DelayRIM result
        self._interface_force = self._compute_interface_force()
        self.rim_state = self.delay_rim_manager.rim_state

        # Publish force and teleop commands
        self._pub_interface_force()
        # self._publish_haptic_state()
        self._publish_rim_state()

        # Log control loop frequency
        self._log_timer_monitoring(log_period=3.0)

        self._loop_count += 1

    def _compute_interface_force(self) -> np.ndarray:
        """Get the latest DelayRIM computation result or step persistent model"""
        interface_force = self.delay_rim_manager.get_interface_force()

        if interface_force is not None:
            return interface_force

        else:
            # Use fallback if no result available
            self.get_logger().debug("No DelayRIM result available, using fallback")
            return np.zeros((3, 1))  # Reset to zero if no force available

    # --- Timers
    def _log_timer_monitoring(self, log_period=1.0):
        """
        To log timer diagnostics at a specified period.
        """
        if self._loop_count % (log_period * self._update_freq) == 0 and self._loop_count > 0:
            avg_period = statistics.mean(self._actual_periods) * 1000  # Convert to ms
            min_period = min(self._actual_periods) * 1000
            max_period = max(self._actual_periods) * 1000
            std_period = statistics.stdev(self._actual_periods) * 1000 if len(self._actual_periods) > 1 else 0

            if MONITOR_TIMING:
                self.get_logger().info(
                    f"Timer diagnostics - Avg: {avg_period:.2f}ms | Target: {self._update_period * 1000:.2f}ms | "
                    # f"Min: {min_period:.2f}ms | Max: {max_period:.2f}ms | "
                    # f"Std: {std_period:.2f}ms | Thread: {threading.current_thread().name}"
                )

            if avg_period > self._update_period * 1.5 * 1000:
                self.get_logger().warn(
                    f"High update period detected: {avg_period:.2f}ms (target: {self._update_period * 1000:.2f}ms)"
                )

    def _log_performance_stats(self):
        """Log performance statistics including frequency monitoring"""
        stats = self.delay_rim_manager.get_performance_stats()

        # Calculate frequency statistics
        if len(self._actual_periods) > 1:
            periods = np.array(self._actual_periods)
            avg_period = np.mean(periods)
            std_period = np.std(periods)
            min_period = np.min(periods)
            max_period = np.max(periods)

            avg_frequency = 1.0 / avg_period if avg_period > 0 else 0.0
            min_frequency = 1.0 / max_period if max_period > 0 else 0.0
            max_frequency = 1.0 / min_period if min_period > 0 else 0.0

            frequency_jitter = std_period / avg_period * 100 if avg_period > 0 else 0.0

            # self.get_logger().info(
            #     f"DelayRIM Stats - "
            #     # f"Total packets: {stats.total_packets}, "
            #     f"Avg comp time: {stats.avg_computation_time:.1f}ms, "
            #     f"Max comp time: {stats.max_computation_time:.1f}ms, "
            #     f"Avg total delay: {stats.avg_total_delay:.1f}ms, "
            #     f"Queue: {stats.packet_queue}, "
            #     # f"Queue length: {stats.queue_length}, "
            #     # f"Dropped: {stats.dropped_packets}, "
            # )

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
    wait_for_debugger(NODE_NAME)  # Wait for debugger if env variables is set

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
