"""
To teleop the FR3 with the Inverse 3. The i3 position is mapped to the robot workspace and sent as /target_pose.

If OSC_PD controller is used, renders the force (scaled) of the virtual coupling.

This needs the `osc_pd_controller` running on the server to work.

Only publishes the state of the I3 in the robot's base frame.
"""

# ROS imports
from typing import Literal
import rclpy
from rclpy.node import Node  # node class makes a ROS2 Node

from geometry_msgs.msg import (
    PoseStamped,
    TwistStamped,
    WrenchStamped,
    Vector3,
    Point,
    Quaternion,
    PointStamped,
)

# Other
import numpy as np
from enum import Enum

from adg_ros2_utils.debug_utils import wait_for_debugger
from adg_ros2_utils.msg_utils import ros2np, np2ros

NODE_NAME = "i3_teleop_virtual_coupling"

DEBUG = False


class I3Teleop(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.get_logger().info("Initializing I3 Teleop with Virtual Coupling...")

        self.declare_parameter("i3_pose_topic_name", "/i3/pose")
        self.declare_parameter("i3_twist_topic_name", "/i3/twist")
        self.declare_parameter("i3_wrench_topic_name", "/i3/wrench")

        self.declare_parameter("target_pose_topic_name", "/fr3/target_pose")
        self.declare_parameter("target_twist_topic_name", "/fr3/target_twist")

        self.declare_parameter("interface_force_topic_name", "/fr3/interface_force")

        self.declare_parameter("frequency", 100.0)  # frequency of the node

        self.declare_parameter("force_scale", 0.05)  # Scale for the forces applied to the i3
        self.declare_parameter("position_scale", 1.0)
        # self.declare_parameter("velocity_scale", 500)

        self.declare_parameter("pos_radius", 0.060)  # Radius for the position control region

        self.declare_parameter(
            "active_axis", "x"
        )  # Active axis for the position control. Options: "x", "y", "z", "xy", "xz", "yz", "xyz"

        #! ROS2 Params
        self._i3_pose_topic_name = self.get_parameter("i3_pose_topic_name").get_parameter_value().string_value
        self._i3_twist_topic_name = self.get_parameter("i3_twist_topic_name").get_parameter_value().string_value
        self._i3_wrench_topic_name = self.get_parameter("i3_wrench_topic_name").get_parameter_value().string_value
        self._interface_force_topic_name = (
            self.get_parameter("interface_force_topic_name").get_parameter_value().string_value
        )

        self._target_pose_topic_name = self.get_parameter("target_pose_topic_name").get_parameter_value().string_value
        self._target_twist_topic_name = self.get_parameter("target_twist_topic_name").get_parameter_value().string_value

        self._cmd_pub_freq = self.get_parameter("frequency").get_parameter_value().double_value

        #! Important Arrays
        self._i3_position = np.zeros(3)  # Position of the i3
        self._i3_velocities = np.zeros(3)  # Velocities of the i3
        self._i3_forces_des = np.zeros(3)  # Forces to apply to the i3

        self._haptic_position = np.zeros(3)  # Position of the haptic device in the robot's base frame
        self._haptic_linear_vel = np.zeros(3)  # Velocities of the haptic device in the robot's base frame

        self._ee_pose = None  # np.zeros(3)  # End effector position for the robot
        self._ee_quat = np.array([0, 0, 0, 1])  # End effector orientation for the robot
        self._ee_vel = np.zeros(3)  # Velocities of the end effector.
        self._ws_center = np.zeros(3)  # Workspace center position of the I3 in the robot's base frame

        self._interface_forces = np.zeros(3)
        self._ee_contact_forces = np.zeros(3)  # Forces applied at the end effector

        #! Parameters
        self._start_time = None
        self._is_initialized = False

        self._i3_scale = self.get_parameter("position_scale").get_parameter_value().double_value
        self._T_i3_robot = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])  # Swap x and y, invert i3_xs

        # Active axis
        active_axis_str = self.get_parameter("active_axis").get_parameter_value().string_value.lower()
        x_val = 0
        y_val = 0
        z_val = 0
        if "x" in active_axis_str:
            x_val = 1
        if "y" in active_axis_str:
            y_val = 1
        if "z" in active_axis_str:
            z_val = 1
        self._active_axis = np.array([[x_val, 0, 0], [0, y_val, 0], [0, 0, z_val]])

        self._force_calibrated = False
        self._n_force_cal_msgs = 0
        self._force_bias = np.zeros(3)  # Bias for the force sensor
        self._force_deadzone = 5.0  # Deadzone for the force to apply

        # Scale for the forces applied to the i3
        self._i3_forces_scale = self.get_parameter("force_scale").get_parameter_value().double_value
        # Scale for the position of the i3
        # self._i3_position_scale = self.get_parameter("position_scale").get_parameter_value().double_value
        # self._i3_vel_scale = self.get_parameter("velocity_scale").get_parameter_value().double_value

        # Radius for the position control region
        self._pos_radius = self.get_parameter("pos_radius").get_parameter_value().double_value

        # Log parameter values
        self.get_logger().info(
            f"Parameters\n"
            f"- Update frequency: {self._cmd_pub_freq}, \n"
            f"- Position radius: {self._pos_radius}, \n"
            f"- Active axis: {active_axis_str}, \n"
            f"- I3 position scale: {self._i3_scale}, \n"
            f"- I3 force scale: {self._i3_forces_scale}, \n"
            f"- Topic, I3 Pose: {self._i3_pose_topic_name}, \n"
            f"- Topic, I3 twist: {self._i3_twist_topic_name}, \n"
            f"- Topic, I3 wrench: {self._i3_wrench_topic_name}, \n"
            f"- Topic, interface force: {self._interface_force_topic_name}, \n"
            f"- Topic, target pose: {self._target_pose_topic_name}, \n"
            f"- Topic, target twist: {self._target_twist_topic_name}, \n"
        )

        #! Init functions
        self._init_subscribers()
        self._init_publishers()

        # self._calibrate()

        self._main_timer = self.create_timer(1 / self._cmd_pub_freq, self._main_loop)

    def _init_subscribers(self):
        self.get_logger().info("Initializing subscribers...")

        # --- I3 State ---
        self._i3_pose_sub = self.create_subscription(
            PoseStamped, self._i3_pose_topic_name, self._i3_pose_topic_callback, 10
        )
        self._i3_twist_sub = self.create_subscription(
            TwistStamped, self._i3_twist_topic_name, self._i3_twist_topic_callback, 10
        )

        # --- EE Pose ---
        robot_pose_topic = "/fr3/current_pose"
        self._robot_sub = self.create_subscription(PoseStamped, robot_pose_topic, self._robot_pose_topic_callback, 10)

        # --- Contact Forces ---
        # Estimated contact forces from the FR3
        contact_forces_topic = "/franka_robot_state_broadcaster/external_wrench_in_base_frame"
        self._contact_forces_sub = self.create_subscription(
            WrenchStamped, contact_forces_topic, self._contact_forces_topic_callback, 10
        )

        # Virtual Coupling force
        self._interface_force_sub = self.create_subscription(
            WrenchStamped,
            self._interface_force_topic_name,
            self._interface_forces_topic_callback,
            10,
        )

        # # --- ee vel ---
        # ee_vel_topic = "/franka_robot_state_broadcaster/desired_end_effector_twist"
        # self._ee_vel_sub = self.create_subscription(TwistStamped, ee_vel_topic, self._ee_vel_topic_callback, 10)

    def _init_publishers(self):
        self.get_logger().info("Initializing publishers...")

        # Haptic State Publisher
        self._haptic_pose_pub = self.create_publisher(PoseStamped, self._target_pose_topic_name, 10)
        self._haptic_twist_pub = self.create_publisher(TwistStamped, self._target_twist_topic_name, 10)

        # Contact forces publisher
        self._i3_forces_pub = self.create_publisher(WrenchStamped, self._i3_wrench_topic_name, 10)
        self._i3_forces_msg = WrenchStamped()

    #! Callbacks
    def _i3_pose_topic_callback(self, msg: PoseStamped):
        """
        Callback to get the state of the inverse 3 form the '/i3/pose' topic.

        Parameters
        ----------
        msg : PoseStamped
            Message received from the topic.
        """
        # Raw values
        i3_position = ros2np(msg.pose.position)

        # Switch axis 0 and 1 of i3_position
        # i3_position[0], i3_position[1] = i3_position[1], -i3_position[0]
        # i3_vel[0], i3_vel[1] = i3_vel[1], -i3_vel[0]

        # Convert axes
        self._i3_position = i3_position

    def _i3_twist_topic_callback(self, msg: TwistStamped):
        """
        Callback to get the state of the inverse 3 form the '/i3/twist' topic.

        Parameters
        ----------
        msg : TwistStamped
            Message received from the topic.
        """
        # Raw values
        i3_vel = ros2np(msg.twist.linear)

        # # Switch axis 0 and 1 of i3_position
        # i3_vel[0], i3_vel[1] = i3_vel[1], -i3_vel[0]

        # Convert axes
        self._i3_velocities = i3_vel

    def _robot_pose_topic_callback(self, msg: PoseStamped):
        """
        Callback for the '/fr3/current_pose' topic.

        Parameters
        ----------
        msg : PoseStamped
        """
        self._ee_pose = ros2np(msg.pose.position)
        self._ee_quat = np.array(
            [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        )

        # self.get_logger().info(f"Robot EE: {self._ee_des}")

        # TODO: Move to main loop

    def _contact_forces_topic_callback(self, msg: WrenchStamped):
        """
        Callback for the 'contact_forces_topic' topic.

        Parameters
        ----------
        msg : WrenchStamped
            Estimated contact forces from the FR3.
        """
        self._ee_forces = ros2np(msg.wrench.force)

        # Switch axes and invert the values
        self._ee_forces[0], self._ee_forces[1] = self._ee_forces[1], -self._ee_forces[0]
        self._ee_forces[2] = -self._ee_forces[2]

        if not self._force_calibrated:
            self._force_bias += self._ee_forces
            self._n_force_cal_msgs += 1

            # For ~2 seconds
            if self._n_force_cal_msgs >= 2000:
                self._force_bias /= self._n_force_cal_msgs
                self._force_calibrated = True
                self.get_logger().info(f"Force bias: {self._force_bias}")

        else:
            self._ee_forces -= self._force_bias

            if DEBUG:
                print(f"F: {self._ee_forces[0]:>8.4f} | {self._ee_forces[1]:>8.4f} | {self._ee_forces[2]:>8.4f}")

            # deadzone
            self._ee_forces = np.where(np.abs(self._ee_forces) < self._force_deadzone, 0, self._ee_forces)

    def _interface_forces_topic_callback(self, msg: WrenchStamped):
        """
        Callback for the '/interface_force' topic.

        Parameters
        ----------
        msg : WrenchStamped
            Message received from the '/interface_force' topic.
        """
        self._interface_forces = ros2np(msg.wrench.force)

    def _ee_vel_topic_callback(self, msg: TwistStamped):
        """
        Callback for the '/franka_broadcaster/...' topic.

        Parameters
        ----------
        msg : TwistStamped
            Message received from the '/franka_broadcaster/...' topic.
        """
        self._ee_vel = ros2np(msg.twist.linear)

    # Publishers
    def _pub_haptic_state(self):
        """
        Publish the desired end effector position to the '/teleop/ee_des_pose' topic.
        """
        # Publish the haptic position
        haptic_pose_msg = PoseStamped()
        haptic_pose_msg.header.stamp = self.get_clock().now().to_msg()
        haptic_pose_msg.pose.position = np2ros(self._haptic_position, msg_type="Point")

        # TODO: Update orientation based on robot orientation or i3 orientation
        # For now, just set to a fixed orientation (pointing downwards)
        haptic_pose_msg.pose.orientation = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)

        self._haptic_pose_pub.publish(haptic_pose_msg)

        # Publish the haptic twist
        haptic_twist_msg = TwistStamped()
        haptic_twist_msg.header.stamp = self.get_clock().now().to_msg()
        haptic_twist_msg.twist.linear = np2ros(self._haptic_linear_vel, msg_type="Vector3")

        self._haptic_twist_pub.publish(haptic_twist_msg)

    def _pub_i3_forces(self):
        """
        Publish the desired forces to the i3 robot.
        """
        # Publish
        self._i3_forces_msg.header.stamp = self.get_clock().now().to_msg()
        self._i3_forces_msg.wrench.force = np2ros(self._i3_forces_des)
        self._i3_forces_pub.publish(self._i3_forces_msg)

    #! Methods
    def _main_loop(self):
        """
        Main loop of the node.
        """
        if not self._is_initialized:
            if self._ee_pose is None:
                self.get_logger().warn("Waiting for the robot pose...", throttle_duration_sec=2.0)
                return

            self.get_logger().info("Initializing teleop node...")
            self._ws_center = self._ee_pose

            # self._n_robot_cal_msgs += 1

            self._is_initialized = True
            self.get_logger().info(f"Robot center: {self._ws_center}")
            return

        # Transform the i3 position to the robot's base frame
        T_i3_ee = self._i3_scale * self._active_axis @ self._T_i3_robot
        self._haptic_position = T_i3_ee @ self._i3_position + self._ws_center
        self._haptic_linear_vel = T_i3_ee @ self._i3_velocities

        # Scale the forces
        T_ee_i3 = -self._i3_forces_scale * self._T_i3_robot.T @ self._active_axis
        self._i3_forces_des = T_ee_i3 @ self._interface_forces

        self._pub_haptic_state()
        self._pub_i3_forces()


def main(args=None):
    wait_for_debugger(NODE_NAME)  # Wait for debugger if env variables is set

    rclpy.init(args=args)
    node = I3Teleop()

    try:
        node.get_logger().info("i3_teleop launched, end with CTRL-C")
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt, shutting down.\n")

    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
