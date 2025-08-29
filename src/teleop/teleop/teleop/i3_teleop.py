# ROS imports
from typing import Literal
import rclpy
from rclpy.node import Node  # node class makes a ROS2 Node

from arm_interfaces.msg import Teleop
from teleop_interfaces.msg import Inverse3State
from geometry_msgs.msg import PoseStamped, Pose, TwistStamped, Twist, WrenchStamped, Vector3, Point, Quaternion

# Other
import numpy as np
from enum import Enum

from adg_ros2_utils.debug_utils import wait_for_debugger

NODE_NAME = "i3_teleop"

DEBUG = False


class ControlModes(Enum):
    POSITION = 0
    VELOCITY = 1


def np2ros(array: np.ndarray, msg_type: Literal["Vector3", "Point"] = "Vector3") -> Vector3:
    """
    Converts a numpy array to a Vector 3 ROS message.

    Args:
        array (np.ndarray): Numpy array to convert.

    Returns:
        Vector3: ROS message.
    """
    # if len(array) != 3:
    #     raise ValueError("Array must have 3 elements.")
    if not isinstance(array, np.ndarray):
        raise ValueError("Input must be a numpy array.")

    if msg_type == "Vector3":
        msg = Vector3()
        msg.x = array[0]
        msg.y = array[1]
        msg.z = array[2]
    elif msg_type == "Point":
        msg = Point()
        msg.x = array[0]
        msg.y = array[1]
        msg.z = array[2]
    elif msg_type == "Quaternion":
        msg = Quaternion()
        msg.x = array[0]
        msg.y = array[1]
        msg.z = array[2]
        msg.w = array[3]
    else:
        raise ValueError("Invalid message type.")

    return msg


def ros2np(msg: Vector3 | Point) -> np.ndarray:
    """
    Converts a Vector 3 ROS message to a numpy array.

    Args:
        msg (Vector3): ROS message to convert.

    Returns:
        np.ndarray: Numpy array.
    """
    array = np.array([msg.x, msg.y, msg.z])

    return array


class I3Teleop(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.get_logger().info("Initializing I3Teleop...")

        #! Attributes
        # Joystick
        self._start_time = None
        self._is_initialized = False
        self._cmd_pub_freq = 100  # Frequency to publish the commands

        self._force_calibrated = False
        self._n_force_cal_msgs = 0
        self._force_bias = np.zeros(3)  # Bias for the force sensor
        self._force_deadzone = 5.0  # Deadzone for the force to apply

        self._i3_position = np.zeros(3)  # Position of the i3
        self._i3_velocities = np.zeros(3)  # Velocities of the i3
        self._i3_forces_des = np.zeros(3)  # Forces to apply to the i3

        # TODO: Move to ros params
        self._i3_forces_scale = 0.05  # Scale for the forces applied to the i3
        self._i3_position_scale = 3  # Scale for the position of the i3
        self._i3_vel_scale = 500  # Scale for the velocity
        self._pos_radius = 0.060  # Radius for the position control region

        # Robot
        self._control_mode = ControlModes.POSITION
        self._ee_pos = np.zeros(3)  # End effector position for the robot

        self._ee_center = np.zeros(3)  # Center position for the ee
        self._ee_des = np.zeros(3)  # Desired end effector position

        self._ee_forces = np.zeros(3)  # Forces applied to the end effector
        self._ee_vel = np.zeros(3)  # Velocities of the end effector. Desired one from control

        #! Init functions
        self._init_subscribers()
        self._init_publishers()

        # self._calibrate()

    def _init_subscribers(self):
        self.get_logger().info("Initializing subscribers...")

        # --- joystick ---
        i3_topic = "/inverse3/state"
        self._i3_sub = self.create_subscription(Inverse3State, i3_topic, self._i3_topic_callback, 10)

        # --- ee pose ---
        robot_topic = "/fr3_pose"
        self._robot_sub = self.create_subscription(PoseStamped, robot_topic, self._robot_topic_callback, 10)

        # --- contact forces ---
        contact_forces_topic = "/franka_robot_state_broadcaster/external_wrench_in_base_frame"
        self._contact_forces_sub = self.create_subscription(
            WrenchStamped, contact_forces_topic, self._contact_forces_topic_callback, 10
        )

        # # --- ee vel ---
        # ee_vel_topic = "/franka_robot_state_broadcaster/desired_end_effector_twist"
        # self._ee_vel_sub = self.create_subscription(TwistStamped, ee_vel_topic, self._ee_vel_topic_callback, 10)

    def _init_publishers(self):
        self.get_logger().info("Initializing publishers...")

        # Pose Publisher
        self._ee_cmd_pub_topic = "/teleop/ee_cmd"
        self._ee_cmd_pub = self.create_publisher(Teleop, self._ee_cmd_pub_topic, 10)
        self._ee_cmd_msg = Teleop()
        self._ee_cmd_timer = self.create_timer(1 / self._cmd_pub_freq, self._pub_ee_des)

        # Contact forces publisher
        self._i3_forces_pub_topic = "/inverse3/wrench_des"
        self._i3_forces_pub = self.create_publisher(WrenchStamped, self._i3_forces_pub_topic, 10)
        self._i3_forces_msg = WrenchStamped()
        self._i3_forces_timer = self.create_timer(0.01, self._pub_i3_forces)

    #! Callbacks
    def _i3_topic_callback(self, msg: Inverse3State):
        """
        Callback to get the state of the inverse 3 form the '/inverse3/state' topic.

        Parameters
        ----------
        msg : Inverse3State
            Message received from the topic.
        """
        # Raw values
        i3_position = ros2np(msg.pose.position)
        i3_vel = ros2np(msg.twist.linear)

        # Switch axis 0 and 1 of i3_position
        i3_position[0], i3_position[1] = i3_position[1], -i3_position[0]
        i3_vel[0], i3_vel[1] = i3_vel[1], -i3_vel[0]

        # Convert axes
        self._i3_position = i3_position
        self._i3_velocities = i3_vel

    def _robot_topic_callback(self, msg: PoseStamped):
        """
        Callback for the '/fr3_pose' topic.

        Parameters
        ----------
        msg : PoseStamped
            Message received from the '/robot' topic.
        """
        self._ee_pos = ros2np(msg.pose.position)
        self._ee_quat = np.array(
            [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        )

        # self.get_logger().info(f"Robot EE: {self._ee_des}")

        if not self._is_initialized:
            self._ee_center = self._ee_pos
            # self._n_robot_cal_msgs += 1
            self._is_initialized = True
            self.get_logger().info(f"Robot center: {self._ee_center}")

    def _contact_forces_topic_callback(self, msg: WrenchStamped):
        """
        Callback for the '/franka_broadcaster/...' topic.

        Parameters
        ----------
        msg : WrenchStamped
            Message received from the '/franka_broadcaster/...' topic.
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

    def _ee_vel_topic_callback(self, msg: TwistStamped):
        """
        Callback for the '/franka_broadcaster/...' topic.

        Parameters
        ----------
        msg : TwistStamped
            Message received from the '/franka_broadcaster/...' topic.
        """
        self._ee_vel = ros2np(msg.twist.linear)

    def _pub_ee_des(self):
        """
        Publish the desired end effector position to the '/teleop/ee_des_pose' topic.
        """
        if not self._is_initialized:
            self.get_logger().warn("Robot not initialized, skipping publishing ee command.", throttle_duration_sec=2.0)

            return

        self._compute_ee_command()

        self._ee_cmd_pub.publish(self._ee_cmd_msg)

    def _pub_i3_forces(self):
        """
        Publish the desired forces to the i3 robot.
        """
        if not self._force_calibrated:
            return

        # Transform the ee forces
        self._i3_forces_des = self._i3_forces_scale * self._ee_forces

        # Deadzone
        # self._i3_forces_des = np.where(np.abs(self._i3_forces_des) < self._force_deadzone, 0, self._i3_forces_des)

        # print(
        #     f"F des: {self._i3_forces_des[0]:>8.4f} | {self._i3_forces_des[1]:>8.4f} | {self._i3_forces_des[2]:>8.4f}"
        # )

        # Publish
        self._i3_forces_msg.header.stamp = self.get_clock().now().to_msg()
        self._i3_forces_msg.wrench.force = np2ros(self._i3_forces_des)
        self._i3_forces_pub.publish(self._i3_forces_msg)

    #! Methods
    def _update_control_mode(self):
        """
        Update the control mode based on the joystick values.

        If RT is pressed, the control mode is changed to velocity.
        """
        dist = np.linalg.norm(self._i3_position)

        # pos -> vel
        if self._control_mode == ControlModes.POSITION and dist > self._pos_radius:
            self.get_logger().info("Control mode changed to VELOCITY.")
            self._control_mode = ControlModes.VELOCITY

        # vel -> pos
        elif self._control_mode == ControlModes.VELOCITY and dist <= self._pos_radius:
            self.get_logger().info("Control mode changed to POSITION.")
            self._control_mode = ControlModes.POSITION

            # Update ee center
            self._ee_center = self._ee_pos - self._i3_position * self._i3_position_scale

    def _compute_ee_command(self):
        """
        Compute the command to send to the robot based on the joystick values.

        If RT is pressed, the control mode is changed to velocity.
        (called from `self._pub_ee_des`)
        """
        self._update_control_mode()

        self._ee_cmd_msg.header.stamp = self.get_clock().now().to_msg()

        if self._control_mode == ControlModes.POSITION:
            ee_pose_des, ee_quat_des = self._compute_ee_des()

            self._ee_cmd_msg.control_mode = ControlModes.POSITION.value
            self._ee_cmd_msg.ee_des.position = np2ros(ee_pose_des, msg_type="Point")
            self._ee_cmd_msg.ee_des.orientation = np2ros(ee_quat_des, msg_type="Quaternion")

            self._ee_cmd_msg.ee_vel_des = Twist()

        else:
            ee_vel_des = self._compute_ee_vel()

            self._ee_cmd_msg.control_mode = ControlModes.VELOCITY.value
            self._ee_cmd_msg.ee_vel_des.linear = np2ros(ee_vel_des)

            self._ee_cmd_msg.ee_des = Pose()

    def _compute_ee_des(self):
        """
        Compute the desired end effector position based on the i3 position.

        Returns
        -------
        np.ndarray
            Desired end effector position.
        """
        # # Apply the deadzone
        # js_axes = np.where(np.abs(js_axes) < self._js_deadzone_thres, 0, js_axes)

        # Compute the desired end effector position
        ee_pos_des = self._ee_center + self._i3_position * self._i3_position_scale

        ee_quat_des = self._ee_quat.copy()

        return ee_pos_des, ee_quat_des

    def _compute_ee_vel(self) -> np.ndarray:
        """Compute the desired end effector velocity based on the inverse 3 position."""
        # Compute the velocity to apply
        distance = np.linalg.norm(self._i3_position)
        direction = self._i3_position / distance
        velocity = direction * ((distance - self._pos_radius) ** 3) * self._i3_vel_scale

        # distance = np.sqrt(sum([(device_pos[i] - center[i]) ** 2 for i in range(3)]))
        # direction = [(device_pos[i] - center[i]) / distance for i in range(3)]
        # velocity = [direction[i] * ((distance - radius) ** 3) * Kd for i in range(3)]

        # get the velocity of movement in vel-ctl region
        # Va = velocity_applied(self._workspace_center, self._R, self._raw_positions, 100)

        # magVa = math.sqrt(Va[0] * Va[0] + Va[1] * Va[1] + Va[2] * Va[2])
        vel_norm = np.linalg.norm(velocity)

        max_vel = 0.1
        if vel_norm > max_vel:
            # Va = [(self._maxVa / Va_norm) * Va[0], (self._maxVa / Va_norm) * Va[1], (self._maxVa / Va_norm) * Va[2]]
            print("MAX VEL")
            velocity = velocity * max_vel / vel_norm

        # self._velocity_ball_center = [(self._velocity_ball_center[i] + Va[i]) for i in range(3)]
        # self._velocity_ball_center += Va

        # # Apply the deadzone
        # js_axes = np.where(np.abs(js_axes) < self._js_deadzone_thres, 0, js_axes)

        # Compute the desired end effector position
        ee_vel_des = velocity

        # # Update ee_center
        # self._ee_center += self._ee_vel * 1 / self._cmd_pub_freq

        # print(f"EE vel des: {ee_vel_des} \t EE center: {self._ee_center}")

        return ee_vel_des.astype(float)


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
