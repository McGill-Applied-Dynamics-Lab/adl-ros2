# ROS imports
import rclpy
from rclpy.node import Node  # node class makes a ROS2 Node

from arm_interfaces.msg import Teleop
from teleop_interfaces.msg import Inverse3State
from geometry_msgs.msg import PoseStamped, Pose, TwistStamped, Twist, WrenchStamped, Vector3
from sensor_msgs.msg import Joy

# Other
import numpy as np
from enum import Enum


class ControlModes(Enum):
    POSITION = 0
    VELOCITY = 1


def np2ros(array: np.ndarray) -> Vector3:
    """
    Converts a numpy array to a Vector 3 ROS message.

    Args:
        array (np.ndarray): Numpy array to convert.

    Returns:
        Vector3: ROS message.
    """
    if len(array) != 3:
        raise ValueError("Array must have 3 elements.")
    if not isinstance(array, np.ndarray):
        raise ValueError("Input must be a numpy array.")

    msg = Vector3()
    msg.x = array[0]
    msg.y = array[1]
    msg.z = array[2]

    return msg


def ros2np(msg: Vector3) -> np.ndarray:
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
        super().__init__("i3_teleop")
        self.get_logger().info("Initializing I3Teleop...")

        #! Attributes
        # Joystick
        self._start_time = None
        self._is_calibrated = True  # TODO: Needed?
        self._is_initialized = False

        self._i3_position = np.zeros(3)  # Position of the i3
        self._i3_velocities = np.zeros(3)  # Velocities of the i3
        self._i3_forces = np.zeros(3)  # Forces to apply to the i3

        # Robot
        self._control_mode = ControlModes.POSITION
        self._ee_pose = np.zeros(3)  # End effector position for the robot

        self._ee_center = np.zeros(3)  # Center position for the ee
        self._ee_des = np.zeros(3)  # Desired end effector position

        self._position_scale = 0.1  # Scale for the position of the joystick to the robot
        self._vel_scale = 0.1  # Scale for the velocity of the joystick to the robot

        #! Init functions
        self._init_subscribers()
        self._init_publishers()

        # self._calibrate()

    def _init_subscribers(self):
        self.get_logger().info("Initializing subscribers...")

        # --- joystick ---
        i3_topic = "/inverse3/state"
        self._i3_sub = self.create_subscription(Inverse3State, i3_topic, self._i3_topic_callback, 10)

        # --- robot ---
        robot_topic = "/fr3_pose"
        self._robot_sub = self.create_subscription(PoseStamped, robot_topic, self._robot_topic_callback, 10)

        # --- contact forces ---
        contact_forces_topic = "/franka_robot_state_broadcaster/external_wrench_in_base_frame"
        self._contact_forces_sub = self.create_subscription(
            WrenchStamped, contact_forces_topic, self._contact_forces_topic_callback, 10
        )

    def _init_publishers(self):
        self.get_logger().info("Initializing publishers...")

        # Pose Publisher
        self._ee_cmd_pub_topic = "/teleop/ee_cmd"
        self._ee_cmd_pub = self.create_publisher(Teleop, self._ee_cmd_pub_topic, 10)
        self._ee_cmd_msg = Teleop()
        self._ee_cmd_timer = self.create_timer(0.01, self._pub_ee_des)

        # Contact forces publisher
        self._i3_forces_pub_topic = "/inverse3/wrench_des"
        self._i3_forces_pub = self.create_publisher(WrenchStamped, self._i3_forces_pub_topic, 10)
        self._i3_forces_msg = WrenchStamped()
        self._i3_forces_timer = self.create_timer(0.01, self._pub_i3_forces)

    #! Callbacks
    def _i3_topic_callback(self, msg: Inverse3State):
        """
        Callback for the '/inverse3/state' topic.

        Parameters
        ----------
        msg : Inverse3State
            Message received from the topic.
        """
        self._i3_position = ros2np(msg.pose.position)
        self._i3_velocities = ros2np(msg.twist.linear)

    def _robot_topic_callback(self, msg: PoseStamped):
        """
        Callback for the '/fr3_pose' topic.

        Parameters
        ----------
        msg : PoseStamped
            Message received from the '/robot' topic.
        """
        self._ee_pose = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        # self.get_logger().info(f"Robot EE: {self._ee_des}")

        if not self._is_initialized:
            self._ee_center = self._ee_pose
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
        self._i3_forces = ros2np(msg.wrench.force)

    def _pub_ee_des(self):
        """
        Publish the desired end effector position to the '/teleop/ee_des_pose' topic.
        """
        if not self._is_calibrated:
            return

        self._compute_ee_command()

        self._ee_cmd_pub.publish(self._ee_cmd_msg)

    def _pub_i3_forces(self):
        """
        Publish the desired forces to the i3 robot.
        """
        self._i3_forces_msg.header.stamp = self.get_clock().now().to_msg()
        self._i3_forces_msg.wrench.force = np2ros(self._i3_forces)

        self._i3_forces_pub.publish(self._i3_forces_msg)

    #! Methods
    def _update_control_mode(self):
        """
        Update the control mode based on the joystick values.

        If RT is pressed, the control mode is changed to velocity.
        """
        dist = np.linalg.norm(self._i3_position)
        thres = 0.05

        if self._control_mode == ControlModes.POSITION and dist == thres:
            self.get_logger().info("Control mode changed to VELOCITY.")
            self._control_mode = ControlModes.VELOCITY

        elif self._control_mode == ControlModes.VELOCITY and dist <= thres:
            self.get_logger().info("Control mode changed to POSITION.")
            self._control_mode = ControlModes.POSITION

            # Update ee center
            self._ee_center = self._ee_pose

    def _compute_ee_command(self):
        """
        Compute the command to send to the robot based on the joystick values.

        If RT is pressed, the control mode is changed to velocity.
        (called from `self._pub_ee_des`)
        """
        self._update_control_mode()

        self._ee_cmd_msg.header.stamp = self.get_clock().now().to_msg()

        if self._control_mode == ControlModes.POSITION:
            ee_des = self._compute_ee_des()

            self._ee_cmd_msg.control_mode = ControlModes.POSITION.value
            # self._ee_cmd_msg.ee_des.position = np2ros(ee_des)
            self._ee_cmd_msg.ee_des.position.x = ee_des[0]
            self._ee_cmd_msg.ee_des.position.y = ee_des[1]
            self._ee_cmd_msg.ee_des.position.z = ee_des[2]

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
        ee_des = self._ee_center + self._i3_position * self._position_scale

        return ee_des

    def _compute_ee_vel(self) -> np.ndarray:
        """Compute the desired end effector velocity based on the inverse 3 position."""
        radius = 0.05
        Kd = 100

        distance = np.linalg.norm(self._i3_position)
        direction = self._i3_position / distance
        velocity = direction * ((distance - radius) ** 3) * Kd

        # distance = np.sqrt(sum([(device_pos[i] - center[i]) ** 2 for i in range(3)]))
        # direction = [(device_pos[i] - center[i]) / distance for i in range(3)]
        # velocity = [direction[i] * ((distance - radius) ** 3) * Kd for i in range(3)]

        # get the velocity of movement in vel-ctl region
        # Va = velocity_applied(self._workspace_center, self._R, self._raw_positions, 100)
        Va = velocity

        # magVa = math.sqrt(Va[0] * Va[0] + Va[1] * Va[1] + Va[2] * Va[2])
        Va_norm = np.linalg.norm(Va)

        max_vel = 0.0005
        if Va_norm > max_vel:
            # Va = [(self._maxVa / Va_norm) * Va[0], (self._maxVa / Va_norm) * Va[1], (self._maxVa / Va_norm) * Va[2]]

            Va = Va * max_vel / Va_norm

        # self._velocity_ball_center = [(self._velocity_ball_center[i] + Va[i]) for i in range(3)]
        # self._velocity_ball_center += Va

        # # Apply the deadzone
        # js_axes = np.where(np.abs(js_axes) < self._js_deadzone_thres, 0, js_axes)

        # Compute the desired end effector position
        ee_vel_des = self._vel_scale * Va

        return ee_vel_des.astype(float)


def main(args=None):
    rclpy.init(args=args)
    node = I3Teleop()

    try:
        node.get_logger().info("joy_teleop launched, end with CTRL-C")
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt, shutting down.\n")

    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
