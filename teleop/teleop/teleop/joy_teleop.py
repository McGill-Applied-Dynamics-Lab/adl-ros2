# ROS2
import rclpy
from rclpy.time import Time, Duration
from rclpy.node import Node  # node class makes a ROS2 Node

from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped, Pose, TwistStamped, Twist
from arm_interfaces.msg import Teleop

# Other
from enum import Enum
import numpy as np


class JoyAxisMap(Enum):
    LS_H = 0  # Left stick horizontal
    LS_V = 1  # Left stick vertical
    LT = 2  # Left trigger
    RS_H = 3  # Right stick horizontal
    RS_V = 4  # Right stick vertical
    RT = 5  # Right trigger
    D_H = 6  # Dpad horizontal
    D_V = 7  # Dpad vertical


class JoyButtonMap(Enum):
    A = 0
    B = 1
    X = 2
    Y = 3
    LB = 4
    RB = 5
    BACK = 6
    START = 7
    GUIDE = 8  # Xbox button
    LS = 9
    RS = 10


class ControlModes(Enum):
    POSITION = 0
    VELOCITY = 1


class JoyTeleop(Node):
    def __init__(self):
        super().__init__("joy_teleop")
        self.get_logger().info("Initializing JoyTeleop...")

        #! Attributes
        # Joystick
        self._start_time = None
        self._axes_values = np.zeros(8)
        self._axes_offset = np.zeros(8)  # Offset for the axes
        self._js_deadzone_thres = 0.1

        self._buttons_values = np.zeros(11)

        self._calibrating = False
        self._n_js_cal_msgs = 0
        self._n_robot_cal_msgs = 0
        self._js_cal_axes_idx = [
            JoyAxisMap.LS_H.value,
            JoyAxisMap.LS_V.value,
            JoyAxisMap.RS_H.value,
            JoyAxisMap.RS_V.value,
        ]  # Indexes of the axes to calibrate
        self._is_calibrated = False

        # Robot
        self._control_mode = ControlModes.POSITION
        self._ee_pose = np.zeros(3)  # End effector position for the robot
        self._ee_center = np.zeros(3)  # Center position for the ee
        self._ee_des = np.zeros(3)  # Desired end effector position
        self._position_scale = 0.1  # Scale for the position of the joystick to the robot
        self._vel_scale = 0.001  # Scale for the velocity of the joystick to the robot

        #! Init functions
        self._init_subscribers()
        self._init_publishers()

        self._calibrate()

    def _init_subscribers(self):
        self.get_logger().info("Initializing subscribers...")

        # --- joystick ---
        joy_topic = "/joy"
        self._joy_sub = self.create_subscription(Joy, joy_topic, self._joy_topic_callback, 10)

        # --- robot ---
        robot_topic = "/fr3_pose"
        self._robot_sub = self.create_subscription(PoseStamped, robot_topic, self._robot_topic_callback, 10)

    def _init_publishers(self):
        self.get_logger().info("Initializing publishers...")

        # Pose Publisher
        self._ee_cmd_pub_topic = "/teleop/ee_cmd"
        self._ee_cmd_pub = self.create_publisher(Teleop, self._ee_cmd_pub_topic, 10)
        self._ee_cmd_msg = Teleop()
        self._ee_cmd_timer = self.create_timer(0.01, self._pub_ee_des)

    def _calibrate(self):
        self.get_logger().info("Calibrating...")
        calibrate_time = Duration(seconds=2.0)
        start_time = self.get_clock().now()
        self._calibrating = True

        while self.get_clock().now() - start_time < calibrate_time:
            rclpy.spin_once(self, timeout_sec=0.1)

        # --- joystick ---
        if self._n_js_cal_msgs < 20:
            self.get_logger().error("Joystick calibration failed. Not enough messages received on '/joy'.")
            raise Exception("Joystick calibration failed.")

        self._axes_offset /= self._n_js_cal_msgs
        self.get_logger().info(f"Joystick offset: {self._axes_offset}")

        # --- robot ---
        if self._n_robot_cal_msgs < 20:
            self.get_logger().error("Robot calibration failed. Not enough messages received on '/robot'.")
            raise Exception("Robot calibration failed.")

        self._ee_center /= self._n_robot_cal_msgs
        self.get_logger().info(f"Robot center: {self._ee_center}")

        self._calibrating = False
        self._is_calibrated = True

    #! Callbacks
    def _joy_topic_callback(self, msg: Joy):
        """
        Callback for the '/joy' topic.

        Parameters
        ----------
        msg : Joy
            Message received from the '/joy' topic.
        """
        self._axes_values = np.array(msg.axes)
        self._buttons_values = np.array(msg.buttons)

        if self._calibrating:
            # self._axes_offset[self._js_cal_axes_idx] += self._axes_values[self._js_cal_axes_idx]
            self._n_js_cal_msgs += 1

        # If joystick is calibrated
        if self._is_calibrated:
            self._axes_values -= self._axes_offset

        # self.get_logger().info(f"Joy Axes: {self._axes_values}")

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

        if self._calibrating:
            self._ee_center += self._ee_pose
            self._n_robot_cal_msgs += 1

    def _pub_ee_des(self):
        """
        Publish the desired end effector position to the '/teleop/ee_des_pose' topic.
        """
        if not self._is_calibrated:
            return

        self._compute_ee_command()

        self._ee_cmd_pub.publish(self._ee_cmd_msg)

    #! Methods
    def _update_control_mode(self):
        """
        Update the control mode based on the joystick values.

        If RT is pressed, the control mode is changed to velocity.
        """
        # print(f"RT: {self._axes_values[JoyAxisMap.RT.value]}")
        trigger_thres = -1.0
        if self._control_mode == ControlModes.POSITION and self._axes_values[JoyAxisMap.RT.value] == trigger_thres:
            self.get_logger().info("Control mode changed to VELOCITY.")
            self._control_mode = ControlModes.VELOCITY

        elif self._control_mode == ControlModes.VELOCITY and self._axes_values[JoyAxisMap.RT.value] != trigger_thres:
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
            self._ee_cmd_msg.ee_des.position.x = ee_des[0]
            self._ee_cmd_msg.ee_des.position.y = ee_des[1]
            self._ee_cmd_msg.ee_des.position.z = ee_des[2]

            self._ee_cmd_msg.ee_vel_des = Twist()

        else:
            ee_vel_des = self._compute_ee_vel()

            self._ee_cmd_msg.control_mode = ControlModes.VELOCITY.value
            self._ee_cmd_msg.ee_vel_des.linear.x = ee_vel_des[0]
            self._ee_cmd_msg.ee_vel_des.linear.y = ee_vel_des[1]
            self._ee_cmd_msg.ee_vel_des.linear.z = ee_vel_des[2]

            self._ee_cmd_msg.ee_des = Pose()

    def _compute_ee_des(self):
        """
        Compute the desired end effector position based on the joystick values.

        The mapping is the following:
        - Left stick vertical: x-axis
        - Left stick horizontal: y-axis
        - Right stick vertical: z-axis

        Returns
        -------
        np.ndarray
            Desired end effector position.
        """
        # Get the interesting axes
        js_axes = self._axes_values[[JoyAxisMap.LS_V.value, JoyAxisMap.LS_H.value, JoyAxisMap.RS_V.value]]

        # Apply the deadzone

        js_axes = np.where(np.abs(js_axes) < self._js_deadzone_thres, 0, js_axes)

        # Compute the desired end effector position
        ee_des = self._ee_center + self._position_scale * js_axes

        return ee_des

    def _compute_ee_vel(self) -> np.ndarray:
        # Get the interesting axes
        js_axes = self._axes_values[[JoyAxisMap.LS_V.value, JoyAxisMap.LS_H.value, JoyAxisMap.RS_V.value]]

        # Apply the deadzone

        js_axes = np.where(np.abs(js_axes) < self._js_deadzone_thres, 0, js_axes)

        # Compute the desired end effector position
        ee_vel_des = self._vel_scale * js_axes

        return ee_vel_des.astype(float)


def main(args=None):
    rclpy.init(args=args)
    node = JoyTeleop()

    try:
        node.get_logger().info("joy_teleop launched, end with CTRL-C")
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt, shutting down.\n")

    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
