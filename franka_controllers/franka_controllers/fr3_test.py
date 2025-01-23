import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped

import numpy as np
from robotic_arm_controller.RobotArm import RobotArm
from spatialmath.base.quaternions import q2r
from spatialmath.pose3d import SO3


class FR3Test(Node):
    def __init__(self):
        super().__init__("fr3_test")

        # Initialize robot arm
        self._robot_arm = RobotArm("fr3")

        joint_states_topic = "/joint_states"

        controller_name = "velocity_controller"
        controller_command_topic = f"/{controller_name}/commands"

        self._joint_state_sub = self.create_subscription(
            JointState, joint_states_topic, self._joint_state_callback, 10
        )

        # Command Publisher
        self._commmand_pub = self.create_publisher(
            Float64MultiArray, controller_command_topic, 10
        )
        self._command_msg = Float64MultiArray()
        self._command_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Pose Publisher
        self._pose_pub = self.create_publisher(PoseStamped, "/fr3_pose", 10)
        self._pose_msg = PoseStamped()

        self.timer = self.create_timer(0.25, self._publish_ee_pose)  # Publish at 10 Hz

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer_tf = self.create_timer(0.25, self._tf_callback)  # Publish at 4 Hz

    def _joint_state_callback(self, joint_msg):
        q = np.array(joint_msg.position)
        self._robot_arm.state.q = q

        # self._commmand_pub.publish(self._command_msg)

    def _publish_ee_pose(self):
        ee_position = self._robot_arm.state.ee_position
        ee_quaternion = self._robot_arm.state.ee_quaternion

        self._pose_msg.pose.position.x = ee_position[0]
        self._pose_msg.pose.position.y = ee_position[1]
        self._pose_msg.pose.position.z = ee_position[2]

        self._pose_msg.pose.orientation.x = ee_quaternion[0]
        self._pose_msg.pose.orientation.y = ee_quaternion[1]
        self._pose_msg.pose.orientation.z = ee_quaternion[2]
        self._pose_msg.pose.orientation.w = ee_quaternion[3]

        # self.get_logger().info(f"Publishing ee pose: {ee_position}")
        self._pose_pub.publish(self._pose_msg)

    def _tf_callback(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                "base", "fr3_hand", rclpy.time.Time()
            )
            self._handle_transform(trans)
        except Exception as e:
            self.get_logger().warn(f"Could not transform: {e}")

    def _handle_transform(self, trans: TransformStamped):
        # Process tf2 transform
        translation = trans.transform.translation
        ee_position_rviz = np.array([translation.x, translation.y, translation.z])
        # self.get_logger().info(f"Received transform: {ee_position_rviz}")

        orientation = trans.transform.rotation
        R_rviz = SO3(
            q2r(
                [orientation.w, orientation.x, orientation.y, orientation.z],
                order="sxyz",
            )
        )
        rpy_rviz = R_rviz.rpy(unit="deg", order="zyx")

        # self.get_logger().info(f"Received transform: \n{R_rviz}")

        # Position from python library
        ee_position_py = self._robot_arm.state.ee_position
        # self.get_logger().info(f"Received transform: {ee_position_py}")

        ee_quaternion_py = self._robot_arm.state.ee_quaternion
        R_py = SO3(q2r(ee_quaternion_py, order="sxyz"))
        rpy_py = R_py.rpy(unit="deg", order="zyx")

        # self.get_logger().info(f"Received transform: \n{R_py}")
        # self.get_logger().info(f"Received transform: {rpy_py}")

        # ERRORS
        # self.get_logger().info(
        #     f"EE Position error: {ee_position_rviz - ee_position_py}"
        # )
        # self.get_logger().info(f"rviz: {rpy_rviz}\tpy: {rpy_py}")
        self.get_logger().info(f"EE Orientation error: {rpy_rviz - rpy_py}")


def main(args=None):
    rclpy.init(args=args)
    node = FR3Test()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
