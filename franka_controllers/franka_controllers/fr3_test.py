import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped

import numpy as np
import roboticstoolbox as rtb
from robotic_arm_controller.RobotArm import RobotArm
from spatialmath.base.quaternions import q2r
from spatialmath.pose3d import SO3

from rclpy.action import ActionServer
from arm_interfaces.action import Empty
import time

from rclpy.executors import MultiThreadedExecutor


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

        # Trajectories
        self.trajectory = None

        # Command Publisher
        self._commmand_pub = self.create_publisher(
            Float64MultiArray, controller_command_topic, 10
        )
        self._command_msg = Float64MultiArray()
        self._command_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self._cmd_timer_freq = 100  # Hz
        self._cmd_pub_timer = self.create_timer(1 / 100, self._pub_joint_vels_cmd)

        # Pose Publisher
        self._pose_pub = self.create_publisher(PoseStamped, "/fr3_pose", 10)
        self._pose_msg = PoseStamped()

        self.timer = self.create_timer(0.25, self._publish_ee_pose)  # Publish at 10 Hz

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer_tf = self.create_timer(0.25, self._tf_callback)  # Publish at 4 Hz

        # Attributes
        self._is_state_initialialized = False

        # Actions
        _as_freq = 100
        self._as_loop_rate = self.create_rate(_as_freq, self.get_clock())  # 100 Hz rate
        self._action_server = ActionServer(
            self, Empty, "go_to_joint_positions", self.go_to_joint_positions
        )

    # Callbacks
    def _joint_state_callback(self, joint_msg):
        """
        Update the robot state with the joint state message from '\joint_states' topic
        """
        q = np.array(joint_msg.position)
        q = q[:7]  # Remove the fingers
        self._robot_arm.state.q = q

        self._is_state_initialialized = True

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
        # self.get_logger().info(f"EE Orientation error: {rpy_rviz - rpy_py}")

    def _pub_joint_vels_cmd(self):
        """
        Publish joint velocities at 100 Hz based on the current trajectory.
        """
        if self.trajectory is None or self.current_index >= len(self.trajectory):
            return

        # Get the current time
        # current_time = self.get_clock().now()

        # Find the current trajectory point
        if self.current_index < len(self.trajectory):
            current_point: np.ndarray = self.trajectory[self.current_index]

            data = current_point.tolist()
            self._command_msg.data = data
            self._commmand_pub.publish(self._command_msg)
            # self.get_logger().info(
            #     f"Published point {self.current_index + 1}: {self._command_msg.data}"
            # )

            # # Send feedback to the client
            # # feedback_msg = GoToCartesianPosition.Feedback()
            # # feedback_msg.current_pose = self.compute_current_pose()  # Replace with actual computation
            # # feedback_msg.distance_to_target = self.compute_distance_to_target(self.goal_handle.request.target_pose)
            # # self.goal_handle.publish_feedback(feedback_msg)
            # feedback_msg = Empty.Feedback()
            # # feedback_msg.current_pose = self.compute_current_pose()  # Replace with actual computation
            # # feedback_msg.distance_to_target = self.compute_distance_to_target(self.goal_handle.request.target_pose)
            # self.goal_handle.publish_feedback(feedback_msg)

            # Move to the next point
            self.current_index += 1

        else:
            print("Done")

    # ---- Actions
    def go_to_joint_positions(self, goal_handle):
        self.get_logger().info("Received goal joint position goal")
        # self.get_logger().info(f"Received goal joint position goal: {goal_handle.request.target_pose}")

        # goal_handle.succeed()

        # Paramters
        # home_position = np.deg2rad([0, -45, 0, -135, 0, 90, 45])
        start_q = self._robot_arm.state.q
        goal = np.deg2rad([-90, -45, 0, -135, 0, 90, 45])

        self.get_logger().info(
            f"Current joint positions: {np.rad2deg(self._robot_arm.state.q)}"
        )
        self.get_logger().info(f"Goal joint positions: {np.rad2deg(goal)}")

        traj_time = 5  # Time to reach goal [s]
        freq = 100  # Frequency [Hz]

        # Compute traj
        n_points = int(traj_time * freq)
        joint_traj = rtb.jtraj(start_q, goal, n_points)
        joint_traj.plot(block=False)

        self.trajectory = joint_traj.qd
        self.current_index = 0

        self.goal_handle = goal_handle
        start_time = self.get_clock().now()

        # Wait for the trajectory to complete or goal to be canceled
        while self.current_index < len(self.trajectory):
            if self.goal_handle.is_cancel_requested:
                self.goal_handle.canceled()
                self.get_logger().info("Goal canceled by client.")
                # return GoToCartesianPosition.Result(success=False, message="Goal canceled.")
                return Empty.Result()
            time.sleep(0.01)

        # Check if trajectory was successfully executed
        end_time = self.get_clock().now()
        duration = (end_time - start_time).nanoseconds / 1e9
        self.get_logger().info(
            f"Trajectory execution completed in {duration:.2f} seconds."
        )

        self.trajectory = None

        # Set result
        goal_handle.succeed()
        result = Empty.Result()
        # result.success = True
        # result.message = "Trajectory executed successfully."
        # result.final_pose = self.compute_current_pose()  # Replace with actual final pose
        return result


def main(args=None):
    rclpy.init(args=args)
    node = FR3Test()
    executor = MultiThreadedExecutor()

    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()

    # rclpy.shutdown()


if __name__ == "__main__":
    main()
