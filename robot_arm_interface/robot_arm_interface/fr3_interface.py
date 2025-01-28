from typing import List
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration

# Interfaces
import rclpy.time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformListener, Buffer
from control_msgs.action import FollowJointTrajectory

from arm_interfaces.action import Empty, GotoJoints, GotoPose, GotoJointVelocities

# from arm_interfaces.msg import Joints
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from builtin_interfaces.msg import Duration

# Python packages
import numpy as np
import time

import roboticstoolbox as rtb

from robotic_arm_controller.RobotArm import RobotArm

from spatialmath.base.quaternions import q2r
from spatialmath.pose3d import SO3, SE3

from robot_arm_interface.utils import PoseStamped2SE3, SE32PoseStamped


class FR3Interface(Node):
    def __init__(self):
        super().__init__("fr3_interface")

        # Initialize robot arm
        self._robot_arm = RobotArm("fr3")

        #! Attributes
        self._is_state_initialialized = False
        self._home_position = np.deg2rad([0, -45, 0, -135, 0, 90, 45])

        # TODO: remove
        self.trajectory = None

        #! Subscribers
        joint_states_topic = "/joint_states"
        self._joint_state_sub = self.create_subscription(
            JointState, joint_states_topic, self._joint_state_callback, 10
        )

        #! Publisher
        self._init_publishers()

        #! Actions
        self._init_actions()

    def _init_publishers(self):
        self.get_logger().info("Initializing publishers...")

        # Pose Publisher
        self._pose_pub = self.create_publisher(PoseStamped, "/fr3_pose", 10)
        self._pose_msg = PoseStamped()
        self._pose_pub_timer = self.create_timer(0.1, self._publish_ee_pose)

        # TF Publisher
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer_tf = self.create_timer(0.25, self._tf_callback)  # Publish at 4 Hz

        # Command Publisher
        # --- velocity_controller ---
        joint_vels_controller: str = "velocity_controller"
        joint_vels_controller_topic = f"/{joint_vels_controller}/commands"
        self._joint_vels_cmd_pub = self.create_publisher(
            Float64MultiArray, joint_vels_controller_topic, 10
        )

        self._joint_vels_cmd_msg = Float64MultiArray()
        self._joint_vels_cmd_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self._joint_vels_cmd_freq = 100  # Hz
        self._joint_vels_cmd_pub_timer = self.create_timer(
            1 / self._joint_vels_cmd_freq, self._pub_joint_vels_cmd
        )

    def _init_actions(self):
        self.get_logger().info("Initializing action servers...")
        # _as_freq = 100  # Asction
        # self._as_loop_rate = self.create_rate(_as_freq, self.get_clock())  # 100 Hz rate

        # goto_joints Action Server
        self._joint_traj_msg = JointTrajectory()

        self._goto_joint_as = ActionServer(
            self, GotoJoints, "goto_joints", self._goto_joints_action
        )

        # goto_pose Action Server
        self._goto_pose_as = ActionServer(
            self, GotoPose, "goto_pose", self._goto_pose_action
        )

        # # joint_trajectory_controller action client (from ros_control)
        # self.get_logger().info("Subscribing to controller's action server...")
        # self._follow_joint_trajectory_ac = ActionClient(
        #     self, FollowJointTrajectory, "/fr3_arm_controller/follow_joint_trajectory"
        # )
        # self._follow_joint_trajectory_ac.wait_for_server()
        # self.get_logger().info("Controller's action server is up!")

        # goto_joint_vels Action Server
        self._goto_joint_vels_as = ActionServer(
            self, GotoJointVelocities, "goto_joint_vels", self._goto_joint_vels_action
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
        Publish joint velocities command at 100 Hz.
        """
        self._joint_vels_cmd_pub.publish(self._joint_vels_cmd_msg)

    # ---- Actions
    def _goto_joints_action(self, goal_handle):
        self.get_logger().info("Received goal joint position goal")

        start_q = np.array(self._robot_arm.state.q)  # [rad]
        goal = np.array(goal_handle.request.joints_goal)  # [rad]
        duration = goal_handle.request.duration  # [s]
        # goal = np.deg2rad([90, -45, 0, -135, 0, 90, 45])

        self.get_logger().info(
            f"Current joint positions: {np.rad2deg(self._robot_arm.state.q)}"
        )
        self.get_logger().info(f"Goal joint positions: {np.rad2deg(goal)}")
        self.get_logger().info(f"Duration [s]: {duration}")

        traj_time_step = 0.01  # time between the trajectory points [s]

        # Compute traj
        n_points = int(duration / traj_time_step)
        joint_traj = rtb.jtraj(start_q, goal, n_points)
        print(joint_traj.q.shape)
        self.get_logger().info(f"\n\nTrajectory shape: {joint_traj.q.shape}")

        start_time = self.get_clock().now()

        # Build the JointTrajectory message
        self._joint_traj_msg.header.stamp = self.get_clock().now().to_msg()
        self._joint_traj_msg.joint_names = [
            "fr3_joint1",
            "fr3_joint2",
            "fr3_joint3",
            "fr3_joint4",
            "fr3_joint5",
            "fr3_joint6",
            "fr3_joint7",
        ]

        self._joint_traj_msg.points = joint_traj_to_msg(joint_traj, traj_time_step)

        #! Send the trajectory to the controller
        # Via the action server
        traj_goal = FollowJointTrajectory.Goal()
        traj_goal.trajectory = self._joint_traj_msg

        self._follow_joint_trajectory_ac.send_goal(traj_goal)
        # future = self._follow_joint_trajectory_ac.send_goal_async(traj_goal)
        # rclpy.spin_until_future_complete(self, future)

        # Via the publisher
        # self._commmand_pub.publish(self._joint_traj_msg)

        # Check if trajectory was successfully executed
        end_time = self.get_clock().now()
        duration = (end_time - start_time).nanoseconds / 1e9
        self.get_logger().info(
            f"Trajectory execution completed in {duration:.2f} seconds."
        )

        # Set result
        goal_handle.succeed()
        result = GotoJoints.Result()
        result.success = True
        self.get_logger().info("Goal succeeded.")
        return result

    def _goto_pose_action(self, goal_handle):
        self.get_logger().info("Received cartesian position goal")

        start_cartesian_pose: SE3 = self._robot_arm.state.ee_pose
        start_q = np.array(self._robot_arm.state.q)  # [rad]

        goal_msg = goal_handle.request.pose_goal
        goal_pose = PoseStamped2SE3(goal_msg)
        duration = goal_handle.request.duration  # [s]

        self.get_logger().info(f"Current cartesian pose:\n {start_cartesian_pose}")
        self.get_logger().info(f"Goal cartesian pose:\n {goal_pose}")
        self.get_logger().info(f"Duration [s]: {duration}")

        traj_time_step = 0.01  # time between the trajectory points [s]

        #! Ikine to find goal joint positions
        goal_q = self._robot_arm.ikine(goal_pose)
        self.get_logger().info(
            f"Goal joint positions (from ikine): {np.rad2deg(goal_q)}"
        )

        #! Compute traj
        n_points = int(duration / traj_time_step)
        joint_traj = rtb.jtraj(start_q, goal_q, n_points)
        print(joint_traj.q.shape)
        self.get_logger().info(f"\n\nTrajectory shape: {joint_traj.q.shape}")

        start_time = self.get_clock().now()

        #! Build the JointTrajectory message
        self._joint_traj_msg.header.stamp = self.get_clock().now().to_msg()
        self._joint_traj_msg.joint_names = [
            "fr3_joint1",
            "fr3_joint2",
            "fr3_joint3",
            "fr3_joint4",
            "fr3_joint5",
            "fr3_joint6",
            "fr3_joint7",
        ]

        self._joint_traj_msg.points = joint_traj_to_msg(joint_traj, traj_time_step)

        #! Send the trajectory to the controller
        # Via the action server
        traj_goal = FollowJointTrajectory.Goal()
        traj_goal.trajectory = self._joint_traj_msg

        self._follow_joint_trajectory_ac.send_goal(traj_goal)

        end_time = self.get_clock().now()
        duration = (end_time - start_time).nanoseconds / 1e9
        self.get_logger().info(
            f"Trajectory execution completed in {duration:.2f} seconds."
        )

        end_pose = self._robot_arm.state.ee_pose

        #! Return results
        goal_handle.succeed()
        result = GotoPose.Result()
        result.success = True
        result.final_pose = SE32PoseStamped(end_pose)
        self.get_logger().info("Goal succeeded.")
        return result

    def _goto_joint_vels_action(self, goal_handle):
        self.get_logger().info("Received goal joint velocities goal")

        joint_vels = np.array(goal_handle.request.joint_velocities)
        duration: rclpy.time.Duration = rclpy.time.Duration.from_msg(
            goal_handle.request.duration
        )

        self.get_logger().info(f"Target joint velocities: {joint_vels}")
        self.get_logger().info(f"Duration [s]: {duration.nanoseconds / 1e9}")

        #! Setup joint velocities
        start_time = self.get_clock().now()

        self._joint_vels_cmd_msg.data = joint_vels.tolist()
        self._joint_vels_cmd_pub_timer.reset()

        self._joint_vels_cmd_pub_timer.reset()
        while (self.get_clock().now() - start_time) < duration:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("Goal canceled by client.")
                return GotoJointVelocities.Result()

            else:
                ...

        # self._joint_vels_cmd_pub_timer.cancel()

        #! Return results
        self._joint_vels_cmd_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        goal_handle.succeed()
        result = GotoJointVelocities.Result()
        result.success = True
        self.get_logger().info("Goal succeeded.")
        return result

    def _continuous_action_example(self, goal_handle):
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


"""
Utilities
"""


def joint_traj_to_msg(
    joint_traj: rtb.tools.trajectory.Trajectory,
    traj_time_step: float,
) -> List[JointTrajectoryPoint]:
    """
    Convert a joint trajectory to a list of JointTrajectoryPoint messages.
    """
    joint_traj_points = []

    for i, (q, qd, qdd) in enumerate(zip(joint_traj.q, joint_traj.qd, joint_traj.qdd)):
        point = JointTrajectoryPoint()
        seconds = i * traj_time_step
        time_from_start = Duration(seconds=seconds)
        # time_from_start.sec = int(seconds)
        # time_from_start.nanosec = int((seconds - time_from_start.sec) * 1e9)

        point.positions = q.tolist()
        point.velocities = qd.tolist()
        point.accelerations = qdd.tolist()

        point.time_from_start = time_from_start.to_msg()
        joint_traj_points.append(point)

    return joint_traj_points


def main(args=None):
    rclpy.init(args=args)
    node = FR3Interface()
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
