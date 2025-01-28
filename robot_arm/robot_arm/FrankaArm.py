# ROS Imports
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from rclpy.action import ActionClient
from arm_interfaces.action import GotoJoints, GotoPose, GotoJointVelocities
from geometry_msgs.msg import PoseStamped

import numpy as np
from typing import List, Literal
from spatialmath.pose3d import SO3, SE3
from robot_arm_interface.utils import SE32PoseStamped, PoseStamped2SE3


class FrankaArm(Node):
    def __init__(self):
        super().__init__("franka_arm_client")
        self.get_logger().info("Starting robot arm client")

        #! Action clients
        self._action_client_list = []

        # Go to joints
        self._goto_joints_ac = ActionClient(self, GotoJoints, "goto_joints")
        self._action_client_list.append(self._goto_joints_ac)

        self._goto_pose_ac = ActionClient(self, GotoPose, "goto_pose")
        self._action_client_list.append(self._goto_pose_ac)

        self._goto_joint_vel_ac = ActionClient(
            self, GotoJointVelocities, "goto_joint_vels"
        )
        self._action_client_list.append(self._goto_joint_vel_ac)

        self._wait_for_server()

    def _wait_for_server(self):
        self.get_logger().info("Waiting for action servers...")
        for ac in self._action_client_list:
            ac.wait_for_server()

        self.get_logger().info("Action servers are up!")

    #! Action clients
    def _wait_for_action(self, future):
        rclpy.spin_until_future_complete(self, future)

        if future.done():
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error("Goal was rejected by the action server!")
                return False

            self.get_logger().info("Goal accepted, waiting for result...")
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)

            if result_future.done():
                result = result_future.result()
                self.get_logger().info(f"Result received: {result.result.success}")
                return result.result.success

            else:
                self.get_logger().error("Failed to get result!")
                return False
        else:
            self.get_logger().error("Failed to send goal!")
            return False

    def goto_joints(self, joint_goal: np.ndarray, duration: float):
        self.get_logger().info(f"Moving joints to goal: {joint_goal}")

        goal_msg = GotoJoints.Goal()
        goal_msg.joints_goal = joint_goal.tolist()
        goal_msg.duration = duration

        future = self._goto_joints_ac.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        if future.done():
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error("Goal was rejected by the action server!")
                return False

            self.get_logger().info("Goal accepted, waiting for result...")
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)

            if result_future.done():
                result = result_future.result()
                self.get_logger().info(f"Result received: {result.result.success}")
                return result.result.success

            else:
                self.get_logger().error("Failed to get result!")
                return False
        else:
            self.get_logger().error("Failed to send goal!")
            return False

    def goto_home(self):
        self.get_logger().info("Moving to home position")

        joint_home_position = np.deg2rad([0, -45, 0, -135, 0, 90, 45])
        self.goto_joints(joint_home_position, 5.0)

    def goto_pose(self, pose_goal: SE3, duration: float):
        self.get_logger().info(f"Moving to cartesian goal:\n {pose_goal}")

        pose_goal_msg = SE32PoseStamped(pose_goal)

        T = PoseStamped2SE3(pose_goal_msg)

        goal_msg = GotoPose.Goal()
        goal_msg.pose_goal = pose_goal_msg
        goal_msg.duration = duration

        future = self._goto_pose_ac.send_goal_async(goal_msg)
        return self._wait_for_action(future)

    def goto_ee_vel(self): ...

    def goto_joint_vels(
        self,
        joint_vels: List | np.ndarray,
        duration: Duration,
        unit: str = Literal["rad", "deg"],
    ):
        # Process the joint vels
        if unit == "deg":
            joint_vels = np.deg2rad(joint_vels)

        if type(joint_vels) is list:
            joint_vels = np.array(joint_vels)

        self.get_logger().info(
            f"Moving joints with velocities: {np.rad2deg(joint_vels)}"
        )

        # Build the goal message
        goal_msg = GotoJointVelocities.Goal()
        goal_msg.joint_velocities = joint_vels.tolist()
        goal_msg.duration = duration.to_msg()

        future = self._goto_joint_vel_ac.send_goal_async(goal_msg)
        return self._wait_for_action(future)

    def open_gripper(self): ...

    def close_gripper(self): ...

    #! Services
    def get_robot_state(self): ...

    def get_joints(self): ...

    def get_pose(self): ...


def main(args=None):
    rclpy.init(args=args)
    node = FrankaArm()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

    # executor = MultiThreadedExecutor()
    # try:
    #     rclpy.spin(node, executor=executor)
    # except KeyboardInterrupt:
    #     pass

    # finally:
    #     node.destroy_node()
    #     rclpy.shutdown()


if __name__ == "__main__":
    main()
