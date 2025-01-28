# ROS Imports
import rclpy
from rclpy.node import Node

from rclpy.action import ActionClient
from arm_interfaces.action import GotoJoints

import numpy as np


class FrankaArm(Node):
    def __init__(self):
        super().__init__("franka_arm_client")
        self.get_logger().info("Starting robot arm client")

        #! Action clients
        self._action_client_list = []

        # Go to joints
        self._goto_joints_ac = ActionClient(self, GotoJoints, "goto_joints")
        self._action_client_list.append(self._goto_joints_ac)

        self._wait_for_server()

    def _wait_for_server(self):
        self.get_logger().info("Waiting for action servers...")
        for ac in self._action_client_list:
            ac.wait_for_server()

        self.get_logger().info("Action servers are up!")

    def goto_joints(self, joint_goal: np.ndarray, duration: float):
        self.get_logger().info(f"Moving joints to goal: {joint_goal}")

        goal_msg = GotoJoints.Goal()
        goal_msg.joints_goal = joint_goal.tolist()
        goal_msg.duration = duration

        return self._goto_joints_ac.send_goal_async(goal_msg)


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
