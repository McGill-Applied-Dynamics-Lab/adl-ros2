"""A ROS2 Action Client for controlling robot joints through trajectory messages.

This module provides a JointTrajectoryControllerClient class that simplifies sending joint
trajectories to a ROS2-controlled robot. It handles the communication with the robot's
joint trajectory controller through ROS2 actions.

The client supports:
- Sending joint configurations with specified execution time
- Blocking and non-blocking execution modes
- Automatic namespace handling for multi-robot setups

Example:
    ```python
    import rclpy
    from rclpy.node import Node
    from joint_trajectory_controller_client import JointTrajectoryControllerClient

    node = Node("my_control_node")
    client = JointTrajectoryControllerClient(node)

    # Send a joint configuration
    joint_names = ["joint1", "joint2", "joint3"]
    joint_positions = [0.5, -0.3, 1.0]
    client.send_joint_config(joint_names, joint_positions, time_to_goal=3.0)
    ```
"""

from typing import Optional

from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from arm_client.planning.types import PlannedJointTrajectory

import numpy as np


class JointTrajectoryControllerClient(ActionClient):
    """A ROS2 Action Client for controlling robot joints through trajectory messages.

    This class provides a high-level interface for sending joint trajectories to a ROS2-controlled
    robot. It inherits from ActionClient and handles the communication with the robot's joint
    trajectory controller through ROS2 actions.

    Args:
        node (Node): The ROS2 node that will own this client.
    """

    def __init__(self, node: Node) -> None:
        """Initialize the JointTrajectoryControllerClient.

        Args:
            node (Node): The ROS2 node that will own this client.
        """
        self.node = node
        super().__init__(node, FollowJointTrajectory, "joint_trajectory_controller/follow_joint_trajectory")
        self._goal = FollowJointTrajectory.Goal()
        namespace = self.node.get_namespace().strip("/")
        self._prefix = ""  # f"{namespace}_" if namespace else ""

        self.trajectory_publisher = self.node.create_publisher(
            JointTrajectory, "joint_trajectory_controller/joint_trajectory", 10
        )

    def send_joint_config(
        self,
        joint_names: list[str],
        joint_config: list[float],
        time_to_goal: float = 5.0,
        blocking: bool = True,
    ) -> Optional[FollowJointTrajectory.Result]:
        """Send a joint configuration to the robot.

        Args:
            joint_names (list[str]): List of joint names to control.
            joint_config (list[float]): List of joint positions corresponding to joint_names.
            time_to_goal (float, optional): Time in seconds to reach the goal position. Defaults to 5.0.
            blocking (bool, optional): Whether to wait for the movement to complete. Defaults to True.

        Returns:
            Optional[FollowJointTrajectory.Result]: If blocking is True, returns the result of the
                trajectory execution. If blocking is False, returns None.
        """
        self._goal.trajectory.joint_names = [self._prefix + joint_name for joint_name in joint_names]
        self._goal.trajectory.header.stamp = self.node.get_clock().now().to_msg()
        self._goal.trajectory.points = []
        self._goal.trajectory.points.append(
            JointTrajectoryPoint(
                positions=joint_config,
                velocities=len(joint_config) * [0.0],
                accelerations=len(joint_config) * [0.0],
                time_from_start=Duration(seconds=int(time_to_goal), nanoseconds=0).to_msg(),
            )
        )
        future = self.send_goal_async(self._goal)

        if blocking:
            while not future.done():
                self.node.get_logger().debug("Waiting for goal answer...", throttle_duration_sec=1.0)

            goal_handle = future.result()

            future = goal_handle.get_result_async()
            while not future.done():
                self.node.get_logger().debug("Waiting for goal result...", throttle_duration_sec=1.0)

            self.node.get_logger().debug(f"Goal result: {future.result()}")
            return future.result()

    def send_joint_trajectory(
        self,
        joint_trajectory: PlannedJointTrajectory,
        blocking: bool = True,
    ) -> Optional[FollowJointTrajectory.Result]:
        """Send a full joint trajectory to the robot.

        Args:
            joint_trajectory (PlannedJointTrajectory): The planned joint trajectory to send.
            blocking (bool, optional): Whether to block until completion.

        Returns:
            Optional[FollowJointTrajectory.Result]: Result when blocking, otherwise None.
        """
        joint_names = joint_trajectory.joint_names

        joint_positions = joint_trajectory.joint_positions
        joint_velocities = joint_trajectory.joint_velocities
        joint_accelerations = joint_trajectory.joint_accelerations
        time_from_start = joint_trajectory.time_from_start

        if len(joint_positions) != len(time_from_start):
            raise ValueError("joint_positions and time_from_start must have the same length")
        if len(joint_positions) == 0:
            raise ValueError("joint_positions cannot be empty")

        self._goal.trajectory.joint_names = [self._prefix + joint_name for joint_name in joint_names]
        self._goal.trajectory.header.stamp = self.node.get_clock().now().to_msg()
        self._goal.trajectory.points = []

        for position, vel, accel, t in zip(joint_positions, joint_velocities, joint_accelerations, time_from_start):
            point = JointTrajectoryPoint()
            point.time_from_start = Duration(seconds=int(t), nanoseconds=int((t % 1) * 1e9)).to_msg()

            point.positions = [float(p) for p in position]

            if not np.isnan(vel).any():
                point.velocities = [float(v) for v in vel]

            if not np.isnan(accel).any():
                point.accelerations = [float(a) for a in accel]

            self._goal.trajectory.points.append(point)

        future = self.send_goal_async(self._goal)

        if blocking:
            while not future.done():
                self.node.get_logger().debug("Waiting for trajectory goal answer...", throttle_duration_sec=1.0)

            goal_handle = future.result()

            future = goal_handle.get_result_async()
            while not future.done():
                self.node.get_logger().debug("Waiting for trajectory result...", throttle_duration_sec=1.0)

            self.node.get_logger().debug(f"Trajectory goal result: {future.result()}")
            return future.result()

    def publish_joint_trajectory(
        self,
        joint_trajectory: PlannedJointTrajectory,
    ) -> None:
        """Publish a full joint trajectory directly to the topic (non-blocking splining).

        Args:
            joint_trajectory (PlannedJointTrajectory): The planned joint trajectory to sequence.
        """
        joint_names = joint_trajectory.joint_names

        joint_positions = joint_trajectory.joint_positions
        joint_velocities = joint_trajectory.joint_velocities
        joint_accelerations = joint_trajectory.joint_accelerations
        time_from_start = joint_trajectory.time_from_start

        if len(joint_positions) != len(time_from_start):
            raise ValueError("joint_positions and time_from_start must have the same length")
        if len(joint_positions) == 0:
            raise ValueError("joint_positions cannot be empty")

        msg = JointTrajectory()
        msg.joint_names = [self._prefix + joint_name for joint_name in joint_names]
        msg.header.stamp = self.node.get_clock().now().to_msg()

        for position, vel, accel, t in zip(joint_positions, joint_velocities, joint_accelerations, time_from_start):
            point = JointTrajectoryPoint()
            point.time_from_start = Duration(seconds=int(t), nanoseconds=int((t % 1) * 1e9)).to_msg()

            point.positions = [float(p) for p in position]

            if not np.isnan(vel).any():
                point.velocities = [float(v) for v in vel]

            if not np.isnan(accel).any():
                point.accelerations = [float(a) for a in accel]

            msg.points.append(point)

        self.trajectory_publisher.publish(msg)
