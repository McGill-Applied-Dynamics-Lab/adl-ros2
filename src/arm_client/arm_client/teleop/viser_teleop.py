import numpy as np

from arm_client.robot import Robot
from arm_client.teleop.base_teleop import BaseTeleop


class ViserTeleop(BaseTeleop):
    """Web-based interactive teleoperation interface using Viser."""

    def __init__(self, robot: Robot):
        """Initialize the Viser interactive teleoperation server.

        Args:
            robot: The active hardware implementation of the robot arm client.
        """
        self._robot = robot

        # Grab target initial pose as the starting transform handle location
        start_pose = self._robot.end_effector_pose.copy()

        self._transform_handle = self._robot.visualizer.server.scene.add_transform_controls(
            "/ik_target", scale=0.2, position=start_pose.position, wxyz=start_pose.orientation.as_quat()[[3, 0, 1, 2]]
        )

        self._target_frame = self._robot.visualizer.server.scene.add_batched_axes(
            "/ik_target_frame",
            axes_length=0.05,
            axes_radius=0.005,
            batched_positions=np.zeros((1, 3)),
            batched_wxyzs=np.array([[1.0, 0.0, 0.0, 0.0]]),
        )

    def update_visualization(
        self, joint_configuration: np.ndarray, target_positions: np.ndarray, target_wxyzs: np.ndarray
    ):
        """Update robot mesh and planned objective path."""

        self._robot.visualizer.update_robot_config(joint_configuration)

        if hasattr(self._target_frame, "batched_positions"):
            self._target_frame.batched_positions = target_positions
            self._target_frame.batched_wxyzs = target_wxyzs
        else:
            self._target_frame.positions_batched = target_positions
            self._target_frame.wxyzs_batched = target_wxyzs

    @property
    def target_position(self) -> np.ndarray:
        return np.array(self._transform_handle.position)

    @property
    def target_orientation(self) -> np.ndarray:
        return np.array(self._transform_handle.wxyz)

    def start(self) -> None:
        print("Visual teleoperation started at http://localhost:8080")

    def stop(self) -> None:
        self._transform_handle.remove()
        self._target_frame.remove()
