"""Provides configuration classes for different robot types."""

from dataclasses import dataclass, field

import numpy as np


@dataclass(kw_only=True)
class RobotConfig:
    """Configuration class for robot parameters.

    This class holds all the necessary configuration parameters for a robot,
    including frame names, controller settings, and topic names.

    Attributes:
        home_config (list): Joint positions for the home configuration
        joint_names (list): List of joint names in order
        base_frame (str): Name of the robot's base frame
        target_frame (str): Name of the robot's end-effector frame
        default_controller (str): Name of the default controller to use
        cartesian_impedance_controller_name (str): Name of the Cartesian impedance controller
        target_pose_topic (str): Topic name for publishing target poses
        target_joint_topic (str): Topic name for publishing target joint states
        current_pose_topic (str): Topic name for subscribing to current poses
        current_joint_topic (str): Topic name for subscribing to current joint states
        publish_frequency (float): Frequency for publishing control commands
        time_to_home (float): Time taken to reach home position
    """

    joint_names: list
    home_config: list

    base_frame: str = "base_link"
    target_frame: str = "end_effector_link"

    default_controller: str = "cartesian_impedance_controller"
    cartesian_impedance_controller_name: str = "/fr3/cartesian_impedance_controller"
    joint_trajectory_controller_name: str = "/fr3/joint_impedance_controller"

    target_pose_topic: str = "target_pose"
    target_joint_topic: str = "target_joint"
    target_trajectory_topic: str = "target_trajectory"

    current_pose_topic: str = "current_pose"
    current_joint_topic: str = "joint_states"

    current_wrench_topic: str = (
        "/fr3/franka_robot_state_broadcaster/external_wrench_in_base_frame"  # added current wrench topic
    )

    publish_frequency: float = 50.0
    time_to_home: float = 5.0

    max_pose_delay: float = 1.0
    max_joint_delay: float = 1.0

    def num_joints(self) -> int:
        """Returns the number of joints in the robot."""
        return len(self.joint_names)


@dataclass
class FR3Config(RobotConfig):
    """Configuration specific to Franka Research 3 robot.

    Provides default values for frame names, joint names, and home configuration..
    """

    joint_names: list = field(
        default_factory=lambda: [
            "fr3_joint1",
            "fr3_joint2",
            "fr3_joint3",
            "fr3_joint4",
            "fr3_joint5",
            "fr3_joint6",
            "fr3_joint7",
        ]
    )
    home_config: list = field(
        default_factory=lambda: [
            0,
            -np.pi / 4,
            0,
            -3 * np.pi / 4,
            0,
            np.pi / 2,
            np.pi / 4,
        ]
    )
    base_frame: str = "base"
    target_frame: str = "fr3_hand_tcp"
    # target_frame: str = "fr3_link8"


def make_robot_config(robot_type: str, **kwargs) -> RobotConfig:  # noqa: ANN003
    """Factory function to create robot configuration objects.

    Args:
        robot_type (str): Type of robot ('franka', 'kinova', 'iiwa', 'so101')
        **kwargs: Additional keyword arguments to override default configuration

    Returns:
        RobotConfig: Configured robot configuration object

    Raises:
        ValueError: If robot_type is not supported
    """
    robot_type = robot_type.lower()

    if robot_type == "fr3":
        return FR3Config(**kwargs)
    # elif robot_type == "kinova":
    #     return KinovaConfig(**kwargs)
    # elif robot_type == "iiwa":
    #     return IiwaConfig(**kwargs)
    # elif robot_type == "so101":
    #     return SO101Config(**kwargs)
    else:
        raise ValueError(f"Unsupported robot type: {robot_type}. Supported types: franka, kinova, iiwa, so101")
