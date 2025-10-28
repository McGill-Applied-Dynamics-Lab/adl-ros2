"""Initialize the gripper module."""

from arm_client.gripper.gripper import Gripper, GripperConfig  # noqa: D104, F401

__import__ = [Gripper, GripperConfig]  # noqa: F405
