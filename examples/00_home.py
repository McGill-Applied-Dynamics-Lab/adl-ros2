"""Home the robot, then set end-effector orientation to (180, 0, 0) deg."""

import time

from scipy.spatial.transform import Rotation

from arm_client import CONFIG_DIR
from arm_client.robot import Pose, Robot


robot = Robot(namespace="fr3")
robot.wait_until_ready(timeout=2.0)

print("Homing robot...")
robot.home()
robot.reset_targets()

print("Switching to fr3_pose_controller...")
robot.controller_switcher_client.switch_controller("fr3_pose_controller")
robot.fr3_pose_controller_parameters_client.load_param_config(
    file_path=CONFIG_DIR / "controllers" / "fr3_pose" / "default.yaml"
)
time.sleep(0.1)

current_pose = robot.end_effector_pose
target_orientation = Rotation.from_euler("xyz", [180.0, 0.0, 0.0], degrees=True)
target_pose = Pose(position=current_pose.position, orientation=target_orientation)

print("Moving to target orientation (roll, pitch, yaw) = (180, 0, 0) deg...")
robot.move_to(pose=target_pose, time_to_move=2.0)
time.sleep(0.1)

print("Done")
robot.shutdown()
