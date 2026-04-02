"""Home the robot, then set end-effector orientation to (90, 0, 0) deg."""

from scipy.spatial.transform import Rotation

from arm_client import CONFIG_DIR
from arm_client.robot import Pose, Robot


robot = Robot(namespace="fr3")
robot.wait_until_ready(timeout=2.0)

print("Homing robot...")
robot.home()

print("Switching to fr3_pose_controller...")
robot.controller_switcher_client.switch_controller("osc_pd_controller")
robot.osc_pd_controller_parameters_client.load_param_config(
    file_path=CONFIG_DIR / "controllers" / "osc_pd" / "default.yaml"
)

current_pose = robot.end_effector_pose
target_orientation = Rotation.from_euler("xyz", [90.0, 0.0, 0.0], degrees=True)
target_pose = Pose(position=current_pose.position, orientation=target_orientation)

print("Moving to target orientation (roll, pitch, yaw) = (90, 0, 0) deg...")
robot.move_to(pose=target_pose, speed=0.05)

print("Done")
robot.shutdown()
