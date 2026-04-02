"""Open gripper, move to pick pose, close gripper, then move to second pose."""

import time

import numpy as np
from scipy.spatial.transform import Rotation

from arm_client import CONFIG_DIR
from arm_client.gripper.franka_hand import Gripper
from arm_client.robot import Pose, Robot
from spots import ORIGIN_MM_FR, Final_MM_FR

CLOSE_WIDTH_M = 0.01

# Input positions are in millimeters
pick_mm = np.array(ORIGIN_MM_FR)
place_mm = np.array(Final_MM_FR)

pick_m = pick_mm / 1000.0
place_m = place_mm / 1000.0
approach_offset_z = 0.15  # 15 cm approach/lift height
final_descent_height_z = 0.015  # Start final vertical-only descent 1.5 cm above pick
final_descent_steps = 20
initial_lift_z = 0.05  # First 5 cm lift after grasp
initial_lift_steps = 20

robot = Robot(namespace="fr3")
gripper = Gripper(namespace="fr3")

robot.wait_until_ready(timeout=5.0)
gripper.wait_until_ready(timeout=5.0)

print("Switching to fr3_pose_controller...")
robot.controller_switcher_client.switch_controller("fr3_pose_controller")
robot.fr3_pose_controller_parameters_client.load_param_config(
    file_path=CONFIG_DIR / "controllers" / "fr3_pose" / "default.yaml"
)
time.sleep(0.5)  # Let controller settle before first command

print("Opening gripper...")
opened = gripper.open(block=True)
if not opened:
    raise RuntimeError("Failed to open gripper.")
time.sleep(0.3)


# Keep end-effector orientation fixed at (roll, pitch, yaw) = (180, 0, 0) deg
target_orientation = Rotation.from_euler("xyz", [180.0, 0.0, 0.0], degrees=True)

pick_approach_m = pick_m.copy()
pick_approach_m[2] += approach_offset_z

print(f"Moving above first pose (m): {pick_approach_m.tolist()}")
pick_approach_pose = Pose(position=pick_approach_m, orientation=target_orientation)
robot.move_to(pose=pick_approach_pose, time_to_move=2.0)

pick_pregrasp_m = pick_m.copy()
pick_pregrasp_m[2] += final_descent_height_z

print(f"Descending to pre-grasp pose (m): {pick_pregrasp_m.tolist()}")
pick_pregrasp_pose = Pose(position=pick_pregrasp_m, orientation=target_orientation)
robot.move_to(pose=pick_pregrasp_pose, time_to_move=1.0)

print(
    f"Final vertical-only descent to first pose in {final_descent_steps} steps (m): {pick_m.tolist()}"
)
for step in range(1, final_descent_steps + 1):
    alpha = step / final_descent_steps
    z = pick_pregrasp_m[2] + alpha * (pick_m[2] - pick_pregrasp_m[2])
    vertical_step_m = pick_m.copy()
    vertical_step_m[0] = pick_pregrasp_m[0]
    vertical_step_m[1] = pick_pregrasp_m[1]
    vertical_step_m[2] = z
    vertical_step_pose = Pose(position=vertical_step_m, orientation=target_orientation)
    robot.move_to(pose=vertical_step_pose, time_to_move=0.5)

time.sleep(5)

print(f"Setting gripper width to {CLOSE_WIDTH_M:.4f} m...")
closed = gripper.set_target(CLOSE_WIDTH_M, speed=0.05, block=True)
if not closed:
    raise RuntimeError(f"Failed to set gripper width to {CLOSE_WIDTH_M:.4f} m.")
actual_close_width_m = gripper.value
print(
    f"Gripper close width target: {CLOSE_WIDTH_M:.4f} m, actual: {actual_close_width_m:.4f} m"
)
time.sleep(0.3)

slow_lift_z = min(initial_lift_z, approach_offset_z)
print(
    f"Slow vertical lift over first {slow_lift_z:.3f} m in {initial_lift_steps} steps from pick pose."
)
for step in range(1, initial_lift_steps + 1):
    alpha = step / initial_lift_steps
    z = pick_m[2] + alpha * slow_lift_z
    lift_step_m = pick_m.copy()
    lift_step_m[2] = z
    lift_step_pose = Pose(position=lift_step_m, orientation=target_orientation)
    robot.move_to(pose=lift_step_pose, time_to_move=0.12)

if approach_offset_z > slow_lift_z:
    print(f"Continuing lift to full approach height (m): {pick_approach_m.tolist()}")
    robot.move_to(pose=pick_approach_pose, time_to_move=1.0)

place_approach_m = place_m.copy()
place_approach_m[2] += approach_offset_z

print(f"Moving above second pose (m): {place_approach_m.tolist()}")
place_approach_pose = Pose(position=place_approach_m, orientation=target_orientation)
robot.move_to(pose=place_approach_pose, time_to_move=2.0)

print(f"Descending to second pose (m): {place_m.tolist()}")
place_pose = Pose(position=place_m, orientation=target_orientation)
robot.move_to(pose=place_pose, time_to_move=1.5)
time.sleep(2)
print("Opening gripper...")
opened = gripper.open(block=True)
if not opened:
    raise RuntimeError("Failed to open gripper.")
time.sleep(0.3)

print("Sequence complete.")
robot.shutdown()
