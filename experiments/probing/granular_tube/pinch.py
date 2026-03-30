from scipy.spatial.transform import Rotation as R
from arm_client.robot import Robot, Pose, Twist
from arm_client import CONFIG_DIR
from pathlib import Path
import numpy as np
import time
from arm_client.gripper.franka_hand import Gripper


SETTLE_SEC = 5.0  # Wait time after moves (s)
SAFE_ORI = R.from_euler("xyz", [-180, 0, 0], degrees=True)
SAFE_POS = np.array([0.40, 0, 0.30]) # safe starting location
PROJECT_ROOT = Path(__file__).resolve().parent
TRAJ_FREQ = 10 # (Hz)

TUBE_CENTER_POS = np.array([0.45, 0, 0.30]) # center of tube
OUTER_RADIUS = 0.15
INNER_RADIUS = 0.05
PINCH_TIME = 1.0

def move_waypoints(start_rad, end_rad, radius, end_ori, end_z, speed, traj_freq):
    delta = np.remainder(end_rad - start_rad, 2 * np.pi)
    waypoints = []
    ds = radius*np.abs(delta)
    dt = 1/traj_freq
    N = int(np.ceil(ds/(speed * dt)))

    for i in range(N + 1):
        current_theta = start_rad + (delta * i / N)
        x = radius * np.cos(current_theta)
        y = radius * np.sin(current_theta)
        twist = Twist(np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0]))
        waypoints.append((Pose(np.array([x, y, end_z]), end_ori), twist))

    return np.linspace(0, N * dt, N + 1), waypoints

def main():
    # Initialization
    robot = Robot(namespace="fr3")
    gripper = Gripper(namespace="fr3")
    robot.wait_until_ready()
    gripper.wait_until_ready(timeout=5.0)

    robot.controller_switcher_client.switch_controller("fr3_pose_controller")
    robot.fr3_pose_controller_parameters_client.load_param_config(
        file_path=CONFIG_DIR / "controllers" / "fr3_pose" / "probing.yaml"
    )

    robot.set_target(pose=Pose(SAFE_POS, SAFE_ORI))
    time.sleep(SETTLE_SEC)

    gripper.open()
    time.sleep(SETTLE_SEC)

    prev_angle = None
    for aidx, angle in enumerate(np.radians(np.arange(0, 360, 90))):
        target_rot = R.from_euler('z', angle, degrees=False) * SAFE_ORI

        if aidx != 0:
            times, waypoints = move_waypoints(prev_angle, angle, OUTER_RADIUS, target_rot, 0.30, speed=0.05, traj_freq=TRAJ_FREQ)
            robot.execute_trajectory(waypoints, times)
            while robot.wait_for_trajectory_completion(times[-1], timeout_margin=0.5):
                time.sleep(0.01)
        # Set orientation
        target_rot = R.from_euler('z', angle, degrees=False) * SAFE_ORI
        robot.set_target(pose=Pose(SAFE_POS, target_rot))
        time.sleep(SETTLE_SEC)
        # Move to outer position
        outer_pos = TUBE_CENTER_POS + np.array([np.cos(angle), np.sin(angle), 0]) * OUTER_RADIUS
        robot.set_target(pose=Pose(outer_pos, target_rot))
        time.sleep(SETTLE_SEC)
        # Move radially inward
        inner_pos = TUBE_CENTER_POS + np.array([np.cos(angle), np.sin(angle), 0]) * INNER_RADIUS
        robot.set_target(pose=Pose(inner_pos, target_rot))
        time.sleep(SETTLE_SEC)
        # Cycle the gripper
        gripper.close()
        time.sleep(PINCH_TIME)
        gripper.open()
        time.sleep(SETTLE_SEC)
        # Move radially outward
        robot.set_target(pose=Pose(outer_pos, target_rot))
        time.sleep(SETTLE_SEC)
        # Save previous angle
        prev_angle = angle

    # Move back to safe position
    print("\nReturning to safe pos...")
    robot.set_target(pose=Pose(SAFE_POS, SAFE_ORI))
    time.sleep(SETTLE_SEC)

    print("\nShutting down...")
    robot.shutdown()

if __name__ == "__main__":
    main()
