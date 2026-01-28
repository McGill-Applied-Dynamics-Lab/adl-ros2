"""
Calibrate button press timing in conjunction with the Speedgoat.
- The current best estimate is 0.018 m depth/0.30 s
"""

import time
import numpy as np
from scipy.spatial.transform import Rotation as R
from arm_client.robot import Robot, Pose, Twist
from arm_client import CONFIG_DIR
from pathlib import Path
import pickle

SETTLE_SEC = 1.00  # wait time after moves (s)
TRAJ_FREQ = 10.0 # Hz

# Helper functions
def probe(
    robot: Robot,
    start_xyz: np.ndarray,
    depth: float,
    probe_time: float,
    traj_freq: float = 5.0,
    fixed_ori: R | None = None,
    stop_depth: float | None = None,
):
    """
    Complete plunge and retract motion.

    Args:
        robot: Robot client (already ready & in Cartesian control).
        start_xyz: np.array([x, y, z_surface]) start point of plunge.
        depth: probe depth (positive indicates downwards).
        probe_time: seconds to complete the probe cycle (plunge + retract).
        traj_freq: Frequency of trajectory points per second.
        stop_depth: optionally stop and retract at this depth.
    """
    # Define starting orientation
    cur = robot.end_effector_pose.copy()  # Pose(position, orientation)
    if fixed_ori is None:
        fixed_ori = cur.orientation  # maintain current orientation
    target_pose = cur.copy()
    target_pose.position = start_xyz.astype(float)

    # Move to input start location
    robot.set_target(pose=target_pose)
    time.sleep(SETTLE_SEC)

    # --- Compute probe trajectory ---
    z_init = float(start_xyz[2])
    N = max(1, int(probe_time * traj_freq))
    dt = 1.0 / traj_freq

    waypoints = []  # list of (Pose, Twist) tuples of the trajectory
    time_from_start = []  # matching time of the trajectory points

    s = np.linspace(0, 1, N + 1)
    z = z_init + depth / 2 * (np.cos(2 * np.pi * s) - 1)
    t = np.arange(N + 1) * dt

    if stop_depth is not None:
        stop_idx = np.abs(z - stop_depth).argmin() # find where the end effector is at stop_depth
        z[stop_idx:] = np.linspace(z[stop_idx], z_init, N - stop_idx + 1) # linear retraction back to z_init

    # Include the endpoint (k = 0..N)
    for k in range(N + 1):
        target_position = np.array([start_xyz[0], start_xyz[1], z[k]], dtype=float)
        target_orientation = fixed_ori

        target_pose = Pose(target_position, target_orientation)
        twist = Twist(np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0]))

        waypoints.append((target_pose, twist))
        time_from_start.append(t[k])

    # --- Execute trajectory ---
    robot.execute_trajectory(waypoints, time_from_start)
    robot.wait_for_trajectory_completion(probe_time, timeout_margin=0.5)

    return None


Z_INIT = 0.15 # m
BUTTON_X = 0.678 # m
BUTTON_Y = -0.144 # m
PROBE_DEPTH = 0.0650 # m
PROBE_TIME = 2.0 # plunge and retract time (s)
TRIG_DEPTH = 0.0220 # m
STOP_DEPTHS = np.linspace(Z_INIT - 0.0175, Z_INIT - TRIG_DEPTH, num=30, endpoint=True)
PLUNGE_DEPTHS = np.linspace(0.0205, 0.0207, num=10, endpoint=True)
BASE_ORI = R.from_euler("xyz", [-180, 0, 0], degrees=True)

def main():
    # Setup
    robot = Robot(namespace="fr3")
    robot.wait_until_ready()

    robot.controller_switcher_client.switch_controller("fr3_pose_controller")
    robot.fr3_pose_controller_parameters_client.load_param_config(
        file_path=CONFIG_DIR / "controllers" / "fr3_pose" / "default.yaml"
    )

    # Parameters
    home_position = np.array([BUTTON_X, BUTTON_Y, Z_INIT])  # button location at initial z-height
    home_pose = Pose(home_position, BASE_ORI)

    # Iterate over probe heights
    prompt = input("Select the calibration method (A/B): ")
    input("Press Enter to start probing...")

    if prompt == "A":
        depths = PLUNGE_DEPTHS
    elif prompt == "B":
        depths = STOP_DEPTHS

    for i, depth in enumerate(depths):
        # --- Travel home ---
        robot.set_target(pose=home_pose)
        time.sleep(SETTLE_SEC)

        # --- Press trigger (1) ---
        print(f"\tPressing trigger: {i+1}/{len(depths)}")
        probe(
            robot,
            start_xyz=home_position,
            depth=TRIG_DEPTH,
            probe_time=PROBE_TIME,
            traj_freq=TRAJ_FREQ,
            fixed_ori=BASE_ORI,
        )
        time.sleep(SETTLE_SEC)

        # --- Press trigger (2) ---
        print("\tSecond plunge...")
        if prompt == "B":
            stop_depth = depth
            trig_depth = TRIG_DEPTH
        elif prompt == "A":
            stop_depth = None
            trig_depth = depth
        probe(
            robot,
            start_xyz=home_position,
            depth=trig_depth,
            probe_time=PROBE_TIME,
            traj_freq=TRAJ_FREQ,
            fixed_ori=BASE_ORI,
            stop_depth=stop_depth,
        )
        time.sleep(SETTLE_SEC)

        # Print depth
        print(f"\tProbe depth: {depth}")

        # Ask user to continue
        input("Press Enter to continue...")


    # Return home at the end
    print("\nReturning home...")
    robot.set_target(pose=home_pose)
    time.sleep(SETTLE_SEC)

    robot.shutdown()
    print("Done.")

if __name__ == "__main__":
    main()
