import re
import time
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from sympy import rf
from arm_client.robot import Robot, Pose, Twist
from arm_client import CONFIG_DIR
from pathlib import Path
import pickle
from waveguide_gripper_grid_generator import fetch_landmarks


# Helper functions
def plunge(
    robot: Robot,
    start_xyz: np.ndarray,
    depth: float,
    plunge_time: float = 1.0,
    traj_freq: float = 100.0,
    fixed_ori: R | None = None,
):
    """
    Quarter-sine plunge from start_xyz to final depth.
    - s in [0,1] -> z(s) = z0 + (zf - z0) * sin(pi/2 * s)
    - Ends at phase 90° (velocity = 0).

    Args:
        robot: Robot client (already ready & in Cartesian control).
        start_xyz: np.array([x, y, z_surface]) start point of plunge.
        depth: positive = move down along -Z (final z = z_surface - depth).
        plunge_time: seconds to complete the plunge.
        traj_freq: Frequency of trajectory points per second.
    """
    # Fix starting orientation and make a working target pose
    cur = robot.end_effector_pose.copy()  # Pose(position, orientation)
    if fixed_ori is None:
        fixed_ori = cur.orientation  # keep orientation constant during plunge
    target_pose = cur.copy()
    target_pose.position = start_xyz.astype(float)

    # Ensure we start exactly at the commanded start pose
    robot.set_target(pose=target_pose)
    time.sleep(0.05)

    # --- Compute plunge trajectory ---
    z0 = float(start_xyz[2])
    zf = z0 - float(depth)  # positive depth goes down
    N = max(1, int(plunge_time * traj_freq))
    dt = 1.0 / traj_freq
    t0 = time.perf_counter()

    waypoints = []  # List of (Pose, Twist) tuples of the trajectory
    time_from_start = []  # Matching time of the trajectory points

    # Include the endpoint (k = 0..N)
    for k in range(N + 1):
        s = k / N  # 0..1
        z = z0 + (zf - z0) * np.sin(0.5 * np.pi * s)
        t = k * dt

        target_position = np.array([start_xyz[0], start_xyz[1], z], dtype=float)
        target_orientation = fixed_ori

        target_pose = Pose(target_position, target_orientation)
        twist = Twist(np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0]))

        waypoints.append((target_pose, twist))
        time_from_start.append(t)

    # --- Execute trajectory
    # Initialize save arrays
    ee_forces = []
    ee_poses = []
    target_poses = []
    ts = []

    # print("[plunge] Sending trajectory to controller...")
    robot.execute_trajectory(waypoints, time_from_start)

    # print("[plunge] Trajectory sent! Waiting for execution to complete...")
    start_time = time.time()
    while robot.wait_for_trajectory_completion(plunge_time, timeout_margin=0.5):
        ee_force = robot.end_effector_wrench["force"]
        ee_pose = robot.end_effector_pose
        time_stamp = time.time() - start_time

        ee_poses.append(ee_pose)
        ts.append(time_stamp)
        ee_forces.append(ee_force)

        # Record data
        ee_poses.append(robot.end_effector_pose.copy())
        ee_forces.append(robot.end_effector_wrench["force"].copy())
        target_poses.append(target_pose.copy())
        ts.append(time.perf_counter() - t0)

    return ts, target_poses, ee_poses, ee_forces


def main():
    # Setup
    robot = Robot(namespace="fr3")
    robot.wait_until_ready()

    # Choose controller that accepts trajectories in Cartesian space
    robot.controller_switcher_client.switch_controller("fr3_pose_controller")
    robot.fr3_pose_controller_parameters_client.load_param_config(
        file_path=CONFIG_DIR / "controllers" / "fr3_pose" / "default.yaml"
    )

    # Parameters
    # Load landmark file
    PROJECT_ROOT = Path(__file__).resolve().parent  # or Path.cwd()
    landmark_file = PROJECT_ROOT / "results" / "grids" / "waveguide_gripper_landmarks.txt"
    try:
        landmarks = fetch_landmarks(landmark_file, ["x", "y", "z"])

    except FileNotFoundError as e:
        landmarks = {"x": 0.4, "y": 0.0, "z": 0.4}
        print(f"Warning: {e}. Using default landmarks.")

    z_offset = 0.0250  # (m) offset from landmark z to surface
    z_surface = landmarks["z"] + z_offset  # (m)
    home_position = np.array([landmarks["x"], landmarks["y"], z_surface])  # home location (m)
    delay_sec = 1.0  # wait time (s) MODIFIED from 0.5
    retraction_sec = 5.0  # time to execute retractions (s)

    depth = z_offset + 0.0050  # plunge depth (m)
    plunge_time = 1.0  # plunge duration (s)
    traj_freq = 200.0  # Hz

    base_ori = R.from_euler("xyz", [-270, 0, 0], degrees=True)  # base orientation ([roll, pitch, yaw], degrees)
    base_pose = Pose(position=home_position, orientation=base_ori)

    # TODO: @Wilfred-Mason Uncomment this line when ready
    # # Probe locations: [x, y, z_surface], load from numpy files
    # base_loc = PROJECT_ROOT / "results" / "grids"
    # grid = np.load(f"{base_loc}/train_wvg.npy")  # (N, 2) array in world/robot frame
    # probe_locations = np.hstack([grid, z_surface * np.ones((len(grid), 1))])  # append z_surface to make (N, 3) arrays
    probe_locations = [
        [0.400, 0.000, 0.3],
        [0.400, -0.200, 0.3],
        [0.400, 0.0, 0.3],
    ]

    # Initialize results
    exp_dict = {
        "ts": [],
        "probe_locations": [],
        "probe_positions": [],
        "target_poses": [],
        "ee_poses": [],
        "ee_forces": [],
        "landmarks": [landmarks["x"], landmarks["y"], landmarks["z"]],
    }

    # Move to home position
    print("Going to home...")
    # robot.move_to(pose=base_pose, time_to_move=retraction_sec)
    robot.set_target(pose=base_pose)
    time.sleep(delay_sec)

    # Iterate over probe locations
    input("Press Enter to start probing...")
    for i, loc in enumerate(probe_locations):
        (x, y, z_surface) = loc
        exp_dict["probe_positions"].append([x, y, z_surface])

        print(
            f"\n=== Probe {i + 1}/{len(probe_locations)} at [{x - landmarks['x']:.3f}, {y - landmarks['y']:.3f}, {z_surface - landmarks['z']:.3f}] ==="
        )

        # Move to probe location
        print(f"\tMoving to probe location...")
        surface_xyz = np.array([x, y, z_surface], dtype=float)

        # TODO: update set_target to move_to method which computes a trajectory.
        # For now, use set_target, but a big delay to make sure robot reaches target.
        # or add a robot.wait_until_at_target() method ...
        robot.set_target(pose=Pose(position=surface_xyz, orientation=base_ori))
        time.sleep(delay_sec)

        input("\tPress Enter to start probing...")

        print(f"\tStarting plunge...")
        # Plunge: quarter-sine to final depth (velocity = 0 at end)
        ts, target_poses, ee_poses, ee_forces = plunge(
            robot,
            start_xyz=surface_xyz,
            depth=depth,
            plunge_time=3.0,
            traj_freq=20,
            fixed_ori=base_ori,
        )
        exp_dict["ts"].append(ts)  # time referenced to button press (ts referenced to plunge start)
        exp_dict["target_poses"].append(target_poses)
        exp_dict["ee_poses"].append(ee_poses)
        exp_dict["ee_forces"].append(ee_forces)
        print(f"\tPlunge complete.")
        time.sleep(delay_sec)

        # Move back home (retract in Z, then move in XY)
        print(f"\tRetracting...")
        retract_xyz = surface_xyz.copy()
        # robot.move_to(position=retract_xyz, time_to_move=retraction_sec)
        robot.set_target(position=retract_xyz)
        time.sleep(delay_sec)

    # Return home at the end
    print("\nReturning home...")
    # robot.move_to(pose=base_pose, time_to_move=retraction_sec)
    robot.set_target(pose=base_pose)
    time.sleep(delay_sec)

    robot.shutdown()
    print("Done.")

    # Save results
    results_dir = PROJECT_ROOT / "results"
    results_dir.mkdir(parents=True, exist_ok=True)
    # Save dict using pickle
    full_dir = results_dir / "TEMP.pkl"
    # Make sure file does not already exist / else save
    if full_dir.exists():
        # Ask user to confirm overwrite
        response = input(f"File already exists: {full_dir}. Overwrite? (y/n) ")
        if response.lower() != "y":
            print("Aborting save.")
            return
    with open(full_dir, "wb") as f:
        pickle.dump(exp_dict, f)
        print(f"Results saved to: {full_dir}")


if __name__ == "__main__":
    main()
