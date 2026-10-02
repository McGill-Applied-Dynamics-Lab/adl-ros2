import pickle
import time
from pathlib import Path

import numpy as np
from arm_client.robot import Pose, Robot, Twist
from scipy.spatial.transform import Rotation as R
from waveguide_gripper_grid_generator import fetch_landmarks

from arm_client import CONFIG_DIR

SETTLE_SEC = 2.00  # wait time after moves (s)
TRAJ_FREQ = 10.0  # Hz


# Helper functions
def probe(
    robot: Robot,
    start_xyz: np.ndarray,
    depth: float,
    probe_time: float,
    traj_freq: float = 5.0,
    fixed_ori: R | None = None,
    probe_func: str = "cos",
):
    """
    Complete plunge and retract motion.

    Args:
        robot: Robot client (already ready & in Cartesian control).
        start_xyz: np.array([x, y, z_surface]) start point of plunge.
        depth: probe depth (positive indicates downwards).
        probe_time: seconds to complete the probe cycle (plunge + retract).
        traj_freq: Frequency of trajectory points per second.
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
    t0 = time.perf_counter()

    waypoints = []  # list of (Pose, Twist) tuples of the trajectory
    time_from_start = []  # matching time of the trajectory points

    # Include the endpoint (k = 0..N)
    for k in range(N + 1):
        s = k / N  # 0..1
        if probe_func == "cos":
            z = z_init + depth / 2 * (np.cos(2 * np.pi * s) - 1)
        elif probe_func == "linear":
            z = z_init + depth * (np.abs(2 * s - 1) - 1)
        t = k * dt

        target_position = np.array([start_xyz[0], start_xyz[1], z], dtype=float)
        target_orientation = fixed_ori

        target_pose = Pose(target_position, target_orientation)
        twist = Twist(np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0]))

        waypoints.append((target_pose, twist))
        time_from_start.append(t)

    # --- Execute trajectory ---
    # Initialize save arrays
    ee_forces = []
    ee_poses = []
    ts = []

    robot.execute_trajectory(waypoints, time_from_start)

    t0 = time.perf_counter()
    t_min = 0.0
    z_min = z_init
    while robot.wait_for_trajectory_completion(probe_time, timeout_margin=0.5):
        ee_force = robot.end_effector_external_wrench["force"]
        ee_pose = robot.end_effector_pose
        time_stamp = time.perf_counter() - t0  # time since trajectory start

        # Time-stamp when z-displacement is max
        if ee_pose.position[2] < z_min:
            z_min = ee_pose.position[2]
            t_min = time.perf_counter()  # absolute time

        # Record data
        ee_poses.append(ee_pose.copy())
        ee_forces.append(ee_force.copy())
        ts.append(time_stamp)

        # Must have sleep
        time.sleep(0.01)

    return ts, ee_poses, ee_forces, t_min


# Probing parameters
Z_OFFSET = 0.0250  # (m) offset from landmark z to surface
PROBE_DEPTH = 0.0200  # m (additional depth beyond z_offset)
PROBE_TIME = 2.0  # plunge and retract (s)
BASE_ORI = R.from_euler("xyz", [-270, 0, 0], degrees=True)


def main():
    # Setup
    robot = Robot(namespace="fr3")
    robot.wait_until_ready()

    robot.controller_switcher_client.switch_controller("fr3_pose_controller")
    robot.fr3_pose_controller_parameters_client.load_param_config(
        file_path=CONFIG_DIR / "controllers" / "fr3_pose" / "probing.yaml"
    )

    # Load landmark file
    PROJECT_ROOT = Path(__file__).resolve().parent
    landmark_file = PROJECT_ROOT / "results" / "grids" / "landmarks.txt"

    # Check if landmark file exists
    if not landmark_file.exists():
        raise FileNotFoundError(
            f"Landmark file not found: {landmark_file}. Please run the landmark detection script first."
        )

    landmarks = fetch_landmarks(landmark_file, ["x", "y", "z"])

    # Parameters - use landmarks for home position
    z_surface = landmarks["z"] + Z_OFFSET  # (m) surface is offset from landmark
    home_position = np.array([landmarks["x"], landmarks["y"], z_surface])
    home_pose = Pose(home_position, BASE_ORI)

    # Load probe locations from grid file
    grid_file = PROJECT_ROOT / "results" / "grids" / "grids.pkl"

    # Check if grid file exists
    if not grid_file.exists():
        raise FileNotFoundError(f"Grid file not found: {grid_file}. Please run grid_generator.py first.")

    with open(grid_file, "rb") as f:
        grids = pickle.load(f)

    # Let user select train or test set
    set_name = input("Select grid set (train/test) [default: test]: ").strip().lower()
    if set_name not in ["train", "test"]:
        set_name = "test"
        print(f"Invalid selection. Using default: {set_name}")

    # Get world frame grid (N, 2) array for robot motion
    grid_xy_world = grids["WORLD_FRAME"][set_name]  # (N, 2) array in world/robot frame

    # Get gripper frame grid (N, 2) array for data saving
    grid_xy_gripper = grids["GRIPPER_FRAME"][set_name]  # (N, 2) array in gripper frame

    # Initialize results
    exp_dict = {
        "ts": [],
        "grid_positions": [],  # Store the (x, y) positions from the grid in GRIPPER_FRAME
        "ee_poses": [],
        "ee_forces": [],
        "set_name": set_name,  # Record which set was used
        "landmarks": landmarks,  # Store landmarks for reference
        "z_offset": Z_OFFSET,  # Store z_offset for reference
    }

    # Iterate over probe locations
    input("Press Enter to start probing...")
    for i, loc in enumerate(grid_xy_world):
        x, y = loc
        # Save gripper frame coordinates
        x_gripper, y_gripper = grid_xy_gripper[i]
        exp_dict["grid_positions"].append([x_gripper, y_gripper])
        print(f"\n Probe {i + 1}/{len(grid_xy_world)}")

        # --- Move to probe location ---
        print("\tMoving to probe location...")
        approach_xy = np.array([x, y, z_surface], dtype=float)
        robot.set_target(position=approach_xy)
        time.sleep(SETTLE_SEC)

        # --- Probe cycle ---
        print("\tStarting probe...")
        ts, ee_poses, ee_forces, _ = probe(
            robot,
            start_xyz=approach_xy,
            depth=Z_OFFSET + PROBE_DEPTH,  # Total depth from surface
            probe_time=PROBE_TIME,
            traj_freq=TRAJ_FREQ,
            fixed_ori=BASE_ORI,
            probe_func="cos",
        )

        # Convert Pose objects to numpy arrays for saving
        ee_positions = [pose.position for pose in ee_poses]
        ee_orientations = [pose.orientation.as_quat() for pose in ee_poses]

        # --- Store results ---
        exp_dict["ts"].append(ts)
        # Store numpy arrays instead of Pose objects
        exp_dict["ee_poses"].append({"positions": ee_positions, "orientations": ee_orientations})
        exp_dict["ee_forces"].append(ee_forces)  # already numpy arrays
        print("\tProbe complete.")

    # Return home at the end
    print("\nReturning home...")
    robot.set_target(pose=home_pose)
    time.sleep(SETTLE_SEC)

    robot.shutdown()
    print("Done.")

    # Save results
    results_dir = PROJECT_ROOT / "results"
    results_dir.mkdir(parents=True, exist_ok=True)

    # Generate filename based on set name and number of points
    # Add number suffix if file exists (starting from 00)
    base_filename = f"{len(grid_xy_world)}_grid_{set_name.upper()}"
    counter = 0
    filename = f"{base_filename}_{counter:02d}.pkl"
    full_path = results_dir / filename

    # Find the next available number
    while full_path.exists():
        counter += 1
        filename = f"{base_filename}_{counter:02d}.pkl"
        full_path = results_dir / filename

    # Save the data
    with open(full_path, "wb") as f:
        pickle.dump(exp_dict, f)
        print(f"Results saved to: {full_path}")


if __name__ == "__main__":
    main()
