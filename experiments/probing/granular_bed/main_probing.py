from re import S
import time
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from sympy import rf
from arm_client import robot
from arm_client.robot import Robot, Pose, Twist
from arm_client import CONFIG_DIR
from pathlib import Path
import pickle

SETTLE_SEC = 0.250  # wait time after moves (s)


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
    - Ends at phase 90 deg. (velocity = 0).

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
    time.sleep(SETTLE_SEC)

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

    # --- Execute trajectory ---
    # Initialize save arrays
    ee_forces = []
    ee_poses = []
    target_poses = []
    ts = []

    # print("[plunge] Sending trajectory to controller...")
    robot.execute_trajectory(waypoints, time_from_start)

    # print("[plunge] Trajectory sent! Waiting for execution to complete...")
    t0 = time.perf_counter()
    while robot.wait_for_trajectory_completion(plunge_time, timeout_margin=0.5):
        ee_force = robot.end_effector_wrench["force"]
        ee_pose = robot.end_effector_pose
        time_stamp = time.perf_counter() - t0

        # Record data
        ee_poses.append(ee_pose.copy())
        ee_forces.append(ee_force.copy())
        target_poses.append(target_pose.copy())
        ts.append(time_stamp)

    return ts, target_poses, ee_poses, ee_forces


def main():
    # Setup
    robot = Robot(namespace="fr3")
    robot.wait_until_ready()

    # Choose controller that accepts /target_pose
    # robot.controller_switcher_client.switch_controller("osc_pd_controller")
    # robot.haptic_controller_parameters_client.load_param_config(
    #     file_path=CONFIG_DIR / "controllers" / "osc_pd" /"probe_controller.yaml"
    # )

    # robot.controller_switcher_client.switch_controller("joint_space_controller")
    # robot.joint_space_controller_parameters_client.load_param_config(
    #     file_path=CONFIG_DIR / "controllers" / "joint_space" / "default.yaml"
    # )

    robot.controller_switcher_client.switch_controller("fr3_pose_controller")
    robot.fr3_pose_controller_parameters_client.load_param_config(
        file_path=CONFIG_DIR / "controllers" / "fr3_pose" / "default.yaml"
    )

    # Parameters
    z_surface = 0.15  # (m)
    home_position = np.array([0.60, -0.060, z_surface])  # button location
    approach_speed = 0.050  # (m/s)

    depth = 0.0575  # plunge depth (m)
    plunge_time = 1.0  # plunge duration (s)
    plunge_rest = 1.0  # rest time at depth (s)
    traj_freq = 200.0  # Hz

    trig_depth = 0.0250  # trigger depth (m)
    trig_plunge_time = 0.5  # trigger plunge duration (s)

    base_ori = R.from_euler(
        "xyz", [-180, 0, 0], degrees=True
    )  # base orientation ([roll, pitch, yaw], degrees)

    # Load probe locations from grid file
    PROJECT_ROOT = Path(__file__).resolve().parent  # or Path.cwd()
    grid_file = PROJECT_ROOT / "results" / "grids" / "grids.pkl"

    # Check if grid file exists
    if not grid_file.exists():
        raise FileNotFoundError(
            f"Grid file not found: {grid_file}. Please run grid_generator.py first."
        )

    with open(grid_file, "rb") as f:
        grids = pickle.load(f)

    # Let user select train or test set
    set_name = input("Select grid set (train/test) [default: test]: ").strip().lower()
    if set_name not in ["train", "test"]:
        set_name = "test"
        print(f"Invalid selection. Using default: {set_name}")

    # Get world frame grid (N, 2) array for robot motion
    grid_xy_world = grids["WORLD_FRAME"][set_name]  # (N, 2) array in world/robot frame
    
    # Get sensor frame grid (N, 2) array for data saving
    grid_xy_sensor = grids["SENSOR_FRAME"][set_name]  # (N, 2) array in sensor frame

    # Add z_surface to create (N, 3) probe locations (use world frame for robot motion)
    probe_locations = np.hstack([grid_xy_world, z_surface * np.ones((len(grid_xy_world), 1))])

    # Initialize results
    exp_dict = {
        "ts": [],
        "grid_positions": [],  # Store the (x, y) positions from the grid in SENSOR_FRAME
        "target_poses": [],
        "ee_poses": [],
        "ee_forces": [],
        "set_name": set_name,  # Record which set was used
    }

    # Move to start position
    print("Going to home...")
    # robot.move_to(position=home_position, speed=approach_speed)
    robot.set_target(position=home_position)
    print("  Waiting for robot to reach target...")
    time.sleep(3.0)  # Give time for trajectory to complete
    time.sleep(SETTLE_SEC)

    # Iterate over probe locations
    input("Press Enter to start probing...")
    for i, loc in enumerate(probe_locations):
        x, y, z = loc
        # Save sensor frame coordinates
        x_sensor, y_sensor = grid_xy_sensor[i]
        exp_dict["grid_positions"].append([x_sensor, y_sensor])
        print(
            f"\n=== Probe {i + 1}/{len(probe_locations)} at [{x:.4f}, {y:.4f}, {z:.4f}] ==="
        )

        # Maintain orientation at surface
        start_pose = robot.end_effector_pose.copy()
        start_pose.orientation = base_ori
        robot.set_target(pose=start_pose)
        time.sleep(SETTLE_SEC)

        # Press trigger
        print("\tPressing trigger...")
        plunge(
            robot,
            start_xyz=home_position,
            depth=trig_depth,
            plunge_time=trig_plunge_time,
            traj_freq=traj_freq,
            fixed_ori=base_ori,
        )  # do not record data here
        t_ref = time.perf_counter()  # reference time for this probe location

        input("\tPress Enter to approach probe location...")

        # Approach XY at home Z (safe height), then go to surface Z
        print("\tMoving to probe location...")
        approach_xy = np.array([x, y, home_position[2]], dtype=float)
        robot.move_to(position=approach_xy, speed=approach_speed)

        surface_xyz = np.array([x, y, z], dtype=float)
        robot.move_to(position=surface_xyz, speed=approach_speed)
        time.sleep(SETTLE_SEC)

        # Maintain orientation at surface
        start_pose = robot.end_effector_pose.copy()
        start_pose.orientation = base_ori
        robot.set_target(pose=start_pose)
        time.sleep(SETTLE_SEC)

        input("\tPress Enter to start plunge...")

        # Plunge: quarter-sine to final depth (velocity = 0 at end)
        print("\tStarting plunge...")
        t_plunge = time.perf_counter()
        ts, target_poses, ee_poses, ee_forces = plunge(
            robot,
            start_xyz=surface_xyz,
            depth=depth,
            plunge_time=plunge_time,
            traj_freq=traj_freq,
            fixed_ori=base_ori,
        )
        exp_dict["ts"].append(
            ts + (t_plunge - t_ref)
        )  # time referenced to button press (ts referenced to plunge start)
        exp_dict["target_poses"].append(target_poses)
        exp_dict["ee_poses"].append(ee_poses)
        exp_dict["ee_forces"].append(ee_forces)
        print("\tPlunge complete.")

        # Brief settle at depth
        time.sleep(plunge_rest)

        # Move back home (retract in Z, then move in XY)
        print("\tRetracting...")
        retract_xyz = surface_xyz.copy()
        robot.move_to(position=retract_xyz, speed=approach_speed)
        time.sleep(SETTLE_SEC)

        robot.move_to(position=home_position, speed=approach_speed)
        time.sleep(SETTLE_SEC)

    # Return home at the end
    print("\nReturning home...")
    robot.move_to(position=home_position, speed=approach_speed)
    time.sleep(SETTLE_SEC)

    robot.shutdown()
    print("Done.")

    # Save results
    results_dir = PROJECT_ROOT / "results"
    results_dir.mkdir(parents=True, exist_ok=True)

    # Generate filename based on set name and number of points
    filename = f"{len(probe_locations)}_grid_{set_name.upper()}.pkl"
    full_dir = results_dir / filename

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
