import time
import numpy as np
from scipy.spatial.transform import Rotation as R
from arm_client.robot import Robot, Pose, Twist
from arm_client import CONFIG_DIR
from pathlib import Path
import pickle
from datetime import datetime
import subprocess

SETTLE_SEC = 1.25  # wait time after moves (s)
TRAJ_FREQ = 10.0 # Hz

# Helper functions
def probe(
    robot: Robot,
    start_xyz: np.ndarray,
    depth: float,
    probe_time: float,
    traj_freq: float = 5.0,
    fixed_ori: R | None = None,
    plunge_func: str = 'cos'
):
    """
    Complete plunge and retract motion.

    Args:
        robot: Robot client (already ready & in Cartesian control).
        start_xyz: np.array([x, y, z_surface]) start point of plunge.
        depth: probe depth (positive indicates downwards).
        probe_time: seconds to complete the probe cycle (plunge + retract).
        traj_freq: frequency of trajectory points per second.
        plunge_func: plunge function (cosine and linear plunge have been implemented)
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

    # Include the endpoint (k = 0..N)
    for k in range(N + 1):
        s = k / N  # 0..1
        if plunge_func == 'cos': # cosine-based plunge function
            z = z_init + depth / 2 * (np.cos(2 * np.pi * s) - 1)
        elif plunge_func == 'linear': # linear
            z = z_init + depth * (np.abs(2*s - 1) - 1)
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

    t_min = 0.0
    z_min = z_init
    while robot.wait_for_trajectory_completion(probe_time, timeout_margin=0.5):
        ee_force = robot.end_effector_wrench["force"]
        ee_pose = robot.end_effector_pose
        time_stamp = time.perf_counter() # absolute time

        # Time-stamp when z-displacement is max
        if ee_pose.position[2] < z_min:
            z_min = ee_pose.position[2]
            t_min = time_stamp # absolute time

        # Record data
        ee_poses.append(ee_pose.copy())
        ee_forces.append(ee_force.copy())
        ts.append(time_stamp)

        # Must have sleep
        time.sleep(0.01)

    return ts, ee_poses, ee_forces, t_min

# Trigger settings
Z_INIT = 0.15 # m
BUTTON_X = 0.681 # m
BUTTON_Y = -0.147 # m
TRIG_DEPTH = 0.0206 # m (previously 0.0250 m)
TRIG_TIME = 2.0 # s

# Orientation/starting position
BASE_ORI = R.from_euler("xyz", [-180, 0, 0], degrees=True)
PROBE_START_Z = 0.10799 # start height for probing at the surface of the sensor (m) - originally 0.112 m

# ***Add depths***
DEPTH_INC = 5e-3  # depth increment (m)
DEPTHS = np.arange(1e-2, 3e-2 + DEPTH_INC, DEPTH_INC) # plunge depths
## DEPTHS = np.array([0.01])
# ***Add times***
TIME_INC = 0.5  # time increment (s)
PLUNGE_TIMES = np.arange(1, 4 + TIME_INC, TIME_INC)  # plunge times (s)
## LUNGE_TIMES = np.array([2.0, 3.0])

def main():
    # Setup
    PROJECT_ROOT = Path(__file__).resolve().parent 
    robot = Robot(namespace="fr3")
    robot.wait_until_ready()

    robot.controller_switcher_client.switch_controller("fr3_pose_controller")
    robot.fr3_pose_controller_parameters_client.load_param_config(
        file_path=CONFIG_DIR / "controllers" / "fr3_pose" / "probing.yaml"
    )

    # Set home position/pose
    home_position = np.array([BUTTON_X, BUTTON_Y, Z_INIT])  # button location at initial z-height
    home_pose = Pose(home_position, BASE_ORI)

    # Run grid generator (eliminate for extended run/incremental saving)
    """
    subprocess.run(["python3", PROJECT_ROOT / "grid_generator.py"]) # run grid generator, ensure params are correct
    """

    # Load probe locations from grid file
    grid_file = PROJECT_ROOT / "results" / "grids" / "MEGA_GRID.pkl" # update if required

    # Check if grid file exists
    if not grid_file.exists():
        raise FileNotFoundError(
            f"Grid file not found: {grid_file}. Please run a grid generator first."
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

    # Initialize results
    exp_dict = {
        "ts": [],
        "grid_positions": [],  # Store the (x, y) positions from the grid in SENSOR_FRAME
        "ee_poses": [],
        "ee_forces": [],
        "plunge_depths": [], # plunge depth from surface (z = 0)
        "plunge_times": [], # plunge times
        "set_name": set_name,  # Record which set was used
    }

    # Open/check if pickle file exists
    results_dir = PROJECT_ROOT / "results"
    results_dir.mkdir(parents=True, exist_ok=True)
    filename = f"R_MEGA_GRID.pkl"  # results filename
    full_path = results_dir / filename

    start_idx = 0 # default start index
    if not full_path.exists():
        with open(full_path, "wb") as f:
            pickle.dump(exp_dict, f)
            print(f"Created new results file: {full_path}")
    else: # open pickle file
        with open(full_path, "rb") as f:
            exp_dict = pickle.load(f)
            print(f"Loaded existing results file: {full_path}")
            # Check current index
            start_idx = len(exp_dict["grid_positions"])

    # Compute starting indices
    H = len(DEPTHS)
    J = len(PLUNGE_TIMES)
    K = len(grid_xy_world)

    si = start_idx // (J*K)
    r = start_idx %  (J*K)
    sj = r // K
    sk = r %  K
    print(f"Resuming from depth i={si}, time j={sj}, location k={sk}")

    input("Press Enter to start probing...")
    # Iterate over depths, plunge times, and locations
    for i in range(si, H):
        depth = DEPTHS[i]
        j0 = sj if i == si else 0

        for j in range(j0, J):
            plunge_time = PLUNGE_TIMES[j]
            k0 = sk if (i == si and j == sj) else 0

            for k in range(k0, K):
                x, y = grid_xy_world[k]
                x_sensor, y_sensor = grid_xy_sensor[k]

                print(f"\n Probe {i*(J*K) + j*K + k + 1} / {H*J*K}")

                # --- Travel home ---
                robot.set_target(pose=home_pose)
                time.sleep(SETTLE_SEC)

                # --- Press trigger ---
                print("\tPressing trigger...")
                _, _, _, t_ref = probe(
                    robot,
                    start_xyz=home_position,
                    depth=TRIG_DEPTH,
                    probe_time=TRIG_TIME,
                    traj_freq=TRAJ_FREQ,
                    fixed_ori=BASE_ORI,
                    plunge_func='cos',
                )

                # --- Move to probe location ---
                print("\tMoving to probe location...")
                approach_xyz = np.array([x, y, Z_INIT], dtype=float)
                robot.set_target(position=approach_xyz)
                time.sleep(SETTLE_SEC)

                # Descend to probe start height
                print("\tDescending to probe start height...")
                approach_xyz = np.array([x, y, PROBE_START_Z], dtype=float)
                robot.set_target(position=approach_xyz)
                time.sleep(SETTLE_SEC)

                # --- Probe cycle ---
                print("\tStarting probe...")
                ts, ee_poses, ee_forces, _ = probe(
                    robot,
                    start_xyz=approach_xyz,
                    depth=depth, # new
                    probe_time=plunge_time, # new
                    traj_freq=TRAJ_FREQ,
                    fixed_ori=BASE_ORI,
                )

                # Retract in z
                print("\tRetracting in z...")
                retract_position = np.array([x, y, Z_INIT], dtype=float)
                robot.set_target(position=retract_position)
                time.sleep(SETTLE_SEC)
                print("\tPlunge complete.")

                # Convert Pose objects to numpy arrays
                ee_positions = [pose.position for pose in ee_poses]
                ee_orientations = [pose.orientation.as_quat() for pose in ee_poses]

                # --- Store results ---
                if np.any(np.asarray(ee_positions)[:,2] <= (PROBE_START_Z - depth + 2.5e-3)): # allow small tolerance
                    ts_adjusted = [t - t_ref for t in ts] # time referenced to trigger
                    exp_dict["ts"].append(ts_adjusted)
                    # Store numpy arrays instead of Pose objects
                    exp_dict["ee_poses"].append({
                        "positions": ee_positions,
                        "orientations": ee_orientations
                    })
                    exp_dict["ee_forces"].append(ee_forces)  # already numpy arrays
                    exp_dict["plunge_depths"].append(depth)
                    exp_dict["plunge_times"].append(plunge_time)
                    exp_dict["grid_positions"].append([x_sensor, y_sensor])

                    # Save data
                    with open(full_path, "wb") as f:
                        pickle.dump(exp_dict, f)
                        print(f"Results saved to: {full_path}")
                else:
                    raise(ValueError(f"Plunge depth not achieved. Check robot operation."))

    # Return home at the end
    print("\nReturning home...")
    robot.set_target(pose=home_pose)
    time.sleep(SETTLE_SEC)

    robot.shutdown()
    print("Done.")


if __name__ == "__main__":
    main()
