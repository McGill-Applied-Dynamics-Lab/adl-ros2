import time
import numpy as np
from scipy.spatial.transform import Rotation as R
from arm_client.robot import Robot, Pose
from arm_client import CONFIG_DIR
from pathlib import Path
import pickle
from waveguide_gripper_grid_generator import fetch_landmarks

SETTLE_SEC = 0.250  # wait time after moves (s)

# Helper functions
def plunge(
    device: Robot,
    start_xyz: np.ndarray,
    depth: float,
    plunge_time: float = 1.0,
    traj_freq: float = 100.0,
    fixed_ori: R = None,
):
    """
    Quarter-sine plunge from start_xyz to final depth.
    - s in [0,1] -> z(s) = z0 + (zf - z0) * sin(pi/2 * s)
    - Ends at phase 90 deg. (velocity = 0).

    Args:
        device: Robot client (already ready & in Cartesian control).
        start_xyz: np.array([x, y, z_surface]) start point of plunge.
        depth: positive = move down along -Z (final z = z_surface - depth).
        plunge_time: seconds to complete the plunge.
        traj_freq: Hz publish rate during plunge.
    """
    # Fix starting orientation and make a working target pose
    cur = device.end_effector_pose.copy()  # Pose(position, orientation)
    if fixed_ori is None:
        fixed_ori = cur.orientation  # keep orientation constant during plunge
    target_pose = cur.copy()
    target_pose.position = start_xyz.astype(float)

    # Ensure we start exactly at the commanded start pose
    device.set_target(pose=target_pose)
    time.sleep(SETTLE_SEC)

    z0 = float(start_xyz[2])
    zf = z0 - float(depth)  # positive depth goes down
    N = max(1, int(plunge_time * traj_freq))
    dt = 1.0 / traj_freq

    # Initialize save arrays
    ee_forces = []
    ee_poses = []
    target_poses = []
    ts = []

    # Include the endpoint (k = 0..N)
    t0 = time.perf_counter() # start counter
    for k in range(N + 1):
        s = k / N  # 0..1
        z = z0 + (zf - z0) * np.sin(0.5 * np.pi * s)
        target_pose.position = np.array([start_xyz[0], start_xyz[1], z], dtype=float)
        target_pose.orientation = fixed_ori
        device.set_target(pose=target_pose)

        # Record data
        ee_poses.append(device.end_effector_pose.copy())
        ee_forces.append(device.end_effector_wrench["force"].copy())
        target_poses.append(target_pose.copy())
        ts.append(time.perf_counter() - t0)

        # Pace the loop
        next_tick = t0 + (k + 1) * dt
        sleep_time = next_tick - time.perf_counter()
        if sleep_time > 0:
            time.sleep(sleep_time)

    return ts, target_poses, ee_poses, ee_forces


def main():
    # Franka setup
    franka = Robot(namespace="fr3")
    franka.wait_until_ready()

    franka.controller_switcher_client.switch_controller("osc_pd_controller")
    franka.osc_pd_controller_parameters_client.load_param_config(
        file_path=CONFIG_DIR / "controllers" / "osc_pd" / "probe_controller.yaml"
    )

    # Load landmark file
    PROJECT_ROOT = Path(__file__).resolve().parent  # or Path.cwd()
    landmark_file = PROJECT_ROOT / 'results' / 'grids' / "waveguide_gripper_landmarks.txt"
    landmarks = fetch_landmarks(landmark_file, ['x', 'y', 'z'])

    # Define probing parameters
    z_offset = 0.0250 # (m) offset from landmark z to surface
    z_surface = landmarks['z'] + z_offset  # (m)
    home_position = np.array([landmarks['x'], landmarks['y'], z_surface])  # home location (m)
    retraction_sec = 5.0 # time to execute retractions (s)
    set_name = 'train'

    depth = z_offset + 0.0050  # plunge depth (m)
    plunge_time = 1.0  # plunge duration (s)
    traj_freq = 200.0  # (Hz)

    base_ori = R.from_euler("xyz", [-270, 0, 0], degrees=True)  # base orientation ([roll, pitch, yaw], degrees)
    base_pose = Pose(position=home_position, orientation=base_ori)

    # Probe locations: [x, y, z_surface], load from numpy files
    grid_loc = PROJECT_ROOT / "results" / "grids" / 'GRIPPER_GRID.pkl'
    with open(grid_loc, "rb") as f:
        grid_dict = pickle.load(f)
    grid = grid_dict[f"{set_name}_world_frame"]
    probe_locations = np.hstack([grid, z_surface * np.ones((len(grid), 1))]) # append z_surface to make (N, 3) arrays

    ts_list = []
    target_poses_list = []
    ee_poses_list = []
    ee_forces_list = []

    # Move to home position
    print("Going to home...")
    franka.move_to(pose=base_pose, time_to_move=retraction_sec)
    time.sleep(SETTLE_SEC)

    # Iterate over probe locations
    for i, loc in enumerate(probe_locations):
        (x, y, z_surface) = loc

        print(f"\n=== Probe {i + 1}/{len(probe_locations)} at [{x - landmarks['x']:.3f}, {y - landmarks['y']:.3f}, {z_surface - landmarks['z']:.3f}] ===")

        # Move to probe location
        surface_xyz = np.array([x, y, z_surface], dtype=float)
        franka.move_to(pose=base_pose, time_to_move=retraction_sec)
        time.sleep(SETTLE_SEC)

        # Plunge: quarter-sine to final depth (velocity = 0 at end)
        ts, target_poses, ee_poses, ee_forces = plunge(
            device=franka,
            start_xyz=surface_xyz,
            depth=depth,
            plunge_time=plunge_time,
            traj_freq=traj_freq,
            fixed_ori=base_ori,
        )
        ts_list.append(ts)  # time stamps
        target_poses_list.append(target_poses)
        ee_poses_list.append(ee_poses)
        ee_forces_list.append(ee_forces)

        # Move back home (retract in Z, then move in XY)
        retract_xyz = surface_xyz.copy()
        franka.move_to(position=retract_xyz, time_to_move=retraction_sec)
        time.sleep(SETTLE_SEC)

    # Return home at the end
    print("\nReturning home...")
    franka.move_to(pose=base_pose, time_to_move=retraction_sec)
    time.sleep(SETTLE_SEC)

    franka.shutdown()
    print("Done.")

    # Convert to numpy safe
    ts_list = np.asarray(ts_list)
    target_positions_list = np.asarray([[pose.position for pose in trial] for trial in target_poses_list])
    target_orientations_list = np.asarray([[pose.orientation.as_quat() for pose in trial] for trial in target_poses_list])
    ee_positions_list = np.array([[pose.position for pose in trial] for trial in ee_poses_list])
    ee_orientations_list = np.array([[pose.orientation.as_quat() for pose in trial] for trial in ee_poses_list])
    ee_forces_list = np.asarray(ee_forces_list)

    # Create experiment dict
    exp_dict = {}
    exp_dict["ts"] = ts_list
    exp_dict['target_positions'] = target_positions_list
    exp_dict['target_orientations'] = target_orientations_list
    exp_dict['ee_positions'] = ee_positions_list
    exp_dict['ee_orientations'] = ee_orientations_list
    exp_dict['ee_forces'] = ee_forces_list
    exp_dict['probe_locations'] = grid[f"{set_name}_gripper_frame"]  # (N, 2) array of (x, y) probe locations in world frame


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
