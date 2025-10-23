import time
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from arm_client import robot
from arm_client.robot import Robot
from arm_client import CONFIG_DIR
from pathlib import Path
import pickle


# Helper functions
def plunge(
    robot: Robot,
    start_xyz: np.ndarray,
    depth: float,
    plunge_time: float = 1.0,
    traj_freq: float = 100.0,
    fixed_ori: R = None,
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
        traj_freq: Hz publish rate during plunge.
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

    z0 = float(start_xyz[2])
    zf = z0 - float(depth)  # positive depth goes down
    N = max(1, int(plunge_time * traj_freq))
    dt = 1.0 / traj_freq
    t0 = time.perf_counter()

    # Initialize save arrays
    ee_forces = []
    ee_poses = []
    target_poses = []
    ts = []

    # Include the endpoint (k = 0..N)
    for k in range(N + 1):
        s = k / N  # 0..1
        z = z0 + (zf - z0) * np.sin(0.5 * np.pi * s)
        target_pose.position = np.array([start_xyz[0], start_xyz[1], z], dtype=float)
        target_pose.orientation = fixed_ori
        robot.set_target(pose=target_pose)

        # Record data
        ee_poses.append(robot.end_effector_pose.copy())
        ee_forces.append(robot.end_effector_wrench["force"].copy())
        target_poses.append(target_pose.copy())
        ts.append(time.perf_counter() - t0)

        # Pace the loop
        next_tick = t0 + (k + 1) * dt
        sleep_time = next_tick - time.perf_counter()
        if sleep_time > 0:
            time.sleep(sleep_time)

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

    robot.controller_switcher_client.switch_controller("joint_space_controller")
    robot.joint_space_controller_parameters_client.load_param_config(
        file_path=CONFIG_DIR / "controllers" / "joint_space" / "default.yaml"
    )

    # Parameters
    z_surface = 0.35  # (m)
    top_right_position = np.array([0.615, -0.0714, z_surface])  # top right dimple
    home_position = np.array([0.560, -0.0714, z_surface])  # button location
    approach_speed = 0.050  # (m/s)
    settle_sec = 0.25  # wait time (s)

    depth = 0.0500  # plunge depth (m)
    plunge_time = 1.0  # plunge duration (s)
    plunge_rest = 1.0  # rest time at depth (s)
    traj_freq = 200.0  # Hz

    trig_depth = 0.0200  # trigger depth (m)
    trig_plunge_time = 0.5  # trigger plunge duration (s)

    base_ori = R.from_euler("xyz", [-180, 0, 0], degrees=True)  # base orientation ([roll, pitch, yaw], degrees)

    # Probe locations: [x, y, z_surface], load from numpy files
    PROJECT_ROOT = Path(__file__).resolve().parent  # or Path.cwd()
    base_loc = PROJECT_ROOT / "grids"
    _grid = np.load(f"{base_loc}/train_grid.npy")

    # Offset all (x,y) values
    probe_locations = np.hstack([_grid, z_surface * np.ones((len(_grid), 1))])
    probe_locations[:, 0] = -probe_locations[:, 0] - 2.5 / 100 + top_right_position[0]  # x offset
    probe_locations[:, 1] = -probe_locations[:, 1] - 2.5 / 100 + top_right_position[1]  # y offset

    # Initialize results
    exp_dict = {
        "ts": [],
        "target_poses": [],
        "ee_poses": [],
        "ee_forces": [],
    }

    # Move to home position
    print("Going to home...")
    robot.move_to(position=home_position, speed=approach_speed)
    time.sleep(settle_sec)

    # Iterate over probe locations
    for i, loc in enumerate(probe_locations):
        x, y, z_surface = loc
        print(f"\n=== Probe {i + 1}/{len(probe_locations)} at [{x:.3f}, {y:.3f}, {z_surface:.3f}] ===")

        # Maintain orientation at surface
        start_pose = robot.end_effector_pose.copy()
        start_pose.orientation = base_ori
        robot.set_target(pose=start_pose)
        time.sleep(settle_sec)

        # Press trigger
        plunge(
            robot,
            start_xyz=home_position,
            depth=trig_depth,
            plunge_time=trig_plunge_time,
            traj_freq=traj_freq,
            fixed_ori=base_ori,
        )  # do not record data here

        # Approach XY at home Z (safe height), then go to surface Z
        approach_xy = np.array([x, y, home_position[2]], dtype=float)
        robot.move_to(position=approach_xy, speed=approach_speed)

        surface_xyz = np.array([x, y, z_surface], dtype=float)
        robot.move_to(position=surface_xyz, speed=approach_speed)
        time.sleep(settle_sec)

        # Maintain orientation at surface
        start_pose = robot.end_effector_pose.copy()
        start_pose.orientation = base_ori
        robot.set_target(pose=start_pose)
        time.sleep(settle_sec)

        # Plunge: quarter-sine to final depth (velocity = 0 at end)
        ts, target_poses, ee_poses, ee_forces = plunge(
            robot,
            start_xyz=surface_xyz,
            depth=depth,
            plunge_time=plunge_time,
            traj_freq=traj_freq,
            fixed_ori=base_ori,
        )
        exp_dict["ts"].append(ts)
        exp_dict["target_poses"].append(target_poses)
        exp_dict["ee_poses"].append(ee_poses)
        exp_dict["ee_forces"].append(ee_forces)

        # Brief settle at depth
        time.sleep(plunge_rest)

        # Move back home (retract in Z, then move in XY)
        retract_xyz = surface_xyz.copy()
        robot.move_to(position=retract_xyz, speed=approach_speed)
        time.sleep(0.1)

        robot.move_to(position=home_position, speed=approach_speed)
        time.sleep(0.1)

    # Return home at the end
    print("\nReturning home...")
    robot.move_to(position=home_position, speed=approach_speed)
    time.sleep(settle_sec)

    robot.shutdown()
    print("Done.")

    # Save results
    results_dir = PROJECT_ROOT / "results"
    results_dir.mkdir(parents=True, exist_ok=True)
    # Save dict using pickle
    with open(results_dir / "64_GRID_TRAIN.pkl", "wb") as f:
        pickle.dump(exp_dict, f)


if __name__ == "__main__":
    main()
