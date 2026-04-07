#!/usr/bin/env python3
import time
import pickle
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R

from arm_client import CONFIG_DIR
from arm_client.robot import Pose, Robot, Twist

from waveguide_gripper_grid_generator import fetch_landmarks


SETTLE_SEC = 1.00  # wait time after moves (s)
TRAJ_FREQ = 10.0   # Hz

# Probing parameters
Z_OFFSET = 0.0250
PROBE_DEPTH = 0.0200
PROBE_TIME = 2.0
BASE_ORI = R.from_euler("xyz", [-270, 0, 0], degrees=True)


def probe(
    robot: Robot,
    start_xyz: np.ndarray,
    depth: float,
    probe_time: float,
    traj_freq: float = 5.0,
    fixed_ori: R | None = None,
    probe_func: str = "cos",
    t0_perf: float | None = None,
):
    cur = robot.end_effector_pose.copy()
    if fixed_ori is None:
        fixed_ori = cur.orientation

    target_pose = cur.copy()
    target_pose.position = start_xyz.astype(float)

    robot.set_target(pose=target_pose)
    time.sleep(SETTLE_SEC)

    z_init = float(start_xyz[2])
    num_steps = max(1, int(probe_time * traj_freq))
    dt = 1.0 / traj_freq

    waypoints = []
    time_from_start = []
    for k in range(num_steps + 1):
        s = k / num_steps
        if probe_func == "cos":
            z = z_init + depth / 2 * (np.cos(2 * np.pi * s) - 1)
        elif probe_func == "linear":
            z = z_init + depth * (np.abs(2 * s - 1) - 1)
        else:
            raise ValueError(f"Unsupported probe_func: {probe_func}")

        target_position = np.array([start_xyz[0], start_xyz[1], z], dtype=float)
        target_pose = Pose(target_position, fixed_ori)
        twist = Twist(np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0]))

        waypoints.append((target_pose, twist))
        time_from_start.append(k * dt)

    ee_forces = []
    ee_poses = []
    ts = []

    robot.execute_trajectory(waypoints, time_from_start)

    if t0_perf is None:
        t0_perf = time.perf_counter()
    t0 = t0_perf

    t_min = 0.0
    z_min = z_init
    while robot.wait_for_trajectory_completion(probe_time, timeout_margin=0.5):
        ee_force = robot.end_effector_wrench["force"]
        ee_pose = robot.end_effector_pose
        time_stamp = time.perf_counter() - t0

        if ee_pose.position[2] < z_min:
            z_min = ee_pose.position[2]
            t_min = time.perf_counter()

        ee_poses.append(ee_pose.copy())
        ee_forces.append(ee_force.copy())
        ts.append(time_stamp)

        time.sleep(0.01)

    return ts, ee_poses, ee_forces, t_min


def main():
    robot = Robot(namespace="fr3")
    robot.wait_until_ready()

    robot.controller_switcher_client.switch_controller("fr3_pose_controller")
    robot.fr3_pose_controller_parameters_client.load_param_config(
        file_path=CONFIG_DIR / "controllers" / "fr3_pose" / "default.yaml"
    )

    project_root = Path(__file__).resolve().parent

    landmark_file = project_root / "results" / "grids" / "landmarks.txt"
    if not landmark_file.exists():
        raise FileNotFoundError(
            f"Landmark file not found: {landmark_file}. Please run the landmark detection script first."
        )

    landmarks = fetch_landmarks(landmark_file, ["x", "y", "z"])

    z_surface = landmarks["z"] + Z_OFFSET
    home_position = np.array([landmarks["x"], landmarks["y"], z_surface])
    home_pose = Pose(home_position, BASE_ORI)

    grid_file = project_root / "results" / "grids" / "grids.pkl"
    if not grid_file.exists():
        raise FileNotFoundError(
            f"Grid file not found: {grid_file}. Please run grid_generator.py first."
        )

    with open(grid_file, "rb") as f:
        grids = pickle.load(f)

    set_name = input("Select grid set (train/test) [default: test]: ").strip().lower()
    if set_name not in ["train", "test"]:
        set_name = "test"
        print(f"Invalid selection. Using default: {set_name}")

    grid_xy_world = grids["WORLD_FRAME"][set_name]
    grid_xy_gripper = grids["GRIPPER_FRAME"][set_name]

    exp_dict = {
        "ts": [],
        "grid_positions": [],
        "ee_poses": [],
        "ee_forces": [],
        "set_name": set_name,
        "landmarks": landmarks,
        "z_offset": Z_OFFSET,
    }

    try:
        robot.set_target(pose=home_pose)
        time.sleep(SETTLE_SEC)

        input("Press Enter to start probing...")
        for i, loc in enumerate(grid_xy_world):
            x, y = loc
            x_gripper, y_gripper = grid_xy_gripper[i]
            exp_dict["grid_positions"].append([x_gripper, y_gripper])
            print(f"\n Probe {i + 1}/{len(grid_xy_world)}")

            print("\tMoving to probe location...")
            approach_xy = np.array([x, y, z_surface], dtype=float)
            robot.set_target(position=approach_xy)
            time.sleep(SETTLE_SEC)

            print("\tStarting probe...")
            t0_perf = time.perf_counter()
            ts, ee_poses, ee_forces, _ = probe(
                robot,
                start_xyz=approach_xy,
                depth=Z_OFFSET + PROBE_DEPTH,
                probe_time=PROBE_TIME,
                traj_freq=TRAJ_FREQ,
                fixed_ori=BASE_ORI,
                probe_func="linear",
                t0_perf=t0_perf,
            )

            ee_positions = [pose.position for pose in ee_poses]
            ee_orientations = [pose.orientation.as_quat() for pose in ee_poses]

            exp_dict["ts"].append(ts)
            exp_dict["ee_poses"].append(
                {"positions": ee_positions, "orientations": ee_orientations}
            )
            exp_dict["ee_forces"].append(ee_forces)
            print("\tProbe complete.")

        print("\nReturning home...")
        robot.set_target(pose=home_pose)
        time.sleep(SETTLE_SEC)
    finally:
        robot.shutdown()

    print("Done.")

    results_dir = project_root / "results"
    results_dir.mkdir(parents=True, exist_ok=True)

    base_filename = f"{len(grid_xy_world)}_grid_{set_name.upper()}_NO_RF"
    counter = 0
    filename = f"{base_filename}_{counter:02d}.pkl"
    full_path = results_dir / filename
    while full_path.exists():
        counter += 1
        filename = f"{base_filename}_{counter:02d}.pkl"
        full_path = results_dir / filename

    with open(full_path, "wb") as f:
        pickle.dump(exp_dict, f)
    print(f"Results saved to: {full_path}")


if __name__ == "__main__":
    main()
