"""
Benchmark IK controller accuracy for plunge trajectories across a grid of
(depth, probe_time) combinations at 9 workspace positions.

For each plunge, three trajectories are recorded:
  - reference:  analytic cosine profile (ground truth)
  - fk:         forward kinematics of the planned joint trajectory (IK accuracy)
  - measured:   actual EE pose recorded during execution (total error)

Results are saved as a pickle for offline analysis in plunge_ik_analysis.ipynb.
Supports crash recovery: progress is checkpointed after every plunge.

Usage:
    pixi run python experiments/benchmarks/ik_controller/plunge_ik_benchmark.py
"""

import pickle
import time
import threading
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
from scipy.spatial.transform import Rotation as R

from arm_client.robot import Robot, Pose
from arm_client.planning.types import CartesianWaypoint
from arm_client.planning.ik_pyroki import compute_fk_trajectory

# ===================== Parameters =====================

# Plunge parameter grid
DEPTHS = np.linspace(0.01, 0.03, 5)  # [0.01, 0.015, 0.02, 0.025, 0.03] m
TIMES = np.linspace(1.0, 4.0, 5)  # [1.0, 1.75, 2.5, 3.25, 4.0] s

# Workspace grid (3x3, defined by bottom-left corner + extent)
# Adjust these to the actual area over the granular bed
GRID_NX = 3
GRID_NY = 3
WORKSPACE_X_MIN = 0.40  # m  — bottom-left corner x
WORKSPACE_Y_MIN = 0.226  # m  — bottom-left corner y
WORKSPACE_X_EXTENT = 0.30  # m  — total span in x
WORKSPACE_Y_EXTENT = 0.45  # m  — total span in y

# Robot motion
BASE_ORI = R.from_euler("xyz", [-180, 0, 0], degrees=True)
PROBE_START_Z = 0.1150  # m — approach height, same as probing script
Z_INIT = 0.15  # m — safe traversal height
MOVE_SPEED = 0.05  # m/s
SETTLE_SEC = 0.5  # s — wait after traversal before plunging

# IK weights (match probing script)
PLUNGE_SIMILARITY_WEIGHT = 0.1
PLUNGE_ORI_WEIGHT = 200.0
PLUNGE_POINTS_PER_SEC = 100

# Crash recovery
ALIVE_TIMEOUT_SEC = 1.0  # robot.is_alive() threshold
ALIVE_RETRIES = 10
ALIVE_WAIT_SEC = 2.0  # wait between is_alive() retries


# ===================== Helpers =====================


def _plan_plunge_traj(robot: Robot, target_xyz, depth, probe_time):
    z_init = float(target_xyz[2])
    n_waypoints = 50
    n_points = max(50, int(probe_time * PLUNGE_POINTS_PER_SEC))
    waypoints = [
        CartesianWaypoint(
            position=np.array(
                [target_xyz[0], target_xyz[1], z_init + depth / 2 * (np.cos(2 * np.pi * k / n_waypoints) - 1)],
                dtype=float,
            ),
            orientation=BASE_ORI,
            s=k / n_waypoints,
        )
        for k in range(n_waypoints + 1)
    ]
    return robot.plan_joint_trajectory(
        waypoints=waypoints,
        duration=probe_time,
        n_points=n_points,
        visualize=False,
        show_progress=False,
        initial_joint_config=robot.q.copy(),
        similarity_weight=PLUNGE_SIMILARITY_WEIGHT,
        ori_weight=PLUNGE_ORI_WEIGHT,
        pin_start=True,
    )


def _execute_plunge(robot: Robot, plunge_traj, probe_time):
    """Execute a plunge and record EE poses. Returns (ts, positions)."""
    ee_positions = []
    ts = []
    t0 = time.perf_counter()
    robot.follow_joint_trajectory(plunge_traj, blocking=False)
    while robot.wait_for_trajectory_completion(probe_time, timeout_margin=0.5):
        ee_positions.append(robot.end_effector_pose.position.copy())
        ts.append(time.perf_counter() - t0)
        time.sleep(0.01)
    return np.array(ts), np.array(ee_positions)


def _reference_z(target_z, depth, probe_time, t_arr):
    s = t_arr / probe_time
    return target_z + depth / 2 * (np.cos(2 * np.pi * s) - 1)


def _check_robot_alive(robot: Robot):
    for attempt in range(ALIVE_RETRIES):
        if robot.is_alive(ALIVE_TIMEOUT_SEC):
            return True
        print(f"  Robot unresponsive (attempt {attempt + 1}/{ALIVE_RETRIES}), waiting {ALIVE_WAIT_SEC:.0f}s...")
        time.sleep(ALIVE_WAIT_SEC)
    return False


# ===================== Checkpoint helpers =====================


def _find_latest_checkpoint(results_dir):
    checkpoints = sorted(results_dir.glob("*_checkpoint.pkl"))
    return checkpoints[-1] if checkpoints else None


def _save_checkpoint(data, path):
    tmp = path.with_suffix(".tmp")
    with open(tmp, "wb") as f:
        pickle.dump(data, f)
    tmp.replace(path)  # atomic replace


def _load_or_create_run(results_dir):
    checkpoint_path = _find_latest_checkpoint(results_dir)
    if checkpoint_path is not None:
        with open(checkpoint_path, "rb") as f:
            data = pickle.load(f)
        n_done = len(data["completed"])
        n_total = GRID_NX * GRID_NY * len(DEPTHS) * len(TIMES)
        ans = input(f"Found checkpoint '{checkpoint_path.name}' ({n_done}/{n_total} complete). Resume? [y/n]: ")
        if ans.strip().lower() == "y":
            print(f"Resuming from checkpoint.")
            return data, checkpoint_path

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    new_checkpoint = results_dir / f"plunge_benchmark_{timestamp}_checkpoint.pkl"
    data = {
        "timestamp": timestamp,
        "depths": DEPTHS,
        "times": TIMES,
        "positions": None,  # filled in main()
        "completed": set(),
        "results": [],
    }
    return data, new_checkpoint


# ===================== Main =====================


def main():
    results_dir = Path(__file__).resolve().parent / "results"
    results_dir.mkdir(exist_ok=True)

    run_data, checkpoint_path = _load_or_create_run(results_dir)

    # Build 3x3 workspace grid
    xs = np.linspace(WORKSPACE_X_MIN, WORKSPACE_X_MIN + WORKSPACE_X_EXTENT, GRID_NX)
    ys = np.linspace(WORKSPACE_Y_MIN, WORKSPACE_Y_MIN - WORKSPACE_Y_EXTENT, GRID_NY)
    positions = [(float(x), float(y)) for y in ys for x in xs]  # row-major, 9 pts
    run_data["positions"] = positions

    n_total = len(positions) * len(DEPTHS) * len(TIMES)
    print(f"Benchmark: {len(positions)} positions × {len(DEPTHS)} depths × {len(TIMES)} times = {n_total} plunges")

    # rclpy.init()
    robot = Robot(namespace="fr3")
    robot.wait_until_ready()
    home_pose = robot.end_effector_pose.copy()

    # input("Press Enter to start...")

    try:
        for pos_idx, (x, y) in enumerate(positions):
            approach_xyz = np.array([x, y, PROBE_START_Z])

            remaining = [
                (di, ti)
                for di in range(len(DEPTHS))
                for ti in range(len(TIMES))
                if (pos_idx, di, ti) not in run_data["completed"]
            ]
            if not remaining:
                print(f"\nPosition {pos_idx + 1}/{len(positions)} already complete, skipping.")
                continue

            print(f"\n[{pos_idx + 1}/{len(positions)}] Moving to ({x:.3f}, {y:.3f}, {PROBE_START_Z:.4f})")
            robot.move_to(
                pose=Pose(approach_xyz, BASE_ORI),
                speed=MOVE_SPEED,
            )
            time.sleep(SETTLE_SEC)

            for depth_idx, depth in enumerate(DEPTHS):
                for time_idx, probe_time in enumerate(TIMES):
                    if (pos_idx, depth_idx, time_idx) in run_data["completed"]:
                        continue

                    label = (
                        f"  depth={depth * 1e3:.0f}mm  "
                        f"time={probe_time:.2f}s  "
                        f"[{len(run_data['completed']) + 1}/{n_total}]"
                    )
                    print(label, end="  ", flush=True)

                    if not _check_robot_alive(robot):
                        print("\nRobot unresponsive after retries. Saving checkpoint and exiting.")
                        _save_checkpoint(run_data, checkpoint_path)
                        return

                    # Plan
                    plunge_traj = _plan_plunge_traj(robot, approach_xyz, depth, probe_time)

                    # FK on planned joint positions
                    fk_positions = compute_fk_trajectory(
                        plunge_traj.joint_positions,
                        robot.config.ik_target_link_name,
                    )

                    # Execute
                    ts, measured_positions = _execute_plunge(robot, plunge_traj, probe_time)

                    run_data["results"].append(
                        {
                            "pos_idx": pos_idx,
                            "depth_idx": depth_idx,
                            "time_idx": time_idx,
                            "target_xyz": approach_xyz.copy(),
                            "depth": depth,
                            "probe_time": probe_time,
                            "planned_time_from_start": np.array(plunge_traj.time_from_start),
                            "planned_joint_positions": plunge_traj.joint_positions.copy(),
                            "fk_positions": fk_positions,
                            "measured_positions": measured_positions,
                            "ts": ts,
                        }
                    )
                    run_data["completed"].add((pos_idx, depth_idx, time_idx))
                    _save_checkpoint(run_data, checkpoint_path)
                    print("done")

    finally:
        print("\nReturning home...")
        robot.move_to(pose=home_pose, speed=MOVE_SPEED)
        robot.shutdown()

    # Save final results and remove checkpoint
    final_path = results_dir / f"plunge_benchmark_{run_data['timestamp']}.pkl"
    with open(final_path, "wb") as f:
        pickle.dump(run_data, f)
    checkpoint_path.unlink(missing_ok=True)
    print(f"\nResults saved to: {final_path}")
    print("Benchmark complete.")


if __name__ == "__main__":
    main()
