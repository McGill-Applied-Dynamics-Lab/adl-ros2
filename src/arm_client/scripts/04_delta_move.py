#!/usr/bin/env python3
"""
Execute a sequence of absolute XYZ targets from a CSV, one-by-one, using the FR3 robot.

Input CSV format (header recommended):
  episode,step,x,y,z,grip
- The 'grip' column is ignored.
- Rows are filtered by --episode and sorted by 'step'.

Example:
  python scripts/04_delta_move.py --csv absolute_actions.csv --episode 0

Optional:
  --speed sets cartesian interpolation speed for robot.move_to()
  --pos-tol sets position tolerance to consider the target reached
"""

import argparse
import csv
import time
from typing import List, Tuple

import numpy as np
import rclpy

from arm_client.robot import Robot

# ---------------------- Defaults ----------------------
NAMESPACE = "fr3"
CONTROLLER_NAME = "osc_pd_controller"

DEFAULT_SPEED = 0.05   # m/s used by Robot.move_to
POS_TOL = 1e-3         # meters
SETTLE_CHECKS = 5      # consecutive checks inside tolerance
CHECK_HZ = 50.0        # Hz
TIMEOUT_S = 5.0        # max time to wait per target


# ---------------------- Helpers ----------------------
def wait_until_reached(
    robot: Robot,
    target_xyz: np.ndarray,
    pos_tol: float = POS_TOL,
    settle_checks: int = SETTLE_CHECKS,
    check_hz: float = CHECK_HZ,
    timeout_s: float = TIMEOUT_S,
) -> bool:
    """Wait until the TCP is within pos_tol of target_xyz for settle_checks consecutive polls."""
    target_xyz = np.asarray(target_xyz, dtype=float)
    rate = robot.node.create_rate(check_hz)

    consec = 0
    elapsed = 0.0
    while rclpy.ok():
        cur = robot.end_effector_pose.position
        err = np.linalg.norm(cur - target_xyz)
        if err <= pos_tol:
            consec += 1
            if consec >= settle_checks:
                return True
        else:
            consec = 0

        rate.sleep()
        elapsed += 1.0 / check_hz
        if elapsed >= timeout_s:
            return False
    return False


def read_absolute_actions(csv_path: str, episode: int) -> List[Tuple[int, float, float, float]]:
    """
    Load actions for a given episode from CSV.
    Returns a list of (step, x, y, z), sorted by step.
    """
    actions = []
    with open(csv_path, newline="") as f:
        sniffer = csv.Sniffer()
        sample = f.read(2048)
        f.seek(0)
        has_header = sniffer.has_header(sample)
        if has_header:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    ep = int(row["episode"])
                except Exception:
                    continue
                if ep != episode:
                    continue
                step = int(row["step"])
                x = float(row["x"])
                y = float(row["y"])
                z = float(row["z"])
                actions.append((step, x, y, z))
        else:
            # No header: expect columns [episode, step, x, y, z, grip]
            reader = csv.reader(f)
            for raw in reader:
                if not raw or len(raw) < 5:
                    continue
                try:
                    ep = int(raw[0])
                    st = int(raw[1])
                    x = float(raw[2])
                    y = float(raw[3])
                    z = float(raw[4])
                except Exception:
                    continue
                if ep != episode:
                    continue
                actions.append((st, x, y, z))

    actions.sort(key=lambda r: r[0])
    return actions


# ---------------------- Main ----------------------
def main():
    ap = argparse.ArgumentParser(description="Execute absolute XYZ actions from CSV (gripper ignored).")
    ap.add_argument("--csv", required=True, help="Path to absolute actions CSV (episode,step,x,y,z,grip).")
    ap.add_argument("--episode", type=int, required=True, help="Episode ID to execute.")
    ap.add_argument("--namespace", default=NAMESPACE, help="ROS2 namespace for the robot (default: fr3).")
    ap.add_argument("--controller", default=CONTROLLER_NAME, help="Controller to switch to (default: osc_pd_controller).")
    ap.add_argument("--speed", type=float, default=DEFAULT_SPEED, help="Linear interpolation speed for move_to (m/s).")
    ap.add_argument("--pos-tol", type=float, default=POS_TOL, help="Position tolerance (m).")
    ap.add_argument("--settle-checks", type=int, default=SETTLE_CHECKS, help="Consecutive checks inside tolerance.")
    ap.add_argument("--check-hz", type=float, default=CHECK_HZ, help="Polling frequency (Hz).")
    ap.add_argument("--timeout", type=float, default=TIMEOUT_S, help="Timeout per target (s).")
    args = ap.parse_args()

    actions = read_absolute_actions(args.csv, args.episode)
    if not actions:
        print(f"No actions found in {args.csv} for episode {args.episode}.")
        return

    rclpy.init(args=None)
    robot = Robot(namespace=args.namespace, spin_node=True)

    try:
        robot.wait_until_ready(timeout=2.0)
        robot.controller_switcher_client.switch_controller(args.controller)

        for idx, (step, x, y, z) in enumerate(actions):
            target = np.array([x, y, z], dtype=float)
            print(f"[Episode {args.episode}] Step {step} (#{idx+1}/{len(actions)}): move_to {target} @ {args.speed} m/s")
            robot.move_to(position=target, speed=args.speed)

            reached = wait_until_reached(
                robot,
                target,
                pos_tol=args.pos_tol,
                settle_checks=args.settle_checks,
                check_hz=args.check_hz,
                timeout_s=args.timeout,
            )
            if not reached:
                print(f"Timeout waiting to reach {target} at step {step}. Stopping sequence.")
                break

            print(f"Reached step {step}.")
            time.sleep(0.1)  # small pause between moves

        print("Sequence complete.")
    finally:
        robot.shutdown()


if __name__ == "__main__":
    main()
