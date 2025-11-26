#!/usr/bin/env python3
"""
Execute a sequence of absolute XYZ targets (plus *hardware* gripper) from a CSV using the FR3 robot.
- Fixed EEF orientation: Euler XYZ = (-180°, 0°, 0°)  (Y axis remains "not moving")
- Blended waypoint streaming + stuck detection
- Gripper is controlled by arm_client.gripper.franka_hand.Gripper

CSV columns (header required):
  episode,step,picker_id,x,y,z,grip_raw,grip_closed,source
  - grip_closed: 0=open, 1=closed

Coordinate handling:
  - Swap CSV Y/Z
  - Recenter so that (386.5, -20.7, -22.5) -> (0,0,0)

Example:
  python3 run_actions_with_hw_gripper.py --csv /path/to/file.csv --episode 0 \
      --blend-dist 0.02 --stuck-window 5 --stuck-threshold 0.005 \
      --open-width 0.08 --closed-width 0.0 --gripper-speed 0.1 --gripper-force 50
"""

import argparse
import csv
import time
from arm_client import CONFIG_DIR
from typing import List, Tuple, Optional
from collections import deque

import numpy as np
import rclpy
from scipy.spatial.transform import Rotation

from arm_client.robot import Robot, Pose
from arm_client.gripper.franka_hand import Gripper, GripperConfig

# ---------------------- Constants ----------------------
NAMESPACE = "fr3"
CONTROLLER_NAME = "osc_pd_controller"

DEFAULT_SPEED = 0.05
POS_TOL = 1e-3
CHECK_HZ = 100.0
TIMEOUT_S = 5.0

# Coordinate adjustments (meters)
REF_POINT = np.array([0.4865, -0.0207, 0.225], dtype=float)  # (386.5mm, -20.7mm, -22.5mm)
# REF_POINT = np.array([0.4865, -0.0207, 0.4225], dtype=float)  # (386.5mm, -20.7mm, -22.5mm)


# ---------------------- Helpers ----------------------
def wait_until_reached(
    robot: Robot,
    target_xyz: np.ndarray,
    pos_tol: float = POS_TOL,
    check_hz: float = CHECK_HZ,
    timeout_s: float = TIMEOUT_S,
) -> bool:
    """Wait until TCP is within pos_tol; print live distance."""
    target_xyz = np.asarray(target_xyz, dtype=float)
    rate = robot.node.create_rate(check_hz)

    elapsed = 0.0
    print(f"--- Monitoring convergence to {target_xyz} ---")
    while rclpy.ok():
        cur = robot.end_effector_pose.position
        err = np.linalg.norm(cur - target_xyz)
        print(f"[track] distance to target: {err:.5f} m", end="\r")

        # Immediately return once within tolerance
        if err <= pos_tol:
            print(f"\n[reach] final distance = {err:.5f} m (tol {pos_tol:.5f})")
            return True

        rate.sleep()
        elapsed += 1.0 / check_hz
        if elapsed >= timeout_s:
            print(f"\n[timeout] distance = {err:.5f} m (tol {pos_tol:.5f})")
            return False
    return False


def _to_bool01(val) -> int:
    if val is None:
        return 0
    if isinstance(val, (int, float)):
        return int(val != 0)
    s = str(val).strip().lower()
    return 1 if s in ("1", "true", "t", "yes", "closed", "close") else 0


def read_actions_with_transform(
    csv_path: str,
    episode: int,
    picker_id: Optional[int] = None
) -> List[Tuple[int, float, float, float, int]]:
    """
    Apply Y<->Z swap and affine:
      X = 0.5*x + REF.x
      Y = 0.5*z + REF.z   (swapped)
      Z = 0.5*y + REF.y   (swapped)
    Returns list of (step, X, Y, Z, grip_closed), sorted by step.
    """
    actions: List[Tuple[int, float, float, float, int]] = []
    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                ep = int(row["episode"])
                if ep != episode:
                    continue
                st = int(row["step"])
                pid = int(row.get("picker_id", "0"))
                if picker_id is not None and pid != picker_id:
                    continue
                x = float(row["x"]); y = float(row["y"]); z = float(row["z"])
                grip_closed = _to_bool01(row.get("grip_closed"))
            except Exception:
                continue

            X = (0.5 * x) + REF_POINT[0]
            Y = (0.5 * z) + REF_POINT[1]  # swapped
            Z = (0.5 * y) + REF_POINT[2]  # swapped
            actions.append((st, X, Y, Z, grip_closed))

    actions.sort(key=lambda r: r[0])
    return actions


def _command_gripper(gripper: Gripper, closed: int, open_width: float, closed_width: float) -> bool:
    """
    Send a blocking width command using the available API.
    Preference order:
      - .set_target(width)
      - .open() / .close()
    """
    try:
        if closed:
            # If set_target is available, use precise closed width; else close()
            if hasattr(gripper, "set_target"):
                ok = gripper.set_target(float(closed_width))
            else:
                ok = gripper.close()
            print(f"[gripper] CLOSE -> width target {closed_width:.3f} m | {'ok' if ok else 'fail'}")
            return ok
        else:
            if hasattr(gripper, "set_target"):
                ok = gripper.set_target(float(open_width))
            else:
                ok = gripper.open()
            print(f"[gripper] OPEN  -> width target {open_width:.3f} m | {'ok' if ok else 'fail'}")
            return ok
    except Exception as e:
        print(f"[gripper] command error: {e}")
        return False


# ---------------------- Main ----------------------
def main():
    ap = argparse.ArgumentParser(description="Execute XYZ actions with FIXED EEF orientation and hardware gripper.")
    ap.add_argument("--csv", required=True, help="Path to actions file.")
    ap.add_argument("--episode", type=int, required=True, help="Episode ID to execute.")
    ap.add_argument("--picker-id", type=int, default=None, help="Optional picker_id filter.")
    ap.add_argument("--namespace", default=NAMESPACE, help="ROS2 namespace (default: fr3).")
    ap.add_argument("--controller", default=CONTROLLER_NAME, help="Controller name (default: osc_pd_controller).")
    ap.add_argument("--speed", type=float, default=DEFAULT_SPEED, help="Cartesian move speed (m/s).")

    # Blending controls
    ap.add_argument("--blend-dist", type=float, default=0.02,
                    help="Distance (m) at which to trigger the next waypoint (blending).")
    ap.add_argument("--final-wait", dest="final_wait", action="store_true", default=True,
                    help="Wait to fully reach the last target (default: on).")
    ap.add_argument("--no-final-wait", dest="final_wait", action="store_false",
                    help="Do not wait to settle on the final target.")

    # Stuck detection
    ap.add_argument("--stuck-window", type=int, default=5,
                    help="Consecutive loop ticks to consider for stuck detection.")
    ap.add_argument("--stuck-threshold", type=float, default=0.005,
                    help="Per-tick movement (m) below which the robot is considered not moving.")

    # Gripper behavior
    ap.add_argument("--open-width", type=float, default=0.08, help="Target width when open (m).")
    ap.add_argument("--closed-width", type=float, default=0.00, help="Target width when closed (m).")
    ap.add_argument("--gripper-speed", type=float, default=0.10, help="Default gripper speed (m/s).")
    ap.add_argument("--gripper-force", type=float, default=50.0, help="Default gripper force (N).")

    args = ap.parse_args()

    actions = read_actions_with_transform(args.csv, args.episode, picker_id=args.picker_id)
    if not actions:
        print(f"No actions found in {args.csv} for episode {args.episode}.")
        return

    rclpy.init(args=None)

    # Robot
    robot = Robot(namespace=args.namespace, spin_node=True)
    # Gripper
    gripper_cfg = GripperConfig(
        max_width=max(args.open_width, args.closed_width),
        min_width=min(args.open_width, args.closed_width),
        default_speed=args.gripper_speed,
        default_force=args.gripper_force,
    )
    gripper = Gripper(namespace=args.namespace, gripper_config=gripper_cfg)

    try:
        robot.wait_until_ready(timeout=2.0)
        gripper.wait_until_ready()
        robot.controller_switcher_client.switch_controller(args.controller)
        robot.osc_pd_controller_parameters_client.load_param_config(
            file_path=CONFIG_DIR / "controllers" / "osc_pd" / "default.yaml"
        )

        # Fixed orientation: Euler XYZ = (-180°, 0°, 0°)
        fixed_rot = Rotation.from_euler('xyz', [-270.0, 0.0, 0.0], degrees=True)
        print("[ori] using FIXED Euler XYZ = (-180°, 0°, 0°)")

        # Seed initial grip state
        idx = 0
        step0, X0, Y0, Z0, grip0 = actions[idx]
        cur_xyz = np.array([X0, Y0, Z0], dtype=float)

        # Move to first waypoint (pos + fixed orientation)
        print(f"[Ep {args.episode}] Step {step0} -> move_to Pose(pos={cur_xyz}, fixed Euler)")
        robot.move_to(pose=Pose(cur_xyz, fixed_rot), speed=args.speed)

        # Command gripper to initial state
        print("Priming gripper to initial state...")
        _command_gripper(gripper, grip0, args.open_width, args.closed_width)
        last_grip = grip0

        rate = robot.node.create_rate(CHECK_HZ)
        total = len(actions)

        # Stuck detection window
        recent_pos = deque(maxlen=max(2, args.stuck_window))
        recent_pos.append(robot.end_effector_pose.position)

        while rclpy.ok():
            # If last scheduled, optionally wait and finish
            if idx >= total - 1:
                if args.final_wait:
                    _ = wait_until_reached(
                        robot, cur_xyz, pos_tol=POS_TOL,
                        check_hz=CHECK_HZ, timeout_s=TIMEOUT_S
                    )
                print("Sequence complete.")
                break

            # Current distance to active target (for live feedback)
            ee_xyz = robot.end_effector_pose.position
            dist = np.linalg.norm(ee_xyz - cur_xyz)
            print(f"[blend] step={actions[idx][0]} dist={dist:.5f} m", end="\r")

            # Update stuck window
            recent_pos.append(ee_xyz)
            stuck = False
            if len(recent_pos) >= args.stuck_window:
                deltas = [np.linalg.norm(recent_pos[i] - recent_pos[i-1]) for i in range(1, len(recent_pos))]
                if all(d < args.stuck_threshold for d in deltas):
                    stuck = True

            # Advance when within blend radius OR robot appears stuck
            if dist <= args.blend_dist or stuck:
                if stuck:
                    print(f"\n[stuck] movement over last {len(recent_pos)-1} ticks "
                          f"< {args.stuck_threshold:.4f} m -> advancing.")

                idx += 1
                step_i, X, Y, Z, grip_closed = actions[idx]
                toggled = (grip_closed != last_grip)

                # Move to next waypoint first (pos + fixed orientation)
                cur_xyz = np.array([X, Y, Z], dtype=float)
                print(f"\n[Ep {args.episode}] Step {step_i} (#{idx+1}/{total}) -> move_to Pose(pos={cur_xyz}, fixed Euler)")
                robot.move_to(pose=Pose(cur_xyz, fixed_rot), speed=args.speed)

                # If gripper toggled at this waypoint, send the hardware command
                if toggled:
                    _command_gripper(gripper, grip_closed, args.open_width, args.closed_width)
                    last_grip = grip_closed

                # Reset stuck window after commanding a new target
                recent_pos.clear()
                recent_pos.append(robot.end_effector_pose.position)

            rate.sleep()

    finally:
        robot.shutdown()


if __name__ == "__main__":
    main()
