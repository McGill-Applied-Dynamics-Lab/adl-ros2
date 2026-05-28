"""
Stiffness characterization benchmark.

Holds the robot at its home pose while the user manually applies force
along each axis. Logs external wrench and TCP position error continuously.
At equilibrium the virtual coupling gives: F_ext ≈ -K * δx, so the slope
of force vs displacement estimates K for each axis.

Usage:
  1. Run the script.
  2. When prompted, push along the indicated axis, hold for a few seconds,
     then release. Repeat for each axis.
  3. Results and estimated stiffness are printed and saved.

Also checks orientation error throughout — pushing should not perturb orientation.

Results saved to results/stiffness_<timestamp>.npz.
"""
from __future__ import annotations

import sys
import time

import numpy as np

sys.path.insert(0, str(__import__("pathlib").Path(__file__).parent))
from _utils import OSC_DEFAULT_CONFIG, collect, save, setup_robot

PUSH_S = 15.0       # recording duration while user applies force
SETTLE_S = 3.0      # settle time between axes
RATE_HZ = 200.0
AXES = [("x", 0), ("y", 1), ("z", 2)]


def estimate_stiffness(positions: np.ndarray, forces: np.ndarray, axis_idx: int) -> float:
    """Least-squares slope of F_axis vs δx_axis. F_ext ≈ -K * δx → K ≈ -slope."""
    delta_x = positions[:, axis_idx] - positions[:, axis_idx].mean()
    f_axis = forces[:, axis_idx]
    # Only use samples where force is non-trivial (> 0.5 N) to avoid noise bias
    mask = np.abs(f_axis) > 0.5
    if mask.sum() < 10:
        return float("nan")
    slope = np.polyfit(delta_x[mask], f_axis[mask], 1)[0]
    return -float(slope)  # sign: external force opposes displacement


def main() -> None:
    robot, home_pose = setup_robot(OSC_DEFAULT_CONFIG)
    ref_ori = home_pose.orientation
    results: dict = {"home_position": home_pose.position.copy()}

    for axis_name, axis_idx in AXES:
        print(f"\n{'='*50}")
        print(f"Axis: {axis_name.upper()}")
        print(f"  Hold the EE and push firmly along {axis_name.upper()}, then release.")
        input("  Press Enter when ready to start recording...")

        robot.set_target(pose=home_pose)
        data = collect(robot, ref_ori, PUSH_S, RATE_HZ)

        k_est = estimate_stiffness(data["position"], data["force"], axis_idx)
        pos_err = np.abs(data["position"][:, axis_idx] - home_pose.position[axis_idx])
        ori_deg = np.degrees(data["ori_err_rad"])
        f_peak = np.abs(data["force"][:, axis_idx]).max()

        print(f"\n  Peak force: {f_peak:.1f} N")
        print(f"  Peak displacement: {pos_err.max()*1e3:.1f} mm")
        print(f"  Estimated K_{axis_name}: {k_est:.0f} N/m")
        print(f"  Orientation error — mean: {ori_deg.mean():.2f}°  max: {ori_deg.max():.2f}°")

        results[axis_name] = {**data, "k_estimated": np.array(k_est)}

        robot.set_target(pose=home_pose)
        time.sleep(SETTLE_S)

    save(results, "stiffness")
    robot.shutdown()


if __name__ == "__main__":
    main()
