"""
Nullspace drift benchmark.

Holds a fixed Cartesian target for HOLD_S seconds and monitors joint
angles over time. Runs two back-to-back trials:

  Trial A — no nullspace damping  (nullspace.damping = 0 via default.yaml)
  Trial B — with nullspace damping (nullspace.damping = 5.0 via rim_controller.yaml)

Joint drift is the RMS change in q relative to the initial configuration.
TCP position and orientation are logged to confirm the task-space target is
maintained regardless of nullspace behaviour.

Results saved to results/nullspace_drift_<timestamp>.npz.
"""
from __future__ import annotations

import sys
import time

import numpy as np

sys.path.insert(0, str(__import__("pathlib").Path(__file__).parent))
from _utils import OSC_DEFAULT_CONFIG, OSC_RIM_CONFIG, collect, save, setup_robot

HOLD_S = 60.0
RATE_HZ = 50.0      # lower rate is fine — drift is slow


def run_trial(label: str, config_path, hold_s: float, rate_hz: float) -> dict:
    print(f"\n{'='*50}")
    print(f"Trial: {label}  (hold {hold_s:.0f} s)")
    robot, home_pose = setup_robot(config_path)
    robot.set_target(pose=home_pose)
    time.sleep(2.0)

    q0 = robot.q.copy()
    print(f"Initial q: {np.round(np.degrees(q0), 1)} deg")

    data = collect(robot, home_pose.orientation, hold_s, rate_hz)

    # Drift metrics
    q_drift = data["q"] - q0  # (N, 7)
    drift_rms = np.sqrt((q_drift**2).mean(axis=0))
    drift_max = np.abs(q_drift).max(axis=0)
    print(f"  Joint drift RMS (deg): {np.round(np.degrees(drift_rms), 2)}")
    print(f"  Joint drift max (deg): {np.round(np.degrees(drift_max), 2)}")

    # Task-space stability
    pos_err = np.linalg.norm(data["position"] - home_pose.position, axis=1)
    ori_deg = np.degrees(data["ori_err_rad"])
    print(f"  TCP position error — mean: {pos_err.mean()*1e3:.2f}mm  max: {pos_err.max()*1e3:.2f}mm")
    print(f"  Orientation error  — mean: {ori_deg.mean():.2f}°  max: {ori_deg.max():.2f}°")

    robot.set_target(pose=home_pose)
    time.sleep(1.0)
    robot.shutdown()

    return {**data, "q0": q0, "home_position": home_pose.position.copy()}


def main() -> None:
    trial_a = run_trial("No nullspace damping (damping=0)", OSC_DEFAULT_CONFIG, HOLD_S, RATE_HZ)
    trial_b = run_trial("Nullspace damping=5.0", OSC_RIM_CONFIG, HOLD_S, RATE_HZ)

    # Flatten for savez: prefix keys by trial
    data = {}
    for key, val in trial_a.items():
        data[f"no_damping_{key}"] = val
    for key, val in trial_b.items():
        data[f"damping_{key}"] = val
    data["hold_s"] = np.array(HOLD_S)

    save(data, "nullspace_drift")


if __name__ == "__main__":
    main()
