"""
Step response benchmark.

Commands a position step along each Cartesian axis and logs:
  - TCP position vs target (tracking error)
  - Orientation error vs home — the tool must keep pointing down
  - External wrench at the EE
  - Joint positions (check for nullspace drift during the step)

Results saved to results/step_response_<timestamp>.npz.
Each axis result is stored under keys "x", "y", "z".
"""

from __future__ import annotations

import sys
import time

import numpy as np
from arm_client.robot import Pose
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(__import__("pathlib").Path(__file__).parent))
from _utils import OSC_DEFAULT_CONFIG, OSC_RIM_CONFIG, collect, print_summary, save, setup_robot

STEP_M = 0.10  # step amplitude (m)
SETTLE_S = 2.0  # settle time at home before/after each step
RECORD_S = 3.0  # recording duration after the step command
RATE_HZ = 200.0


def main() -> None:
    robot, home_pose = setup_robot(OSC_RIM_CONFIG)
    ref_ori = home_pose.orientation
    results: dict = {
        "home_position": home_pose.position.copy(),
        "step_m": np.array(STEP_M),
    }

    for axis_name, axis_idx in [("x", 0), ("y", 1), ("z", 2)]:
        print(f"\n{'=' * 50}")
        print(f"Step along {axis_name.upper()}  ({STEP_M * 100:.0f} cm)")

        robot.set_target(pose=home_pose)
        time.sleep(SETTLE_S)

        target_pos = home_pose.position.copy()
        target_pos[axis_idx] += STEP_M
        step_pose = Pose(position=target_pos, orientation=ref_ori)
        robot.set_target(pose=step_pose)
        print(f"Target: {target_pos}")

        data = collect(robot, ref_ori, RECORD_S, RATE_HZ)
        data["target"] = target_pos

        # Tracking error along the step axis
        axis_err = np.abs(data["position"][:, axis_idx] - target_pos[axis_idx])
        ss_err = axis_err[-int(RATE_HZ * 0.5) :]  # last 0.5 s = steady state
        print(f"  Tracking error (axis) — peak: {axis_err.max() * 1e3:.1f}mm  SS: {ss_err.mean() * 1e3:.1f}mm")
        print_summary(f"{axis_name.upper()} step", data, home_pose.position)

        results[axis_name] = data

        robot.set_target(pose=home_pose)
        time.sleep(SETTLE_S)

    save(results, "step_response")
    robot.shutdown()


if __name__ == "__main__":
    main()
