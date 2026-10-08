"""Hold benchmark: osc_controller holds its current pose, no motion commanded.

Measures the static error, drift, and torque chatter (what you hear when gains are too high).

Usage:
    python hold.py [--duration 10] [--config default] [--set gains.k_pos_x=1000 ...] [--home] [--tag T]
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).parent))
from _bench import (  # noqa: E402
    Recorder,
    add_common_args,
    chatter_metrics,
    fmt,
    orientation_error_mrad,
    save,
    setup,
    teardown,
)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--duration", type=float, default=10.0, help="Hold duration (s)")
    add_common_args(parser)
    args = parser.parse_args()

    robot, params = setup(args)
    rec = Recorder(robot)
    start = robot.end_effector_pose.copy()
    robot.set_target(pose=start)
    time.sleep(0.5)

    rec.start()
    t0 = rec.mark("hold")
    time.sleep(args.duration)
    t1 = rec.mark("end")
    rec.stop()
    data = rec.arrays()

    err = data["err"]
    pos = data["ee"][:, :3]
    metrics = {
        "duration_s": args.duration,
        "err_pos_mean_mm": np.linalg.norm(err[:, :3], axis=1).mean() * 1e3,
        "err_pos_max_mm": np.linalg.norm(err[:, :3], axis=1).max() * 1e3,
        "err_rot_mean_mrad": np.linalg.norm(err[:, 3:], axis=1).mean() * 1e3,
        "err_rot_max_mrad": np.linalg.norm(err[:, 3:], axis=1).max() * 1e3,
        "drift_mm": np.linalg.norm(pos[-1] - pos[0]) * 1e3,
        "pos_std_mm": pos.std(0) * 1e3,
        "rot_from_start_max_mrad": orientation_error_mrad(data["ee"][:, 3:7], Rotation.from_quat(data["ee"][0, 3:7])).max(),
        **chatter_metrics(data, t0, t1),
    }

    print(f"\n=== hold {args.duration:.0f} s ===")
    print(
        f"task error  pos mean {metrics['err_pos_mean_mm']:.2f} mm (max {metrics['err_pos_max_mm']:.2f})  "
        f"rot mean {metrics['err_rot_mean_mrad']:.1f} mrad (max {metrics['err_rot_max_mrad']:.1f})"
    )
    print(f"drift {metrics['drift_mm']:.2f} mm   pos std {fmt(metrics['pos_std_mm'], digits=3)} mm")
    print(f"chatter (>15 Hz RMS) tau_cmd {fmt(metrics['tau_cmd_rms_hf_Nm'], digits=3)} Nm  peak Hz {fmt(metrics['tau_cmd_peak_hz'], digits=0)}")
    print(f"                     dq      {fmt(metrics['dq_rms_hf_mrad_s'], digits=2)} mrad/s  peak Hz {fmt(metrics['dq_peak_hz'], digits=0)}")
    print(f"                     tau_J   {fmt(metrics['tau_J_rms_hf_Nm'], digits=3)} Nm")
    save("hold", args, params, data, metrics)
    teardown(robot)


if __name__ == "__main__":
    main()
