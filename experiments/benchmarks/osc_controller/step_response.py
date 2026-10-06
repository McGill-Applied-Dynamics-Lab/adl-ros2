"""Step response benchmark: position steps out and back along each base axis, orientation held.

Per step: rise time (10-90 %), overshoot, settling time (to within --settle-band of the final
value), steady-state error, off-axis deviation, orientation error, peak speed, torque chatter.

Usage:
    python step_response.py [--amplitude 0.03] [--axes xyz] [--hold 2.5] [--config default]
                            [--set gains.k_pos_x=1000 ...] [--home] [--tag T]
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from _bench import (  # noqa: E402
    Recorder,
    add_common_args,
    chatter_metrics,
    orientation_error_mrad,
    rate_limit_metrics,
    save,
    setup,
    teardown,
)

AXES = {"x": 0, "y": 1, "z": 2}
SETTLED_AFTER_S = 1.0  # chatter after this delay from the step command is reported as "settled_*"
TRANSIENT_S = 0.3  # window after the step command reported as "transient_*"


def step_metrics(data: dict, t0: float, t1: float, p_from: np.ndarray, p_to: np.ndarray, ref_rot, band: float) -> dict:
    m = (data["ee_t"] >= t0) & (data["ee_t"] < t1)
    tt = data["ee_t"][m] - t0
    pos, quat, vel = data["ee"][m, :3], data["ee"][m, 3:7], data["ee"][m, 7:10]
    length = np.linalg.norm(p_to - p_from)
    axis = (p_to - p_from) / length
    s = (pos - p_from) @ axis
    off = pos - p_from - np.outer(s, axis)

    def first_time(cond):
        return tt[np.argmax(cond)] if cond.any() else np.nan

    s0 = s[0]
    rise = first_time(s >= s0 + 0.9 * (length - s0)) - first_time(s >= s0 + 0.1 * (length - s0))
    final = s[tt > tt[-1] - 0.5].mean()
    outside = np.abs(s - final) > band
    settle = tt[np.where(outside)[0][-1]] if outside.any() else 0.0
    return {
        "rise_ms": rise * 1e3,
        "overshoot_mm": (s.max() - length) * 1e3 if s.max() > length else 0.0,
        "settle_ms": settle * 1e3,
        "ss_error_mm": (length - final) * 1e3,
        "start_offset_mm": s0 * 1e3,
        "off_axis_max_mm": np.linalg.norm(off, axis=1).max() * 1e3,
        "rot_err_max_mrad": orientation_error_mrad(quat, ref_rot).max(),
        "peak_speed_mm_s": np.linalg.norm(vel, axis=1).max() * 1e3,
        **chatter_metrics(data, t0, t1),
        # After the transient: a sustained oscillation (limit cycle) shows here, the step itself does not
        **{f"settled_{k}": v for k, v in chatter_metrics(data, t0 + SETTLED_AFTER_S, t1).items()},
        # Rate-limit saturation right after the target jump: what triggers the limit cycle
        **{f"transient_{k}": v for k, v in rate_limit_metrics(data, (data["tau_t"] >= t0) & (data["tau_t"] < t0 + TRANSIENT_S)).items()},
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--amplitude", type=float, default=0.03, help="Step size (m), max 0.05 (controller error clamp)")
    parser.add_argument("--axes", default="xyz", help="Axes to step along, e.g. 'xyz' or 'y'")
    parser.add_argument("--hold", type=float, default=2.5, help="Time recorded after each step command (s)")
    parser.add_argument("--settle-band", type=float, default=0.001, help="Settling band around the final value (m)")
    add_common_args(parser)
    args = parser.parse_args()
    if not 0.0 < args.amplitude <= 0.1:
        parser.error("--amplitude must be in (0, 0.05] m: larger steps saturate limits.max_position_error")

    robot, params = setup(args)
    rec = Recorder(robot)
    start = robot.end_effector_pose.copy()
    robot.set_target(pose=start)
    time.sleep(1.0)

    steps = []  # (label, t0, t1, p_from, p_to)
    rec.start()
    for name in args.axes:
        out = start.copy()
        out.position = start.position.copy()
        out.position[AXES[name]] += args.amplitude
        for label, p_from, target in ((f"{name}+", start.position, out), (f"{name}-", out.position, start)):
            rec.check_live()  # stop commanding steps if the controller went away
            robot.set_target(pose=target)
            t0 = rec.mark(label)
            time.sleep(args.hold)
            steps.append((label, t0, time.time(), p_from.copy(), target.position.copy()))
    rec.stop()
    data = rec.arrays()
    data.update(
        step_label=np.array([s[0] for s in steps]),
        step_t0=np.array([s[1] for s in steps]),
        step_t1=np.array([s[2] for s in steps]),
        step_from=np.array([s[3] for s in steps]),
        step_to=np.array([s[4] for s in steps]),
    )

    metrics: dict = {"amplitude_m": args.amplitude, "steps": {}}
    print(f"\n=== step response, {args.amplitude * 1e3:.0f} mm ===")
    print(
        f"{'step':5s} {'rise ms':>8s} {'overshoot':>9s} {'settle ms':>9s} {'ss err':>7s} {'off-axis':>8s} {'rot max':>8s} {'v max':>7s} {'tau_hf max':>10s} "
        f"{'lim tr':>6s} {'settled':>8s} {'lim st':>6s} {'peak':>5s}"
    )
    print(f"{'':5s} {'':>8s} {'mm':>9s} {'':>9s} {'mm':>7s} {'mm':>8s} {'mrad':>8s} {'mm/s':>7s} {'Nm':>10s} {'%':>6s} {'Nm':>8s} {'%':>6s} {'Hz':>5s}")
    for label, t0, t1, p_from, p_to in steps:
        sm = step_metrics(data, t0, t1, p_from, p_to, start.orientation, args.settle_band)
        metrics["steps"][label] = sm
        worst = int(np.nanargmax(sm["settled_tau_cmd_rms_hf_Nm"]))
        print(
            f"{label:5s} {sm['rise_ms']:8.0f} {sm['overshoot_mm']:9.2f} {sm['settle_ms']:9.0f} {sm['ss_error_mm']:7.2f} "
            f"{sm['off_axis_max_mm']:8.2f} {sm['rot_err_max_mrad']:8.1f} {sm['peak_speed_mm_s']:7.0f} "
            f"{sm['tau_cmd_rms_hf_max_Nm']:10.3f} {sm['transient_tau_rate_limited_pct_max']:6.1f} "
            f"{sm['settled_tau_cmd_rms_hf_max_Nm']:8.3f} {sm['settled_tau_rate_limited_pct_max']:6.1f} {sm['settled_tau_cmd_peak_hz'][worst]:5.0f}"
        )

    all_steps = list(metrics["steps"].values())
    for key in (
        "rise_ms",
        "overshoot_mm",
        "settle_ms",
        "ss_error_mm",
        "off_axis_max_mm",
        "rot_err_max_mrad",
        "tau_cmd_rms_hf_max_Nm",
        "dq_rms_hf_max_mrad_s",
        "settled_tau_cmd_rms_hf_max_Nm",
        "settled_dq_rms_hf_max_mrad_s",
        "transient_tau_rate_limited_pct_max",
        "settled_tau_rate_limited_pct_max",
    ):
        vals = np.array([s[key] for s in all_steps], dtype=float)
        metrics[f"{key}_mean"] = float(np.nanmean(np.abs(vals)))
        metrics[f"{key}_max"] = float(np.nanmax(np.abs(vals)))
    print(
        f"mean |ss err| {metrics['ss_error_mm_mean']:.2f} mm, max overshoot {metrics['overshoot_mm_max']:.2f} mm, "
        f"mean rise {metrics['rise_ms_mean']:.0f} ms, max rot err {metrics['rot_err_max_mrad_max']:.1f} mrad, "
        f"max tau chatter {metrics['tau_cmd_rms_hf_max_Nm_max']:.3f} Nm (settled {metrics['settled_tau_cmd_rms_hf_max_Nm_max']:.3f} Nm)"
    )
    save("step_response", args, params, data, metrics)
    teardown(robot)


if __name__ == "__main__":
    main()
