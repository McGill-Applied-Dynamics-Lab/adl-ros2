"""Trajectory tracking benchmark: horizontal figure eight (or vertical plunge) from the start pose, orientation held.

The eight is a 2:1 Lissajous in the xy plane (as in examples/03_figure_eight.py): y = A_y sin(w t),
x = A_x sin(2 w t); --shape plunge goes down by --depth and back up in z. Time is warped with smooth ramps so the path starts and ends at rest. Targets
are streamed at --rate with Robot's streaming mode; the analytic velocity is sent on target_twist
unless --no-twist-ff. The robot is homed first (JTC) unless --no-home.

Metrics: position error vs the commanded target at the same instant (RMS / max / per axis), the
lag that best aligns measured and commanded paths, orientation error, torque chatter.

Usage:
    python tracking.py [--period 8] [--cycles 2] [--amp-x 0.08] [--amp-y 0.2] [--rate 500]
                       [--no-twist-ff] [--config default] [--set ...] [--no-home] [--tag T]
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from _bench import (  # noqa: E402
    Pose,
    Recorder,
    Twist,
    add_common_args,
    chatter_metrics,
    fmt,
    orientation_error_mrad,
    save,
    setup,
    teardown,
)


def figure_eight(args, center: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Time grid, positions (N x 3) and velocities (N x 3) of the warped figure eight."""
    dt = 1.0 / args.rate
    t_ramp = args.ramp
    duration = args.cycles * args.period + t_ramp  # each ramp integrates to t_ramp / 2
    t = np.arange(0.0, duration + dt, dt)

    def smoothstep(u):
        u = np.clip(u, 0.0, 1.0)
        return u * u * (3.0 - 2.0 * u)

    rate = np.minimum(smoothstep(t / t_ramp), smoothstep((duration - t) / t_ramp))  # d tau / dt
    tau = np.concatenate([[0.0], np.cumsum(0.5 * (rate[1:] + rate[:-1]) * dt)])
    w = 2.0 * np.pi / args.period
    pos = np.zeros((len(t), 3))
    vel = np.zeros((len(t), 3))
    if args.shape == "plunge":
        # Down by --depth and back up along z each period (examples/07b's trajectory)
        half = 0.5 * args.depth
        pos[:, 2] = -half * (1.0 - np.cos(w * tau))
        vel[:, 2] = -half * w * np.sin(w * tau) * rate
    else:
        pos[:, 0] = args.amp_x * np.sin(2.0 * w * tau)
        pos[:, 1] = args.amp_y * np.sin(w * tau)
        vel[:, 0] = args.amp_x * 2.0 * w * np.cos(2.0 * w * tau) * rate
        vel[:, 1] = args.amp_y * w * np.cos(w * tau) * rate
    return t, pos + center, vel


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--shape", choices=("eight", "plunge"), default="eight", help="Figure eight in xy, or a vertical plunge in z")
    parser.add_argument("--depth", type=float, default=0.2, help="Plunge depth (m), for --shape plunge")
    parser.add_argument("--period", type=float, default=8.0, help="Period of one eight / plunge (s)")
    parser.add_argument("--cycles", type=int, default=2, help="Number of eights / plunges")
    parser.add_argument("--amp-x", type=float, default=0.08, help="Half-width of the lobes along x (m)")
    parser.add_argument("--amp-y", type=float, default=0.2, help="Half-length of the eight along y (m)")
    parser.add_argument("--ramp", type=float, default=2.0, help="Speed ramp at start and end (s)")
    parser.add_argument("--rate", type=float, default=500.0, help="Target streaming rate (Hz)")
    parser.add_argument("--no-twist-ff", action="store_true", help="Do not stream the target velocity")
    add_common_args(parser)
    # Home by default: the eight has a fixed size, so it must start where it fits in the workspace
    parser.add_argument("--no-home", dest="home", action="store_false", help="Start from the current pose instead of homing")
    parser.set_defaults(home=True)
    args = parser.parse_args()

    robot, params = setup(args)
    rec = Recorder(robot)
    start = robot.end_effector_pose.copy()
    t_ref, p_ref, v_ref = figure_eight(args, start.position)
    print(
        f"{args.shape}: {args.cycles} x {args.period:.1f} s, peak speed {np.linalg.norm(v_ref, axis=1).max() * 1e3:.0f} mm/s, "
        f"x [{p_ref[:, 0].min():.3f}, {p_ref[:, 0].max():.3f}] y [{p_ref[:, 1].min():.3f}, {p_ref[:, 1].max():.3f}]"
    )

    robot.set_target(pose=start)
    time.sleep(1.0)

    zero = np.zeros(3)
    sent_t = np.zeros(len(t_ref))
    rec.start()
    robot.set_target_streaming(True)
    try:
        t0 = rec.mark("track")
        t0_perf = time.perf_counter()
        for i in range(len(t_ref)):
            delay = t0_perf + t_ref[i] - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            if i % 50 == 0:
                rec.check_live()  # abort (holding the last target) if the controller went away
            target = Pose(p_ref[i], start.orientation)
            twist = None if args.no_twist_ff else Twist(v_ref[i], zero.copy())
            robot.publish_target(pose=target, twist=twist)
            sent_t[i] = time.time()
        if not args.no_twist_ff:
            robot.publish_target(twist=Twist(zero.copy(), zero.copy()))
    finally:
        robot.set_target_streaming(False)  # resumes republishing the last streamed target (the center)
    time.sleep(1.0)
    t1 = rec.mark("end")
    rec.stop()
    data = rec.arrays()
    data.update(ref_t=sent_t, ref_pos=p_ref, ref_vel=v_ref)

    # Measured position at each instant a target was sent
    m = (data["ee_t"] >= t0) & (data["ee_t"] <= t1)
    ee_t, ee_pos = data["ee_t"][m], data["ee"][m, :3]
    meas = np.column_stack([np.interp(sent_t, ee_t, ee_pos[:, k]) for k in range(3)])
    err = meas - p_ref
    err_norm = np.linalg.norm(err, axis=1)

    # Lag: shift of the commanded path that best matches the measured one
    lags = np.arange(0, int(0.3 * args.rate))
    cost = [np.mean(np.linalg.norm(meas[lag:] - p_ref[: len(p_ref) - lag], axis=1)) for lag in lags]
    best = int(np.argmin(cost))
    jitter = np.diff(sent_t)

    rot = orientation_error_mrad(data["ee"][m, 3:7], start.orientation)
    metrics = {
        "peak_speed_ref_mm_s": np.linalg.norm(v_ref, axis=1).max() * 1e3,
        "twist_ff": not args.no_twist_ff,
        "err_rms_mm": np.sqrt((err_norm**2).mean()) * 1e3,
        "err_max_mm": err_norm.max() * 1e3,
        "err_rms_axis_mm": np.sqrt((err**2).mean(0)) * 1e3,
        "lag_ms": best / args.rate * 1e3,
        "err_rms_after_lag_mm": cost[best] * 1e3,
        "rot_err_rms_mrad": np.sqrt((rot**2).mean()),
        "rot_err_max_mrad": rot.max(),
        "stream_rate_hz": 1.0 / jitter.mean(),
        "stream_gap_max_ms": jitter.max() * 1e3,
        **chatter_metrics(data, t0, t1),
    }

    print(f"\n=== tracking ({args.shape}) ===")
    print(f"position error  RMS {metrics['err_rms_mm']:.2f} mm  max {metrics['err_max_mm']:.2f} mm  per axis RMS {fmt(metrics['err_rms_axis_mm'])} mm")
    print(f"lag {metrics['lag_ms']:.0f} ms (RMS after removing lag {metrics['err_rms_after_lag_mm']:.2f} mm)")
    print(f"orientation error RMS {metrics['rot_err_rms_mrad']:.1f} mrad  max {metrics['rot_err_max_mrad']:.1f} mrad")
    print(f"target stream {metrics['stream_rate_hz']:.0f} Hz, max gap {metrics['stream_gap_max_ms']:.1f} ms")
    print(f"chatter (>15 Hz RMS) tau_cmd {fmt(metrics['tau_cmd_rms_hf_Nm'], digits=3)} Nm  peak Hz {fmt(metrics['tau_cmd_peak_hz'], digits=0)}")
    print(f"                     dq      {fmt(metrics['dq_rms_hf_mrad_s'], digits=2)} mrad/s")
    print(f"torque rate limit: ticks at limit {fmt(metrics['tau_rate_limited_pct'], digits=2)} %  |dtau| p99 {fmt(metrics['dtau_p99_Nm'], digits=2)} Nm")
    save("tracking", args, params, data, metrics)
    teardown(robot)


if __name__ == "__main__":
    main()
