r"""T1 — OSC response: does ``osc_controller`` realize the gains it is given, and what is the arm behind it?

Drives the OSC target directly (no haptics, no operator): on each axis in turn, a step out and back,
then sines at a few frequencies. Then reports, per axis:

- **gains**: least squares of the controller's task force on the tracking error,
  $F \approx \hat K (x_t - x) + \hat D (v_t - v)$, against the set $K_r, D_r$;
- **arm**: least squares of the same force on the motion it produces (free space),
  $F \approx \hat\Lambda a + \hat b v + \hat F_c \tanh(v / v_0)$ — inertia, viscous and Coulomb friction;
- **steps**: rise time (10-90 %), overshoot, steady-state error (friction shows up here, as $F_c / K_r$);
- **sines**: amplitude ratio and phase lag per frequency (closed-loop bandwidth).

These are the closed-loop parameters RIM's closed-loop variant (M6) models.

    pixi run -e humble python experiments/rim-delays/pretests/t1_osc_response.py --sim
    pixi run -e humble python experiments/rim-delays/pretests/t1_osc_response.py --kv 1000 --dv 70
    pixi run -e humble python experiments/rim-delays/pretests/t1_osc_response.py --preset   # osc_teleop.yaml gains

The robot starts at ``--start-position`` (joint trajectory), then the OSC holds orientation and
posture while the target moves. Ctrl-C stops early (the data so far is still analysed).
"""

from __future__ import annotations

import argparse
import datetime
import json
import sys
import time
from pathlib import Path

import numpy as np
import pandas as pd

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
REPO = HERE.parents[2]
import matplotlib.pyplot as plt  # noqa: E402
from runs import INK, INK_2, INK_3, save  # noqa: E402

AXES = "xyz"
COLS = ("#2a78d6", "#eb6834", "#1baf7a")  # x, y, z
OUT = Path("~/data/franka/rim-delays").expanduser()


# ------------------------------------------------------------------ the target profile


def build_profile(step: float, amp: float, freqs: list[float], cycles: int, rest: float, axes: str):
    """``[(axis, kind, t0, t1, param)]`` segments and the total duration [s].

    ``kind`` is ``rest``, ``step`` (``param`` = offset held over the segment) or ``sine``
    (``param`` = frequency [Hz]; amplitude ``amp``, starting and ending at 0 with zero velocity)."""
    segs, t = [], 0.0
    for a in axes:
        i = AXES.index(a)
        for kind, param, dur in (("rest", 0.0, rest), ("step", step, 2.0), ("step", 0.0, 2.0),
                                 ("step", -step, 2.0), ("step", 0.0, 2.0)):
            segs.append((i, kind, t, t + dur, param))
            t += dur
        for f in freqs:
            dur = cycles / f
            segs.append((i, "sine", t, t + dur, f))
            t += dur
            segs.append((i, "rest", t, t + rest, 0.0))
            t += rest
    return segs, t


def target_at(segs, t: float, amp: float) -> tuple[np.ndarray, np.ndarray, int, str]:
    """Offset and velocity ``(3,)`` of the target at ``t``, the active axis, the segment kind."""
    off, vel = np.zeros(3), np.zeros(3)
    for i, kind, t0, t1, p in segs:
        if t0 <= t < t1:
            if kind == "step":
                off[i] = p
            elif kind == "sine":
                # whole cycles: starts and ends at 0 (the velocity jumps there, as at a step)
                w = 2 * np.pi * p
                off[i] = amp * np.sin(w * (t - t0))
                vel[i] = amp * w * np.cos(w * (t - t0))
            return off, vel, i, kind
    return off, vel, -1, "done"


# ------------------------------------------------------------------ backends


class SimBackend:
    """`sim_plant.SimFR3` with the same gains on every axis (or the teleop preset)."""

    def __init__(self, args) -> None:
        from pyrim import InterfaceFrame
        from sim_plant import PRESET_K, SimConfig, SimFR3, osc_gains

        class _Sink:
            def write(self, *a, **k):
                pass

        coupling = None if args.preset else (args.kv, args.dv)
        cfg = SimConfig(start=tuple(args.start_position), table_z=-1.0, friction=0.0)
        self.gains = osc_gains("xyz", coupling, cfg.mass)
        self.sim = SimFR3(cfg, InterfaceFrame(np.eye(3)), self.gains, sink=_Sink(), sample_hz=1000.0)
        self.start = np.asarray(args.start_position, dtype=float)
        self.preset_k = PRESET_K

    def begin(self) -> np.ndarray:
        self.sim.start()
        return self.start.copy()

    def aim(self, x, v) -> None:
        self.sim.aim(x, v)

    def end(self) -> pd.DataFrame:
        self.sim.close()
        from sim_plant import TRUTH_COLS

        d = pd.DataFrame(self.sim.truth, columns=TRUTH_COLS)
        return pd.DataFrame({"t": d.t, **{a: d[f"x_{a}"] for a in AXES}, **{f"v{a}": d[f"v_{a}"] for a in AXES},
                             **{f"f{a}": d[f"f_task_{a}"] for a in AXES}})


class RobotBackend:
    """The real FR3 under ``osc_controller``, set up in ``fr3_teleop``'s safe order."""

    def __init__(self, args) -> None:
        import rclpy
        from arm_client.control.parameters_client import ParametersClient
        from arm_client.robot import Robot
        from fr3_haptic.plant import FR3Plant
        from fr3_haptic.teleop import move_to_start
        from pyrim import InterfaceFrame
        from rclpy.signals import SignalHandlerOptions

        if not rclpy.ok():
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.robot = Robot(namespace=args.namespace)
        self.robot.wait_until_ready()
        move_to_start(self.robot, args.start_position, args.start_speed)
        self.robot.set_target_streaming(True)  # before the switch: no stale target reaches the controller
        self.plant = FR3Plant(self.robot, InterfaceFrame(np.eye(3)), record=True)
        self.robot.controller_switcher_client.switch_controller(args.controller)
        params = ParametersClient(self.robot.node, target_node=args.controller)
        params.wait_until_ready()
        params.load_param_config(str(REPO / args.osc_params))
        gains = [("control.inertia_decoupling", False), ("feedforward.twist", True), ("feedforward.wrench", False),
                 ("feedforward.accel", False)]
        if not args.preset:
            for a in AXES:
                gains += [(f"gains.k_pos_{a}", float(args.kv)), (f"gains.d_pos_{a}", float(args.dv))]
        params.set_parameters(gains)
        self.params = params
        self.gains_set = gains
        first = self.plant.wait_for_sample()
        self.plant.set_hold_pose(first.ee_pose)
        reach = args.step + args.amp + 0.02
        self.plant.set_interface_limits((first.x_i - reach, first.x_i + reach))
        self.start = first.x_i.copy()

    def begin(self) -> np.ndarray:
        return self.start.copy()

    def aim(self, x, v) -> None:
        self.plant.aim(x, v)

    def end(self) -> pd.DataFrame:
        try:
            self.plant.freeze()
        finally:
            hist = list(self.plant.history)
            self.plant.close()
            self.robot.shutdown()
        rows = [(s.rx_s, *s.ee_position, *s.ee_velocity, *s.task_force) for s in hist]
        return pd.DataFrame(rows, columns=["t", *AXES, *(f"v{a}" for a in AXES), *(f"f{a}" for a in AXES)])


# ------------------------------------------------------------------ analysis


def lstsq(A: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, float]:
    """Coefficients and R² of ``y ≈ A c``."""
    ok = np.all(np.isfinite(A), axis=1) & np.isfinite(y)
    c, *_ = np.linalg.lstsq(A[ok], y[ok], rcond=None)
    res = y[ok] - A[ok] @ c
    return c, 1.0 - res.var() / max(y[ok].var(), 1e-12)


def analyse(robot: pd.DataFrame, tgt: pd.DataFrame, segs, start: np.ndarray, amp: float, gains_set: dict) -> tuple[dict, dict]:
    """Per-axis results and the series the plot needs."""
    t = robot.t.to_numpy()
    xt = np.stack([np.interp(t, tgt.t, tgt[f"x{a}"]) for a in AXES], 1)
    vt = np.stack([np.interp(t, tgt.t, tgt[f"v{a}"]) for a in AXES], 1)
    seg_axis = np.interp(t, tgt.t, tgt.axis, left=-1, right=-1).round().astype(int)
    x = robot[list(AXES)].to_numpy()
    v = robot[[f"v{a}" for a in AXES]].to_numpy()
    f = robot[[f"f{a}" for a in AXES]].to_numpy()
    dt = np.gradient(t)
    k = max(5, int(round(0.01 / np.median(dt))))  # ~10 ms moving average before differentiating
    kern = np.ones(k) / k
    a_ = np.stack([np.gradient(np.convolve(v[:, i], kern, mode="same")) / dt for i in range(3)], 1)
    results, series = {}, {"t": t, "x": x, "xt": xt, "f": f}
    for i, ax in enumerate(AXES):
        on = seg_axis == i
        if on.sum() < 50:
            continue
        r = {"K_set": gains_set.get(f"k_pos_{ax}"), "D_set": gains_set.get(f"d_pos_{ax}")}
        e, de = xt[on, i] - x[on, i], vt[on, i] - v[on, i]
        (K, D, c0), r2 = lstsq(np.stack([e, de, np.ones_like(e)], 1), f[on, i])
        r.update(K_hat=K, D_hat=D, gains_r2=r2)
        v0 = 0.002
        (lam, b, fc), r2 = lstsq(np.stack([a_[on, i], v[on, i], np.tanh(v[on, i] / v0)], 1), f[on, i])
        r.update(Lambda_hat=lam, b_hat=b, Fc_hat=fc, arm_r2=r2)
        steps = []
        for (si, kind, t0, t1, p) in segs:
            if si != i or kind != "step":
                continue
            w = (t >= t0) & (t < t1)
            if w.sum() < 20:
                continue
            x0 = x[w, i][0] - start[i]
            y = x[w, i] - start[i]
            span = p - x0
            if abs(span) < 1e-4:
                continue
            frac = (y - x0) / span
            tt = t[w] - t0
            t10 = tt[np.argmax(frac >= 0.1)] if (frac >= 0.1).any() else np.nan
            t90 = tt[np.argmax(frac >= 0.9)] if (frac >= 0.9).any() else np.nan
            tail = tt > (t1 - t0) - 0.5
            steps.append({"rise_s": t90 - t10, "overshoot_pct": 100 * max(0.0, frac.max() - 1.0),
                          "ess_mm": 1e3 * float(np.mean(p - y[tail]))})
        r["steps"] = steps
        sines = []
        for (si, kind, t0, t1, fr) in segs:
            if si != i or kind != "sine":
                continue
            w = (t >= t0 + 1.0 / fr) & (t < t1)  # skip the first cycle
            if w.sum() < 20:
                continue
            ph = 2 * np.pi * fr * (t[w] - t0)
            A = np.stack([np.sin(ph), np.cos(ph), np.ones_like(ph)], 1)
            (s, c, _), _ = lstsq(A, x[w, i] - start[i])
            sines.append({"f_hz": fr, "gain": float(np.hypot(s, c) / amp), "phase_deg": float(np.degrees(np.arctan2(c, s)))})
        r["sines"] = sines
        results[ax] = r
    return results, series


def report(results: dict) -> str:
    lines = ["axis |   K set →  K̂ [N/m] |  D set →  D̂ [N·s/m] |  Λ̂ [kg] | b̂ [N·s/m] | F̂c [N] | R² gains/arm"]
    for ax, r in results.items():
        ks = "preset" if r["K_set"] is None else f"{r['K_set']:7.0f}"
        ds = "auto" if r["D_set"] is None else f"{r['D_set']:6.1f}"
        lines.append(f"  {ax}  | {ks} → {r['K_hat']:7.0f}    | {ds} → {r['D_hat']:6.1f}      | {r['Lambda_hat']:7.2f} |"
                     f" {r['b_hat']:8.1f}  | {r['Fc_hat']:6.2f} | {r['gains_r2']:.3f}/{r['arm_r2']:.3f}")
    lines.append("")
    lines.append("axis | steps: rise [ms], overshoot [%], steady-state error [mm]   | sines: f [Hz] gain / phase [deg]")
    for ax, r in results.items():
        st = "  ".join(f"{1e3 * s['rise_s']:.0f}/{s['overshoot_pct']:.0f}/{s['ess_mm']:+.2f}" for s in r["steps"])
        sn = "  ".join(f"{s['f_hz']:g}: {s['gain']:.2f}/{s['phase_deg']:+.0f}" for s in r["sines"])
        lines.append(f"  {ax}  | {st} | {sn}")
    return "\n".join(lines)


def plot(series: dict, results: dict, segs, start: np.ndarray, out: Path) -> list[Path]:
    t, x, xt, f = series["t"], series["x"], series["xt"], series["f"]
    tested = [ax for ax in AXES if ax in results]
    fig, axs = plt.subplots(len(tested), 3, figsize=(13, 2.9 * len(tested) + 0.4), squeeze=False)
    for row, ax in enumerate(tested):
        i = AXES.index(ax)
        c = COLS[i]
        steps = [(t0, t1, p) for si, kind, t0, t1, p in segs if si == i and kind == "step"]
        a = axs[row, 0]  # the step responses, overlaid from each step's start
        for t0, t1, p in steps:
            w = (t >= t0) & (t < t1)
            if w.any() and abs(p) > 0:
                a.plot((t[w] - t0) * 1e3, (x[w, i] - start[i]) * 1e3 / (p * 1e3), color=c, linewidth=1.5)
        a.axhline(1.0, color=INK_3, linewidth=1, linestyle="--")
        a.set_xlim(0, 600)
        a.set_title(f"{ax}: step response (normalized)", loc="left")
        a.set_xlabel("time from the step [ms]")
        a = axs[row, 1]  # the force law: measured task force vs. the set gains' prediction
        r = results[ax]
        a.plot(t, f[:, i], color=c, linewidth=1.0, label="task force")
        a.plot(t, r["K_hat"] * (xt[:, i] - x[:, i]), color=INK_2, linewidth=0.8, label="K̂ · error")
        a.set_title(f"{ax}: task force [N]", loc="left")
        a.set_xlabel("time [s]")
        if row == 0:
            a.legend(loc="upper right", fontsize=8)
        a = axs[row, 2]  # frequency response
        if r["sines"]:
            fr = [s["f_hz"] for s in r["sines"]]
            a.plot(fr, [s["gain"] for s in r["sines"]], marker="o", color=c, label="gain")
            a2 = a.twinx()
            a2.plot(fr, [s["phase_deg"] for s in r["sines"]], marker="s", color=INK_2, linewidth=1.2, label="phase")
            a2.set_ylabel("phase [deg]", color=INK_2)
            a2.grid(False)
            a.set_xscale("log")
            a.set_xticks(fr, [f"{v:g}" for v in fr])
            a.minorticks_off()
            a.set_ylim(0, max(1.2, 1.1 * max(s["gain"] for s in r["sines"])))
        a.set_title(f"{ax}: tracking gain (sines)", loc="left")
        a.set_xlabel("frequency [Hz]")
    fig.suptitle("T1 — OSC response", x=0.01, ha="left", color=INK, fontsize=12)
    fig.tight_layout()
    return save(fig, out, "t1_osc_response")


# ------------------------------------------------------------------ main


def main(argv: list[str] | None = None) -> None:
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--sim", action="store_true", help="simulated FR3 instead of the robot")
    p.add_argument("--kv", type=float, default=1000.0, help="stiffness on every axis [N/m]")
    p.add_argument("--dv", type=float, default=70.0, help="damping on every axis [N·s/m]")
    p.add_argument("--preset", action="store_true", help="keep the --osc-params gains (no --kv/--dv override)")
    p.add_argument("--axes", default="xyz", help="axes to test, in order")
    p.add_argument("--step", type=float, default=0.01, help="step size [m]")
    p.add_argument("--amp", type=float, default=0.005, help="sine amplitude [m]")
    p.add_argument("--freqs", type=float, nargs="+", default=[0.5, 1.0, 2.0, 4.0], help="sine frequencies [Hz]")
    p.add_argument("--cycles", type=int, default=3, help="cycles per frequency")
    p.add_argument("--rest", type=float, default=1.0, help="rest between segments [s]")
    p.add_argument("--command-hz", type=float, default=1000.0, help="target rate [Hz]")
    p.add_argument("--start-position", type=float, nargs=3, default=[0.4, 0.0, 0.25], help="start [m, base frame]")
    p.add_argument("--start-speed", type=float, default=0.05)
    p.add_argument("--namespace", default="fr3")
    p.add_argument("--controller", default="osc_controller")
    p.add_argument("--osc-params", default="python/fr3_haptic/configs/osc_teleop.yaml")
    p.add_argument("--out", type=Path, default=None, help="run directory; default ~/data/franka/rim-delays/<backend>/pretests/t1_<time>")
    args = p.parse_args(argv)

    from haptic_teleop import RateTicker

    backend_name = "sim" if args.sim else "robot"
    out = args.out or OUT / backend_name / "pretests" / datetime.datetime.now().strftime("t1_%Y%m%d_%H%M%S")
    segs, total = build_profile(args.step, args.amp, args.freqs, args.cycles, args.rest, args.axes)
    print(f"[T1] {backend_name}: {len(args.axes)} axes, {total:.0f} s; "
          f"{'preset gains' if args.preset else f'K = {args.kv:g} N/m, D = {args.dv:g} N·s/m on every axis'}")

    backend = SimBackend(args) if args.sim else RobotBackend(args)
    start = backend.begin()
    ticker = RateTicker(args.command_hz)
    rows = []
    t0 = time.monotonic()
    try:
        while True:
            now = time.monotonic()
            off, vel, axis, kind = target_at(segs, now - t0, args.amp)
            if kind == "done":
                break
            backend.aim(start + off, vel)
            rows.append((now, *(start + off), *vel, axis))
            ticker.sleep()
    except KeyboardInterrupt:
        print("\n[T1] interrupted: analysing what was recorded")
    robot = backend.end()
    tgt = pd.DataFrame(rows, columns=["t", "xx", "xy", "xz", "vx", "vy", "vz", "axis"])
    robot = robot[(robot.t >= t0)].copy()
    tgt.t -= t0
    robot.t -= t0
    segs_rel = segs  # segment times are already relative to t0

    gains_set = {}
    if not args.preset:
        gains_set = {**{f"k_pos_{a}": args.kv for a in AXES}, **{f"d_pos_{a}": args.dv for a in AXES}}
    results, series = analyse(robot, tgt, segs_rel, start, args.amp, gains_set)
    text = report(results)
    print(text)
    out.mkdir(parents=True, exist_ok=True)
    robot.to_csv(out / "robot.csv", index=False)
    tgt.to_csv(out / "target.csv", index=False)
    (out / "results.json").write_text(json.dumps({"args": {k: (str(v) if isinstance(v, Path) else v) for k, v in vars(args).items()},
                                                  "results": results}, indent=2, default=float))
    (out / "report.txt").write_text(text + "\n")
    paths = plot(series, results, segs_rel, start, out)
    print(f"[T1] -> {out} ({paths[0].name})")


if __name__ == "__main__":
    main()
