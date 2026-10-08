"""Plot an ``fr3_teleop`` run: tracking, timing and safety, joint torques, end-effector.

Reads a run directory written with ``fr3_teleop --save`` (``samples.mcap`` + ``metadata.json``)
through ``experiment_logger.load_streams``, prints a summary, and draws four figures, each a
stack of subplots on one shared time axis (run time [s], haptic t = 0):

- ``tracking``: position and velocity along the interface axis (handle vs robot), the rendered
  force with the coupling and external forces, and the tracking error;
- ``timing``: plant-sample and device age seen by the haptic loop, passivity energy, watchdog
  gain and guard state;
- ``torques``: per joint, commanded torque vs the robot's external-torque estimate;
- ``ee``: end-effector drift on the held axes, orientation error, task force.

Colors follow the entity across figures: handle / leader / rendered = blue, robot / measured =
orange, estimate / external = aqua (validated categorical slots 1-3 of the dataviz palette).

    pixi run -e humble fr3_plot                      # latest run in the default output dir
    pixi run -e humble fr3_plot <run_dir> --figs tracking,torques --t0 2 --t1 8
    pixi run -e humble fr3_plot <run_dir> --save     # PNGs in <run_dir>/plots, no window
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd

DEFAULT_RUNS = Path("/home/athena/csirois/data/franka/fr3_haptic")
FIGS = ("tracking", "timing", "torques", "ee")

# Categorical slots 1-3 (light surface), fixed by entity, never cycled
BLUE, ORANGE, AQUA = "#2a78d6", "#eb6834", "#1baf7a"
SURFACE, INK, INK_2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e6e5e0"
AXIS_COLORS = {"x": BLUE, "y": ORANGE, "z": AQUA}


def _style(plt) -> None:
    plt.rcParams.update(
        {
            "figure.facecolor": SURFACE,
            "axes.facecolor": SURFACE,
            "savefig.facecolor": SURFACE,
            "axes.edgecolor": GRID,
            "axes.labelcolor": INK_2,
            "axes.titlecolor": INK,
            "axes.titlesize": 10,
            "axes.titleweight": "bold",
            "axes.titlelocation": "left",
            "axes.labelsize": 9,
            "axes.grid": True,
            "grid.color": GRID,
            "grid.linewidth": 0.6,
            "grid.linestyle": "-",
            "axes.spines.top": False,
            "axes.spines.right": False,
            "xtick.color": INK_2,
            "ytick.color": INK_2,
            "xtick.labelsize": 8,
            "ytick.labelsize": 8,
            "lines.linewidth": 1.3,
            "legend.fontsize": 8,
            "legend.frameon": False,
            "legend.labelcolor": INK,
            "text.color": INK,
            "font.size": 9,
        }
    )


# ------------------------------------------------------------------------------------- data


def latest_run(root: Path) -> Path:
    """The most recently modified ``run_*`` directory under ``root``."""
    runs = sorted(root.glob("run_*"), key=lambda p: p.stat().st_mtime)
    if not runs:
        raise FileNotFoundError(f"no run_* directory in {root}")
    return runs[-1]


def load_run(run_dir: Path) -> tuple[dict[str, pd.DataFrame], dict]:
    """Streams (one DataFrame each, time column ``ts``) and the run metadata."""
    from experiment_logger import load_streams

    streams = load_streams(run_dir)
    meta_path = run_dir / "metadata.json"
    meta = json.loads(meta_path.read_text()) if meta_path.exists() else {}
    return streams, meta


def _window(df: pd.DataFrame | None, t0: float | None, t1: float | None) -> pd.DataFrame | None:
    if df is None or df.empty:
        return None
    m = np.ones(len(df), dtype=bool)
    if t0 is not None:
        m &= df["ts"].to_numpy() >= t0
    if t1 is not None:
        m &= df["ts"].to_numpy() <= t1
    return df[m]


def _args(meta: dict) -> dict:
    return meta.get("metadata", meta).get("args", {}) if meta else {}


def _hold(meta: dict) -> np.ndarray | None:
    hold = meta.get("metadata", meta).get("hold_position") if meta else None
    return None if hold is None else np.asarray(hold, dtype=float)


def _rates(meta: dict) -> str:
    """Rates as recorded: ``rates`` metadata (current runs), else the older ``plant_hz`` flag."""
    rates = meta.get("metadata", meta).get("rates") if meta else None
    if rates:
        text = f"model update {rates.get('model_update_hz', '?'):g} Hz, commands {rates.get('command_hz', '?'):g} Hz"
        if rates.get("rim_update_hz"):
            text += f", RIM {rates['rim_update_hz']:g} Hz"
        return text
    return f"plant {_args(meta).get('plant_hz', '?')} Hz"


def summary(streams: dict[str, pd.DataFrame], meta: dict) -> list[str]:
    """Key numbers of the run, one line each."""
    a = _args(meta)
    lines = [
        f"method {a.get('method', '?')}, kv {a.get('kv', '?')} N/m, dv {a.get('dv', '?')} N.s/m, "
        f"{_rates(meta)}, force {'on' if a.get('force') else 'off'}"
    ]
    h, p = streams.get("haptic"), streams.get("plant")
    if h is not None and len(h):
        dur = h["ts"].iloc[-1]
        lines.append(f"haptic: {len(h)} ticks over {dur:.1f} s ({len(h) / max(dur, 1e-9):.0f} Hz nominal stamps)")
        age = h["plant_age_s"].replace([np.inf], np.nan).dropna() * 1e3
        if len(age):
            lines.append(f"plant age seen by the haptic loop: p50 {age.median():.1f} ms, p99 {age.quantile(0.99):.1f} ms, max {age.max():.1f} ms")
        f = h["f_0"].abs()
        lines.append(f"rendered force: max {f.max():.2f} N, rms {np.sqrt((h['f_0'] ** 2).mean()):.2f} N (world)")
        lines.append(f"guard trips: {int((h['guard_tripped'].diff() > 0).sum())}, watchdog gain min {h['watchdog_gain'].iloc[100:].min():.2f}")
    if p is not None and len(p) > 1 and h is not None:
        lines.append(f"plant samples: {len(p)} ({(len(p) - 1) / (p['ts'].iloc[-1] - p['ts'].iloc[0]):.1f} Hz)")
        err = (h["x_l_0"] - np.interp(h["ts"], p["ts"], p["x_i"])) * 1e3
        lines.append(f"tracking error (leader - robot): rms {np.sqrt((err**2).mean()):.1f} mm, max {err.abs().max():.1f} mm")
    stop = meta.get("metadata", meta).get("stop_reason") if meta else None
    if stop:
        lines.append(f"stop reason: {stop}")
    return lines


# ------------------------------------------------------------------------------------ figures


def _legend(ax) -> None:
    """Legend above the plot, right-aligned on the title line: never over the data."""
    handles = [h for h in ax.get_lines() if not h.get_label().startswith("_")]
    if len(handles) > 1:
        ax.legend(handles=handles, loc="lower right", bbox_to_anchor=(1.0, 1.0), ncols=len(handles), borderaxespad=0.2)


def _finish(ax, ylabel: str, legend: bool = True) -> None:
    ax.set_ylabel(ylabel)
    if legend:
        _legend(ax)


def fig_tracking(plt, s: dict[str, pd.DataFrame], meta: dict):
    h, p = s.get("haptic"), s.get("plant")
    if h is None:
        return None
    x_ref = h["x_l_0"].iloc[0]
    fig, axes = plt.subplots(4, 1, sharex=True, figsize=(11, 8.5), constrained_layout=True)
    ax = axes[0]
    ax.set_title("Position along the interface axis (relative to start)")
    ax.plot(h["ts"], (h["x_l_0"] - x_ref) * 1e3, color=BLUE, label="handle (leader)")
    if p is not None:
        ax.plot(p["ts"], (p["x_i"] - x_ref) * 1e3, color=ORANGE, label="robot")
    half_range = _args(meta).get("interface_range")
    if half_range:  # --interface-range: the robot's commanded position stops here
        for sign in (-1, 1):
            ax.axhline(sign * half_range * 1e3, color=INK_2, linewidth=0.8, label="_limit")
        ax.annotate(f"interface range ±{half_range * 1e3:g} mm", (h["ts"].iloc[0], -half_range * 1e3),
                    xytext=(4, 4), textcoords="offset points", color=INK_2, fontsize=8)  # fmt: skip
    _finish(ax, "mm")

    ax = axes[1]
    ax.set_title("Velocity along the interface axis")
    ax.plot(h["ts"], h["v_l_0"] * 1e3, color=BLUE, label="handle (leader)")
    if p is not None:
        ax.plot(p["ts"], p["v_i"] * 1e3, color=ORANGE, label="robot")
    _finish(ax, "mm/s")

    ax = axes[2]
    ax.set_title("Force along the interface axis")
    ax.plot(h["ts"], h["f_0"], color=BLUE, label="rendered to the handle")
    if p is not None:
        ax.plot(p["ts"], p["lam"], color=ORANGE, label="coupling (robot)")
    rs = s.get("robot_state")
    axis = _args(meta).get("interface_axis", "z")
    col = f"o_f_ext_hat_k.wrench.force.{axis}"
    if rs is not None and col in rs:
        ax.plot(rs["ts"], -rs[col], color=AQUA, label="external estimate")
    _finish(ax, "N, as felt by the operator")

    ax = axes[3]
    ax.set_title("Tracking error: handle - robot")
    if p is not None:
        err = (h["x_l_0"] - np.interp(h["ts"], p["ts"], p["x_i"])) * 1e3
        ax.plot(h["ts"], err, color=BLUE)
    ax.axhline(0.0, color=INK_2, linewidth=0.8)
    _finish(ax, "mm", legend=False)
    axes[-1].set_xlabel("run time [s]")
    fig.suptitle(_title(meta, "Tracking"), x=0.01, ha="left", fontsize=11, fontweight="bold")
    return fig


def fig_timing(plt, s: dict[str, pd.DataFrame], meta: dict):
    h = s.get("haptic")
    if h is None:
        return None
    links = [(name, s[name]) for name in ("feedback_link", "command_link") if s.get(name) is not None and len(s[name])]
    fig, axes = plt.subplots(4 + bool(links), 1, sharex=True, figsize=(11, 7.5 + 1.8 * bool(links)), constrained_layout=True)
    a = _args(meta)
    ax = axes[0]
    ax.set_title("Plant sample age at the haptic loop (max over 100 ms)")
    age = h["plant_age_s"].replace([np.inf], np.nan) * 1e3
    ax.plot(h["ts"], age.rolling(100, min_periods=1).max(), color=ORANGE, label="since measured (incl. delay)")
    if "plant_delivery_age_s" in h and links:
        gap = h["plant_delivery_age_s"].replace([np.inf], np.nan) * 1e3
        ax.plot(h["ts"], gap.rolling(100, min_periods=1).max(), color=BLUE, label="since delivered (watchdog)")
    limit = a.get("plant_max_age_ms")
    if limit:
        ax.axhline(limit, color=INK_2, linewidth=0.8)
        ax.annotate(f"watchdog limit {limit:g} ms", (h["ts"].iloc[0], limit), xytext=(4, -10),
                    textcoords="offset points", color=INK_2, fontsize=8)  # fmt: skip
    _finish(ax, "ms")

    ax = axes[1]
    ax.set_title("Device sample age")
    ax.plot(h["ts"], h["device_age_s"] * 1e3, color=BLUE)
    _finish(ax, "ms", legend=False)

    ax = axes[2]
    ax.set_title("Passivity observer energy (handle; positive = energy given to the operator)")
    ax.plot(h["ts"], h["energy_j"] * 1e3, color=BLUE)
    _finish(ax, "mJ", legend=False)

    ax = axes[3]
    ax.set_title("Force gates")
    ax.plot(h["ts"], h["watchdog_gain"], color=BLUE, label="watchdog gain")
    ax.plot(h["ts"], h["guard_tripped"], color=ORANGE, label="guard tripped")
    ax.set_ylim(-0.05, 1.1)
    _finish(ax, "")
    if links:
        ax = axes[4]
        ax.set_title("Simulated delay, per delivered item")
        colors = {"feedback_link": ORANGE, "command_link": BLUE}
        names = {"feedback_link": "feedback (robot -> haptic)", "command_link": "command (haptic -> robot)"}
        for name, df in links:
            ax.plot(df["ts"], df["delay_ms"], color=colors[name], label=names[name], linewidth=0.9)
        _finish(ax, "ms")
    axes[-1].set_xlabel("run time [s]")
    fig.suptitle(_title(meta, "Timing and safety"), x=0.01, ha="left", fontsize=11, fontweight="bold")
    return fig


def fig_torques(plt, s: dict[str, pd.DataFrame], meta: dict):
    cmd, rs = s.get("joint_torques_cmd"), s.get("robot_state")
    if cmd is None and rs is None:
        return None
    fig, axes = plt.subplots(7, 1, sharex=True, figsize=(11, 11), constrained_layout=True)
    for j, ax in enumerate(axes):
        ax.set_title(f"Joint {j + 1}", fontsize=9)
        if cmd is not None and f"effort_{j}" in cmd:
            ax.plot(cmd["ts"], cmd[f"effort_{j}"], color=BLUE, label="commanded (controller, no gravity)")
        col = f"tau_ext_hat_filtered.effort_{j}"
        if rs is not None and col in rs:
            ax.plot(rs["ts"], rs[col], color=AQUA, label="external torque estimate")
        ax.set_ylabel("Nm")
    _legend(axes[0])
    axes[-1].set_xlabel("run time [s]")
    fig.suptitle(_title(meta, "Joint torques (sampled at --robot-log-hz)"), x=0.01, ha="left", fontsize=11, fontweight="bold")
    return fig


def fig_ee(plt, s: dict[str, pd.DataFrame], meta: dict):
    ee, err, wr = s.get("ee_state"), s.get("task_error"), s.get("task_wrench")
    if ee is None and err is None and wr is None:
        return None
    hold = _hold(meta)
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(11, 7), constrained_layout=True)
    ax = axes[0]
    ax.set_title("End-effector position relative to the hold pose")
    if ee is not None:
        for k, name in enumerate("xyz"):
            col = f"pose.pose.position.{name}"
            ref = hold[k] if hold is not None else ee[col].iloc[0]
            ax.plot(ee["ts"], (ee[col] - ref) * 1e3, color=AXIS_COLORS[name], label=name)
    _finish(ax, "mm")

    ax = axes[1]
    ax.set_title("Orientation error (controller task error, rotation vector)")
    if err is not None:
        for name in "xyz":
            ax.plot(err["ts"], err[f"twist.angular.{name}"] * 1e3, color=AXIS_COLORS[name], label=f"about {name}")
    _finish(ax, "mrad")

    ax = axes[2]
    ax.set_title("Task force commanded by the controller")
    if wr is not None:
        for name in "xyz":
            ax.plot(wr["ts"], wr[f"wrench.force.{name}"], color=AXIS_COLORS[name], label=name)
    _finish(ax, "N")
    axes[-1].set_xlabel("run time [s]")
    fig.suptitle(_title(meta, "End-effector"), x=0.01, ha="left", fontsize=11, fontweight="bold")
    return fig


def _title(meta: dict, what: str) -> str:
    a = _args(meta)
    return f"{what} - {a.get('method', '?')}, kv {a.get('kv', '?')} N/m, dv {a.get('dv', '?')} N.s/m, {_rates(meta)}"


BUILDERS = {"tracking": fig_tracking, "timing": fig_timing, "torques": fig_torques, "ee": fig_ee}


def main(argv: list[str] | None = None) -> None:
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("run_dir", nargs="?", type=Path, default=None, help=f"run directory (default: latest in {DEFAULT_RUNS})")
    p.add_argument("--figs", default=",".join(FIGS), help=f"comma-separated subset of {', '.join(FIGS)}")
    p.add_argument("--t0", type=float, default=None, help="start of the time window [s]")
    p.add_argument("--t1", type=float, default=None, help="end of the time window [s]")
    p.add_argument("--save", action="store_true", help="write PNGs to <run_dir>/plots instead of opening windows")
    args = p.parse_args(argv)

    run_dir = args.run_dir or latest_run(DEFAULT_RUNS)
    streams, meta = load_run(run_dir)
    print(f"[fr3_plot] {run_dir}")
    for line in summary(streams, meta):
        print(f"[fr3_plot]   {line}")
    streams = {k: _window(v, args.t0, args.t1) for k, v in streams.items()}
    streams = {k: v for k, v in streams.items() if v is not None}

    import matplotlib

    if args.save:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    _style(plt)
    wanted = [f.strip() for f in args.figs.split(",") if f.strip()]
    unknown = set(wanted) - set(FIGS)
    if unknown:
        raise SystemExit(f"unknown figure(s): {sorted(unknown)}; choose from {FIGS}")
    out = run_dir / "plots"
    for name in wanted:
        fig = BUILDERS[name](plt, streams, meta)
        if fig is None:
            print(f"[fr3_plot]   {name}: no data")
            continue
        if args.save:
            out.mkdir(exist_ok=True)
            path = out / f"{name}.png"
            fig.savefig(path, dpi=130)
            plt.close(fig)
            print(f"[fr3_plot]   {name} -> {path}")
    if not args.save:
        plt.show()


if __name__ == "__main__":
    main()
