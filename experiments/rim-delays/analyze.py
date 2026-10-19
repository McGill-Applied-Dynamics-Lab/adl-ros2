r"""Metrics and figures for a rim-delays session (sim or robot runs, same code).

    pixi run -e humble python experiments/rim-delays/analyze.py data/rim-delays/sim/dev
    pixi run -e humble python experiments/rim-delays/analyze.py data/rim-delays/robot/day1 --rtt-detail 2600

Writes, into ``<session>/analysis/``: ``metrics.csv`` (one row per run), ``summary.csv`` and
``summary.md`` (mean ± std per task, method, delay), and figures (PNG + PDF).

Metrics, per run (z is the contact axis, ``s`` the table height, ``K`` the coupling stiffness):

- ``free_force_rms`` [N]: RMS of the rendered force while the leader is > 5 mm above the table —
  what free space feels like (0 is perfect).
- ``render_err_rms`` [N]: RMS of ``f - f_ideal``, with ``f_ideal = K max(0, s - x_l,z)`` along z:
  the gap to a rigid table rendered with no delay.
- ``penetration_max`` [mm]: deepest the leader goes below the table.
- ``robot_force_max`` [N]: peak table force on the robot (sim: the contact model; robot: the
  task force pushing down, ``-task_force_z``).
- ``energy_min`` [J]: minimum of the loop's passivity observer (negative: energy generated).
- ``stable``: the run ended on its own (no guard or watchdog stop).
- circle only: ``radius_err_rms`` [mm] (robot path vs the intended circle), ``robot_fz_std`` [N] while drawing,
  ``drag_rms`` [N] (tangential rendered force while drawing).
"""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402
from experiment_logger import load_streams  # noqa: E402

NAME = re.compile(r"(?P<task>[a-z]+)_(?P<method>[a-z-]+)_rtt(?P<rtt>\d+)_r(?P<rep>\d+)$")
METHODS = ("proxy-rim", "zoh", "tdpa-zoh")
LABELS = {"proxy-rim": "RIM (ours)", "zoh": "ZOH", "tdpa-zoh": "TDPA"}
COLORS = {"proxy-rim": "#2a78d6", "zoh": "#eb6834", "tdpa-zoh": "#1baf7a"}  # categorical slots 1-3
INK, INK_2, INK_3, GRID, SURFACE = "#0b0b0b", "#52514e", "#8a8984", "#e6e5e0", "#fcfcfb"
FREE_MARGIN = 0.005  # leader this far above the table counts as free motion [m]

plt.rcParams.update({
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
    "axes.edgecolor": GRID, "axes.labelcolor": INK_2, "xtick.color": INK_2, "ytick.color": INK_2,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8, "axes.spines.top": False,
    "axes.spines.right": False, "font.size": 10, "axes.titlesize": 11, "axes.titlecolor": INK,
    "legend.frameon": False, "lines.linewidth": 2.0,
})


# ------------------------------------------------------------------ loading


def load_run(run_dir: Path) -> dict | None:
    """Streams and metadata of one run, with the robot side normalized to ``robot`` columns
    ``ts, x, y, z, fz`` (table force on the robot, up = positive)."""
    meta_path = run_dir / "metadata.json"
    if not meta_path.exists() or not (run_dir / "samples.mcap").exists():
        return None
    meta = json.loads(meta_path.read_text())
    meta = meta.get("metadata", meta)
    streams = load_streams(run_dir, ["haptic", "plant", "sim", "command"])
    if "haptic" not in streams:
        return None
    if "sim" in streams:
        s = streams["sim"]
        robot = pd.DataFrame({"ts": s.ts, "x": s.x_x, "y": s.x_y, "z": s.x_z, "fz": s.f_contact_z})
    elif "plant" in streams and "ee_position_0" in streams["plant"]:
        s = streams["plant"]
        robot = pd.DataFrame({"ts": s.ts, "x": s.ee_position_0, "y": s.ee_position_1, "z": s.ee_position_2,
                              "fz": (-s.task_force_2).clip(lower=0.0)})
    else:
        robot = None
    return {"meta": meta, "haptic": streams["haptic"], "robot": robot}


def find(meta: dict, key: str, default=None):
    """``key`` from the metadata, wherever it sits (top level or under ``args``)."""
    if key in meta and meta[key] is not None:
        return meta[key]
    return meta.get("args", {}).get(key, default)


# ------------------------------------------------------------------ metrics


def run_metrics(run: dict) -> dict:
    meta, h, robot = run["meta"], run["haptic"], run["robot"]
    args = meta.get("args", {})
    dim = sum(1 for c in h.columns if c.startswith("x_l_"))
    ca = int(meta.get("contact_axis", 0))
    script = meta.get("script") or {}
    surface = script.get("surface_z", find(meta, "contact_surface"))
    K = float(args.get("kv", 500.0))
    xl = h[[f"x_l_{k}" for k in range(dim)]].to_numpy()
    f = h[[f"f_{k}" for k in range(dim)]].to_numpy()
    z = xl[:, ca]
    f_ideal = np.zeros_like(f)
    f_ideal[:, ca] = K * np.clip(surface - z, 0.0, None)
    free = z > surface + FREE_MARGIN
    out = {
        "free_force_rms": float(np.sqrt(np.mean(np.sum(f[free] ** 2, axis=1)))) if free.any() else np.nan,
        "render_err_rms": float(np.sqrt(np.mean(np.sum((f - f_ideal) ** 2, axis=1)))),
        "penetration_max": float(max(0.0, surface - z.min()) * 1e3),
        "energy_min": float(h.energy_j.min()) if "energy_j" in h else np.nan,
        "stable": str(meta.get("stop_reason", "")) in ("run ended", "run length reached", ""),
        "stop_reason": meta.get("stop_reason", ""),
    }
    if robot is not None and len(robot):
        out["robot_force_max"] = float(robot.fz.max())
    if script.get("profile") == "circle" and dim == 3 and robot is not None:
        origin = np.asarray(meta.get("leader_origin", meta.get("hold_position")), dtype=float)
        center, radius = origin[:2], float(script.get("radius", 0.05))
        # drawing = the leader on the table, away from the start/end of the circle
        on = (z < surface) & (np.hypot(xl[:, 0] - center[0], xl[:, 1] - center[1]) > 0.5 * radius)
        out["drag_rms"] = float(np.sqrt(np.mean(np.sum(f[on][:, :2] ** 2, axis=1)))) if on.any() else np.nan
        r_on = robot[(robot.z < surface + 0.002)]
        if len(r_on):
            r = np.hypot(r_on.x - center[0], r_on.y - center[1])
            drawing = r > 0.5 * radius
            out["radius_err_rms"] = float(np.sqrt(np.mean((r[drawing] - radius) ** 2)) * 1e3) if drawing.any() else np.nan
            out["robot_fz_std"] = float(r_on.fz[drawing].std()) if drawing.any() else np.nan
    return out


def collect(session: Path) -> tuple[pd.DataFrame, dict]:
    rows, runs = [], {}
    for run_dir in sorted(p for p in session.iterdir() if p.is_dir()):
        m = NAME.match(run_dir.name)
        if not m:
            continue
        run = load_run(run_dir)
        if run is None:
            print(f"[analyze] skipping {run_dir.name}: incomplete")
            continue
        key = (m["task"], m["method"], int(m["rtt"]), int(m["rep"]))
        runs[key] = run
        rows.append({"run": run_dir.name, "task": key[0], "method": key[1], "rtt_ms": key[2], "rep": key[3],
                     **run_metrics(run)})
    return pd.DataFrame(rows), runs


# ------------------------------------------------------------------ figures

METRIC_LABELS = {
    "free_force_rms": "Free-motion force felt, RMS [N]",
    "render_err_rms": "Error vs. ideal rigid table, RMS [N]",
    "penetration_max": "Operator overshoot into the table [mm]",
    "robot_force_max": "Peak table force on the robot [N]",
    "radius_err_rms": "Robot path vs. intended circle, RMS [mm]",
    "robot_fz_std": "Robot normal force spread [N]",
    "drag_rms": "Tangential drag felt, RMS [N]",
}


def methods_in(df: pd.DataFrame) -> list[str]:
    return [m for m in METHODS if m in set(df.method)] + sorted(set(df.method) - set(METHODS))


def save(fig, out: Path, name: str) -> None:
    for ext in ("png", "pdf"):
        fig.savefig(out / f"{name}.{ext}", dpi=200, bbox_inches="tight")
    plt.close(fig)


def fig_metrics(df: pd.DataFrame, task: str, metrics: list[str], out: Path) -> None:
    """Small multiples: each metric against the round-trip delay, one line per method (mean ± std)."""
    d = df[df.task == task]
    metrics = [m for m in metrics if m in d and d[m].notna().any()]
    if not metrics:
        return
    rtts = sorted(d.rtt_ms.unique())
    pos = {r: i for i, r in enumerate(rtts)}  # categorical x: the delays span 3 decades
    ncol = min(2, len(metrics))
    nrow = int(np.ceil(len(metrics) / ncol))
    fig, axes = plt.subplots(nrow, ncol, figsize=(4.6 * ncol, 3.2 * nrow), squeeze=False)
    for ax, metric in zip(axes.flat, metrics):
        for method in methods_in(d):
            g = d[d.method == method].groupby("rtt_ms")[metric].agg(["mean", "std"]).reindex(rtts)
            x = [pos[r] for r in g.index]
            ax.errorbar(x, g["mean"], yerr=g["std"].fillna(0), color=COLORS.get(method, INK_3), marker="o",
                        markersize=6, capsize=3, label=LABELS.get(method, method))
        ax.set_xticks(range(len(rtts)), [f"{r:g}" if r < 1000 else f"{r / 1000:g} s" for r in rtts])
        ax.set_xlabel("Round-trip delay [ms]")
        ax.set_title(METRIC_LABELS.get(metric, metric), loc="left")
        ax.set_ylim(bottom=0)
    for ax in list(axes.flat)[len(metrics):]:
        ax.set_visible(False)
    handles, labels = axes.flat[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=len(labels), bbox_to_anchor=(0.5, 1.02))
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    save(fig, out, f"metrics_{task}")


def fig_press_timeseries(runs: dict, rtt: int, out: Path, rep: int = 0) -> None:
    """One column per method at one delay: z of the leader and the robot, and the force along z
    rendered to the operator vs. the table force on the robot."""
    cells = [(m, runs.get(("press", m, rtt, rep))) for m in METHODS]
    cells = [(m, r) for m, r in cells if r is not None]
    if not cells:
        return
    fig, axes = plt.subplots(2, len(cells), figsize=(4.2 * len(cells), 5.2), sharex=True, sharey="row", squeeze=False)
    for j, (method, run) in enumerate(cells):
        meta, h, robot = run["meta"], run["haptic"], run["robot"]
        ca = int(meta.get("contact_axis", 0))
        surface = (meta.get("script") or {}).get("surface_z", find(meta, "contact_surface"))
        ax = axes[0, j]
        ax.axhline(surface * 1e3, color=INK_3, linewidth=1, linestyle="--")
        ax.annotate("table", (h.ts.iloc[-1], surface * 1e3), xytext=(-2, 3), textcoords="offset points",
                    ha="right", color=INK_3, fontsize=8)
        ax.plot(h.ts, h[f"x_l_{ca}"] * 1e3, color=COLORS[method], label="operator (leader)")
        if robot is not None:
            ax.plot(robot.ts, robot.z * 1e3, color=INK_2, linewidth=1.5, label="robot")
        ax.set_title(f"{LABELS[method]}", loc="left", color=INK)
        ax = axes[1, j]
        ax.plot(h.ts, h[f"f_{ca}"], color=COLORS[method], label="felt by the operator")
        if robot is not None:
            ax.plot(robot.ts, robot.fz, color=INK_2, linewidth=1.5, label="on the robot (table)")
        ax.set_xlabel("Time [s]")
    axes[0, 0].set_ylabel("Height z [mm]")
    axes[1, 0].set_ylabel("Force along z [N]")
    for ax in axes[:, -1]:
        ax.legend(loc="upper right", fontsize=8)
    fig.suptitle(f"Press task, round-trip delay {rtt:g} ms", x=0.01, ha="left", color=INK, fontsize=12)
    fig.tight_layout()
    save(fig, out, f"press_timeseries_rtt{rtt:04d}")


def fig_circle_paths(runs: dict, rtt: int, out: Path, rep: int = 0) -> None:
    cells = [(m, runs.get(("circle", m, rtt, rep))) for m in METHODS]
    cells = [(m, r) for m, r in cells if r is not None and r["robot"] is not None]
    if not cells:
        return
    fig, axes = plt.subplots(1, len(cells), figsize=(3.6 * len(cells), 3.8), sharex=True, sharey=True, squeeze=False)
    for ax, (method, run) in zip(axes[0], cells):
        h, robot, meta = run["haptic"], run["robot"], run["meta"]
        surface = (meta.get("script") or {}).get("surface_z", find(meta, "contact_surface"))
        on_l = h.x_l_2 < surface
        on_r = robot.z < surface + 0.002
        ax.plot(h.x_l_0[on_l] * 1e3, h.x_l_1[on_l] * 1e3, color=COLORS[method], linewidth=1.5, label="operator")
        ax.plot(robot.x[on_r] * 1e3, robot.y[on_r] * 1e3, color=INK_2, linewidth=1.2, label="robot, on the table")
        ax.set_aspect("equal")
        ax.set_title(LABELS[method], loc="left")
        ax.set_xlabel("x [mm]")
    axes[0, 0].set_ylabel("y [mm]")
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels,
               loc="lower center", ncol=2, bbox_to_anchor=(0.5, -0.04), fontsize=9)
    fig.suptitle(f"Circle task, round-trip delay {rtt:g} ms", x=0.01, ha="left", color=INK, fontsize=12)
    fig.tight_layout()
    save(fig, out, f"circle_paths_rtt{rtt:04d}")


# ------------------------------------------------------------------ main


def summarize(df: pd.DataFrame) -> pd.DataFrame:
    metrics = [c for c in METRIC_LABELS if c in df] + ["energy_min"]
    g = df.groupby(["task", "method", "rtt_ms"])
    summary = g[metrics].agg(["mean", "std"])
    summary.columns = [f"{a}_{b}" for a, b in summary.columns]
    summary["runs"] = g.size()
    summary["stable_runs"] = g["stable"].sum()
    return summary.reset_index()


def markdown(summary: pd.DataFrame) -> str:
    lines = []
    for task, d in summary.groupby("task"):
        metrics = [m for m in METRIC_LABELS if f"{m}_mean" in d and d[f"{m}_mean"].notna().any()]
        lines += [f"## {task}", "", "| method | RTT [ms] | " + " | ".join(METRIC_LABELS[m] for m in metrics) + " | stable |",
                  "|---|---:|" + "---:|" * len(metrics) + "---:|"]
        for _, r in d.sort_values(["rtt_ms", "method"]).iterrows():
            cells = [f"{r[f'{m}_mean']:.2f} ± {0 if np.isnan(r[f'{m}_std']) else r[f'{m}_std']:.2f}" for m in metrics]
            lines.append(f"| {LABELS.get(r.method, r.method)} | {r.rtt_ms:g} | " + " | ".join(cells)
                         + f" | {int(r.stable_runs)}/{int(r.runs)} |")
        lines.append("")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> None:
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("session", type=Path, help="session directory (holds the run directories)")
    p.add_argument("--rtt-detail", type=int, nargs="+", default=None, help="delays for the time-series figures")
    args = p.parse_args(argv)

    df, runs = collect(args.session)
    if df.empty:
        raise SystemExit(f"no runs in {args.session}")
    out = args.session / "analysis"
    out.mkdir(exist_ok=True)
    df.to_csv(out / "metrics.csv", index=False)
    summary = summarize(df)
    summary.to_csv(out / "summary.csv", index=False)
    (out / "summary.md").write_text(markdown(summary))
    print(markdown(summary))

    fig_metrics(df, "press", ["free_force_rms", "render_err_rms", "penetration_max", "robot_force_max"], out)
    fig_metrics(df, "circle", ["drag_rms", "render_err_rms", "radius_err_rms", "robot_fz_std"], out)
    for rtt in args.rtt_detail or sorted(df.rtt_ms.unique()):
        fig_press_timeseries(runs, int(rtt), out)
        fig_circle_paths(runs, int(rtt), out)
    print(f"[analyze] {len(df)} runs -> {out}")


if __name__ == "__main__":
    main()
