#!/usr/bin/env python3
from __future__ import annotations

import argparse
import pickle
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.patches import FancyBboxPatch, Rectangle

CHANNEL_NAMES = ("S0", "S1", "S2", "S3")
RESULTS_DIR = Path(__file__).resolve().parent / "results"
RF_PLOT_X_MIN = 350  # match live_rolling_view.py viewport

# ── colour palette ───────────────────────────────────────────────────────────
BG        = "#12121e"
PANEL     = "#1c1c30"
BORDER    = "#2e2e50"
TEXT      = "#d4d4f0"
DIM_TEXT  = "#7070a0"
ACCENT    = "#4a90e2"
CH_COLORS = ["#00d4ff", "#ff7043", "#b388ff", "#69f0ae"]
BAND_CLR  = "#ff3b30"
SENSOR_FC = "#1e3558"
SENSOR_EC = "#4a90e2"
PHASE_CLR = {"compress": "#ffcc44", "slide": "#44ff99", "decompress": "#ff9944", "done": DIM_TEXT}


def load_result(path: Path) -> dict:
    with open(path, "rb") as f:
        return pickle.load(f)


def _pick_trial_dir(results_dir: Path) -> Path:
    """Pick any subdirectory of *results_dir* that contains .pkl result files.

    Accepts both auto-numbered `trial_NN/` folders and operator-named ones
    like `1mm_depth_dia24mm/`.  Folders without .pkl files are hidden.
    """
    candidate_dirs = sorted(
        [p for p in results_dir.iterdir()
         if p.is_dir() and any(p.glob("*.pkl"))]
    )
    if not candidate_dirs:
        raise FileNotFoundError(f"No result folders with .pkl files found in {results_dir}")
    print("\nAvailable result folders:")
    for i, path in enumerate(candidate_dirs):
        n_pkls = len(list(path.glob("*.pkl")))
        print(f"  [{i}] {path.name}  ({n_pkls} file{'s' if n_pkls != 1 else ''})")
    raw = input(f"Select folder [0-{len(candidate_dirs) - 1}]: ").strip()
    try:
        return candidate_dirs[int(raw)]
    except (ValueError, IndexError):
        print("Invalid selection, using 0.")
        return candidate_dirs[0]


def _pick_speed_file(trial_dir: Path) -> Path:
    pkls = sorted(trial_dir.glob("*.pkl"))
    if not pkls:
        raise FileNotFoundError(f"No result files found in {trial_dir}")
    print(f"\nAvailable rolling files in {trial_dir.name}:")
    for i, path in enumerate(pkls):
        print(f"  [{i}] {path.stem}")
    raw = input(f"Select file [0-{len(pkls) - 1}]: ").strip()
    try:
        return pkls[int(raw)]
    except (ValueError, IndexError):
        print("Invalid selection, using 0.")
        return pkls[0]


def _extract_rf_array(trial: dict, expected_samples: int) -> np.ndarray:
    frames = trial.get("frames", [])
    if not frames:
        raise ValueError("No RF frames stored in this rolling file.")

    valid_frames = []
    for frame_data, _t in frames:
        if len(frame_data) != len(CHANNEL_NAMES):
            continue
        if any(len(ch) != expected_samples for ch in frame_data):
            continue
        valid_frames.append(frame_data)

    if not valid_frames:
        raise ValueError("No structurally valid RF frames found in this rolling file.")
    return np.asarray(valid_frames, dtype=np.float32)


def _frame_times(trial: dict, expected_samples: int) -> np.ndarray:
    ts = []
    for frame_data, t in trial.get("frames", []):
        if len(frame_data) != len(CHANNEL_NAMES):
            continue
        if any(len(ch) != expected_samples for ch in frame_data):
            continue
        ts.append(float(t))
    return np.asarray(ts, dtype=float)


def _eef_series(trial: dict, frame_times: np.ndarray, key: str, none_as_nan: bool = False) -> np.ndarray:
    """Generic (N_frames, 3) series interpolated from ee_poses. Returns NaN array if unavailable."""
    ee_poses = trial.get("ee_poses_slide_only") or trial.get("ee_poses", [])
    nan_arr = np.full((max(len(frame_times), 0), 3), np.nan, dtype=float)
    if not ee_poses or len(frame_times) == 0:
        return nan_arr
    ee_ts = np.asarray([p["t"] for p in ee_poses], dtype=float)
    if none_as_nan:
        raw = [p.get(key) for p in ee_poses]
        if all(v is None for v in raw):
            return nan_arr
        vals = np.asarray([[np.nan] * 3 if v is None else v for v in raw], dtype=float)
    else:
        vals = np.asarray([p.get(key, [np.nan] * 3) for p in ee_poses], dtype=float)
    out = np.empty((len(frame_times), 3), dtype=float)
    for axis in range(3):
        out[:, axis] = np.interp(frame_times, ee_ts, vals[:, axis],
                                 left=vals[0, axis], right=vals[-1, axis])
    return out


def _cmd_eef_series(trial: dict, frame_times: np.ndarray, key: str) -> np.ndarray:
    """Same as _eef_series but reads from commanded_ee_poses."""
    cmd_poses = trial.get("commanded_ee_poses", [])
    nan_arr = np.full((max(len(frame_times), 0), 3), np.nan, dtype=float)
    if not cmd_poses or len(frame_times) == 0:
        return nan_arr
    cmd_ts = np.asarray([p["t"] for p in cmd_poses], dtype=float)
    vals = np.asarray([p.get(key, [np.nan] * 3) for p in cmd_poses], dtype=float)
    out = np.empty((len(frame_times), 3), dtype=float)
    for axis in range(3):
        out[:, axis] = np.interp(frame_times, cmd_ts, vals[:, axis],
                                 left=vals[0, axis], right=vals[-1, axis])
    return out


def _eef_rotation_series_deg(trial: dict, frame_times: np.ndarray) -> np.ndarray:
    ee_poses = trial.get("ee_poses_slide_only") or trial.get("ee_poses", [])
    if ee_poses:
        ee_ts = np.asarray([pose["t"] for pose in ee_poses], dtype=float)
        ee_rot = np.asarray(
            [pose.get("orientation_euler_deg_xyz", [np.nan, np.nan, np.nan]) for pose in ee_poses],
            dtype=float,
        )
        if len(ee_ts) > 0 and len(frame_times) > 0:
            rot_interp = np.empty((len(frame_times), 3), dtype=float)
            for axis in range(3):
                rot_interp[:, axis] = np.interp(
                    frame_times,
                    ee_ts,
                    ee_rot[:, axis],
                    left=ee_rot[0, axis],
                    right=ee_rot[-1, axis],
                )
            return rot_interp

    planned = trial.get("planned_slide_poses", [])
    if planned and len(frame_times) > 0:
        planned_rot = np.asarray(
            [pose.get("orientation_euler_deg_xyz", [np.nan, np.nan, np.nan]) for pose in planned],
            dtype=float,
        )
        idx = np.linspace(0, len(planned_rot) - 1, num=len(frame_times))
        out = np.empty((len(frame_times), 3), dtype=float)
        base = np.arange(len(planned_rot), dtype=float)
        for axis in range(3):
            out[:, axis] = np.interp(idx, base, planned_rot[:, axis])
        return out

    return np.full((len(frame_times), 3), np.nan, dtype=float)


def _eef_position_series_m(trial: dict, frame_times: np.ndarray) -> np.ndarray:
    ee_poses = trial.get("ee_poses_slide_only") or trial.get("ee_poses", [])
    if ee_poses:
        ee_ts = np.asarray([pose["t"] for pose in ee_poses], dtype=float)
        ee_pos = np.asarray([pose.get("position", [np.nan, np.nan, np.nan]) for pose in ee_poses], dtype=float)
        if len(ee_ts) > 0 and len(frame_times) > 0:
            pos_interp = np.empty((len(frame_times), 3), dtype=float)
            for axis in range(3):
                pos_interp[:, axis] = np.interp(
                    frame_times,
                    ee_ts,
                    ee_pos[:, axis],
                    left=ee_pos[0, axis],
                    right=ee_pos[-1, axis],
                )
            return pos_interp

    planned = trial.get("planned_slide_poses", [])
    if planned and len(frame_times) > 0:
        planned_pos = np.asarray([pose.get("position", [np.nan, np.nan, np.nan]) for pose in planned], dtype=float)
        idx = np.linspace(0, len(planned_pos) - 1, num=len(frame_times))
        out = np.empty((len(frame_times), 3), dtype=float)
        base = np.arange(len(planned_pos), dtype=float)
        for axis in range(3):
            out[:, axis] = np.interp(idx, base, planned_pos[:, axis])
        return out

    return np.full((len(frame_times), 3), np.nan, dtype=float)


def print_summary(path: Path, payload: dict) -> None:
    trial = payload["trial"]
    session = payload["session"]
    summary = trial.get("rf_capture_summary", {})
    static_rf = trial.get("static_rf") or {}
    print("\n" + "=" * 72)
    print(f"File: {path}")
    print(f"Speed: {trial['speed_m_s'] * 1000.0:.1f} mm/s")
    speed_idx = trial.get("speed_idx")
    rep_idx = trial.get("repeat_idx")
    if speed_idx is not None and rep_idx is not None:
        print(f"Speed/repeat index: speed_idx={speed_idx}, repeat_idx={rep_idx}")
    diam = trial.get("roller_diameter_m", session.get("roller_diameter_m"))
    if diam is not None:
        print(f"Roller diameter: {float(diam) * 1000.0:.1f} mm")
    roll_dist = trial.get("roll_distance_m")
    roll_off = trial.get("roll_end_offset_m", session.get("roll_end_offset_m"))
    if roll_dist is not None and roll_off is not None:
        print(
            f"Roll distance: {float(roll_dist) * 1000.0:.1f} mm  "
            f"(end offset {float(roll_off) * 1000.0:.1f} mm)"
        )
    print(f"RF frames: {summary.get('frame_count', 0)}")
    print(f"RF coverage: {summary.get('coverage_ratio', 0.0) * 100.0:.1f}%")
    print(f"Expected RF samples/frame/channel: {session.get('expected_rf_samples', '?')}")
    if static_rf:
        sb_frames = len(static_rf.get("frames") or [])
        print(
            f"Static (stationary) RF baseline: {sb_frames} frames"
            f" (requested {static_rf.get('n_frames_requested', '?')})"
        )
    print(f"Compress duration: {trial.get('compress_duration_s', float('nan')):.2f} s")
    print(f"Slide duration: {trial.get('slide_duration_s', float('nan')):.2f} s")
    print(f"Decompress duration: {trial.get('decompress_duration_s', float('nan')):.2f} s")
    print("=" * 72)


def _style_ax(ax: plt.Axes, *, xlabel: str = "", ylabel: str = "", title: str = "") -> None:
    """Apply consistent dark styling to an axes."""
    ax.set_facecolor(PANEL)
    for spine in ax.spines.values():
        spine.set_edgecolor(BORDER)
        spine.set_linewidth(0.8)
    ax.tick_params(colors=DIM_TEXT, labelsize=7, length=3, width=0.6)
    if xlabel:
        ax.set_xlabel(xlabel, fontsize=7.5, color=DIM_TEXT, labelpad=4)
    if ylabel:
        ax.set_ylabel(ylabel, fontsize=7.5, color=DIM_TEXT, labelpad=4)
    if title:
        ax.set_title(title, fontsize=9, fontweight="bold", color=TEXT, pad=5)
    ax.grid(True, color=BORDER, linewidth=0.5, alpha=0.6)


def replay(payload: dict, fps: float) -> FuncAnimation:
    trial = payload["trial"]
    session = payload["session"]
    expected_samples = int(session["expected_rf_samples"])
    arr = _extract_rf_array(trial, expected_samples)
    ts = _frame_times(trial, expected_samples)
    pos_m   = _eef_position_series_m(trial, ts)
    rot_deg = _eef_rotation_series_deg(trial, ts)
    frc_N   = _eef_series(trial, ts, "force_xyz")
    vel_ms  = _eef_series(trial, ts, "velocity_xyz", none_as_nan=True)
    acc_ms2 = _eef_series(trial, ts, "acceleration_xyz", none_as_nan=True)
    cmd_pos_m   = _cmd_eef_series(trial, ts, "position")
    cmd_rot_deg = _cmd_eef_series(trial, ts, "orientation_euler_deg_xyz")
    n_frames, n_ch, n_samples = arr.shape

    compress_duration  = float(trial["compress_duration_s"])
    slide_duration     = float(trial["slide_duration_s"])
    decompress_duration = float(trial["decompress_duration_s"])
    rf_window_duration  = float(trial["rf_window_duration_s"])

    # ── figure & grid layout ─────────────────────────────────────────────────
    plt.rcParams.update({"font.size": 9, "font.family": "sans-serif"})
    fig = plt.figure(figsize=(20, 10), facecolor=BG)

    # Two outer columns: RF charts (left) | right panel (contact viz + info)
    outer = fig.add_gridspec(
        1, 2,
        width_ratios=[3.6, 1.4],
        wspace=0.06,
        left=0.055, right=0.975,
        top=0.92, bottom=0.07,
    )
    chart_gs = outer[0, 0].subgridspec(1, 4, wspace=0.35)
    axes = [fig.add_subplot(chart_gs[0, i]) for i in range(4)]

    # Right side: contact viz (tall) on top, info panel below
    right_gs = outer[0, 1].subgridspec(2, 1, height_ratios=[1.5, 1.7], hspace=0.28)
    contact_ax = fig.add_subplot(right_gs[0, 0])
    info_ax    = fig.add_subplot(right_gs[1, 0])

    # ── RF channel axes ──────────────────────────────────────────────────────
    x    = np.arange(n_samples)
    vmin = float(np.percentile(arr, 1))
    vmax = float(np.percentile(arr, 99))
    margin = (vmax - vmin) * 0.05
    lines = []
    for idx, ax in enumerate(axes):
        _style_ax(ax, ylabel="ADC" if idx == 0 else "",
                  xlabel="Sample index", title=CHANNEL_NAMES[idx])
        (line,) = ax.plot(x, arr[0, idx], lw=1.1, color=CH_COLORS[idx], alpha=0.92)
        ax.set_xlim(RF_PLOT_X_MIN, n_samples - 1)
        ax.set_ylim(vmin - margin, vmax + margin)
        lines.append(line)

    # ── contact visualisation ────────────────────────────────────────────────
    _style_ax(contact_ax, title="Contact Position")
    contact_ax.grid(False)
    contact_ax.set_xlim(0.0, 1.0)
    contact_ax.set_ylim(0.0, 1.0)
    contact_ax.set_xticks([])
    contact_ax.set_yticks([0.07, 0.5, 0.91], ["base", "mid", "tip"])
    contact_ax.tick_params(left=True, labelleft=True, colors=DIM_TEXT, labelsize=7.5)

    sensor_rect = FancyBboxPatch(
        (0.28, 0.06), 0.44, 0.88,
        boxstyle="round,pad=0.02",
        facecolor=SENSOR_FC, edgecolor=SENSOR_EC, linewidth=1.5, alpha=0.85,
    )
    contact_band = Rectangle(
        (0.30, 0.06), 0.40, 0.045,
        facecolor=BAND_CLR, edgecolor="#ff0000", linewidth=0.8, alpha=0.90,
    )
    contact_ax.add_patch(sensor_rect)
    contact_ax.add_patch(contact_band)

    phase_text = contact_ax.text(
        0.5, 0.975, "—",
        ha="center", va="top",
        fontsize=8.5, fontweight="bold", color=PHASE_CLR["slide"],
        transform=contact_ax.transAxes,
    )

    # ── info panel ───────────────────────────────────────────────────────────
    info_ax.set_facecolor(PANEL)
    for spine in info_ax.spines.values():
        spine.set_edgecolor(BORDER)
        spine.set_linewidth(0.8)
    info_ax.set_xticks([])
    info_ax.set_yticks([])
    info_ax.set_xlim(0, 1)
    info_ax.set_ylim(0, 1)
    info_ax.set_title("EEF State", fontsize=9, fontweight="bold", color=TEXT, pad=4)

    info_text = info_ax.text(
        0.07, 0.96, "",
        ha="left", va="top",
        fontsize=7.5,
        color=TEXT,
        fontfamily="monospace",
        transform=info_ax.transAxes,
        linespacing=1.45,
    )
    info_meas_lbl = info_ax.text(
        0.13, 0.846, "meas",
        ha="left", va="top", fontsize=7.5, color="#ff4444",
        fontfamily="monospace", transform=info_ax.transAxes,
    )

    # ── title ────────────────────────────────────────────────────────────────
    title = fig.suptitle(
        "",
        fontsize=11,
        color=TEXT,
        fontweight="bold",
        y=0.975,
    )

    interval_ms = int(1000.0 / max(1.0, fps))

    # ── helpers ──────────────────────────────────────────────────────────────
    def _contact_progress(t_now: float) -> float:
        if t_now <= compress_duration:
            return 0.0
        t_slide = t_now - compress_duration
        if t_slide <= slide_duration:
            return t_slide / max(slide_duration, 1e-6)
        return 1.0

    def _phase(t_now: float) -> str:
        if t_now <= compress_duration:
            return "compress"
        if t_now <= compress_duration + slide_duration:
            return "slide"
        if t_now <= compress_duration + slide_duration + decompress_duration:
            return "decompress"
        return "done"

    _phase_labels = {
        "compress":   "▼  compress",
        "slide":      "↑  slide",
        "decompress": "▲  decompress",
        "done":       "✓  done",
    }

    # ── animation update ─────────────────────────────────────────────────────
    def update(frame_idx: int):
        for ch_idx, line in enumerate(lines):
            line.set_ydata(arr[frame_idx, ch_idx])

        t_now    = float(ts[frame_idx]) if frame_idx < len(ts) else 0.0
        progress = float(np.clip(_contact_progress(t_now), 0.0, 1.0))
        ph       = _phase(t_now)

        # slide-band position (travels from base to tip)
        band_y = 0.06 + 0.84 * progress
        contact_band.set_y(band_y)
        phase_text.set_text(_phase_labels[ph])
        phase_text.set_color(PHASE_CLR.get(ph, TEXT))

        title.set_text(
            f"Rolling Replay  ·  {trial['speed_m_s'] * 1000.0:.1f} mm/s  ·  "
            f"frame {frame_idx + 1} / {n_frames}"
        )

        px, py, pz = pos_m[frame_idx]
        rx, ry, rz = rot_deg[frame_idx]
        fx, fy, fz = frc_N[frame_idx]
        vx, vy, vz = vel_ms[frame_idx]
        acx, acy, acz = acc_ms2[frame_idx]
        cpx, cpy, cpz = cmd_pos_m[frame_idx]
        crx, cry, crz = cmd_rot_deg[frame_idx]

        def _ff(v: float) -> str:
            return f"{v:+.2f}" if not np.isnan(v) else "  n/a"

        def _fv(v: float) -> str:
            return f"{v:+.5f}" if not np.isnan(v) else "   n/a  "

        def _fa(v: float) -> str:
            return f"{v:+.3f}" if not np.isnan(v) else "  n/a"

        def _ferr(m, c):
            return f"{(m-c)*1000:+.2f}" if not (np.isnan(m) or np.isnan(c)) else "  n/a"

        def _rerr(m, c):
            return f"{m-c:+.3f}" if not (np.isnan(m) or np.isnan(c)) else " n/a"

        info_text.set_text(
            f"t   {t_now:6.2f} / {rf_window_duration:.2f} s   [{ph}]\n"
            f"cmp {compress_duration:.2f}s  sld {slide_duration:.2f}s  dcmp {decompress_duration:.2f}s\n"
            f"\n"
            f"               cmd      err\n"
            f" x  {px:+.4f}  {cpx:+.4f}  {_ferr(px,cpx)} mm\n"
            f" y  {py:+.4f}  {cpy:+.4f}  {_ferr(py,cpy)} mm\n"
            f" z  {pz:+.4f}  {cpz:+.4f}  {_ferr(pz,cpz)} mm\n"
            f"\n"
            f"rx  {rx:+.2f}°  {crx:+.2f}°  {_rerr(rx,crx)}°\n"
            f"ry  {ry:+.2f}°  {cry:+.2f}°  {_rerr(ry,cry)}°\n"
            f"rz  {rz:+.2f}°  {crz:+.2f}°  {_rerr(rz,crz)}°\n"
            f"\n"
            f"Fx {_ff(fx)} N   vx {_fv(vx)} m/s\n"
            f"Fy {_ff(fy)} N   vy {_fv(vy)} m/s\n"
            f"Fz {_ff(fz)} N   vz {_fv(vz)} m/s"
        )
        return [*lines, contact_band, phase_text, info_text, title]

    ani = FuncAnimation(fig, update, frames=n_frames, interval=interval_ms, blit=False, repeat=True)
    plt.show()
    return ani


def _resolve_root(arg_path: Path | None, env_path: str | None) -> Path:
    """Decide which directory to scan for result folders.

    Priority:
      1. `path` argument if it's a directory (use as root)
      2. `--results-dir` flag (handled by argparse above this)
      3. ROLLING_RESULTS_DIR environment variable
      4. default RESULTS_DIR next to this script
    """
    if arg_path is not None and arg_path.is_dir():
        return arg_path
    if env_path:
        return Path(env_path).expanduser().resolve()
    return RESULTS_DIR


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Replay rolling-testing RF result files.\n\n"
            "Positional 'path' can be:\n"
            "  - a single .pkl file → replay it directly\n"
            "  - a directory that holds *.pkl files → pick one of those\n"
            "  - a directory of result folders (each with *.pkl) → pick folder then file\n"
            "If omitted, scans --results-dir (or $ROLLING_RESULTS_DIR, or the default\n"
            "results/ folder next to this script)."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("path", nargs="?", help="Optional .pkl file OR folder")
    parser.add_argument(
        "--results-dir",
        help=(
            "Root directory of result folders (overrides the default and the "
            "ROLLING_RESULTS_DIR env var).  Used only when 'path' is not a file."
        ),
    )
    parser.add_argument("--fps", type=float, default=20.0, help="Replay fps")
    args = parser.parse_args()

    arg_path = Path(args.path).expanduser().resolve() if args.path else None

    if arg_path is not None and arg_path.is_file():
        # Direct .pkl path
        path = arg_path
    else:
        # Decide root: explicit --results-dir, then env, then positional dir, then default
        if args.results_dir:
            root = Path(args.results_dir).expanduser().resolve()
        else:
            import os
            root = _resolve_root(arg_path, os.environ.get("ROLLING_RESULTS_DIR"))

        if not root.exists():
            raise SystemExit(f"Results root not found: {root}")

        # If the root itself contains .pkl files, treat it as a single trial folder.
        if any(root.glob("*.pkl")):
            trial_dir = root
        else:
            trial_dir = _pick_trial_dir(root)
        path = _pick_speed_file(trial_dir)

    payload = load_result(path)
    print_summary(path, payload)
    replay(payload, fps=args.fps)


if __name__ == "__main__":
    main()
