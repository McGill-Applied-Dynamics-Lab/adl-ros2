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
    trial_dirs = sorted([p for p in results_dir.glob("trial_*") if p.is_dir()])
    if not trial_dirs:
        raise FileNotFoundError(f"No trial folders found in {results_dir}")
    print("\nAvailable trials:")
    for i, path in enumerate(trial_dirs):
        print(f"  [{i}] {path.name}")
    raw = input(f"Select trial [0-{len(trial_dirs) - 1}]: ").strip()
    try:
        return trial_dirs[int(raw)]
    except (ValueError, IndexError):
        print("Invalid selection, using 0.")
        return trial_dirs[0]


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
    print("\n" + "=" * 72)
    print(f"File: {path}")
    print(f"Speed: {trial['speed_m_s'] * 1000.0:.1f} mm/s")
    print(f"RF frames: {summary.get('frame_count', 0)}")
    print(f"RF coverage: {summary.get('coverage_ratio', 0.0) * 100.0:.1f}%")
    print(f"Expected RF samples/frame/channel: {session.get('expected_rf_samples', '?')}")
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
    pos_m = _eef_position_series_m(trial, ts)
    rot_deg = _eef_rotation_series_deg(trial, ts)
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
    chart_gs = outer[0, 0].subgridspec(4, 1, hspace=0.55)
    axes = [fig.add_subplot(chart_gs[i, 0]) for i in range(4)]

    # Right side: contact viz (tall) on top, info panel below
    right_gs = outer[0, 1].subgridspec(2, 1, height_ratios=[2.2, 1.0], hspace=0.28)
    contact_ax = fig.add_subplot(right_gs[0, 0])
    info_ax    = fig.add_subplot(right_gs[1, 0])

    # ── RF channel axes ──────────────────────────────────────────────────────
    x    = np.arange(n_samples)
    vmin = float(np.percentile(arr, 1))
    vmax = float(np.percentile(arr, 99))
    margin = (vmax - vmin) * 0.05
    lines = []
    for idx, ax in enumerate(axes):
        xlabel = "Sample index" if idx == 3 else ""
        _style_ax(ax, ylabel="ADC", xlabel=xlabel, title=CHANNEL_NAMES[idx])
        (line,) = ax.plot(x, arr[0, idx], lw=1.1, color=CH_COLORS[idx], alpha=0.92)
        ax.set_xlim(0, n_samples - 1)
        ax.set_ylim(vmin - margin, vmax + margin)
        if idx < 3:
            ax.set_xticklabels([])
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
        fontsize=8.2,
        color=TEXT,
        fontfamily="monospace",
        transform=info_ax.transAxes,
        linespacing=1.65,
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

        info_text.set_text(
            f"t   {t_now:6.2f} / {rf_window_duration:.2f} s\n"
            f"\n"
            f"compress   {compress_duration:.2f} s\n"
            f"slide      {slide_duration:.2f} s\n"
            f"decompress {decompress_duration:.2f} s\n"
            f"\n"
            f"x   {px:+.4f} m\n"
            f"y   {py:+.4f} m\n"
            f"z   {pz:+.4f} m\n"
            f"\n"
            f"rx  {rx:+.2f}°\n"
            f"ry  {ry:+.2f}°\n"
            f"rz  {rz:+.2f}°"
        )
        return [*lines, contact_band, phase_text, info_text, title]

    ani = FuncAnimation(fig, update, frames=n_frames, interval=interval_ms, blit=False, repeat=True)
    plt.show()
    return ani


def main() -> None:
    parser = argparse.ArgumentParser(description="Replay rolling-testing RF result files.")
    parser.add_argument("path", nargs="?", help="Optional direct path to a per-speed result .pkl")
    parser.add_argument("--fps", type=float, default=20.0, help="Replay fps")
    args = parser.parse_args()

    if args.path:
        path = Path(args.path).expanduser().resolve()
    else:
        trial_dir = _pick_trial_dir(RESULTS_DIR)
        path = _pick_speed_file(trial_dir)

    payload = load_result(path)
    print_summary(path, payload)
    replay(payload, fps=args.fps)


if __name__ == "__main__":
    main()
