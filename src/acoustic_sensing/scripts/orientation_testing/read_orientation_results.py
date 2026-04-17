#!/usr/bin/env python3
"""Visualiser for orientation-testing result files.

Layout
------
Left  : 4 RF channel waveforms, animated through hold frames.
        Vertical dashed lines show the expected contact sample index
        derived geometrically from the entered bar angle.
Right : Angular sensor diagram (static) — sensor body, waveguides, bar
        at the entered angle, and measured contact points from the
        Hilbert envelope peak.
        Info panel below with EEF state and trial metadata.

Usage
-----
    python read_orientation_results.py [path/to/file.pkl] [--fps 15]
"""

from __future__ import annotations

import argparse
import pickle
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
from matplotlib.animation import FuncAnimation
from scipy.signal import hilbert

CHANNEL_NAMES = ("S0", "S1", "S2", "S3")
RESULTS_DIR = Path(__file__).resolve().parent / "results"

# ── colour palette (matches rolling reader) ───────────────────────────────────
BG        = "#12121e"
PANEL     = "#1c1c30"
BORDER    = "#2e2e50"
TEXT      = "#d4d4f0"
DIM_TEXT  = "#7070a0"
ACCENT    = "#4a90e2"
CH_COLORS = ["#00d4ff", "#ff7043", "#b388ff", "#69f0ae"]
SENSOR_FC = "#1e3558"
SENSOR_EC = "#4a90e2"
BAR_CLR   = "#ffcc44"

# Sensor geometry in display coords (normalised [0,1])
_SX0, _SX1 = 0.22, 0.78   # sensor left / right edges
_SY0, _SY1 = 0.08, 0.92   # sensor bottom / top edges
_SCX = (_SX0 + _SX1) / 2  # sensor centre x
_SCY = (_SY0 + _SY1) / 2  # sensor centre y


# ── file helpers ──────────────────────────────────────────────────────────────

def load_result(path: Path) -> dict:
    with open(path, "rb") as f:
        return pickle.load(f)


def _pick_trial_dir(results_dir: Path) -> Path:
    trial_dirs = sorted([p for p in results_dir.glob("trial_*") if p.is_dir()])
    if not trial_dirs:
        raise FileNotFoundError(f"No trial folders found in {results_dir}")
    print("\nAvailable trials:")
    for i, p in enumerate(trial_dirs):
        print(f"  [{i}] {p.name}")
    raw = input(f"Select trial [0-{len(trial_dirs)-1}]: ").strip()
    try:
        return trial_dirs[int(raw)]
    except (ValueError, IndexError):
        return trial_dirs[-1]


def _pick_pkl(trial_dir: Path) -> Path:
    # Prefer the subdirectory layout written by live_orientation_view.py.
    known_subdirs = ["full_exp", "20frames"]
    sub_pools = [(trial_dir / sd, sorted((trial_dir / sd).glob("*.pkl")))
                 for sd in known_subdirs
                 if (trial_dir / sd).is_dir() and any((trial_dir / sd).glob("*.pkl"))]

    if sub_pools:
        if len(sub_pools) > 1:
            print("\nFile sets:")
            for i, (sd, pkls) in enumerate(sub_pools):
                print(f"  [{i}] {sd.name}  ({len(pkls)} files)")
            raw = input(f"Select set [0-{len(sub_pools)-1}]: ").strip()
            try:
                chosen_dir, pkls = sub_pools[int(raw)]
            except (ValueError, IndexError):
                chosen_dir, pkls = sub_pools[0]
        else:
            chosen_dir, pkls = sub_pools[0]
    else:
        # Flat layout fallback
        pkls = sorted(trial_dir.glob("*.pkl"))
        if not pkls:
            raise FileNotFoundError(f"No .pkl files in {trial_dir}")
        chosen_dir = trial_dir

    print(f"\nFiles in {chosen_dir.name}:")
    for i, p in enumerate(pkls):
        print(f"  [{i}] {p.stem}")
    raw = input(f"Select file [0-{len(pkls)-1}]: ").strip()
    try:
        return pkls[int(raw)]
    except (ValueError, IndexError):
        return pkls[0]


# ── RF helpers ────────────────────────────────────────────────────────────────

def _valid_frames(payload: dict) -> tuple[np.ndarray, np.ndarray]:
    """Return (arr [N,4,S], timestamps [N]) for all structurally valid frames."""
    raw = payload.get("frames", [])
    if not raw:
        return np.empty((0, 4, 0), dtype=np.float32), np.empty(0)
    # infer expected sample count from majority
    lengths = [len(ch) for fd, _ in raw for ch in fd if isinstance(fd, list) and len(fd) == 4]
    if not lengths:
        return np.empty((0, 4, 0), dtype=np.float32), np.empty(0)
    expected = int(np.bincount(lengths).argmax())
    good_fd, good_ts = [], []
    for fd, t in raw:
        if not (isinstance(fd, list) and len(fd) == 4):
            continue
        if any(len(ch) != expected for ch in fd):
            continue
        good_fd.append(fd)
        good_ts.append(t)
    if not good_fd:
        return np.empty((0, 4, 0), dtype=np.float32), np.empty(0)
    return np.asarray(good_fd, dtype=np.float32), np.asarray(good_ts, dtype=float)


def _hilbert_peak(waveform: np.ndarray) -> float:
    """Return sample index of the Hilbert envelope peak."""
    env = np.abs(hilbert(waveform.astype(float)))
    return float(np.argmax(env))


# ── geometry ─────────────────────────────────────────────────────────────────

def _expected_contact_samples(
    angle_deg: float,
    waveguide_lateral_mm: np.ndarray,
    sample_pitch_mm: float,
    n_samples: int,
) -> np.ndarray:
    """Expected sample index of bar-waveguide intersection for each channel.

    Convention: angle=0 → bar perpendicular to gripper axis (horizontal),
    crossing all waveguides at the centre sample.  Positive angle tilts the
    bar so waveguides with larger lateral position see a higher sample index.
    """
    centre = n_samples / 2.0
    phi = np.radians(angle_deg)
    offsets_mm = waveguide_lateral_mm * np.tan(phi)
    return centre + offsets_mm / sample_pitch_mm


def _bar_endpoints_display(
    angle_deg: float,
    waveguide_lateral_mm: np.ndarray,
    sample_pitch_mm: float,
    n_samples: int,
) -> tuple[tuple[float, float], tuple[float, float]]:
    """Compute bar line endpoints in display coords.

    The sensor face spans _SX0.._SX1 laterally and _SY0.._SY1 along-waveguide.
    """
    phys_width  = float(waveguide_lateral_mm[-1] - waveguide_lateral_mm[0])  # mm
    phys_height = n_samples * sample_pitch_mm                                  # mm
    # extend bar slightly beyond sensor edges
    x_ext_mm = phys_width * 0.7
    phi = np.radians(angle_deg)

    def _to_disp(x_mm: float, y_mm: float) -> tuple[float, float]:
        xd = _SCX + (x_mm / phys_width) * (_SX1 - _SX0)
        yd = _SCY + (y_mm / phys_height) * (_SY1 - _SY0)
        return xd, yd

    x1, x2 = -x_ext_mm, x_ext_mm
    y1 = x1 * np.tan(phi)
    y2 = x2 * np.tan(phi)
    return _to_disp(x1, y1), _to_disp(x2, y2)


def _waveguide_x_display(
    waveguide_lateral_mm: np.ndarray,
) -> np.ndarray:
    """Display x coordinate for each waveguide."""
    w_min = waveguide_lateral_mm.min()
    w_max = waveguide_lateral_mm.max()
    return _SX0 + (waveguide_lateral_mm - w_min) / (w_max - w_min) * (_SX1 - _SX0)


def _sample_to_disp_y(sample_idx: float, n_samples: int) -> float:
    """Convert sample index to display y coord (0 = bottom = sample 0)."""
    return _SY0 + (sample_idx / max(n_samples - 1, 1)) * (_SY1 - _SY0)


# ── axes styling ──────────────────────────────────────────────────────────────

def _style_ax(ax, *, xlabel="", ylabel="", title="") -> None:
    ax.set_facecolor(PANEL)
    for sp in ax.spines.values():
        sp.set_edgecolor(BORDER)
        sp.set_linewidth(0.8)
    ax.tick_params(colors=DIM_TEXT, labelsize=7, length=3, width=0.6)
    if xlabel:
        ax.set_xlabel(xlabel, fontsize=7.5, color=DIM_TEXT, labelpad=4)
    if ylabel:
        ax.set_ylabel(ylabel, fontsize=7.5, color=DIM_TEXT, labelpad=4)
    if title:
        ax.set_title(title, fontsize=9, fontweight="bold", color=TEXT, pad=5)
    ax.grid(True, color=BORDER, linewidth=0.5, alpha=0.6)


# ── summary ───────────────────────────────────────────────────────────────────

def print_summary(path: Path, payload: dict) -> None:
    print("\n" + "=" * 72)
    print(f"File  : {path}")
    print(f"Angle : {payload.get('angle_deg', '?')} °")
    print(f"Depth : {payload.get('compress_depth_m', 0)*1000:.1f} mm")
    print(f"Hold  : {payload.get('hold_at_contact_sec', '?')} s")
    print(f"Frames: {len(payload.get('frames', []))}")
    ee = payload.get("ee_poses") or payload.get("ee_poses_during_rf", [])
    if ee:
        print(f"EEF samples: {len(ee)}  ({ee[0]['t']:.2f} – {ee[-1]['t']:.2f} s)")
    print("=" * 72)


# ── main visualiser ───────────────────────────────────────────────────────────

def visualise(payload: dict, fps: float) -> FuncAnimation | None:
    arr, ts = _valid_frames(payload)
    angle_deg = float(payload.get("angle_deg", 0.0))
    depth_mm  = float(payload.get("compress_depth_m", 0.0)) * 1000.0
    hold_sec  = float(payload.get("hold_at_contact_sec", 0.0))
    wg_mm     = np.asarray(payload.get("waveguide_lateral_positions_mm", [-4.5, -1.5, 1.5, 4.5]))
    pitch_mm  = float(payload.get("waveguide_sample_pitch_mm", 0.1))
    ee_poses     = payload.get("ee_poses") or payload.get("ee_poses_during_rf", [])
    cmd_ee_poses = payload.get("commanded_ee_poses") or payload.get("commanded_ee_poses_during_rf", [])

    has_rf = arr.ndim == 3 and arr.shape[0] > 0
    n_frames  = arr.shape[0] if has_rf else 1
    n_samples = arr.shape[2] if has_rf else 1000

    expected_samples = _expected_contact_samples(angle_deg, wg_mm, pitch_mm, n_samples)
    wg_x_disp = _waveguide_x_display(wg_mm)
    bar_p1, bar_p2 = _bar_endpoints_display(angle_deg, wg_mm, pitch_mm, n_samples)

    # ── figure & grid ─────────────────────────────────────────────────────────
    plt.rcParams.update({"font.size": 9, "font.family": "sans-serif"})
    fig = plt.figure(figsize=(20, 10), facecolor=BG)

    outer = fig.add_gridspec(
        1, 2, width_ratios=[3.6, 1.4], wspace=0.06,
        left=0.055, right=0.975, top=0.92, bottom=0.07,
    )
    chart_gs = outer[0, 0].subgridspec(1, 4, wspace=0.35)
    axes = [fig.add_subplot(chart_gs[0, i]) for i in range(4)]

    right_gs = outer[0, 1].subgridspec(2, 1, height_ratios=[1.5, 1.7], hspace=0.28)
    sensor_ax = fig.add_subplot(right_gs[0, 0])
    info_ax   = fig.add_subplot(right_gs[1, 0])

    # ── RF channel axes ───────────────────────────────────────────────────────
    x_idx = np.arange(n_samples)
    if has_rf:
        vmin = float(np.percentile(arr, 1))
        vmax = float(np.percentile(arr, 99))
    else:
        vmin, vmax = -1.0, 1.0
    margin = (vmax - vmin) * 0.05

    lines, vlines = [], []
    for i, ax in enumerate(axes):
        _style_ax(ax, ylabel="ADC" if i == 0 else "",
                  xlabel="Sample index",
                  title=CHANNEL_NAMES[i])
        ydata = arr[0, i] if has_rf else np.zeros(n_samples)
        (ln,) = ax.plot(x_idx, ydata, lw=1.1, color=CH_COLORS[i], alpha=0.92)
        ax.set_xlim(500, n_samples - 1)
        ax.set_ylim(vmin - margin, vmax + margin)
        # expected contact sample — dashed vertical line
        vl = ax.axvline(expected_samples[i], color=CH_COLORS[i],
                        lw=1.4, ls="--", alpha=0.7)
        lines.append(ln)
        vlines.append(vl)

    # ── sensor / angular diagram ──────────────────────────────────────────────
    _style_ax(sensor_ax, title=f"Bar angle: {angle_deg:+.1f}°")
    sensor_ax.grid(False)
    sensor_ax.set_xlim(0.0, 1.0)
    sensor_ax.set_ylim(0.0, 1.0)
    sensor_ax.set_xticks([])
    sensor_ax.set_yticks([])

    # sensor body
    sensor_rect = mpatches.FancyBboxPatch(
        (_SX0, _SY0), _SX1 - _SX0, _SY1 - _SY0,
        boxstyle="round,pad=0.01",
        facecolor=SENSOR_FC, edgecolor=SENSOR_EC, linewidth=1.5, alpha=0.85,
        transform=sensor_ax.transData, zorder=2,
    )
    sensor_ax.add_patch(sensor_rect)

    # waveguide lines (vertical, faint)
    for xi, color in zip(wg_x_disp, CH_COLORS):
        sensor_ax.plot([xi, xi], [_SY0 + 0.02, _SY1 - 0.02],
                       color=color, lw=0.9, alpha=0.45, zorder=3)

    # bar line at entered angle
    sensor_ax.plot([bar_p1[0], bar_p2[0]], [bar_p1[1], bar_p2[1]],
                   color=BAR_CLR, lw=2.2, alpha=0.9, zorder=4,
                   solid_capstyle="round")

    # angle arc label
    arc_r = 0.10
    arc_theta = np.linspace(np.pi / 2, np.pi / 2 - np.radians(angle_deg), 40)
    sensor_ax.plot(_SCX + arc_r * np.cos(arc_theta),
                   _SCY + arc_r * np.sin(arc_theta),
                   color=BAR_CLR, lw=1.2, alpha=0.7, zorder=5)
    sensor_ax.text(_SCX + arc_r * 1.3, _SCY + 0.01,
                   f"{angle_deg:+.1f}°",
                   fontsize=8, color=BAR_CLR, va="center", ha="left", zorder=6)

    # contact point markers (expected = open circle, measured = filled)
    exp_markers, meas_markers = [], []
    for xi, exp_s, color in zip(wg_x_disp, expected_samples, CH_COLORS):
        ey = _sample_to_disp_y(exp_s, n_samples)
        (em,) = sensor_ax.plot(xi, ey, "o", ms=7, mfc="none",
                               mec=color, mew=1.8, zorder=7)
        (mm,) = sensor_ax.plot(xi, ey, "o", ms=5, color=color,
                               alpha=0.0, zorder=8)   # hidden until RF available
        exp_markers.append(em)
        meas_markers.append(mm)

    # legend
    sensor_ax.plot([], [], "o", ms=6, mfc="none", mec=TEXT, mew=1.5,
                   label="expected", transform=sensor_ax.transAxes)
    sensor_ax.plot([], [], "o", ms=5, color=TEXT,
                   label="measured", transform=sensor_ax.transAxes)
    leg = sensor_ax.legend(loc="lower right", fontsize=7,
                           facecolor=PANEL, edgecolor=BORDER,
                           labelcolor=DIM_TEXT, handlelength=1.2)
    for txt in leg.get_texts():
        if txt.get_text() == "measured":
            txt.set_color("#ff4444")

    # gripper axis arrow (top centre → indicates sensor orientation)
    sensor_ax.annotate(
        "", xy=(_SCX, _SY1 + 0.04), xytext=(_SCX, _SY1 - 0.10),
        arrowprops=dict(arrowstyle="->", color=DIM_TEXT, lw=1.2),
        annotation_clip=False,
    )
    sensor_ax.text(_SCX + 0.04, _SY1 + 0.01, "gripper\naxis",
                   fontsize=6.5, color=DIM_TEXT, va="bottom", ha="left",
                   clip_on=False)

    # ── info panel ────────────────────────────────────────────────────────────
    info_ax.set_facecolor(PANEL)
    for sp in info_ax.spines.values():
        sp.set_edgecolor(BORDER); sp.set_linewidth(0.8)
    info_ax.set_xticks([]); info_ax.set_yticks([])
    info_ax.set_xlim(0, 1); info_ax.set_ylim(0, 1)
    info_ax.set_title("EEF State", fontsize=9, fontweight="bold", color=TEXT, pad=4)

    info_text = info_ax.text(
        0.07, 0.96, "",
        ha="left", va="top", fontsize=7.5, color=TEXT,
        fontfamily="monospace", transform=info_ax.transAxes, linespacing=1.45,
    )
    # Red caption for the measured-pose column; x/y are approximate for the
    # monospace layout — tweak if your DPI shifts it slightly.
    info_meas_lbl = info_ax.text(
        0.13, 0.884, "meas",
        ha="left", va="top", fontsize=7.5, color="#ff4444",
        fontfamily="monospace", transform=info_ax.transAxes,
        visible=False,
    )

    # ── title ─────────────────────────────────────────────────────────────────
    title_obj = fig.suptitle(
        f"Orientation Result  ·  angle {angle_deg:+.1f}°  ·  depth {depth_mm:.1f} mm",
        fontsize=11, color=TEXT, fontweight="bold", y=0.975,
    )

    # precompute ee pose lookup arrays
    if ee_poses:
        ee_ts  = np.asarray([p["t"] for p in ee_poses], dtype=float)
        ee_pos = np.asarray([p.get("position", [np.nan]*3) for p in ee_poses], dtype=float)
        ee_rot = np.asarray([p.get("orientation_euler_deg_xyz", [np.nan]*3) for p in ee_poses], dtype=float)
        ee_frc = np.asarray([p.get("force_xyz", [np.nan]*3) for p in ee_poses], dtype=float)
        ee_vel = np.asarray(
            [[np.nan]*3 if p.get("velocity_xyz") is None else p["velocity_xyz"] for p in ee_poses],
            dtype=float,
        )
        ee_acc = np.asarray(
            [[np.nan]*3 if p.get("acceleration_xyz") is None else p["acceleration_xyz"] for p in ee_poses],
            dtype=float,
        )
    else:
        ee_ts = ee_pos = ee_rot = ee_frc = ee_vel = ee_acc = None

    if cmd_ee_poses:
        cmd_ts  = np.asarray([p["t"] for p in cmd_ee_poses], dtype=float)
        cmd_pos = np.asarray([p.get("position", [np.nan]*3) for p in cmd_ee_poses], dtype=float)
        cmd_rot = np.asarray([p.get("orientation_euler_deg_xyz", [np.nan]*3) for p in cmd_ee_poses], dtype=float)
    else:
        cmd_ts = cmd_pos = cmd_rot = None

    # ── animation update ──────────────────────────────────────────────────────
    def update(frame_idx: int):
        if has_rf:
            for ch, (ln, vl) in enumerate(zip(lines, vlines)):
                ln.set_ydata(arr[frame_idx, ch])
                meas_s = _hilbert_peak(arr[frame_idx, ch])
                meas_markers[ch].set_ydata([_sample_to_disp_y(meas_s, n_samples)])
                meas_markers[ch].set_alpha(0.9)
                vl.set_xdata([meas_s, meas_s])

        t_now = float(ts[frame_idx]) if (has_rf and frame_idx < len(ts)) else 0.0
        title_obj.set_text(
            f"Orientation Result  ·  angle {angle_deg:+.1f}°  ·  "
            f"depth {depth_mm:.1f} mm  ·  "
            f"frame {frame_idx+1}/{n_frames}"
        )

        if ee_ts is not None and len(ee_ts) > 0:
            i = int(np.argmin(np.abs(ee_ts - t_now)))
            px, py, pz = ee_pos[i]
            rx, ry, rz = ee_rot[i]
            fx, fy, fz = ee_frc[i]
            vx, vy, vz = ee_vel[i]
            acx, acy, acz = ee_acc[i]

            if cmd_ts is not None and len(cmd_ts) > 0:
                ci = int(np.argmin(np.abs(cmd_ts - t_now)))
                cpx, cpy, cpz = cmd_pos[ci]
                crx, cry, crz = cmd_rot[ci]
            else:
                cpx = cpy = cpz = crx = cry = crz = float("nan")

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

            info_meas_lbl.set_visible(True)
            info_text.set_text(
                f"t      {t_now:6.2f} s\n"
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
        else:
            info_meas_lbl.set_visible(False)
            info_text.set_text(
                f"angle   {angle_deg:+.1f}°\n"
                f"depth   {depth_mm:.1f} mm\n"
                f"hold    {hold_sec:.1f} s\n"
                f"\n(no EEF data)"
            )

        return [*lines, *vlines, *meas_markers, info_text, title_obj]

    interval_ms = int(1000.0 / max(1.0, fps))
    ani = FuncAnimation(fig, update, frames=n_frames,
                        interval=interval_ms, blit=False, repeat=True)
    plt.show()
    return ani


# ── entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Visualise orientation-testing result files."
    )
    parser.add_argument("path", nargs="?", help="Direct path to a .pkl result file")
    parser.add_argument("--fps", type=float, default=10.0, help="Replay fps (default 10)")
    args = parser.parse_args()

    if args.path:
        path = Path(args.path).expanduser().resolve()
        payload = load_result(path)
        print_summary(path, payload)
        visualise(payload, fps=args.fps)
        return

    trial_dir = _pick_trial_dir(RESULTS_DIR)
    while True:
        path = _pick_pkl(trial_dir)
        payload = load_result(path)
        print_summary(path, payload)
        visualise(payload, fps=args.fps)
        raw = input("\nOpen another file? [y = same trial / t = new trial / n = quit]: ").strip().lower()
        if raw == "t":
            trial_dir = _pick_trial_dir(RESULTS_DIR)
        elif raw != "y":
            break


if __name__ == "__main__":
    main()
