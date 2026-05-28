#!/usr/bin/env python3
"""Replay force-controlled rolling contact results.

Layout mirrors pose/read_rolling_results.py but swaps the roller-position
viz for an Fz-vs-time tracking plot (target band shaded) and adds
force-control diagnostics (termination reason, in-band %, Kp, z correction).
"""
from __future__ import annotations

import argparse
import pickle
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

CHANNEL_NAMES = ("S0", "S1", "S2", "S3")
RESULTS_DIR = Path(__file__).resolve().parent / "results"

BG        = "#12121e"
PANEL     = "#1c1c30"
BORDER    = "#2e2e50"
TEXT      = "#d4d4f0"
DIM_TEXT  = "#7070a0"
CH_COLORS = ["#00d4ff", "#ff7043", "#b388ff", "#69f0ae"]
BAND_CLR  = "#44ff99"
FZ_LINE   = "#ff3b30"
CURSOR    = "#ffcc44"
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


def _pick_file(trial_dir: Path) -> Path:
    pkls = sorted(trial_dir.glob("*.pkl"))
    if not pkls:
        raise FileNotFoundError(f"No result files found in {trial_dir}")
    print(f"\nAvailable files in {trial_dir.name}:")
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
        raise ValueError("No RF frames stored.")
    valid = []
    for frame_data, _t in frames:
        if len(frame_data) != len(CHANNEL_NAMES):
            continue
        if any(len(ch) != expected_samples for ch in frame_data):
            continue
        valid.append(frame_data)
    if not valid:
        raise ValueError("No structurally valid RF frames found.")
    return np.asarray(valid, dtype=np.float32)


def _frame_times(trial: dict, expected_samples: int) -> np.ndarray:
    ts = []
    for frame_data, t in trial.get("frames", []):
        if len(frame_data) != len(CHANNEL_NAMES):
            continue
        if any(len(ch) != expected_samples for ch in frame_data):
            continue
        ts.append(float(t))
    return np.asarray(ts, dtype=float)


def _ee_series(trial: dict, frame_times: np.ndarray, key: str, none_as_nan: bool = False) -> np.ndarray:
    ee_poses = trial.get("ee_poses", [])
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


def _ee_pos_series(trial: dict, frame_times: np.ndarray) -> np.ndarray:
    return _ee_series(trial, frame_times, "position")


def _ee_rot_series_deg(trial: dict, frame_times: np.ndarray) -> np.ndarray:
    return _ee_series(trial, frame_times, "orientation_euler_deg_xyz")


def _cmd_ee_series(trial: dict, frame_times: np.ndarray, key: str) -> np.ndarray:
    """Same as _ee_series but reads from commanded_ee_poses."""
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


def _force_control_series(trial: dict):
    """Return (t, fz, z_corr, kp, in_band, f_error) from ee_poses_slide_force."""
    sf = trial.get("ee_poses_slide_force", [])
    if not sf:
        return (np.array([]),) * 6
    t      = np.asarray([s["t"] for s in sf], dtype=float)
    fz     = np.asarray([s["fz_measured"] for s in sf], dtype=float)
    z_corr = np.asarray([s.get("z_correction_m", s.get("z_error_m", np.nan)) for s in sf], dtype=float)
    kp     = np.asarray([s.get("kp", np.nan) for s in sf], dtype=float)
    inband = np.asarray([bool(s.get("in_band", False)) for s in sf], dtype=bool)
    ferr   = np.asarray([s.get("f_error", np.nan) for s in sf], dtype=float)
    return t, fz, z_corr, kp, inband, ferr


def print_summary(path: Path, payload: dict) -> None:
    trial = payload["trial"]
    session = payload["session"]
    summary = trial.get("rf_capture_summary", {})
    static_rf = trial.get("static_rf") or {}
    sf = trial.get("ee_poses_slide_force", [])
    fz_lo = trial.get("force_lower_n", session.get("force_lower_n", float("nan")))
    fz_hi = trial.get("force_upper_n", session.get("force_upper_n", float("nan")))
    in_band_pct = (100.0 * sum(1 for s in sf if s.get("in_band")) / len(sf)) if sf else float("nan")
    print("\n" + "=" * 72)
    print(f"File: {path}")
    print(f"Slide speed: {trial.get('slide_speed_m_s', float('nan')) * 1000.0:.1f} mm/s")
    speed_idx = trial.get("speed_idx")
    rep_idx = trial.get("repeat_idx")
    if speed_idx is not None and rep_idx is not None:
        print(f"Speed/repeat index: speed_idx={speed_idx}, repeat_idx={rep_idx}")
    diam = trial.get("roller_diameter_m", session.get("roller_diameter_m"))
    if diam is not None:
        print(f"Roller diameter: {float(diam) * 1000.0:.1f} mm")
    print(f"Force band:  [{fz_lo:.1f}, {fz_hi:.1f}] N")
    print(f"Termination: {trial.get('termination_reason', '?')}")
    print(f"Roll distance: {trial.get('roll_distance_m', float('nan')) * 1000.0:.1f} mm")
    print(f"RF frames: {summary.get('frame_count', 0)}  coverage: {summary.get('coverage_ratio', 0.0) * 100:.1f}%")
    if static_rf:
        sb_frames = len(static_rf.get("frames") or [])
        print(
            f"Static (stationary) RF baseline: {sb_frames} frames"
            f" (requested {static_rf.get('n_frames_requested', '?')})"
        )
    print(f"Force-control samples: {len(sf)}  in-band: {in_band_pct:.1f}%")
    print(f"Compress duration: {trial.get('compress_duration_s', float('nan')):.2f} s")
    print(f"Decompress duration: {trial.get('decompress_duration_s', float('nan')):.2f} s")
    print("=" * 72)


def _style_ax(ax: plt.Axes, *, xlabel: str = "", ylabel: str = "", title: str = "") -> None:
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
    ts  = _frame_times(trial, expected_samples)
    pos_m   = _ee_pos_series(trial, ts)
    rot_deg = _ee_rot_series_deg(trial, ts)
    frc_N   = _ee_series(trial, ts, "force_xyz")
    vel_ms  = _ee_series(trial, ts, "velocity_xyz", none_as_nan=True)
    acc_ms2 = _ee_series(trial, ts, "acceleration_xyz", none_as_nan=True)
    cmd_pos_m   = _cmd_ee_series(trial, ts, "position")
    cmd_rot_deg = _cmd_ee_series(trial, ts, "orientation_euler_deg_xyz")
    n_frames, _n_ch, n_samples = arr.shape

    fz_lo = trial.get("force_lower_n", session.get("force_lower_n", -11.0))
    fz_hi = trial.get("force_upper_n", session.get("force_upper_n", -10.0))
    speed_m_s = trial.get("slide_speed_m_s", session.get("slide_speed_m_s", 0.0))
    termination = trial.get("termination_reason", "?")
    compress_duration = float(trial.get("compress_duration_s", 0.0))
    decompress_duration = float(trial.get("decompress_duration_s", 0.0))
    rf_window_duration = float(trial.get("rf_window_duration_s", ts[-1] if len(ts) else 0.0))

    fc_t, fc_fz, fc_zc, fc_kp, fc_inband, fc_ferr = _force_control_series(trial)
    slide_t_start = fc_t[0] if len(fc_t) else compress_duration
    slide_t_end   = fc_t[-1] if len(fc_t) else compress_duration

    plt.rcParams.update({"font.size": 9, "font.family": "sans-serif"})
    fig = plt.figure(figsize=(20, 10), facecolor=BG)

    outer = fig.add_gridspec(
        1, 2, width_ratios=[3.6, 1.4], wspace=0.06,
        left=0.055, right=0.975, top=0.92, bottom=0.07,
    )
    chart_gs = outer[0, 0].subgridspec(1, 4, wspace=0.35)
    axes = [fig.add_subplot(chart_gs[0, i]) for i in range(4)]

    right_gs  = outer[0, 1].subgridspec(3, 1, height_ratios=[1.5, 0.28, 1.7], hspace=0.22)
    fz_ax     = fig.add_subplot(right_gs[0, 0])
    roller_ax = fig.add_subplot(right_gs[1, 0])
    info_ax   = fig.add_subplot(right_gs[2, 0])

    x = np.arange(n_samples)
    vmin = float(np.percentile(arr, 1))
    vmax = float(np.percentile(arr, 99))
    margin = (vmax - vmin) * 0.05
    lines = []
    for idx, ax in enumerate(axes):
        _style_ax(ax, ylabel="ADC" if idx == 0 else "",
                  xlabel="Sample index", title=CHANNEL_NAMES[idx])
        (line,) = ax.plot(x, arr[0, idx], lw=1.1, color=CH_COLORS[idx], alpha=0.92)
        ax.set_xlim(350, n_samples - 1)
        ax.set_ylim(vmin - margin, vmax + margin)
        lines.append(line)

    _style_ax(fz_ax, xlabel="Time [s]", ylabel="Fz [N]", title="Force tracking")
    if len(fc_t):
        fz_ax.axhspan(fz_lo, fz_hi, color=BAND_CLR, alpha=0.15, linewidth=0)
        fz_ax.axhline(fz_lo, color=BAND_CLR, linewidth=0.7, linestyle="--", alpha=0.6)
        fz_ax.axhline(fz_hi, color=BAND_CLR, linewidth=0.7, linestyle="--", alpha=0.6)
        fz_ax.plot(fc_t, fc_fz, color=FZ_LINE, linewidth=1.0, alpha=0.9)
        pad = max(1.0, 0.15 * abs(fz_hi - fz_lo))
        fz_ax.set_xlim(fc_t[0], fc_t[-1])
        fz_ax.set_ylim(min(fz_lo, float(np.min(fc_fz))) - pad,
                       max(fz_hi, float(np.max(fc_fz))) + pad)
    else:
        fz_ax.text(0.5, 0.5, "no force-control telemetry",
                   ha="center", va="center", color=DIM_TEXT,
                   transform=fz_ax.transAxes)
    fz_cursor = fz_ax.axvline(0.0, color=CURSOR, linewidth=1.0, alpha=0.8)

    # ── Roller contact-position strip ─────────────────────────────────────────
    t_roll_l = float(fc_t[0])  if len(fc_t) else (float(ts[0])  if len(ts) else 0.0)
    t_roll_r = float(fc_t[-1]) if len(fc_t) else (float(ts[-1]) if len(ts) else 1.0)

    roller_ax.set_facecolor(PANEL)
    for spine in roller_ax.spines.values():
        spine.set_edgecolor(BORDER)
        spine.set_linewidth(0.7)
    roller_ax.set_xticks([])
    roller_ax.set_yticks([])
    roller_ax.set_xlim(t_roll_l, t_roll_r)
    roller_ax.set_ylim(-1, 1)
    roller_ax.set_title("contact position →", fontsize=7, color=DIM_TEXT, pad=2)
    # Track
    roller_ax.axhline(0, color=DIM_TEXT, linewidth=2.5, alpha=0.35,
                      solid_capstyle="round", zorder=1)
    # End caps
    roller_ax.plot([t_roll_l, t_roll_r], [0, 0], "|",
                   color=DIM_TEXT, markersize=8, markeredgewidth=1.0, alpha=0.5)
    # Moving roller marker
    roller_dot, = roller_ax.plot([t_roll_l], [0], "o",
                                  color=CURSOR, markersize=12,
                                  markeredgecolor=TEXT, markeredgewidth=0.8, zorder=5)

    info_ax.set_facecolor(PANEL)
    for spine in info_ax.spines.values():
        spine.set_edgecolor(BORDER)
        spine.set_linewidth(0.8)
    info_ax.set_xticks([])
    info_ax.set_yticks([])
    info_ax.set_xlim(0, 1)
    info_ax.set_ylim(0, 1)
    info_ax.set_title("EEF + Force State", fontsize=9, fontweight="bold", color=TEXT, pad=4)

    contact_z = float(trial.get("contact_position", [np.nan, np.nan, np.nan])[2])

    # Column x positions — freely chosen, no char-width estimation needed
    _X_LBL, _X_MEAS, _X_CMD, _X_ERR = 0.04, 0.22, 0.47, 0.72
    _kw = dict(ha="left", va="top", fontsize=7.5, fontfamily="monospace",
               transform=info_ax.transAxes, linespacing=1.45)

    txt_top = info_ax.text(_X_LBL, 0.97, "", color=TEXT, **_kw)

    # Column headers (static)
    info_ax.text(_X_MEAS, 0.76, "meas", color="#ff4444", ha="left", va="top",
                 fontsize=7.5, fontfamily="monospace", transform=info_ax.transAxes)
    info_ax.text(_X_CMD,  0.76, "cmd",  color=DIM_TEXT, ha="left", va="top",
                 fontsize=7.5, fontfamily="monospace", transform=info_ax.transAxes)
    info_ax.text(_X_ERR,  0.76, "err",  color=DIM_TEXT, ha="left", va="top",
                 fontsize=7.5, fontfamily="monospace", transform=info_ax.transAxes)

    # Row labels (static)
    info_ax.text(_X_LBL, 0.71, "x\ny\nz\n\nrx\nry\nrz\n\nΔz",
                 color=DIM_TEXT, **_kw)

    # Data columns (updated each frame)
    txt_meas = info_ax.text(_X_MEAS, 0.71, "", color="#ff4444", **_kw)
    txt_cmd  = info_ax.text(_X_CMD,  0.71, "", color=TEXT,      **_kw)
    txt_err  = info_ax.text(_X_ERR,  0.71, "", color=DIM_TEXT,  **_kw)

    # Force / velocity / force-control section (below data table)
    txt_frc = info_ax.text(_X_LBL, 0.35, "", color=TEXT, **_kw)

    title = fig.suptitle("", fontsize=11, color=TEXT, fontweight="bold", y=0.975)

    interval_ms = int(1000.0 / max(1.0, fps))

    def _phase(t_now: float) -> str:
        if t_now < slide_t_start:
            return "compress"
        if t_now <= slide_t_end:
            return "slide"
        if t_now <= slide_t_end + decompress_duration:
            return "decompress"
        return "done"

    def _fc_at(t_now: float):
        if len(fc_t) == 0:
            return (np.nan,) * 5
        i = int(np.clip(np.searchsorted(fc_t, t_now), 0, len(fc_t) - 1))
        return float(fc_fz[i]), float(fc_zc[i]), float(fc_kp[i]), bool(fc_inband[i]), float(fc_ferr[i])

    def update(frame_idx: int):
        for ch_idx, line in enumerate(lines):
            line.set_ydata(arr[frame_idx, ch_idx])

        t_now = float(ts[frame_idx]) if frame_idx < len(ts) else 0.0
        ph = _phase(t_now)
        fz_cursor.set_xdata([t_now, t_now])

        t_roller = float(np.clip(t_now, t_roll_l, t_roll_r))
        roller_dot.set_data([t_roller], [0])

        title.set_text(
            f"Force-Controlled Rolling Replay  ·  {speed_m_s*1000.0:.1f} mm/s  ·  "
            f"band=[{fz_lo:.1f},{fz_hi:.1f}] N  ·  "
            f"frame {frame_idx + 1} / {n_frames}  ·  termination: {termination}"
        )

        px, py, pz = pos_m[frame_idx]
        rx, ry, rz = rot_deg[frame_idx]
        fx, fy, fz_ee = frc_N[frame_idx]
        vx, vy, vz = vel_ms[frame_idx]
        cpx, cpy, cpz = cmd_pos_m[frame_idx]
        crx, cry, crz = cmd_rot_deg[frame_idx]
        fz_ctrl, z_corr, kp_now, inband, ferr = _fc_at(t_now)
        depth_mm = (contact_z - pz) * 1000.0 if not np.isnan(contact_z) else np.nan

        def _f(v, p=2):
            return f"{v:+.{p}f}" if not np.isnan(v) else "   n/a"
        def _pe(m, c):
            return f"{(m-c)*1000:+.2f}" if not (np.isnan(m) or np.isnan(c)) else "  n/a"
        def _re(m, c):
            return f"{m-c:+.3f}" if not (np.isnan(m) or np.isnan(c)) else " n/a"

        band_str = "IN " if inband else ("?? " if np.isnan(fz_ctrl) else "OUT")
        kp_str = f"{kp_now:.5f}" if not np.isnan(kp_now) else "   n/a"
        z_corr_mm = z_corr * 1000.0 if not np.isnan(z_corr) else np.nan

        txt_top.set_text(
            f"t   {t_now:6.2f} / {rf_window_duration:.2f} s   [{ph}]\n"
            f"cmp {compress_duration:.2f}s  dcmp {decompress_duration:.2f}s"
        )
        txt_meas.set_text(
            f"{px:+.4f}\n{py:+.4f}\n{pz:+.4f}\n\n"
            f"{rx:+.2f}°\n{ry:+.2f}°\n{rz:+.2f}°\n\n"
            f"{_f(depth_mm, 2)}"
        )
        txt_cmd.set_text(
            f"{cpx:+.4f}\n{cpy:+.4f}\n{cpz:+.4f}\n\n"
            f"{crx:+.2f}°\n{cry:+.2f}°\n{crz:+.2f}°\n\n"
        )
        txt_err.set_text(
            f"{_pe(px,cpx)} mm\n{_pe(py,cpy)} mm\n{_pe(pz,cpz)} mm\n\n"
            f"{_re(rx,crx)}°\n{_re(ry,cry)}°\n{_re(rz,crz)}°\n\n"
            f"mm"
        )
        txt_frc.set_text(
            f"     F[N]       v[m/s]\n"
            f"x  {_f(fx)}  {_f(vx,4)}\n"
            f"y  {_f(fy)}  {_f(vy,4)}\n"
            f"z  {_f(fz_ee)}  {_f(vz,4)}\n"
            f"\n"
            f"Force ctrl [{band_str}]  [{fz_lo:.1f},{fz_hi:.1f}] N\n"
            f" Fz(ctrl) {_f(fz_ctrl)} N   err {_f(ferr, 3)} N\n"
            f" z_corr   {_f(z_corr_mm, 3)} mm\n"
            f" Kp       {kp_str} m/N"
        )
        return [*lines, fz_cursor, roller_dot, txt_top, txt_meas, txt_cmd, txt_err, txt_frc, title]

    ani = FuncAnimation(fig, update, frames=n_frames, interval=interval_ms, blit=False, repeat=True)
    plt.show()
    return ani


def main() -> None:
    parser = argparse.ArgumentParser(description="Replay force-controlled rolling RF result files.")
    parser.add_argument("path", nargs="?", help="Optional direct path to a per-trial result .pkl")
    parser.add_argument("--fps", type=float, default=20.0, help="Replay fps")
    args = parser.parse_args()

    if args.path:
        path = Path(args.path).expanduser().resolve()
    else:
        trial_dir = _pick_trial_dir(RESULTS_DIR)
        path = _pick_file(trial_dir)

    payload = load_result(path)
    print_summary(path, payload)
    replay(payload, fps=args.fps)


if __name__ == "__main__":
    main()
