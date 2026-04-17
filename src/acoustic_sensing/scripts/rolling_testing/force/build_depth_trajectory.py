#!/usr/bin/env python3
"""Fit a k=5 smoothing spline to the Z-depth profile of a force-controlled rolling trial
and export a smooth robot trajectory (Y, Z_world) that can be used as a feedforward path.

Usage:
    python3 build_depth_trajectory.py                  # uses latest trial
    python3 build_depth_trajectory.py path/to/trial.pkl

Outputs (in depth_profiles/):
    <trial>_depth_spline.png   — plot of raw data + spline
    <trial>_depth_spline.npz   — arrays: y_mm, z_world_mm, z_rel_mm  (N_TRAJ points)
"""
from __future__ import annotations

import argparse
import pickle
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import UnivariateSpline
from scipy.signal import savgol_filter

# ── config ────────────────────────────────────────────────────────────────────
RESULTS_DIR   = Path(__file__).resolve().parent / "results"
OUTPUT_DIR    = Path(__file__).resolve().parent / "depth_profiles"

BASE_Z_MM     = 97.7      # CONTACT_POSITION_M[2] * 1000 from rolling_force_config.py

SAVGOL_WIN    = 101       # pre-filter window (odd) before spline fitting
SAVGOL_POLY   = 3

SPLINE_K      = 5         # spline degree
SPLINE_S_FACTOR = 0.015   # s = factor * n_samples  (larger = smoother)

N_TRAJ        = 500       # number of waypoints in output trajectory


# ── helpers ───────────────────────────────────────────────────────────────────

def _latest_pkl(results_dir: Path) -> Path:
    pkls = sorted(results_dir.glob("trial_*/rolling_force_*.pkl"),
                  key=lambda p: p.stat().st_mtime)
    if not pkls:
        raise FileNotFoundError(f"No .pkl files under {results_dir}")
    return pkls[-1]


def load_slide_data(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    """Return (y_mm, z_rel_mm, fz_N, z_initial_mm)."""
    with open(path, "rb") as f:
        payload = pickle.load(f)
    sf = payload["trial"].get("ee_poses_slide_force", [])
    if not sf:
        raise ValueError("No force-control telemetry found in this trial.")

    y_mm        = np.array([s["y_traveled_m"]   for s in sf]) * 1000.0
    z_corr_mm   = np.array([s["z_correction_m"] for s in sf]) * 1000.0
    fz_n        = np.array([s["fz_measured"]    for s in sf])

    z_initial_mm = sf[0]["z_cmd"] * 1000.0 - sf[0]["z_correction_m"] * 1000.0
    z_actual_mm  = z_initial_mm + z_corr_mm
    z_rel_mm     = z_actual_mm - BASE_Z_MM     # negative = below base contact Z

    return y_mm, z_rel_mm, fz_n, z_initial_mm


def fit_spline(y_mm: np.ndarray, z_rel_mm: np.ndarray
               ) -> tuple[UnivariateSpline, np.ndarray, np.ndarray]:
    """Fit k=5 smoothing spline.  Returns (spline, y_out, z_out)."""
    order  = np.argsort(y_mm)
    y_s    = y_mm[order]
    z_s    = z_rel_mm[order]

    # Savitzky-Golay pre-filter
    win    = min(SAVGOL_WIN, len(z_s) if len(z_s) % 2 == 1 else len(z_s) - 1)
    z_sg   = savgol_filter(z_s, window_length=win, polyorder=SAVGOL_POLY)

    spl    = UnivariateSpline(y_s, z_sg, k=SPLINE_K, s=SPLINE_S_FACTOR * len(y_s))

    y_out  = np.linspace(y_s[0], y_s[-1], N_TRAJ)
    z_out  = spl(y_out)
    return spl, y_out, z_out


def plot(path: Path,
         y_raw: np.ndarray, z_rel_raw: np.ndarray, fz: np.ndarray,
         y_out: np.ndarray, z_out: np.ndarray) -> None:

    fig, axes = plt.subplots(2, 1, figsize=(13, 8), sharex=True)
    fig.suptitle(f"Depth trajectory spline — {path.parent.name}/{path.stem}", fontsize=12)

    ax = axes[0]
    ax.scatter(y_raw, z_rel_raw, s=2, alpha=0.25, color="#60aaff", label="raw z_rel")
    ax.plot(y_out, z_out, color="#ff4444", linewidth=2.5, label=f"k={SPLINE_K} spline (trajectory)")
    ax.axhline(0, color="white", linewidth=1.0, linestyle="--", alpha=0.5,
               label=f"base contact Z ({BASE_Z_MM} mm)")
    ax.set_ylabel("Z relative to base contact [mm]")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.4)
    ax.set_title(f"Z range: {z_out.min():.1f} to {z_out.max():.1f} mm  "
                 f"|  world Z range: {BASE_Z_MM+z_out.min():.1f} to {BASE_Z_MM+z_out.max():.1f} mm")

    ax2 = axes[1]
    ax2.plot(y_raw, fz, color="#aaaaaa", linewidth=0.8, alpha=0.7, label="Fz measured")
    ax2.axhline(fz.mean(), color="#ffcc44", linewidth=1.2, linestyle="--",
                label=f"mean {fz.mean():.1f} N")
    ax2.set_ylabel("Fz [N]")
    ax2.set_xlabel("Y traveled [mm]")
    ax2.legend(fontsize=8)
    ax2.grid(True, alpha=0.4)

    plt.tight_layout()

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    out = OUTPUT_DIR / f"{path.parent.name}_{path.stem}_depth_spline.png"
    fig.savefig(out, dpi=150)
    print(f"  Plot saved: {out}")
    plt.show()


def build_trajectory(y_out: np.ndarray, z_out: np.ndarray
                     ) -> tuple[np.ndarray, np.ndarray]:
    """Return (y_mm, z_world_mm) trajectory arrays.

    z_world_mm = BASE_Z_MM + z_rel_mm  (absolute world-frame Z for the robot).
    Feed these into the force-controlled slide as a Z feedforward correction.
    """
    z_world = BASE_Z_MM + z_out
    return y_out, z_world


def save_trajectory(path: Path, y_mm: np.ndarray,
                    z_world_mm: np.ndarray, z_rel_mm: np.ndarray) -> Path:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    out = OUTPUT_DIR / f"{path.parent.name}_{path.stem}_depth_spline.npz"
    np.savez(out,
             y_mm=y_mm,
             z_world_mm=z_world_mm,
             z_rel_mm=z_rel_mm,
             base_z_mm=np.array([BASE_Z_MM]))
    print(f"  Trajectory saved: {out}")
    print(f"  Waypoints: {len(y_mm)}  |  "
          f"Y: {y_mm[0]:.1f}–{y_mm[-1]:.1f} mm  |  "
          f"Z world: {z_world_mm.min():.1f}–{z_world_mm.max():.1f} mm")
    return out


# ── main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("path", nargs="?", help="Path to .pkl file (default: latest trial)")
    args = parser.parse_args()

    pkl_path = Path(args.path).resolve() if args.path else _latest_pkl(RESULTS_DIR)
    print(f"Loading: {pkl_path}")

    y_raw, z_rel_raw, fz, z_initial_mm = load_slide_data(pkl_path)
    print(f"  Samples: {len(y_raw)}  |  z_initial: {z_initial_mm:.2f} mm  |  "
          f"z_rel: {z_rel_raw.min():.1f} to {z_rel_raw.max():.1f} mm")

    spl, y_out, z_out = fit_spline(y_raw, z_rel_raw)

    y_traj, z_world_traj = build_trajectory(y_out, z_out)
    save_trajectory(pkl_path, y_traj, z_world_traj, z_out)

    plot(pkl_path, y_raw, z_rel_raw, fz, y_out, z_out)


if __name__ == "__main__":
    main()
