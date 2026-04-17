#!/usr/bin/env python3
"""Extract and smooth the Z-depth profile from the latest force-controlled rolling trial.

Reads the most recent trial .pkl, extracts z_correction_m vs y_traveled_m from the
force-control telemetry, fits a smooth spline, and saves a plot alongside a .npz
containing the smoothed (y, z_corr) arrays for use as a feedforward trajectory.
"""
from __future__ import annotations

import pickle
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import UnivariateSpline
from scipy.signal import savgol_filter

RESULTS_DIR = Path(__file__).resolve().parent / "results"
OUTPUT_DIR  = Path(__file__).resolve().parent / "depth_profiles"

# Savitzky-Golay pre-filter before spline fitting (removes high-freq noise)
SAVGOL_WINDOW = 51   # must be odd; larger = smoother
SAVGOL_POLY   = 3

# Spline smoothing factor (s): larger = smoother, less faithful to data
SPLINE_SMOOTH = 5e-6   # m² units (z_corr is in metres)

# Resolution of the output smoothed trajectory
N_INTERP = 500


def _latest_pkl(results_dir: Path) -> Path:
    pkls = sorted(results_dir.glob("trial_*/rolling_force_*.pkl"),
                  key=lambda p: p.stat().st_mtime)
    if not pkls:
        raise FileNotFoundError(f"No .pkl files found under {results_dir}")
    return pkls[-1]


def load_slide_data(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return (y_mm, z_corr_mm, fz_N) arrays from ee_poses_slide_force."""
    with open(path, "rb") as f:
        payload = pickle.load(f)
    sf = payload["trial"].get("ee_poses_slide_force", [])
    if not sf:
        raise ValueError("No force-control telemetry in this trial.")
    y      = np.array([s["y_traveled_m"]   for s in sf]) * 1000.0   # mm
    z_corr = np.array([s["z_correction_m"] for s in sf]) * 1000.0   # mm
    fz     = np.array([s["fz_measured"]    for s in sf])             # N
    return y, z_corr, fz


def smooth_profile(y: np.ndarray, z: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Return (y_smooth, z_smooth) on a uniform grid."""
    # 1) sort by Y (force control can have slight back-steps)
    order  = np.argsort(y)
    y_s, z_s = y[order], z[order]

    # 2) Savitzky-Golay to knock down frame-to-frame noise
    win = min(SAVGOL_WINDOW, len(z_s) if len(z_s) % 2 == 1 else len(z_s) - 1)
    z_sg = savgol_filter(z_s, window_length=win, polyorder=SAVGOL_POLY)

    # 3) Spline on the SG-filtered signal
    spl = UnivariateSpline(y_s, z_sg, s=SPLINE_SMOOTH * len(y_s), k=5)

    y_out = np.linspace(y_s[0], y_s[-1], N_INTERP)
    z_out = spl(y_out)
    return y_out, z_out


def plot_and_save(path: Path, y_raw: np.ndarray, z_raw: np.ndarray,
                  fz: np.ndarray, y_sm: np.ndarray, z_sm: np.ndarray) -> None:
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    fig.suptitle(f"Depth profile — {path.parent.name}/{path.stem}", fontsize=12)

    # ── top: Z correction ──────────────────────────────────────────────────
    ax = axes[0]
    ax.scatter(y_raw, z_raw, s=2, alpha=0.3, color="#60aaff", label="raw z_corr")
    ax.plot(y_sm, z_sm, color="#ff4444", linewidth=2, label="smoothed spline")
    ax.set_ylabel("Z correction [mm]")
    ax.legend(loc="upper left", fontsize=8)
    ax.grid(True, alpha=0.4)
    ax.invert_yaxis()   # deeper = more negative (display convention)
    ax.set_title("Z depth correction vs. Y travel")

    # ── bottom: Fz ────────────────────────────────────────────────────────
    ax2 = axes[1]
    ax2.plot(y_raw, fz, color="#aaaaaa", linewidth=0.8, alpha=0.7, label="Fz measured")
    ax2.axhline(fz.mean(), color="#ffcc44", linewidth=1.2, linestyle="--",
                label=f"mean {fz.mean():.1f} N")
    ax2.set_ylabel("Fz [N]")
    ax2.set_xlabel("Y traveled [mm]")
    ax2.legend(loc="upper right", fontsize=8)
    ax2.grid(True, alpha=0.4)
    ax2.set_title("Force vs. Y travel")

    plt.tight_layout()
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    out_png = OUTPUT_DIR / f"{path.parent.name}_{path.stem}_depth_profile.png"
    fig.savefig(out_png, dpi=150)
    print(f"  Plot saved: {out_png}")
    plt.show()


def main() -> None:
    pkl_path = _latest_pkl(RESULTS_DIR)
    print(f"Loading: {pkl_path}")

    y_raw, z_raw, fz = load_slide_data(pkl_path)
    print(f"  Samples: {len(y_raw)}  Y: {y_raw.min():.1f}–{y_raw.max():.1f} mm"
          f"  Z_corr: {z_raw.min():.2f}–{z_raw.max():.2f} mm")

    y_sm, z_sm = smooth_profile(y_raw, z_raw)

    # Save smoothed profile as .npz for use as feedforward trajectory
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    npz_path = OUTPUT_DIR / f"{pkl_path.parent.name}_{pkl_path.stem}_depth_profile.npz"
    np.savez(npz_path, y_mm=y_sm, z_corr_mm=z_sm)
    print(f"  Profile saved: {npz_path}")

    plot_and_save(pkl_path, y_raw, z_raw, fz, y_sm, z_sm)


if __name__ == "__main__":
    main()
