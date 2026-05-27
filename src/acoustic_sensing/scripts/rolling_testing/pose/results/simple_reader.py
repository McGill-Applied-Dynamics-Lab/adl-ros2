#!/usr/bin/env python3
"""Minimal example reader for a rolling-contact trial pickle.

Usage:
    python simple_reader.py path/to/rolling_speed_0010.0_mm_s_rep_00.pkl

Prints a short summary of the trial and the static-RF baseline. Use it as a
starting point for your own analysis — the loading code at the top is all
you need to get the data into numpy.

See README.md (same folder) for the full pickle schema.
"""
from __future__ import annotations

import pickle
import sys
from pathlib import Path

import numpy as np


def load_trial(path: Path) -> dict:
    """Load a rolling-trial pickle and return its `{trial, session}` dict."""
    with open(path, "rb") as f:
        return pickle.load(f)


def rf_frames_as_array(frames: list) -> tuple[np.ndarray, np.ndarray]:
    """Stack a `[(rf, t), ...]` list into (rf_array, t_array).

    Returns:
        rf  : np.ndarray, shape (n_frames, 4_channels, 1000_samples), float32
        t_s : np.ndarray, shape (n_frames,), seconds since capture start
    """
    if not frames:
        return np.empty((0, 4, 0), dtype=np.float32), np.empty((0,), dtype=float)
    rf = np.stack([np.stack(arrs, axis=0) for arrs, _t in frames], axis=0)
    t_s = np.array([t for _arrs, t in frames], dtype=float)
    return rf, t_s


def main() -> None:
    if len(sys.argv) != 2:
        print(__doc__.strip())
        sys.exit(1)

    path = Path(sys.argv[1]).expanduser().resolve()
    if not path.is_file():
        print(f"Not a file: {path}")
        sys.exit(1)

    data = load_trial(path)
    trial = data["trial"]
    session = data["session"]

    print(f"=== {path.name} ===")
    print(f"controller          : {session.get('controller')}")
    print(f"speed (mm/s)        : {trial['speed_m_s'] * 1000:.1f}   "
          f"[speed_idx={trial.get('speed_idx')}  repeat_idx={trial.get('repeat_idx')}]")
    print(f"roller diameter (m) : {trial.get('roller_diameter_m')}")
    print(f"roll distance (m)   : {trial['roll_distance_m']:.4f}   "
          f"(end offset {trial.get('roll_end_offset_m', 0.0)*1000:.1f} mm)")
    print(f"slide duration (s)  : {trial['slide_duration_s']:.2f}")

    # ── Rolling RF ────────────────────────────────────────────────────────
    rf, t_rf = rf_frames_as_array(trial["frames"])
    print()
    print("[rolling RF]")
    print(f"  frames            : {rf.shape[0]}   "
          f"(coverage {trial['rf_capture_summary']['coverage_ratio']*100:.1f}%)")
    if rf.size:
        print(f"  shape             : {rf.shape}  (n_frames, n_channels, n_samples)")
        print(f"  per-channel mean  : {rf.mean(axis=(0, 2)).tolist()}")

    # ── Static-RF baseline ────────────────────────────────────────────────
    static = trial.get("static_rf") or {}
    static_rf, _ = rf_frames_as_array(static.get("frames", []))
    print()
    print("[static-RF baseline]")
    print(f"  frames captured   : {static_rf.shape[0]} "
          f"(requested {static.get('n_frames_requested', 0)})")
    if static_rf.size:
        print(f"  shape             : {static_rf.shape}")
        print(f"  per-channel mean  : {static_rf.mean(axis=(0, 2)).tolist()}")

    # ── EE telemetry ──────────────────────────────────────────────────────
    ee = trial["ee_poses"]
    ee_slide = trial["ee_poses_slide_only"]
    print()
    print("[end-effector telemetry]")
    print(f"  total samples     : {len(ee)}")
    print(f"  slide-only samples: {len(ee_slide)}")
    if ee:
        forces = np.array([s["force_xyz"] for s in ee])
        print(f"  |F| max (N)       : {np.linalg.norm(forces, axis=1).max():.2f}")
        print(f"  Fz min/max (N)    : {forces[:, 2].min():.2f} / {forces[:, 2].max():.2f}")


if __name__ == "__main__":
    main()
