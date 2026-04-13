#!/usr/bin/env python3
"""
Inspect timestamps for:
  - Robot forces (ee_forces) vs robot timestamps (exp["ts"])
  - Acoustic packets timestamps (t_rel_perf, t_abs_perf, BBB time_offset_ms)
Supports 1 or 2 RF channels (grouped by pkt["rf_id"]).
Prints per-probe summaries + optional detailed dump + simple alignment plots.

Usage:
  python inspect_timestamps_force_acoustic.py /path/to/your.pkl --probe 0 --plot
  python inspect_timestamps_force_acoustic.py /path/to/your.pkl --all
"""

import argparse
import pickle
from pathlib import Path
from typing import Dict, List, Any, Tuple

import numpy as np
import matplotlib.pyplot as plt


# ----------------------- Loading -----------------------

def load_exp(pkl_path: str | Path) -> dict:
    pkl_path = Path(pkl_path)
    with open(pkl_path, "rb") as f:
        return pickle.load(f)


# ----------------------- Helpers -----------------------

def _as_array(x):
    return np.asarray(x)


def _safe_ptp(x: np.ndarray) -> float:
    if x.size == 0:
        return float("nan")
    return float(np.nanmax(x) - np.nanmin(x))


def _stats_1d(t: np.ndarray) -> Dict[str, float]:
    t = np.asarray(t, dtype=float)
    t = t[np.isfinite(t)]
    if t.size == 0:
        return {"n": 0, "min": np.nan, "max": np.nan, "span": np.nan, "median_dt": np.nan, "mean_dt": np.nan}
    dt = np.diff(t) if t.size >= 2 else np.array([], dtype=float)
    return {
        "n": int(t.size),
        "min": float(t.min()),
        "max": float(t.max()),
        "span": float(t.max() - t.min()),
        "median_dt": float(np.median(dt)) if dt.size else np.nan,
        "mean_dt": float(dt.mean()) if dt.size else np.nan,
    }


def _extract_acoustic_by_rf(seg: dict) -> Dict[int, List[dict]]:
    """
    Supports:
      (A) seg["packets_by_rf"] = {"1":[...], "2":[...]} (or int keys)
      (B) seg["packets"] = [...] with pkt["rf_id"]
    Returns {rf_id: [packets...]}.
    """
    by_rf: Dict[int, List[dict]] = {}

    if "packets_by_rf" in seg:
        pbrf = seg.get("packets_by_rf", {})
        for k, v in pbrf.items():
            try:
                rf = int(k)
            except Exception:
                continue
            if isinstance(v, list):
                by_rf[rf] = v
        return by_rf

    pkts = seg.get("packets", [])
    for p in pkts:
        rf = int(p.get("rf_id", 1))
        by_rf.setdefault(rf, []).append(p)
    return by_rf


def _force_magnitudes(force_list: List[Any]) -> np.ndarray:
    """
    ee_forces entries are typically numpy arrays like [fx, fy, fz] or dict-like.
    We try to convert to Nx3 and compute norms.
    """
    if len(force_list) == 0:
        return np.array([], dtype=float)

    arr = []
    for f in force_list:
        if isinstance(f, dict):
            # try common keys
            if "force" in f:
                vec = np.asarray(f["force"], dtype=float).reshape(-1)
            else:
                # best effort: take first 3 values
                vec = np.asarray(list(f.values()), dtype=float).reshape(-1)
        else:
            vec = np.asarray(f, dtype=float).reshape(-1)
        if vec.size >= 3:
            arr.append(vec[:3])
        else:
            arr.append(np.pad(vec, (0, 3 - vec.size), constant_values=np.nan))
    arr = np.asarray(arr, dtype=float)
    mag = np.linalg.norm(arr, axis=1)
    return mag


def _print_probe_summary(exp: dict, probe_idx: int):
    print("\n" + "=" * 90)
    print(f"Probe {probe_idx}")
    print("=" * 90)

    # --- Robot timestamps + forces ---
    t_robot = np.asarray(exp["ts"][probe_idx], dtype=float) if "ts" in exp and probe_idx < len(exp["ts"]) else np.array([], dtype=float)
    forces = exp["ee_forces"][probe_idx] if "ee_forces" in exp and probe_idx < len(exp["ee_forces"]) else []
    fmag = _force_magnitudes(forces)

    s_tr = _stats_1d(t_robot)
    print("[Robot]")
    print(f"  samples: {s_tr['n']}")
    print(f"  t_robot: {s_tr['min']:.6f} → {s_tr['max']:.6f}  (span {s_tr['span']:.6f} s)")
    if np.isfinite(s_tr["median_dt"]):
        print(f"  dt: median {s_tr['median_dt']:.6f} s | mean {s_tr['mean_dt']:.6f} s")
    else:
        print("  dt: n/a")

    if fmag.size:
        print(f"  forces: N={fmag.size} | |F| min/mean/max = {np.nanmin(fmag):.3f} / {np.nanmean(fmag):.3f} / {np.nanmax(fmag):.3f}")
        if t_robot.size != fmag.size:
            print(f"  WARNING: len(ts)={t_robot.size} != len(ee_forces)={fmag.size} (sampling mismatch)")
    else:
        print("  forces: none")

    # --- Acoustic ---
    seg = exp["acoustic"][probe_idx] if "acoustic" in exp and probe_idx < len(exp["acoustic"]) else {}
    by_rf = _extract_acoustic_by_rf(seg)

    if len(by_rf) == 0:
        print("\n[Acoustic] NO DATA in this probe.")
        return

    print("\n[Acoustic]")
    for rf in sorted(by_rf.keys()):
        pkts = by_rf[rf]
        t_rel = np.asarray([p.get("t_rel_perf", np.nan) for p in pkts], dtype=float)
        t_abs = np.asarray([p.get("t_abs_perf", np.nan) for p in pkts], dtype=float)
        t_bbb_ms = np.asarray([p.get("time_offset_ms", np.nan) for p in pkts], dtype=float)

        s_rel = _stats_1d(t_rel)
        s_abs = _stats_1d(t_abs)

        # BBB stats: treat as ms; show span in seconds
        t_bbb_ms_f = t_bbb_ms[np.isfinite(t_bbb_ms)]
        if t_bbb_ms_f.size:
            bbb_min = float(t_bbb_ms_f.min())
            bbb_max = float(t_bbb_ms_f.max())
            bbb_span_s = (bbb_max - bbb_min) / 1000.0
            bbb_dt = np.diff(t_bbb_ms_f) if t_bbb_ms_f.size >= 2 else np.array([], dtype=float)
            bbb_med_dt_s = float(np.median(bbb_dt) / 1000.0) if bbb_dt.size else np.nan
        else:
            bbb_min = bbb_max = bbb_span_s = bbb_med_dt_s = np.nan

        print(f"  RF{rf}: packets={len(pkts)}")
        print(f"    t_rel_perf: {s_rel['min']:.6f} → {s_rel['max']:.6f} (span {s_rel['span']:.6f} s) | median dt {s_rel['median_dt']:.6f} s")
        print(f"    t_abs_perf: {s_abs['min']:.6f} → {s_abs['max']:.6f} (span {s_abs['span']:.6f} s) | median dt {s_abs['median_dt']:.6f} s")
        print(f"    BBB ms:     {bbb_min:.0f} → {bbb_max:.0f} (span {bbb_span_s:.6f} s) | median dt {bbb_med_dt_s:.6f} s")

    # --- Basic alignment check: robot span vs acoustic span ---
    if t_robot.size:
        rob_span = float(np.nanmax(t_robot) - np.nanmin(t_robot))
        print("\n[Alignment sanity]")
        print(f"  Robot span:   {rob_span:.6f} s")
        for rf in sorted(by_rf.keys()):
            t_rel = np.asarray([p.get("t_rel_perf", np.nan) for p in by_rf[rf]], dtype=float)
            t_rel = t_rel[np.isfinite(t_rel)]
            if t_rel.size:
                print(f"  Acoustic RF{rf} span (t_rel_perf): {float(t_rel.max()-t_rel.min()):.6f} s")
            else:
                print(f"  Acoustic RF{rf} span: n/a")


def _dump_detailed(exp: dict, probe_idx: int, limit: int = 20):
    seg = exp["acoustic"][probe_idx]
    by_rf = _extract_acoustic_by_rf(seg)

    print("\n" + "-" * 90)
    print(f"Detailed dump (first {limit} rows per RF) — Probe {probe_idx}")
    print("-" * 90)

    for rf in sorted(by_rf.keys()):
        pkts = by_rf[rf]
        print(f"\nRF{rf}:")
        for i, p in enumerate(pkts[:limit]):
            print(
                f"  {i:04d} | "
                f"t_rel_perf={p.get('t_rel_perf', np.nan):.6f} s | "
                f"t_abs_perf={p.get('t_abs_perf', np.nan):.6f} s | "
                f"time_offset_ms={int(p.get('time_offset_ms', -1))}"
            )


def _plot_alignment(exp: dict, probe_idx: int):
    # Robot
    t_robot = np.asarray(exp["ts"][probe_idx], dtype=float) if probe_idx < len(exp.get("ts", [])) else np.array([], dtype=float)
    forces = exp["ee_forces"][probe_idx] if probe_idx < len(exp.get("ee_forces", [])) else []
    fmag = _force_magnitudes(forces)

    # Acoustic
    seg = exp["acoustic"][probe_idx] if probe_idx < len(exp.get("acoustic", [])) else {}
    by_rf = _extract_acoustic_by_rf(seg)

    # Figure 1: Force magnitude vs robot t
    if t_robot.size and fmag.size:
        plt.figure(figsize=(9, 4))
        n = min(t_robot.size, fmag.size)
        plt.plot(t_robot[:n], fmag[:n], lw=1)
        plt.xlabel("Robot time since probe start (s)")
        plt.ylabel("|F| (N)")
        plt.title(f"Probe {probe_idx}: Robot force magnitude vs time")
        plt.grid(True)

    # Figure 2: Packet times (t_rel_perf) per RF as raster
    if len(by_rf):
        plt.figure(figsize=(9, 4))
        y_base = 0
        yticks = []
        yticklabels = []
        for k, rf in enumerate(sorted(by_rf.keys())):
            pkts = by_rf[rf]
            t_rel = np.asarray([p.get("t_rel_perf", np.nan) for p in pkts], dtype=float)
            t_rel = t_rel[np.isfinite(t_rel)]
            y = np.full_like(t_rel, y_base + k, dtype=float)
            plt.plot(t_rel, y, "|", markersize=10)
            yticks.append(y_base + k)
            yticklabels.append(f"RF{rf}")

        plt.yticks(yticks, yticklabels)
        plt.xlabel("Acoustic t_rel_perf (s)")
        plt.title(f"Probe {probe_idx}: Acoustic packet times (raster)")
        plt.grid(True)

    # Figure 3: Overlay robot t range with acoustic t range (span only)
    if t_robot.size and len(by_rf):
        plt.figure(figsize=(9, 2.8))
        rob_min, rob_max = float(np.nanmin(t_robot)), float(np.nanmax(t_robot))
        plt.plot([rob_min, rob_max], [0, 0], lw=6, solid_capstyle="butt")
        plt.text(rob_min, 0.08, "Robot span", va="bottom")

        for k, rf in enumerate(sorted(by_rf.keys())):
            t_rel = np.asarray([p.get("t_rel_perf", np.nan) for p in by_rf[rf]], dtype=float)
            t_rel = t_rel[np.isfinite(t_rel)]
            if t_rel.size:
                ac_min, ac_max = float(t_rel.min()), float(t_rel.max())
                plt.plot([ac_min, ac_max], [k + 1, k + 1], lw=6, solid_capstyle="butt")
                plt.text(ac_min, k + 1 + 0.08, f"RF{rf} span", va="bottom")

        plt.yticks([0] + [k + 1 for k in range(len(by_rf))], ["Robot"] + [f"RF{rf}" for rf in sorted(by_rf.keys())])
        plt.xlabel("Time (s)")
        plt.title(f"Probe {probe_idx}: Time span comparison")
        plt.grid(True)

    plt.show()


# ----------------------- Main CLI -----------------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("pkl", type=str, help="Path to experiment .pkl file")
    ap.add_argument("--probe", type=int, default=0, help="Probe index to inspect")
    ap.add_argument("--all", action="store_true", help="Inspect all probes")
    ap.add_argument("--detail", action="store_true", help="Print detailed per-packet timestamps (first N rows)")
    ap.add_argument("--detail_n", type=int, default=20, help="Rows to print per RF if --detail")
    ap.add_argument("--plot", action="store_true", help="Plot force vs time and acoustic timestamp rasters")
    args = ap.parse_args()

    exp = load_exp(args.pkl)

    if "ts" not in exp or "ee_forces" not in exp or "acoustic" not in exp:
        print("WARNING: expected keys ['ts','ee_forces','acoustic'] not all found in this pkl.")
        print(f"Keys present: {list(exp.keys())}")

    if args.all:
        n = len(exp.get("acoustic", []))
        for i in range(n):
            _print_probe_summary(exp, i)
            if args.detail:
                _dump_detailed(exp, i, limit=args.detail_n)
            if args.plot:
                _plot_alignment(exp, i)
    else:
        i = args.probe
        _print_probe_summary(exp, i)
        if args.detail:
            _dump_detailed(exp, i, limit=args.detail_n)
        if args.plot:
            _plot_alignment(exp, i)


if __name__ == "__main__":
    main()
