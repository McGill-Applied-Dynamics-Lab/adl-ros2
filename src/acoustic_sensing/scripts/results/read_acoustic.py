#!/usr/bin/env python3
import pickle
from pathlib import Path
from typing import Union, Dict, List, Any

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


def load_exp(pkl_path: Union[str, Path]) -> dict:
    pkl_path = Path(pkl_path)
    with open(pkl_path, "rb") as f:
        return pickle.load(f)


def _extract_packets_by_channel(seg: dict) -> Dict[int, List[dict]]:
    """
    Supports both formats:
      (A) new: seg["packets_by_rf"] = {"1":[...], "2":[...]} (or int keys)
      (B) old: seg["packets"] = [...] where each pkt has pkt["rf_id"]
    Returns {rf_id: [packets...]} with rf_id as int.
    """
    by_rf: Dict[int, List[dict]] = {}

    if "packets_by_rf" in seg:
        pbrf = seg.get("packets_by_rf", {})
        for k, v in pbrf.items():
            try:
                rf = int(k)
            except Exception:
                continue
            if isinstance(v, list) and len(v) > 0:
                by_rf[rf] = v
        return by_rf

    pkts = seg.get("packets", [])
    for p in pkts:
        rf = int(p.get("rf_id", 1))
        by_rf.setdefault(rf, []).append(p)
    return by_rf


def get_packets(exp: dict, probe_idx: int) -> Dict[int, List[dict]]:
    seg = exp["acoustic"][probe_idx]
    by_rf = _extract_packets_by_channel(seg)
    if len(by_rf) == 0:
        raise ValueError(f"Probe {probe_idx}: no acoustic packets stored.")
    return by_rf


def _frames_from_packets(pkts: List[dict], mode: str) -> List[np.ndarray]:
    if len(pkts) == 0:
        return []

    if mode == "processed":
        frames = [np.asarray(p["waveform"], dtype=np.float32) for p in pkts]
        return frames

    if mode == "raw":
        if "samples_u16" not in pkts[0]:
            raise ValueError("Raw samples not found (samples_u16 missing). Re-run recorder with store_raw=True.")
        frames = [np.asarray(p["samples_u16"], dtype=np.uint16).astype(np.int16).astype(np.float32) for p in pkts]
        return frames

    raise ValueError("mode must be 'processed' or 'raw'")


def _robust_ylims(frames: List[np.ndarray], fallback=(0.0, 1.0)):
    if len(frames) == 0:
        return fallback
    sample_idx = np.linspace(0, len(frames) - 1, num=min(30, len(frames)), dtype=int)
    stack = np.concatenate([frames[i].ravel() for i in sample_idx])
    lo, hi = np.percentile(stack, [1, 99])
    pad = 0.1 * max(1e-6, (hi - lo))
    return (lo - pad, hi + pad)


def replay_probe(
    exp: dict,
    probe_idx: int,
    mode: str = "processed",   # "processed" or "raw"
    fps: float = 30.0,
    repeat: bool = True,
    ylims=None,                # None or dict like {1:(..), 2:(..)} or tuple(..)
):
    by_rf = get_packets(exp, probe_idx)
    rf_ids = sorted(by_rf.keys())

    # Detect 1 vs 2 channels (or more)
    if len(rf_ids) == 1:
        layout = "single"
    else:
        layout = "stacked"

    # Build frames per channel
    frames_by_rf: Dict[int, List[np.ndarray]] = {}
    t_rel_by_rf: Dict[int, np.ndarray] = {}
    title_mode = "processed waveform" if mode == "processed" else "raw samples (int16-like)"

    for rf in rf_ids:
        pkts = by_rf[rf]
        frames = _frames_from_packets(pkts, mode=mode)
        if len(frames) == 0:
            continue
        frames_by_rf[rf] = frames
        t_rel_by_rf[rf] = np.array([p.get("t_rel_perf", np.nan) for p in pkts], dtype=float)

    if len(frames_by_rf) == 0:
        raise ValueError(f"Probe {probe_idx}: no valid frames found for mode='{mode}'.")

    # Choose x from first available channel
    first_rf = sorted(frames_by_rf.keys())[0]
    x = np.arange(frames_by_rf[first_rf][0].shape[0], dtype=int)

    # y-lims handling:
    # - if ylims is dict: use per-channel
    # - if ylims is tuple: use for all
    # - if None: compute robust per-channel
    if isinstance(ylims, dict):
        ylims_by_rf = {int(k): v for k, v in ylims.items()}
    elif isinstance(ylims, tuple) and len(ylims) == 2:
        ylims_by_rf = {rf: ylims for rf in frames_by_rf.keys()}
    else:
        ylims_by_rf = {rf: _robust_ylims(frames_by_rf[rf]) for rf in frames_by_rf.keys()}

    interval_ms = int(1000.0 / max(1e-6, fps))

    # ---------- Figure(s) ----------
    if layout == "single":
        rf = first_rf
        frames = frames_by_rf[rf]
        t_rel = t_rel_by_rf[rf]

        fig, ax = plt.subplots(figsize=(9, 4))
        (line,) = ax.plot(x, frames[0], lw=1)
        ax.set_xlim(x[0], x[-1])
        ax.set_ylim(*ylims_by_rf[rf])
        ax.set_xlabel("Sample index")
        ax.set_ylabel("Amplitude")
        ttl = ax.set_title("")
        txt = ax.text(0.01, 0.95, "", transform=ax.transAxes, va="top", ha="left")

        def update(i):
            y = frames[i]
            line.set_ydata(y)
            ttl.set_text(f"Probe {probe_idx} | RF{rf} | {title_mode} | frame {i+1}/{len(frames)}")
            t = t_rel[i] if i < len(t_rel) else np.nan
            txt.set_text(f"t_rel_perf = {t:.3f} s" if np.isfinite(t) else "")
            return line, ttl, txt

        ani = FuncAnimation(fig, update, frames=len(frames), interval=interval_ms, blit=False, repeat=repeat)
        plt.tight_layout()
        plt.show()
        return ani

    # stacked: two axes (top/bottom). If >2 channels exist, plot first two.
    rf_top = rf_ids[0]
    rf_bot = rf_ids[1]

    frames_top = frames_by_rf.get(rf_top, [])
    frames_bot = frames_by_rf.get(rf_bot, [])
    if len(frames_top) == 0 or len(frames_bot) == 0:
        # fall back: pick any two channels that have frames
        avail = [rf for rf in rf_ids if len(frames_by_rf.get(rf, [])) > 0]
        if len(avail) < 2:
            # back to single
            return replay_probe(exp, probe_idx, mode=mode, fps=fps, repeat=repeat, ylims=ylims)
        rf_top, rf_bot = avail[0], avail[1]
        frames_top, frames_bot = frames_by_rf[rf_top], frames_by_rf[rf_bot]

    t_top = t_rel_by_rf[rf_top]
    t_bot = t_rel_by_rf[rf_bot]

    n_frames = min(len(frames_top), len(frames_bot))  # keep sync, avoid mismatch

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)

    (line1,) = ax1.plot(x, frames_top[0], lw=1)
    (line2,) = ax2.plot(x, frames_bot[0], lw=1)

    ax1.set_ylim(*ylims_by_rf.get(rf_top, _robust_ylims(frames_top)))
    ax2.set_ylim(*ylims_by_rf.get(rf_bot, _robust_ylims(frames_bot)))

    ax2.set_xlabel("Sample index")
    ax1.set_ylabel("Amplitude")
    ax2.set_ylabel("Amplitude")

    ttl = fig.suptitle("", y=0.98)
    txt1 = ax1.text(0.01, 0.92, "", transform=ax1.transAxes, va="top", ha="left")
    txt2 = ax2.text(0.01, 0.92, "", transform=ax2.transAxes, va="top", ha="left")

    ax1.set_title(f"RF{rf_top}", loc="left")
    ax2.set_title(f"RF{rf_bot}", loc="left")

    def update(i):
        line1.set_ydata(frames_top[i])
        line2.set_ydata(frames_bot[i])

        t1 = t_top[i] if i < len(t_top) else np.nan
        t2 = t_bot[i] if i < len(t_bot) else np.nan

        ttl.set_text(f"Probe {probe_idx} | {title_mode} | frame {i+1}/{n_frames}")
        txt1.set_text(f"t_rel_perf = {t1:.3f} s" if np.isfinite(t1) else "")
        txt2.set_text(f"t_rel_perf = {t2:.3f} s" if np.isfinite(t2) else "")
        return line1, line2, ttl, txt1, txt2

    ani = FuncAnimation(fig, update, frames=n_frames, interval=interval_ms, blit=False, repeat=repeat)
    plt.tight_layout()
    plt.show()
    return ani


if __name__ == "__main__":
    # 1) set this
    pkl_path = "/home/ros/ros2_ws/src/adl-ros2/src/acoustic_sensing/scripts/results/7_grid_TEST_01.pkl"
    exp = load_exp(pkl_path)

    # 2) choose which probe
    probe_idx = 4

    # 3) replay processed waveforms
    replay_probe(exp, probe_idx, mode="processed", fps=30.0)

    # If you stored raw, you can also do:
    # replay_probe(exp, probe_idx, mode="raw", fps=30.0)
