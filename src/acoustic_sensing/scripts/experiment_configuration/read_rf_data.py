#!/usr/bin/env python3
"""Interactive reader and live-replay tool for RF probing pickle files."""

import hashlib
import pickle
import time
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np

CHANNEL_NAMES = ("S0", "S1", "S2", "S3")
REPLAY_INTERVAL_MS = 200


def _rf_sha(frames: list) -> str:
    buf = bytearray()
    for frame in frames:
        for ch in frame:
            buf += np.array(ch, dtype=np.int32).tobytes()
    return hashlib.sha256(buf).hexdigest()[:16]


def _results_dir() -> Path:
    return Path(__file__).resolve().parent / "results"


def _load(path: Path) -> dict:
    """Load a pkl file written in either the old (single dump) or new (append) format."""
    records = []
    with open(path, "rb") as f:
        while True:
            try:
                records.append(pickle.load(f))
            except EOFError:
                break

    if not records:
        raise ValueError(f"Empty file: {path}")

    # Legacy single-dump format: one dict with grid_positions/rf_data at top level
    if len(records) == 1:
        return records[0]

    # New append format: first record is the header, rest are per-location dicts
    header = records[0]
    data = {
        "set_name":       header.get("set_name", ""),
        "landmarks":      header.get("landmarks", {}),
        "z_offset":       header.get("z_offset", 0.0),
        "probe_depth":    header.get("probe_depth", []),
        "grid_positions": [],
        "rf_data":        [],
        "robot_data":     [],
    }
    for rec in records[1:]:
        data["grid_positions"].append(rec["grid_position"])
        data["rf_data"].append(rec["rf_data"])
        data["robot_data"].append(rec.get("robot_data", []))
    return data


def _has_robot_data(data: dict) -> bool:
    return bool(data.get("robot_data"))


# ---------------------------------------------------------------------------
# Step 1: pick a pkl file
# ---------------------------------------------------------------------------

def pick_file() -> Path:
    results_dir = _results_dir()
    pkls = sorted(results_dir.glob("*.pkl"))
    if not pkls:
        raise FileNotFoundError(f"No .pkl files found in {results_dir}")

    print(f"\nFound {len(pkls)} result file(s) in {results_dir}:\n")
    for i, p in enumerate(pkls):
        size_kb = p.stat().st_size / 1024
        print(f"  [{i}] {p.name}  ({size_kb:.1f} KB)")

    print()
    while True:
        raw = input("Select file number: ").strip()
        if raw.isdigit() and 0 <= int(raw) < len(pkls):
            return pkls[int(raw)]
        print(f"  Enter a number between 0 and {len(pkls) - 1}.")


# ---------------------------------------------------------------------------
# Step 2: pick a location
# ---------------------------------------------------------------------------

def pick_location(data: dict) -> int:
    n_loc = len(data["grid_positions"])
    n_rf  = len(data["rf_data"])
    probe_depth_mm = [round(d * 1000.0, 3) for d in data["probe_depth"]]

    print(f"\nSet: {data['set_name']}  |  depths: {probe_depth_mm} mm  |  "
          f"{n_loc} grid positions, {n_rf} with RF data")
    print(f"Robot data: {'yes' if _has_robot_data(data) else 'no'}\n")

    print(f"  {'#':>4}  {'XY gripper (mm)':>24}  frames/depth")
    print("  " + "-" * 50)
    for i, xy in enumerate(data["grid_positions"]):
        xy_mm = [round(v * 1000.0, 2) for v in xy]
        if i < n_rf:
            frames_per_depth = [len(d) for d in data["rf_data"][i]]
        else:
            frames_per_depth = []
        print(f"  {i:>4}  {str(xy_mm):>24}  {frames_per_depth}")

    print()
    while True:
        raw = input(f"Select location to replay [0-{n_loc - 1}]: ").strip()
        if raw.isdigit() and 0 <= int(raw) < n_loc:
            loc = int(raw)
            if loc >= n_rf or not data["rf_data"][loc]:
                print(f"  Location {loc} has no RF data, pick another.")
                continue
            return loc
        print(f"  Enter a number between 0 and {n_loc - 1}.")


# ---------------------------------------------------------------------------
# Replay
# ---------------------------------------------------------------------------

def replay(data: dict, loc_idx: int) -> None:
    loc_rf = data["rf_data"][loc_idx]
    xy = data["grid_positions"][loc_idx]
    has_robot = _has_robot_data(data)
    all_xy = np.array(data["grid_positions"]) * 1000.0  # mm

    n_depths = len(loc_rf)
    probe_depth_mm = [round(d * 1000.0, 3) for d in data["probe_depth"]]

    # --- figure layout ---
    n_ch = len(CHANNEL_NAMES)
    if has_robot:
        fig = plt.figure(figsize=(18, max(6, n_ch * 1.8)))
        gs = gridspec.GridSpec(max(n_ch, 2), 3, figure=fig,
                               width_ratios=[1, 1.2, 2.2], hspace=0.55, wspace=0.4)
        ax_grid  = fig.add_subplot(gs[:, 0])
        ax_z     = fig.add_subplot(gs[0, 1])
        ax_force = fig.add_subplot(gs[1, 1])
        rf_axes  = [fig.add_subplot(gs[r, 2]) for r in range(n_ch)]
    else:
        fig = plt.figure(figsize=(14, max(5, n_ch * 1.8)))
        gs = gridspec.GridSpec(n_ch, 2, figure=fig,
                               width_ratios=[1, 2.5], hspace=0.45, wspace=0.35)
        ax_grid  = fig.add_subplot(gs[:, 0])
        rf_axes  = [fig.add_subplot(gs[r, 1]) for r in range(n_ch)]
        ax_z = ax_force = None

    # Grid
    ax_grid.scatter(all_xy[:, 0], all_xy[:, 1], c="lightgray", s=40, zorder=1)
    xy_mm = np.array(xy) * 1000.0
    ax_grid.scatter([xy_mm[0]], [xy_mm[1]], c="red", s=80, zorder=3)
    ax_grid.set_xlabel("X gripper (mm)")
    ax_grid.set_ylabel("Y gripper (mm)")
    ax_grid.set_aspect("equal")
    ax_grid.set_title(f"Grid — loc {loc_idx}")

    # Robot panels
    z_line = None
    force_lines = []
    if has_robot and ax_z is not None:
        (z_line,) = ax_z.plot([], [], lw=1.0, color="steelblue")
        ax_z.set_ylabel("EE Z (mm)")
        ax_z.set_xlabel("t (s)")
        ax_z.set_title("Plunge trajectory")
        ax_z.tick_params(labelsize=7)

        for label, color in zip(["Fx", "Fy", "Fz"], ["tab:red", "tab:green", "tab:blue"]):
            (ln,) = ax_force.plot([], [], lw=1.0, color=color, label=label)
            force_lines.append(ln)
        ax_force.set_ylabel("Force (N)")
        ax_force.set_xlabel("t (s)")
        ax_force.set_title("EE forces")
        ax_force.legend(fontsize=7, loc="upper right")
        ax_force.tick_params(labelsize=7)

    # RF panels
    rf_lines = []
    for ax, name in zip(rf_axes, CHANNEL_NAMES):
        (line,) = ax.plot([], [], lw=0.8)
        ax.set_ylabel(name, fontsize=8)
        ax.tick_params(labelsize=7)
        rf_lines.append(line)
    rf_axes[-1].set_xlabel("Sample index")

    plt.ion()
    plt.show()

    for depth_idx in range(n_depths):
        frames = loc_rf[depth_idx]
        if not frames:
            continue

        # Robot trajectory for this depth (static)
        if has_robot and z_line is not None:
            robot_data = data.get("robot_data", [])
            if loc_idx < len(robot_data) and depth_idx < len(robot_data[loc_idx]):
                rd = robot_data[loc_idx][depth_idx]
                ts_arr    = np.array(rd["ts"])
                pos_arr   = np.array(rd["positions"])
                force_arr = np.array(rd["forces"])
                z_line.set_data(ts_arr, pos_arr[:, 2] * 1000.0)
                ax_z.relim()
                ax_z.autoscale_view()
                for fi, ln in enumerate(force_lines):
                    ln.set_data(ts_arr, force_arr[:, fi])
                ax_force.relim()
                ax_force.autoscale_view()

        n_frames = len(frames)
        sha = _rf_sha(frames)
        for f_idx, frame in enumerate(frames):
            t0 = time.monotonic()

            fig.suptitle(
                f"Location {loc_idx}  |  depth {probe_depth_mm[depth_idx]:.2f} mm  "
                f"|  RF frame {f_idx + 1}/{n_frames}  "
                f"|  set: {data['set_name']}\n"
                f"sha256={sha}",
                fontsize=10,
            )

            for line, ch_i in zip(rf_lines, range(len(CHANNEL_NAMES))):
                samples = np.array(frame[ch_i], dtype=float)
                line.set_data(np.arange(len(samples)), samples)
                rf_axes[ch_i].relim()
                rf_axes[ch_i].autoscale_view()

            fig.canvas.draw_idle()
            fig.canvas.flush_events()

            elapsed = time.monotonic() - t0
            wait = REPLAY_INTERVAL_MS / 1000.0 - elapsed
            if wait > 0:
                time.sleep(wait)

    plt.ioff()
    print("Replay complete. Close the window to continue.")
    plt.show(block=True)


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

def main() -> None:
    path = pick_file()
    print(f"\nLoading {path.name}...")
    data = _load(path)
    n_loc = len(data["grid_positions"])

    loc_idx = pick_location(data)
    while True:
        replay(data, loc_idx)

        raw = input("\nEnter location number to replay again, or press Enter to quit: ").strip()
        if not raw.isdigit():
            break
        loc_idx = int(raw)
        if not (0 <= loc_idx < n_loc) or loc_idx >= len(data["rf_data"]) or not data["rf_data"][loc_idx]:
            print(f"  Invalid or empty location {loc_idx}, quitting.")
            break


if __name__ == "__main__":
    main()
