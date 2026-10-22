"""Read and plot a waveguide run recorded with `--record` (wg_teleop.py or sensor.py).

Standalone: needs only numpy, matplotlib and mcap (`pip install numpy matplotlib mcap`), not this repository.

    python plot_run.py                                    # the newest run in ~/data/waveguide/*/
    python plot_run.py ~/data/waveguide/teleop/20261022-154716
    python plot_run.py RUN_DIR --start 10 --end 40        # seconds from the start of the run
    python plot_run.py RUN_DIR --save run.png --no-show

The figure, against the time from the start of the run [s]:
1. Echo: |averaged frame - background| over the echo time (darker: larger change), with the detected echo times
2. Detected hole of every frame, and the swipes (arrow from the start to the end hole; green: sent as a command,
   grey: ignored)
3. Echo amplitude of every frame, against the detection threshold
4. End-effector position (x, y, z, relative to the start) and its commanded target (dashed); the executed
   commands are the ticks at the top (wg_teleop runs only)

To use the data in your own code:

    from plot_run import load_run, stack
    run = load_run(run_dir)                    # {stream: [sample, ...]}, see the streams below
    frames = stack(run["sensor/frame"], "frame")       # (n_frames, 1500) array
    t = stack(run["sensor/detection"], "t_mono")       # CLOCK_MONOTONIC [s], shared by all streams

Each sample is a dict: `ts` (wall time [s]) and the recorded fields. Streams: `sensor/frame` (t_mono, frame),
`sensor/detection` (t_mono, detected, hole, hole2, distance_cm, amplitude, echo_time_ms, pressed),
`sensor/background` (t_mono, frame), `sensor/calibration` (t_mono, file, threshold, hole_ids), `sensor/swipe`
(t_mono, kind, start_hole, end_hole, duration_s, axis, distance, speed, ignored_reason), `robot/command` (t_mono,
status, detail, axis, distance, speed, t_created, target), `robot/state` (t_mono, position, orientation, target).
"""

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection

DATA_DIR = Path.home() / "data" / "waveguide"

# Okabe-Ito (colorblind-safe), as the live plot: signal blue, touch vermillion, command green, reference orange
BLUE, VERMILLION, GREEN, ORANGE, PURPLE, SKY = "#0072B2", "#D55E00", "#009E73", "#E69F00", "#CC79A7", "#56B4E9"
GREY, INK, MUTED = "#8C8C8C", "#222222", "#6B6B6B"
AXIS_COLORS = {"x": BLUE, "y": ORANGE, "z": GREEN}


# --- Reading
def load_run(run_dir: str | Path) -> dict[str, list[dict]]:
    """The samples of a run, per stream, in time order: {stream: [{"ts": wall time [s], field: value, ...}, ...]}."""
    from mcap.reader import make_reader

    path = Path(run_dir).expanduser()
    path = path / "samples.mcap" if path.is_dir() else path
    streams: dict[str, list[dict]] = {}
    with open(path, "rb") as file:
        for _, channel, message in make_reader(file).iter_messages():
            if channel.message_encoding != "json":
                continue
            sample = json.loads(message.data)
            streams.setdefault(sample["stream"], []).append({"ts": sample["ts"], **sample["values"]})
    for samples in streams.values():
        samples.sort(key=lambda s: s["ts"])
    return streams


def stack(samples: list[dict], field: str, dtype=float) -> np.ndarray:
    """One field of the samples as an array (None -> nan for numbers); arrays are stacked into rows."""
    values = [s.get(field) for s in samples]
    if dtype is float:
        return np.array([np.nan if v is None else v for v in values], dtype=float)
    return np.array(values, dtype=dtype)


def latest_run(data_dir: Path = DATA_DIR) -> Path:
    runs = sorted(data_dir.glob("*/*/samples.mcap"), key=lambda p: p.stat().st_mtime)
    if not runs:
        raise FileNotFoundError(f"No run in {data_dir}/*/ (give the run folder)")
    return runs[-1].parent


def summary(run: dict[str, list[dict]]) -> str:
    detections = run.get("sensor/detection", [])
    lines = [f"{name}: {len(samples)} samples" for name, samples in sorted(run.items())]
    if detections:
        t = stack(detections, "t_mono")
        detected = stack(detections, "detected", dtype=bool)
        lines.append(f"{t[-1] - t[0]:.1f} s of frames ({len(t) / max(t[-1] - t[0], 1e-9):.1f}/s), {detected.sum()} detected")
    for swipe in run.get("sensor/swipe", []):
        what = (
            f"{swipe['axis']} {swipe['distance'] * 100:+.1f} cm at {swipe['speed'] * 100:.1f} cm/s"
            if swipe["axis"] is not None
            else f"ignored: {swipe['ignored_reason']}"
        )
        lines.append(f"  {swipe['kind']} {swipe['start_hole']} -> {swipe['end_hole']} ({swipe['duration_s']:.2f} s): {what}")
    return "\n".join(lines)


# --- Plotting
def _legend(ax, ncols: int = 4) -> None:
    """Above the panel, out of the data."""
    ax.legend(loc="lower left", bbox_to_anchor=(0, 1.0), ncols=ncols, frameon=False, labelcolor=INK, fontsize=9,
              borderaxespad=0.2, handlelength=1.8)


def _style(ax, ylabel: str) -> None:
    ax.set_ylabel(ylabel, color=INK)
    ax.grid(True, color="#E6E6E6", linewidth=0.8)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color("#BDBDBD")
    ax.tick_params(colors=MUTED, labelcolor=INK)
    ax.tick_params(axis="x", length=0)


def plot_run(
    run: dict[str, list[dict]],
    title: str = "",
    start: float | None = None,
    end: float | None = None,
    debounce_s: float = 0.3,
):
    """The figure of a run (module docstring); `start`/`end` limit the time axis [s from the start]. `debounce_s`
    (the run's GestureConfig.debounce_s) places the swipes: they are stamped when the touch ended, after it."""
    t0 = min(s["t_mono"] for samples in run.values() for s in samples if s.get("t_mono") is not None)
    frames, detections = run.get("sensor/frame", []), run.get("sensor/detection", [])
    robot_states, commands = run.get("robot/state", []), run.get("robot/command", [])
    has_robot = bool(robot_states)

    n_panels = 4 if has_robot else 3
    fig, axes = plt.subplots(
        n_panels, 1, figsize=(11, 2.6 * n_panels + 1), sharex=True,
        gridspec_kw={"height_ratios": [2.2, 1.6, 1.2] + ([1.6] if has_robot else [])},
    )
    plt.rcParams.update({"font.size": 10})
    fig.suptitle(title, x=0.01, ha="left", color=INK, fontsize=12)

    # 1. Echo waterfall: |averaged frame - latest background|
    ax = axes[0]
    if frames:
        t_frames = stack(frames, "t_mono") - t0
        images = np.array([s["frame"] for s in frames], dtype=float)
        backgrounds = run.get("sensor/background", [])
        if backgrounds:  # Each frame against the latest background recorded before it
            t_bg = stack(backgrounds, "t_mono") - t0
            which = np.clip(np.searchsorted(t_bg, t_frames, side="right") - 1, 0, len(backgrounds) - 1)
            bg = np.array([s["frame"] for s in backgrounds], dtype=float)
            images = np.abs(images - bg[which])
        sampling_hz = 183150.2  # wg_helpers.SAMPLING_RATE_HZ
        echo_ms = np.arange(images.shape[1]) / sampling_hz * 1000
        vmax = np.percentile(images, 99.5) if images.size else 1.0
        dt = np.median(np.diff(t_frames)) if len(t_frames) > 1 else 0.05
        ax.imshow(
            images.T, aspect="auto", origin="lower", cmap="Blues", vmin=0, vmax=vmax, interpolation="nearest",
            extent=(t_frames[0] - dt / 2, t_frames[-1] + dt / 2, echo_ms[0], echo_ms[-1]),
        )
        if detections:
            t_det = stack(detections, "t_mono") - t0
            echo = stack(detections, "echo_time_ms")
            hit = stack(detections, "detected", dtype=bool) & np.isfinite(echo)
            ax.scatter(t_det[hit], echo[hit], s=10, color=VERMILLION, linewidths=0, label="detected echo")
        ax.plot([], [], color="#4A90C8", lw=6, label="|frame - background| (darker: larger)")
        _legend(ax)
        ax.set_ylim(0, min(echo_ms[-1], 5.0))
    _style(ax, "Echo time (ms)")

    # 2. Detected hole and swipes
    ax = axes[1]
    if detections:
        t_det = stack(detections, "t_mono") - t0
        hole = stack(detections, "hole")
        hit = stack(detections, "detected", dtype=bool)
        ax.scatter(t_det[hit], hole[hit], s=12, color=VERMILLION, linewidths=0, label="detected hole")
        second = stack(detections, "hole2")
        if np.any(np.isfinite(second[hit])):
            ax.scatter(t_det[hit], second[hit], s=12, color=PURPLE, linewidths=0, label="second hole")
    for swipe in run.get("sensor/swipe", []):
        t_end = swipe["t_mono"] - t0 - debounce_s  # Last touch
        color = GREEN if swipe["axis"] is not None else GREY
        ax.annotate(
            "", xy=(t_end, swipe["end_hole"]), xytext=(t_end - swipe["duration_s"], swipe["start_hole"]),
            arrowprops=dict(arrowstyle="-|>", color=color, lw=2, shrinkA=0, shrinkB=0),
        )
        label = f"{swipe['axis']} {swipe['distance'] * 100:+.1f} cm" if swipe["axis"] is not None else "ignored"
        ax.annotate(label, (t_end, swipe["end_hole"]), xytext=(4, 4), textcoords="offset points", color=INK, fontsize=8)
    ax.plot([], [], color=GREEN, lw=2, label="swipe (command)")
    ax.plot([], [], color=GREY, lw=2, label="swipe (ignored)")
    _legend(ax)
    _style(ax, "Hole")

    # 3. Amplitude against the threshold
    ax = axes[2]
    if detections:
        t_det = stack(detections, "t_mono") - t0
        ax.plot(t_det, stack(detections, "amplitude"), color=BLUE, lw=1.5, label="amplitude")
    calibrations = run.get("sensor/calibration", [])
    if calibrations:  # The threshold of each calibration, from when it was loaded
        t_cal = list(stack(calibrations, "t_mono") - t0)
        last = (t_det[-1] if detections else t_cal[-1]) if t_cal else 0
        for i, cal in enumerate(calibrations):
            t_next = t_cal[i + 1] if i + 1 < len(t_cal) else last
            ax.hlines(cal["threshold"], t_cal[i], t_next, colors=VERMILLION, linestyles="--", lw=1.2,
                      label="threshold" if i == 0 else None)
    _legend(ax)
    _style(ax, "Amplitude (ADC)")

    # 4. End-effector position and target, relative to the start
    if has_robot:
        ax = axes[3]
        t_state = stack(robot_states, "t_mono") - t0
        position = np.array([s["position"] for s in robot_states], dtype=float)
        target = np.array([s["target"] if s.get("target") is not None else [np.nan] * 3 for s in robot_states], float)
        origin = position[0]
        for i, axis in enumerate("xyz"):
            ax.plot(t_state, (position[:, i] - origin[i]) * 100, color=AXIS_COLORS[axis], lw=1.8, label=axis)
            ax.plot(t_state, (target[:, i] - origin[i]) * 100, color=AXIS_COLORS[axis], lw=1.2, ls="--")
        ax.plot([], [], color=MUTED, lw=1.2, ls="--", label="target")
        executed = [c for c in commands if c["status"] == "executed"]
        if executed:
            t_cmd = stack(executed, "t_mono") - t0
            ax.add_collection(
                LineCollection([[(t, 0.92), (t, 1.0)] for t in t_cmd], colors=INK, lw=1.5,
                               transform=ax.get_xaxis_transform())
            )
        if executed:
            ax.plot([], [], color=INK, lw=1.5, label="command executed")
        _legend(ax, ncols=5)
        _style(ax, "Position (cm)")

    axes[-1].set_xlabel("Time (s)", color=INK)
    axes[-1].tick_params(axis="x", length=3.5)
    if start is not None or end is not None:
        axes[-1].set_xlim(start, end)
    fig.tight_layout()
    return fig


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("run_dir", nargs="?", type=Path, help=f"run folder (default: the newest in {DATA_DIR}/*/)")
    parser.add_argument("--start", type=float, help="start of the time axis [s from the start of the run]")
    parser.add_argument("--end", type=float, help="end of the time axis [s]")
    parser.add_argument("--save", type=Path, help="save the figure (png, pdf, svg, ...)")
    parser.add_argument("--no-show", action="store_true", help="do not open the figure window")
    args = parser.parse_args()

    run_dir = args.run_dir.expanduser() if args.run_dir else latest_run()
    run = load_run(run_dir)
    print(f"Run {run_dir}")
    metadata_path = run_dir / "metadata.json"
    metadata = json.loads(metadata_path.read_text()) if metadata_path.exists() else {}
    notes = metadata.get("notes", "")
    if notes:
        print(f"Notes: {notes}")
    print(summary(run))

    debounce_s = metadata.get("gesture", {}).get("debounce_s", 0.3)
    fig = plot_run(run, title=f"{run_dir.name}  {notes}".strip(), start=args.start, end=args.end, debounce_s=debounce_s)
    if args.save:
        fig.savefig(args.save, dpi=200)
        print(f"Saved {args.save}")
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
