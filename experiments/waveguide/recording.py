"""Data recording of the waveguide runs (`--record`), with the lab's `experiment_logger`.

A run is a folder `~/data/waveguide/<kind>/<YYYYmmdd-HHMMSS>/` (kind: "teleop" for `wg_teleop.py`, "sensor"
for `sensor.py`) with `samples.mcap` (Foxglove-openable; `experiment_logger.load_streams(run_dir)` gives one
DataFrame per stream), `metadata.json` (git provenance, configs, notes) and a copy of every calibration used.
The loader spreads lists over numbered columns (`frame_0` ... `frame_1499`, `position_0..2`, `pressed_0`, ...; a
1-element list is a single column): `df.filter(regex=r"^frame_\d+$").to_numpy()` gives the frames.

Streams. Every sample is stamped with the wall time (MCAP log time) and has `t_mono`, CLOCK_MONOTONIC [s], shared
by the sensor and robot processes (the `t_created` of the commands is on the same clock):
- `sensor/frame`: the averaged frame each estimate used (`frame`, ADC), one per frame once detecting
- `sensor/detection`: the estimate of that frame: `detected`, `hole`, `hole2`, `distance_cm`, `amplitude`,
  `echo_time_ms` (None: not available), `pressed` (the holes the gesture layer saw)
- `sensor/background`: each background recorded (`frame`), at start and after a recalibration
- `sensor/calibration`: each calibration loaded: `file` (its copy in the run folder), `threshold`, `hole_ids`
- `sensor/swipe`: each finished touch, ignored ones included: `kind`, `start_hole`, `end_hole`, `duration_s`,
  `axis`, `distance`, `speed` (None if ignored), `ignored_reason`
- `robot/command` (wg_teleop): each command and what the robot did with it: `status` ("executed",
  "ignored_moving", "invalid", "home"), `detail`, `axis`, `distance`, `speed`, `t_created`, `target` [m]
- `robot/state` (wg_teleop, `STATE_RATE_HZ`): `position` [m] and `orientation` (xyzw) of the end-effector, and
  `target` [m], the commanded position

Nothing here imports ROS.
"""

import shutil
import threading
import time
from collections.abc import Callable
from pathlib import Path

import numpy as np

DATA_DIR = Path.home() / "data" / "waveguide"
STATE_RATE_HZ = 50.0

LogFn = Callable[[str, dict, float], None]  # (stream, values, wall time [s])


def start_logger(kind: str, notes: str = "", metadata: dict | None = None):
    """A started `ExperimentLogger` writing the run folder `DATA_DIR / kind / <YYYYmmdd-HHMMSS>` (file only: a long
    run must not accumulate in memory). Stop it at the end of the run."""
    from experiment_logger import ExperimentLogger, LoggingConfig
    from experiment_logger.config import FileSinkConfig

    cfg = LoggingConfig(
        sinks=["file_sink"],
        file_sink=FileSinkConfig(
            output_dir=str(DATA_DIR / kind), run_name=time.strftime("%Y%m%d-%H%M%S"), notes=notes
        ),
    )
    logger = ExperimentLogger(cfg, metadata=metadata)
    logger.start()
    return logger


def wall_time(t_mono: float) -> float:
    """Wall time [s] of a CLOCK_MONOTONIC time."""
    return time.time() - (time.monotonic() - t_mono)


def _number(value) -> float | None:
    return None if value is None else float(value)


class SensorRecorder:
    """Records what the sensor loop sees (see the module docstring). Called from the sensor loop thread.

    `log` receives each sample: `ExperimentLogger.log_sample` in the same process, or `QueueLog` from the sensor
    process. `run_dir` receives the copies of the calibrations.
    """

    def __init__(self, log: LogFn, run_dir: Path):
        self.log, self.run_dir = log, Path(run_dir)
        self._detector = self._averaged = self._background = None
        self._n_calibrations = 0

    def frame(self, t_mono: float, detector, pressed: frozenset[int]) -> None:
        """After `detector.update` of a frame: its averaged frame and estimate, a new background or calibration."""
        if detector is not self._detector:  # Started, or reloaded after a recalibration
            self._detector = detector
            self._calibration(t_mono, detector)
        if detector.background is not None and detector.background is not self._background:
            self._background = detector.background
            self.log("sensor/background", {"t_mono": t_mono, "frame": np.asarray(detector.background)}, wall_time(t_mono))
        if detector.last_averaged is None or detector.last_averaged is self._averaged:
            return  # No estimate for this frame (background still recording, or paused)
        self._averaged = detector.last_averaged
        ts = wall_time(t_mono)
        result = detector.last_result
        self.log("sensor/frame", {"t_mono": t_mono, "frame": np.asarray(self._averaged)}, ts)
        self.log(
            "sensor/detection",
            {
                "t_mono": t_mono,
                "detected": bool(result["detected"]),
                "hole": int(result["hole"]) if result.get("hole") is not None else None,
                "hole2": int(result["hole2"]) if result.get("hole2") is not None else None,
                "distance_cm": _number(result.get("distance_cm")),
                "amplitude": _number(result.get("amplitude")),
                "echo_time_ms": _number(result.get("echo_time_ms")),
                "pressed": sorted(pressed),
            },
            ts,
        )

    def swipe(self, swipe) -> None:
        command = swipe.command
        self.log(
            "sensor/swipe",
            {
                "t_mono": swipe.t,
                "kind": swipe.kind,
                "start_hole": swipe.start_hole,
                "end_hole": swipe.end_hole,
                "duration_s": swipe.duration_s,
                "axis": None if command is None else command.axis,
                "distance": None if command is None else command.distance,
                "speed": None if command is None else command.speed,
                "ignored_reason": swipe.ignored_reason,
            },
            wall_time(swipe.t),
        )

    def _calibration(self, t_mono: float, detector) -> None:
        calibration = detector.calibration_params
        if calibration is None:
            return
        source = Path(getattr(detector, "calibration_path", ""))
        copy = None
        if source.is_file():
            self._n_calibrations += 1
            copy = self.run_dir / f"calibration_{self._n_calibrations}{source.suffix}"
            shutil.copy2(source, copy)
        self.log(
            "sensor/calibration",
            {
                "t_mono": t_mono,
                "file": None if copy is None else copy.name,
                "source": str(source),
                "threshold": float(calibration["threshold"]),
                "hole_ids": np.asarray(calibration["hole_ids"]).astype(int).tolist(),
            },
            wall_time(t_mono),
        )


class QueueLog:
    """A `LogFn` for another process: puts the samples on a multiprocessing queue, drained by `QueueDrain`."""

    def __init__(self, samples):
        self.samples = samples

    def __call__(self, stream: str, values: dict, ts: float) -> None:
        self.samples.put((stream, values, ts))


class QueueDrain:
    """Moves the samples of a `QueueLog` (other process) to `logger`, from a background thread."""

    def __init__(self, samples, logger):
        self.samples, self.logger = samples, logger
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="waveguide_record_drain", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def _run(self) -> None:
        import queue

        while True:
            try:
                stream, values, ts = self.samples.get(timeout=0.1)
            except queue.Empty:
                if self._stop.is_set():
                    return  # Stopped and nothing left
                continue
            self.logger.log_sample(stream, values, ts)

    def stop(self) -> None:
        """Stop once the queue is empty (call after the other process ended)."""
        self._stop.set()
        self._thread.join(timeout=5.0)


class RobotRecorder:
    """Records the commands and the end-effector state of wg_teleop (see the module docstring).

    `state()` returns (position, orientation xyzw, target position) or None if not available yet; it is called
    from a background thread at `STATE_RATE_HZ`.
    """

    def __init__(self, logger, state: Callable[[], tuple | None], rate_hz: float = STATE_RATE_HZ):
        self.logger, self._state, self._period = logger, state, 1.0 / rate_hz
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="waveguide_record_state", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)

    def command(self, command, status: str, detail: str = "", target=None) -> None:
        """`command`: a MoveCommand, or None (e.g. home); `target`: the commanded position after it [m]."""
        t = time.monotonic()
        self.logger.log_sample(
            "robot/command",
            {
                "t_mono": t,
                "status": status,
                "detail": detail,
                "axis": getattr(command, "axis", None),
                "distance": getattr(command, "distance", None),
                "speed": getattr(command, "speed", None),
                "t_created": getattr(command, "t_created", None),
                "target": None if target is None else np.asarray(target, dtype=float),
            },
            wall_time(t),
        )

    def _run(self) -> None:
        t_next = time.monotonic()
        while not self._stop.is_set():
            t_next += self._period
            self._stop.wait(max(0.0, t_next - time.monotonic()))
            try:
                state = self._state()
            except Exception:  # Not ready (no pose yet): skip this sample
                continue
            if state is None:
                continue
            position, orientation, target = state
            t = time.monotonic()
            self.logger.log_sample(
                "robot/state",
                {
                    "t_mono": t,
                    "position": np.asarray(position, dtype=float),
                    "orientation": np.asarray(orientation, dtype=float),
                    "target": None if target is None else np.asarray(target, dtype=float),
                },
                wall_time(t),
            )
