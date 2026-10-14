"""Waveguide sensor processing.

Reads the Teensy over serial and turns the echo frames into teleop commands, in two layers:
1. Hole detection (on every frame, > 500 Hz): frame -> set of pressed holes
2. Gesture processing: changes of the pressed holes -> `MoveCommand` (axis, distance, speed)

Runs in its own process (`SensorProcess`), so the robot (rclpy, the GIL) never stalls the fast
loop. Imports no ROS: it can be developed and tested without the robot.

Standalone (the port defaults to /dev/ttyACM0, see --port):
    pixi run -e wg python experiments/waveguide/sensor.py               # frame rate, pressed holes, commands
    pixi run -e wg python experiments/waveguide/sensor.py --live-plot   # raw frames
    pixi run -e wg python experiments/waveguide/sensor.py --calibrate   # guided calibration, see CalibrationConfig
"""

import argparse
import json
import multiprocessing as mp
import queue
import sqlite3
import threading
import time
from collections import deque
from contextlib import closing
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import serial
import wg_helpers
from commands import MoveCommand


# --- Update configs as desired
@dataclass(frozen=True)
class CalibrationConfig:
    """Calibration (`--calibrate`) settings."""

    directory: Path = Path(__file__).resolve().parent / "calibration"
    filename: str = "calibration_dummy.db"  # Recording (SAVE_TO_SQLITE.py format); the fit is saved next to it (.npz)
    n_holes: int = 31
    n_passes: int = 3  # Recordings of every hole, >= 2 for a detection threshold
    first_hole_cm: float = 25.0  # Distance to hole 1
    hole_spacing_cm: float = 1.0  # Distance between holes

    @property
    def db_path(self) -> Path:
        return self.directory / self.filename

    @property
    def npz_path(self) -> Path:
        return self.db_path.with_suffix(".npz")


@dataclass(frozen=True)
class DetectionConfig:
    """Hole detection (layer 1) settings."""

    n_background_frames: int = 50  # Frames averaged at startup, with nothing covered
    # Latest frames averaged for each estimate: the threshold comes from 5-frame averages (CYCLES_PER_PRESS),
    # single frames give false detections. More: less noise, more lag
    n_average_frames: int = 5
    method: str = "leading_edge"  # One of wg_helpers.METHODS


@dataclass(frozen=True)
class GestureConfig:
    """Gesture processing (layer 2) settings: swipe -> MoveCommand(axis, distance, speed)."""

    hole_idx: int = 15  # Hole where the waveguide bends: swipes starting at holes <= hole_idx are along x, else y
    x_sign: int = 1  # +1: increasing hole index moves +x; -1 to flip
    y_sign: int = 1  # +1: increasing hole index moves +y; -1 to flip
    distance_scale: float = 0.01  # Robot motion [m] per cm of swipe (0.01: 1:1)
    max_speed: float = 0.15  # [m/s] Faster swipes are capped (waveguide_teleop.MAX_SPEED)
    max_distance: float = 0.2  # [m] Longer swipes are capped (waveguide_teleop.MAX_DISTANCE)
    min_holes: int = 2  # Holes a swipe must span (start and end included); shorter touches send nothing
    debounce_s: float = 0.3  # The swipe ends after no hole is detected for this long


# Configs
DETECTION_CFG = DetectionConfig()
GESTURE_CFG = GestureConfig()
CAL_CFG = CalibrationConfig()


DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUDRATE = 3_000_000
SERIAL_TIMEOUT = 0.1  # s, so the loop checks the stop event even without data

# Teensy packet: 3-byte header + FRAME_SIZE uint16 samples (little-endian, 12-bit ADC)
FRAME_SIZE = 1500
HEADER = b"\xaa\x55\x00"
PACKET_SIZE = len(HEADER) + 2 * FRAME_SIZE


def check_port(port: str) -> None:
    """Raise FileNotFoundError if the serial port does not exist."""
    if not Path(port).exists():
        raise FileNotFoundError(f"Serial port {port} not found: is the Teensy plugged in? (--port to use another one)")


def open_serial(port: str, baudrate: int) -> serial.Serial:
    check_port(port)

    print(f"Waveguide sensor loop started on {port} at {baudrate} baud (Ctrl+C to quit)")

    return serial.Serial(port, baudrate, timeout=SERIAL_TIMEOUT)


def load_calibration(path: Path = CAL_CFG.npz_path) -> dict:
    """The fitted calibration (`wg_helpers.load_calibration`); FileNotFoundError if there is none yet."""
    if not path.exists():
        raise FileNotFoundError(f"No calibration at {path}: run sensor.py --calibrate first")

    calibration_params = wg_helpers.load_calibration(path)

    calibration_params["threshold"] = float(calibration_params["threshold"]) * 1.5  # temp, to avoid false detections

    return calibration_params


@dataclass(frozen=True)
class HoleState:
    """Holes pressed at time `t` (CLOCK_MONOTONIC) [s], and the estimated touch distance [cm] if any."""

    t: float
    pressed: frozenset[int]
    distance_cm: float | None = None


class FrameReader:
    """Reads the Teensy echo frames from the serial port."""

    def __init__(self, ser: serial.Serial):
        self.ser = ser
        self._buffer = bytearray()

    def flush(self) -> None:
        """Drop what is buffered: the next frame is one sampled after this call."""
        self.ser.reset_input_buffer()
        self._buffer.clear()

    def read_frame(self) -> np.ndarray | None:
        """Next complete frame (FRAME_SIZE uint16), or None if none arrived within the serial timeout."""
        while (frame := self._pop_frame()) is None:
            chunk = self.ser.read(max(self.ser.in_waiting, 1))
            if not chunk:
                return None
            self._buffer.extend(chunk)
        return frame

    def _pop_frame(self) -> np.ndarray | None:
        """Take the first complete frame out of the buffer, if there is one."""
        idx = self._buffer.find(HEADER)
        if idx < 0:
            del self._buffer[: -(len(HEADER) - 1)]  # Keep a possible partial header
            return None
        del self._buffer[:idx]
        if len(self._buffer) < PACKET_SIZE:
            return None
        frame = np.frombuffer(self._buffer, dtype="<u2", count=FRAME_SIZE, offset=len(HEADER)).copy()
        del self._buffer[:PACKET_SIZE]
        return frame


# --- Layer 1: hole detection ---
class HoleDetector:
    """Detect the pressed hole from each frame (`wg_helpers.estimate`), against a background recorded at startup.

    The first `n_background_frames` frames (nothing must be covered) are averaged into the background;
    detection starts after. Each estimate uses the mean of the latest `n_average_frames` frames.
    Without a calibration (None), nothing is detected.
    """

    def __init__(self, calibration_file_path: Path, detection_cfg: DetectionConfig = DETECTION_CFG):
        if detection_cfg.method not in wg_helpers.METHODS:
            raise ValueError(f"Detection method must be one of {wg_helpers.METHODS}, got {detection_cfg.method}")

        # Load calibration
        calibration_params = load_calibration(calibration_file_path)
        self.calibration_params = calibration_params

        self.detection_cfg = detection_cfg
        self.background = None
        self.last_result = None  # Latest estimate() output (detected, hole, distance_cm, amplitude, ...)
        self.last_averaged = None  # Averaged frame the latest estimate used
        self._background_frames = []
        self._frames = deque(maxlen=detection_cfg.n_average_frames)

    def update(self, frame: np.ndarray) -> frozenset[int]:
        if self.calibration_params is None:
            return frozenset()

        if self.background is None:
            if not self._background_frames:
                print(
                    f"Recording the background ({self.detection_cfg.n_background_frames} frames): don't touch the sensor"
                )
            self._background_frames.append(frame)
            if len(self._background_frames) == self.detection_cfg.n_background_frames:
                self.background = wg_helpers.make_background(self._background_frames)
                self._background_frames = []
                print("Background calibration recorded")
            return frozenset()

        self._frames.append(frame)
        averaged = self._frames[0] if len(self._frames) == 1 else wg_helpers.make_background(self._frames)
        self.last_averaged = averaged

        self.last_result = wg_helpers.estimate(
            averaged, self.calibration_params, self.background, method=self.detection_cfg.method
        )

        if self.last_result["detected"]:
            print(
                f"Detected hole {self.last_result['hole']} at {self.last_result['distance_cm']:.1f} cm (amplitude {self.last_result['amplitude']:.1f})"
            )

            return frozenset({self.last_result["hole"]})

        else:
            return frozenset()


# --- Layer 2: gesture processing ---
class GestureProcessor:
    """Turn swipes along the waveguide into teleop commands.

    A swipe starts at the first detected hole and ends once no hole is detected for `debounce_s`. Only its
    first and last hole count (noisy or skipped holes in between do not matter):
    - axis: x if it starts at or before the bend (`hole_idx`), else y. A swipe crossing the bend stops there.
    - distance: swipe length [cm, from the calibration] * `distance_scale`, signed by the direction
    - speed: swipe speed (start to reaching the last hole, without the debounce) * `distance_scale`
    Distance and speed are capped at `max_distance` and `max_speed`; swipes shorter than `min_holes` send nothing.
    """

    def __init__(self, calibration: dict, gesture_cfg: GestureConfig = GESTURE_CFG):
        self.cfg = gesture_cfg
        self.hole_ids = np.asarray(calibration["hole_ids"]).astype(int)
        self.hole_cm = dict(zip(self.hole_ids.tolist(), np.asarray(calibration["hole_cm"]).tolist()))
        self._reset()

    def _reset(self) -> None:
        self._t_start = None  # Time the swipe started (first hole detected), None: no swipe
        self._t_last_touch = None  # Last time a hole was detected
        self._reached = {}  # Hole -> time it was first reached in this swipe
        self._start_hole = self._last_hole = None

    def update(self, t: float, pressed: frozenset[int]) -> MoveCommand | None:
        """Call on every frame (the swipe end needs the time to advance); returns a command when a swipe ends."""
        if pressed:
            hole = min(pressed)  # One hole at a time from the detection
            if self._t_start is None:
                self._t_start, self._start_hole = t, hole
            self._reached.setdefault(hole, t)
            self._last_hole, self._t_last_touch = hole, t
            return None

        if self._t_start is None or t - self._t_last_touch < self.cfg.debounce_s:
            return None
        command = self._swipe_command()
        self._reset()
        return command

    def _swipe_command(self) -> MoveCommand | None:
        cfg = self.cfg
        start, end = self._start_hole, self._last_hole

        # Axis from the start hole; a swipe crossing the bend stops at the end of the start segment
        on_x = start <= cfg.hole_idx
        segment = self.hole_ids[self.hole_ids <= cfg.hole_idx] if on_x else self.hole_ids[self.hole_ids > cfg.hole_idx]
        end = int(np.clip(end, segment.min(), segment.max()))

        if abs(end - start) + 1 < cfg.min_holes:
            print(f"Swipe {start} -> {end}: shorter than {cfg.min_holes} holes, ignored")
            return None

        axis, sign = ("x", cfg.x_sign) if on_x else ("y", cfg.y_sign)
        distance = sign * (self.hole_cm[end] - self.hole_cm[start]) * cfg.distance_scale
        distance = float(np.clip(distance, -cfg.max_distance, cfg.max_distance))

        # Time to reach the end hole (the last touched one if the end was clipped at the bend and never detected)
        duration = self._reached.get(end, self._reached[self._last_hole]) - self._t_start
        speed = min(abs(distance) / duration, cfg.max_speed) if duration > 0 else cfg.max_speed

        print(f"Swipe {start} -> {end} in {duration:.2f} s: {axis} {distance * 100:+.1f} cm at {speed * 100:.1f} cm/s")
        return MoveCommand(axis, distance, speed)


# --- Process ---
def run_sensor_loop(
    port: str, baudrate: int, on_holes, on_command, stop_event=None, on_frame=None, require_calibration: bool = True
) -> None:
    """Read the serial port and run both layers until `stop_event` is set (or forever).

    `on_holes(HoleState)` is called when the pressed holes change, `on_command(MoveCommand)` for
    each command, and `on_frame(t, frame, detector)` (if given) for every frame, after the detection
    (`detector.last_result`, `detector.background`, ...). Without a calibration, raises
    FileNotFoundError if `require_calibration`, else runs with the detection off.
    """
    detector = HoleDetector(calibration_file_path=CAL_CFG.npz_path)
    processor = GestureProcessor(detector.calibration_params)
    pressed = frozenset()

    with open_serial(port, baudrate) as ser:
        reader = FrameReader(ser)
        reader.flush()  # Drop what accumulated before we started
        while stop_event is None or not stop_event.is_set():
            frame = reader.read_frame()
            t = time.monotonic()

            if frame is not None:
                new_pressed = detector.update(frame)
                if on_frame is not None:
                    on_frame(t, frame, detector)

                if new_pressed != pressed:
                    pressed = new_pressed
                    on_holes(HoleState(t, pressed, detector.last_result["distance_cm"] if pressed else None))

            # Every iteration, even without a frame: the swipe ends after a time without holes
            command = processor.update(t, pressed)
            if command is not None:
                on_command(command)


def _process_main(port: str, baudrate: int, commands: mp.Queue, stop_event) -> None:
    try:
        run_sensor_loop(port, baudrate, on_holes=lambda state: None, on_command=commands.put, stop_event=stop_event)
    except KeyboardInterrupt:
        pass  # Ctrl+C reaches the whole process group; the parent handles it


class SensorProcess:
    """Runs `run_sensor_loop` in a separate process and hands its commands over a queue."""

    def __init__(self, port: str = DEFAULT_PORT, baudrate: int = DEFAULT_BAUDRATE):
        # Checked here, not only in the child: there it would just end the process
        check_port(port)
        load_calibration()
        # Spawn, not fork: the child must not inherit the parent's rclpy state and threads
        ctx = mp.get_context("spawn")
        self._commands = ctx.Queue()
        self._stop_event = ctx.Event()
        self._process = ctx.Process(
            target=_process_main,
            args=(port, baudrate, self._commands, self._stop_event),
            name="waveguide_sensor",
            daemon=True,
        )

    def start(self) -> None:
        self._process.start()

    def is_alive(self) -> bool:
        return self._process.is_alive()

    def get_command(self, timeout: float) -> MoveCommand | None:
        try:
            return self._commands.get(timeout=timeout)
        except queue.Empty:
            return None

    def stop(self, timeout: float = 1.0) -> None:
        self._stop_event.set()
        self._process.join(timeout)
        if self._process.is_alive():
            self._process.terminate()


# --- Calibration ---
def record_calibration(port: str, baudrate: int, config: CalibrationConfig) -> None:
    """Guided single-touch calibration recording, in the SAVE_TO_SQLITE.py format `wg_helpers.calibrate` reads.

    Each pass records nothing covered (key [0]), then holes 1..n_holes one at a time
    (wg_helpers.CYCLES_PER_PRESS frames each).
    """
    n_frames = wg_helpers.CYCLES_PER_PRESS
    with closing(sqlite3.connect(config.db_path)) as conn, open_serial(port, baudrate) as ser:
        conn.execute(
            """CREATE TABLE IF NOT EXISTS signals (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                key_vector TEXT NOT NULL,
                run_number INTEGER NOT NULL,
                timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
                signal_blob BLOB NOT NULL
            )"""
        )
        reader = FrameReader(ser)
        run_number = 0
        for pass_idx in range(config.n_passes):
            print(f"\n--- Pass {pass_idx + 1}/{config.n_passes}")
            for key in range(config.n_holes + 1):
                what = "Uncover ALL holes" if key == 0 else f"Cover hole {key} only"
                input(f"[{key}/{config.n_holes}] {what}, then press Enter ")

                reader.flush()  # Only frames sampled after the press
                frames = []
                while len(frames) < n_frames:
                    frame = reader.read_frame()
                    if frame is None:
                        raise RuntimeError(f"No frame from {port} within {SERIAL_TIMEOUT} s")
                    frames.append(frame)

                conn.executemany(
                    "INSERT INTO signals (key_vector, run_number, signal_blob) VALUES (?, ?, ?)",
                    [(json.dumps([key]), run_number, frame.tobytes()) for frame in frames],
                )
                conn.commit()
                run_number += 1


def calibrate(port: str, baudrate: int, config: CalibrationConfig = CAL_CFG) -> None:
    """Record a calibration (or reuse the existing recording) and fit it."""
    db_path = config.db_path
    if db_path.exists():
        answer = input(f"{db_path} exists: [f]it it, [r]ecord a new one over it, or [q]uit? ").strip().lower()
        if answer == "r":
            db_path.unlink()
        elif answer != "f":
            return

    if not db_path.exists():
        config.directory.mkdir(parents=True, exist_ok=True)
        print(f"Calibration: {config.n_holes} holes, {config.n_passes} passes, recording to {db_path}")
        try:
            record_calibration(port, baudrate, config)
        except BaseException:
            db_path.unlink(missing_ok=True)  # An incomplete recording does not calibrate
            raise
        print()

    wg_helpers.calibrate(db_path, config.first_hole_cm, config.hole_spacing_cm, save_path=config.npz_path)


# --- Standalone modes ---
def print_frames_and_holes(port: str, baudrate: int, plot: bool = False) -> None:
    """Run the sensor loop and print what it detects; with `plot`, also plot the raw frames (`run_with_live_plot`)."""
    n_frames, t_last = 0, time.monotonic()

    def on_frame(t, frame, detector):
        nonlocal n_frames, t_last
        n_frames += 1
        if t - t_last >= 1.0:
            # print(f"{n_frames / (t - t_last):.0f} frames/s")
            n_frames, t_last = 0, t

    def on_holes(state):
        # distance = "" if state.distance_cm is None else f"  distance: {state.distance_cm:.1f} cm"
        # print(f"{state.t:.3f}  holes: {sorted(state.pressed)}{distance}")
        ...

    def on_command(command):
        print(f"{command.t_created:.3f}  command: {command}")

    if plot:
        run_with_live_plot(port, baudrate, on_holes=on_holes, on_command=on_command, on_frame=on_frame)
    else:
        run_sensor_loop(
            port, baudrate, on_holes=on_holes, on_command=on_command, on_frame=on_frame, require_calibration=False
        )


def run_with_live_plot(port: str, baudrate: int, on_holes, on_command, on_frame=None, trail_s: float = 2.0) -> None:
    """`run_sensor_loop` with a live view of the processing, refreshed at ~60 Hz:

    1. Raw frame: the latest frame (cyan), the background (orange), the echo search window (shaded) and the
       detected echo (red line); the last command in the title
    2. Detection signal: envelope of the averaged frame minus the background, as `wg_helpers.estimate` computes
       it, against the detection threshold (dashed)
    3. Waveguide: the holes at their calibrated distance and the estimated distance (yellow). The covered hole
       is bright red (white outline); released holes fade back to grey over `trail_s` [s], showing the swipe.

    The loop runs in a background thread (Qt needs the main thread), so the plot shows what the processing
    sees. The callbacks are passed on to the loop, and called from its thread.
    """
    check_port(port)
    calibration = load_calibration(CAL_CFG.npz_path)
    import pyqtgraph as pg  # Only needed here
    from pyqtgraph.Qt import QtCore

    # Written by the sensor loop thread, read by the plot
    latest = {"frame": None, "detector": None, "n_frames": 0, "command": None}

    def plot_on_command(command):
        latest["command"] = command
        on_command(command)
    errors = []
    stop_event = threading.Event()

    hole_ids = np.asarray(calibration["hole_ids"])
    hole_index = {int(hole): i for i, hole in enumerate(hole_ids)}
    last_covered = np.full(len(hole_ids), -np.inf)  # Last time each hole was covered (CLOCK_MONOTONIC) [s]

    def plot_on_frame(t, frame, detector):
        latest["frame"], latest["detector"] = frame, detector
        latest["n_frames"] += 1
        # Every frame, not every redraw: a fast swipe can cover a hole for less than a redraw period
        result = detector.last_result
        if result is not None and result["detected"]:
            last_covered[hole_index[result["hole"]]] = t
        if on_frame is not None:
            on_frame(t, frame, detector)

    def sensor_loop():
        try:
            run_sensor_loop(
                port,
                baudrate,
                on_holes=on_holes,
                on_command=plot_on_command,
                stop_event=stop_event,
                on_frame=plot_on_frame,
                require_calibration=False,
            )
        except Exception as e:
            errors.append(e)

    start, stop = int(calibration["search_start"]), int(calibration["search_stop"])
    hole_cm = np.asarray(calibration["hole_cm"])
    threshold = float(calibration["threshold"])
    samples_per_ms = calibration["sampling_rate_hz"] / 1000

    app = pg.mkQApp("Waveguide sensor")
    win = pg.GraphicsLayoutWidget(show=True, title="Waveguide sensor")
    win.resize(1200, 850)

    def echo_line(color):
        line = pg.InfiniteLine(angle=90, pen=pg.mkPen(color, width=2))
        line.hide()
        return line

    def search_window():
        return pg.LinearRegionItem(
            values=(start, stop), movable=False, brush=(255, 255, 255, 20), pen=pg.mkPen((255, 255, 255, 60))
        )

    def set_status(text, color):
        guide.setTitle(text, size="16pt", color=color)

    # 1. Raw frame
    raw = win.addPlot(title="Raw frame")
    raw.setYRange(0, 4096)
    raw.setXRange(0, FRAME_SIZE)
    raw.setLabel("left", "ADC raw amplitude (12-bit)")
    raw.showGrid(x=True, y=True, alpha=0.3)
    raw.addLegend(offset=(-10, 10))
    raw.addItem(search_window())
    frame_curve = raw.plot(pen=pg.mkPen("c", width=1.5), name="latest frame")
    background_curve = raw.plot(
        pen=pg.mkPen((255, 150, 0, 200), width=1.5), name="background"
    )  # On top: the echo hides it otherwise
    raw_echo = echo_line("r")
    raw.addItem(raw_echo)

    # 2. Detection signal
    win.nextRow()
    signal = win.addPlot(title="Detection signal")
    signal.setXLink(raw)
    signal.setLabel("bottom", "Sample index (distance = index * c / (2 * fs))")
    signal.setLabel("left", "Envelope (ADC)")
    signal.showGrid(x=True, y=True, alpha=0.3)
    signal.addItem(search_window())
    signal.addLine(y=threshold, pen=pg.mkPen((255, 80, 80), width=1, style=QtCore.Qt.PenStyle.DashLine))
    envelope_curve = signal.plot(pen=pg.mkPen("m", width=1.5))
    signal_echo = echo_line("r")
    signal.addItem(signal_echo)

    # 3. Waveguide
    win.nextRow()
    guide = win.addPlot()
    guide.setLabel("bottom", "Distance along the waveguide (cm)")
    guide.hideAxis("left")
    guide.setMouseEnabled(x=False, y=False)
    set_status("Recording the background: don't touch the sensor", "orange")
    guide.setYRange(-1.0, 1.0)
    margin = 2 * float(np.min(np.diff(hole_cm))) if len(hole_cm) > 1 else 1.0
    guide.setXRange(hole_cm.min() - margin, hole_cm.max() + margin)
    # Holes are shown by recolouring their spots: the spots never move or resize, as removing a spot
    # (ScatterPlotItem.clear/setData) does not repaint where it was and leaves it on screen
    idle_color, covered_color = np.array([70, 70, 70]), np.array([255, 60, 60])
    idle_pen, covered_pen = pg.mkPen((160, 160, 160)), pg.mkPen("w", width=2)
    holes = pg.ScatterPlotItem(
        x=hole_cm, y=np.zeros(len(hole_cm)), size=18, brush=pg.mkBrush(*idle_color), pen=idle_pen
    )
    guide.addItem(holes)
    for hole, x in zip(hole_ids, hole_cm):
        label = pg.TextItem(str(hole), anchor=(0.5, 0.0), color=(170, 170, 170))
        label.setPos(x, -0.3)
        guide.addItem(label)
    shown_trail = None

    def show_trail(covered_hole: int | None) -> None:
        """Covered hole bright red; the others from red (just released) back to grey over trail_s."""
        nonlocal shown_trail
        level = np.clip(1 - (time.monotonic() - last_covered) / trail_s, 0, 1)
        colors = np.round(idle_color + level[:, None] * (covered_color - idle_color)).astype(int)
        if shown_trail is not None and covered_hole == shown_trail[0] and np.array_equal(colors, shown_trail[1]):
            return  # Nothing visibly changed
        holes.setBrush([pg.mkBrush(*color) for color in colors])
        holes.setPen([covered_pen if hole == covered_hole else idle_pen for hole in hole_ids])
        shown_trail = (covered_hole, colors)

    distance_line = echo_line("y")
    guide.addItem(distance_line)

    win.ci.layout.setRowStretchFactor(0, 3)
    win.ci.layout.setRowStretchFactor(1, 2)
    win.ci.layout.setRowStretchFactor(2, 1)

    n_frames_last, t_last = 0, time.monotonic()
    shown_background = None

    def update():
        nonlocal n_frames_last, t_last, shown_background
        if errors:
            app.quit()
            return

        if (command := latest["command"]) is not None:
            raw.setTitle(
                f"Last command: {command.axis} {command.distance * 100:+.1f} cm at {command.speed * 100:.1f} cm/s"
                f" ({time.monotonic() - command.t_created:.1f} s ago)"
            )

        if latest["frame"] is not None:
            frame_curve.setData(latest["frame"])  # Only the latest; the plot cannot keep up with every frame

        detector = latest["detector"]
        result = detector.last_result if detector is not None else None
        if detector is not None and detector.background is not None and detector.background is not shown_background:
            background_curve.setData(detector.background)
            shown_background = detector.background

        if result is not None:
            averaged = detector.last_averaged
            envelope = wg_helpers.ema(np.abs(np.asarray(averaged[:stop], dtype=float) - detector.background[:stop]))
            envelope_curve.setData(envelope)
            signal.setTitle(f"Detection signal: amplitude {result['amplitude']:.1f}, threshold {threshold:.1f}")

            if result["detected"]:
                set_status(f"Hole {result['hole']}  —  {result['distance_cm']:.1f} cm", "r")
                distance_line.setPos(result["distance_cm"])
                distance_line.show()
                if result["echo_time_ms"] is not None:  # None for template matching
                    sample = result["echo_time_ms"] * samples_per_ms
                    for line in (raw_echo, signal_echo):
                        line.setPos(sample)
                        line.show()
            else:
                set_status("No touch", (200, 200, 200))
                for line in (raw_echo, signal_echo, distance_line):
                    line.hide()
        show_trail(result["hole"] if result is not None and result["detected"] else None)

        t = time.monotonic()
        if t - t_last >= 1.0:
            rate = (latest["n_frames"] - n_frames_last) / (t - t_last)
            win.setWindowTitle(f"Waveguide sensor ({rate:.0f} frames/s)")
            n_frames_last, t_last = latest["n_frames"], t

    thread = threading.Thread(target=sensor_loop, name="waveguide_sensor_loop", daemon=True)
    thread.start()
    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(16)
    try:
        app.exec()
    finally:
        stop_event.set()
        thread.join(timeout=1.0)
    if errors:
        raise errors[0]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default=DEFAULT_PORT, help=f"Serial port of the Teensy (default: {DEFAULT_PORT})")
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--calibrate", action="store_true", help="Record a calibration and fit it (CalibrationConfig)")
    mode.add_argument("--live-plot", action="store_true", help="Default mode, also plotting the raw frames")
    args = parser.parse_args()

    try:
        if args.calibrate:
            print("---- Calibration mode ----")
            calibrate(args.port, args.baudrate)
        elif args.live_plot:
            print("---- Live plot mode ----")
            print_frames_and_holes(args.port, args.baudrate, plot=True)
        else:
            print("---- Default mode ----")
            print_frames_and_holes(args.port, args.baudrate)

    except (FileNotFoundError, serial.SerialException) as e:
        parser.exit(1, f"Error: {e}\n")
    except KeyboardInterrupt:
        print()


if __name__ == "__main__":
    main()
