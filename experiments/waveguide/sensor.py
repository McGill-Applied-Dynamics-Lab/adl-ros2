"""Waveguide sensor processing.

Reads the Teensy over serial and turns the echo frames into teleop commands, in two layers:
1. Hole detection (on every frame, > 500 Hz): frame -> set of pressed holes
2. Gesture processing: changes of the pressed holes -> `MoveCommand` (axis, distance, speed)

Runs in its own process (`SensorProcess`), so the robot (rclpy, the GIL) never stalls the fast
loop. Imports no ROS: it can be developed and tested without the robot.

Standalone (the port defaults to the Teensy sending frames, see --port):
    pixi run -e wg python experiments/waveguide/sensor.py               # frame rate, pressed holes, commands
    pixi run -e wg python experiments/waveguide/sensor.py --live-plot   # raw frames
    pixi run -e wg python experiments/waveguide/sensor.py --live-match  # live trace vs the newest calibration/reference_*.csv
    pixi run -e wg python experiments/waveguide/sensor.py --calibrate   # guided calibration, see CalibrationConfig
"""

import argparse
import glob
import json
import multiprocessing as mp
import os
import queue
import signal
import sqlite3
import threading
import time
from collections import deque
from contextlib import closing
from dataclasses import asdict, dataclass
from pathlib import Path

import live_calibration
import numpy as np
import recording
import serial
import wg_helpers
from commands import MoveCommand


# --- Update configs as desired
@dataclass(frozen=True)
class CalibrationConfig:
    """Config for recording the Calibration (`--calibrate`)"""

    directory: Path = Path(__file__).resolve().parent / "calibration"
    filename: str = "demo_calibration2.db"  # Recording (SAVE_TO_SQLITE.py format); the fit is saved next to it (.npz)
    n_holes: int = 51
    n_passes: int = 3  # Recordings of every hole, >= 2 for a detection threshold
    first_hole_cm: float = 25.0  # Distance to hole 1
    hole_spacing_cm: float = 0.6  # Distance between holes

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
    n_average_frames: int = 2
    method: str = "leading_edge"  # One of wg_helpers.METHODS


@dataclass(frozen=True)
class GestureConfig:
    """Gesture processing (layer 2) settings: swipe -> MoveCommand(axis, distance, speed)."""

    hole_idx: int = 25  # Hole where the waveguide bends: it splits the swipes in two segments (see segment_axes)
    # Robot axis of the swipes starting at holes <= hole_idx, and of those after: ("y", "x") swaps them
    segment_axes: tuple[str, str] = ("y", "x")
    x_sign: int = -1  # +1: increasing hole index moves +x (whichever segment drives x); -1 to flip
    y_sign: int = 1  # +1: increasing hole index moves +y (whichever segment drives y); -1 to flip
    distance_scale: float = 0.005  # Robot motion [m] per cm of swipe (0.01: 1:1)
    max_speed: float = 0.15  # [m/s] Faster swipes are capped (wg_teleop.MAX_SPEED)
    max_distance: float = 0.2  # [m] Longer swipes are capped (wg_teleop.MAX_DISTANCE)
    min_holes: int = 2  # Holes a swipe must span (start and end included); shorter touches send nothing
    debounce_s: float = 0.3  # A touch (swipe or z hold) ends after no hole is detected for this long

    # z: hold a z hole; on release, move z by z_speed * hold time, at z_speed. The holes from the lowest to the
    # highest z hole (the z zone, with the bend) are reserved for z: swipes stop before them
    z_up_holes: tuple[int, ...] = (21, 22, 23, 24)
    z_down_holes: tuple[int, ...] = (26, 27, 28, 29, 30)
    z_speed: float = 0.1  # [m/s] z motion per second of hold, and the speed of that motion
    z_min_hold_s: float = 0.2  # Shorter z touches send nothing

    @property
    def z_zone(self) -> range:
        z_holes = self.z_up_holes + self.z_down_holes
        return range(min(z_holes), max(z_holes) + 1)


# Configs
DETECTION_CFG = DetectionConfig()
GESTURE_CFG = GestureConfig()
CAL_CFG = CalibrationConfig()


DEFAULT_PORT = "auto"  # The connected Teensy that sends frames (`find_frame_port`)
TEENSY_PORTS = "/dev/serial/by-id/usb-Teensyduino*"  # By serial number: ttyACM<n> depends on the plug-in order
PORT_PROBE_S = 0.5  # Listening time per port when looking for the frames (~20 frames/s)
NO_FRAME_WARN_S = 2.0  # Warn when no frame arrived for this long
DEFAULT_BAUDRATE = 3_000_000
SERIAL_TIMEOUT = 0.1  # s, so the loop checks the stop event even without data

# Teensy packet: 3-byte header + FRAME_SIZE uint16 samples (little-endian, 12-bit ADC)
FRAME_SIZE = 1500
HEADER = b"\xaa\x55\x00"
PACKET_SIZE = len(HEADER) + 2 * FRAME_SIZE

THRESHOLD_SCALE = 1.2


def teensy_ports() -> list[str]:
    """Serial ports of the connected Teensys (any /dev/ttyACM* if none is identified as one)."""
    return sorted(glob.glob(TEENSY_PORTS)) or sorted(glob.glob("/dev/ttyACM*"))


def find_frame_port(baudrate: int) -> str:
    """The port of the Teensy sending frames (TEENSY_FIRMWARE, or TEENSY_BLUETOOTH while its port is open), among
    the connected ones.

    Another Teensy can be connected too, and ttyACM0/1 swap with the plug-in order, so each one is listened to for
    `PORT_PROBE_S` (the first one sending frames is used: give --port if both are).
    """
    ports = teensy_ports()
    if not ports:
        raise FileNotFoundError("No Teensy found: is it plugged in? (--port to give the port)")
    for port in ports:
        try:
            with serial.Serial(port, baudrate, timeout=0.05) as ser:
                ser.reset_input_buffer()
                data, t_end = bytearray(), time.monotonic() + PORT_PROBE_S
                while time.monotonic() < t_end and HEADER not in data:
                    data += ser.read(max(ser.in_waiting, 1))
        except serial.SerialException:  # Busy (e.g. another script) or gone
            continue
        if HEADER in data:
            return os.path.realpath(port)
    raise FileNotFoundError(f"No Teensy sending frames among {ports}: is TEENSY_FIRMWARE flashed? (--port)")


def check_port(port: str, baudrate: int = DEFAULT_BAUDRATE) -> str:
    """The port to open: `port`, or the Teensy sending frames if "auto". FileNotFoundError if there is none."""
    if port == "auto":
        port = find_frame_port(baudrate)
        print(f"Teensy sending frames: {port}")
    if not Path(port).exists():
        raise FileNotFoundError(f"Serial port {port} not found: is the Teensy plugged in? (--port to use another one)")
    return port


def open_serial(port: str, baudrate: int) -> serial.Serial:
    port = check_port(port, baudrate)

    print(f"Waveguide sensor loop started on {port} at {baudrate} baud (Ctrl+C to quit)")

    return serial.Serial(port, baudrate, timeout=SERIAL_TIMEOUT)


def load_calibration(path: Path = CAL_CFG.npz_path) -> dict:
    """The fitted calibration (`wg_helpers.load_calibration`); FileNotFoundError if there is none yet."""
    if not path.exists():
        raise FileNotFoundError(f"No calibration at {path}: run sensor.py --calibrate first")

    calibration_params = wg_helpers.load_calibration(path)

    calibration_params["threshold"] = float(calibration_params["threshold"]) * THRESHOLD_SCALE

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

    def __init__(
        self, calibration_file_path: Path, detection_cfg: DetectionConfig = DETECTION_CFG, required: bool = True
    ):
        if detection_cfg.method not in wg_helpers.METHODS:
            raise ValueError(f"Detection method must be one of {wg_helpers.METHODS}, got {detection_cfg.method}")

        # Load calibration; without one (and not `required`), nothing is detected
        if not required and not calibration_file_path.exists():
            print(f"No calibration at {calibration_file_path}: detection off")
            calibration_params = None
        else:
            calibration_params = load_calibration(calibration_file_path)
        self.calibration_params = calibration_params
        self.calibration_path = calibration_file_path

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
            # print(
            #     f"Detected hole {self.last_result['hole']} at {self.last_result['distance_cm']:.1f} cm (amplitude {self.last_result['amplitude']:.1f})"
            # )

            return frozenset({self.last_result["hole"]})

        else:
            return frozenset()


# --- Layer 2: gesture processing ---
@dataclass(frozen=True)
class Swipe:
    """A finished touch and its command: a swipe over holes `start_hole` -> `end_hole` (end stopped before the
    z zone), or a z hold (`kind` "z", first and last hole touched)."""

    t: float  # Time the touch ended (after the debounce), CLOCK_MONOTONIC [s]
    start_hole: int
    end_hole: int
    duration_s: float  # Swipe: from the first touch to reaching the end hole. z hold: hold time
    command: MoveCommand | None  # None: ignored, see `ignored_reason`
    ignored_reason: str = ""
    kind: str = "swipe"  # "swipe" or "z"


class GestureProcessor:
    """Turn touches along the waveguide into teleop commands: swipes (x, y) and holds (z).

    A touch starts at the first detected hole and ends once no hole is detected for `debounce_s`. Its first
    hole tells which gesture it is.

    Swipe (first hole outside the z zone). Only its first and last hole count (noisy or skipped holes in
    between do not matter):
    - axis: `segment_axes[0]` if it starts at or before the bend (`hole_idx`), else `segment_axes[1]`. A swipe
      stops before the z zone.
    - distance: swipe length [cm, from the calibration] * `distance_scale`, signed by the direction
    - speed: swipe speed (start to reaching the last hole, without the debounce) * `distance_scale`
    Distance and speed are capped at `max_distance` and `max_speed`; swipes shorter than `min_holes` send nothing.

    z hold (first hole in the z zone): moves z by `z_speed` * hold time (first to last touch, without the
    debounce), at `z_speed`. The detection may jump between holes while held (e.g. 13 <-> 14): the direction
    is the one of the z holes detected most often. Holds shorter than `z_min_hold_s` send nothing.
    """

    def __init__(self, calibration: dict, gesture_cfg: GestureConfig = GESTURE_CFG):
        if sorted(gesture_cfg.segment_axes) != ["x", "y"]:
            raise ValueError(f"segment_axes must be ('x', 'y') or ('y', 'x'), got {gesture_cfg.segment_axes}")
        self.cfg = gesture_cfg
        self.hole_ids = np.asarray(calibration["hole_ids"]).astype(int)
        self.hole_cm = dict(zip(self.hole_ids.tolist(), np.asarray(calibration["hole_cm"]).tolist()))
        swipe_holes = self.hole_ids[~np.isin(self.hole_ids, list(gesture_cfg.z_zone))]
        self._before_bend_holes = swipe_holes[swipe_holes <= gesture_cfg.hole_idx]
        self._after_bend_holes = swipe_holes[swipe_holes > gesture_cfg.hole_idx]
        self._reset()

    def _reset(self) -> None:
        self._t_start = None  # Time the touch started (first hole detected), None: no touch
        self._t_last_touch = None  # Last time a hole was detected
        self._reached = {}  # Hole -> time it was first reached in this touch
        self._start_hole = self._last_hole = None
        self._z_counts = {1: 0, -1: 0}  # z hold: frames on the up (+1) and down (-1) holes

    def update(self, t: float, pressed: frozenset[int]) -> Swipe | None:
        """Call on every frame (the touch end needs the time to advance); returns the touch when it ends."""
        if pressed:
            hole = min(pressed)  # One hole at a time from the detection
            if self._t_start is None:
                self._t_start, self._start_hole = t, hole
            self._reached.setdefault(hole, t)
            self._last_hole, self._t_last_touch = hole, t
            if hole in self.cfg.z_up_holes:
                self._z_counts[1] += 1
            elif hole in self.cfg.z_down_holes:
                self._z_counts[-1] += 1
            return None

        if self._t_start is None or t - self._t_last_touch < self.cfg.debounce_s:
            return None
        is_z = self._start_hole in self.cfg.z_zone
        swipe = self._finish_z_hold(t) if is_z else self._finish_swipe(t)
        self._reset()
        return swipe

    def _finish_z_hold(self, t: float) -> Swipe:
        cfg = self.cfg
        hold_s = self._t_last_touch - self._t_start

        if self._z_counts[1] == self._z_counts[-1]:  # Only the bend, or as long on both
            reason = "no z direction"
        elif hold_s < cfg.z_min_hold_s:
            reason = f"held less than {cfg.z_min_hold_s:g} s"
        else:
            direction = 1 if self._z_counts[1] > self._z_counts[-1] else -1
            distance = float(np.clip(direction * cfg.z_speed * hold_s, -cfg.max_distance, cfg.max_distance))
            speed = min(cfg.z_speed, cfg.max_speed)
            print(f"z hold {hold_s:.2f} s: z {distance * 100:+.1f} cm at {speed * 100:.1f} cm/s")
            return Swipe(t, self._start_hole, self._last_hole, hold_s, MoveCommand("z", distance, speed), kind="z")

        print(f"z hold {hold_s:.2f} s: {reason}, ignored")
        return Swipe(t, self._start_hole, self._last_hole, hold_s, None, reason, kind="z")

    def _finish_swipe(self, t: float) -> Swipe:
        cfg = self.cfg
        start, end = self._start_hole, self._last_hole

        # Axis from the start hole; a swipe stops at the end of its segment (before the z zone)
        before_bend = start <= cfg.hole_idx
        segment = self._before_bend_holes if before_bend else self._after_bend_holes
        end = int(np.clip(end, segment.min(), segment.max()))

        # Time to reach the end hole (the last touched one if the end was clipped at the bend and never detected)
        duration = self._reached.get(end, self._reached[self._last_hole]) - self._t_start

        if abs(end - start) + 1 < cfg.min_holes:
            reason = f"shorter than {cfg.min_holes} holes"
            print(f"Swipe {start} -> {end}: {reason}, ignored")
            return Swipe(t, start, end, duration, None, reason)

        axis = cfg.segment_axes[0] if before_bend else cfg.segment_axes[1]
        sign = cfg.x_sign if axis == "x" else cfg.y_sign
        distance = sign * (self.hole_cm[end] - self.hole_cm[start]) * cfg.distance_scale
        distance = float(np.clip(distance, -cfg.max_distance, cfg.max_distance))
        speed = min(abs(distance) / duration, cfg.max_speed) if duration > 0 else cfg.max_speed

        print(f"Swipe {start} -> {end} in {duration:.2f} s: {axis} {distance * 100:+.1f} cm at {speed * 100:.1f} cm/s")
        return Swipe(t, start, end, duration, MoveCommand(axis, distance, speed))


# --- Process ---
def run_sensor_loop(
    port: str,
    baudrate: int,
    on_holes,
    on_command,
    stop_event=None,
    on_frame=None,
    require_calibration: bool = True,
    on_swipe=None,
    paused=None,
    reload_event=None,
    recorder=None,
) -> None:
    """Read the serial port and run both layers until `stop_event` is set (or forever).

    `on_holes(HoleState)` is called when the pressed holes change, `on_command(MoveCommand)` for
    each command, `on_frame(t, frame, detector)` (if given) for every frame, after the detection
    (`detector.last_result`, `detector.background`, ...), and `on_swipe(Swipe)` (if given) for every
    finished swipe, ignored ones included. Without a calibration, raises
    FileNotFoundError if `require_calibration`, else runs with the detection off.

    While `paused` (an Event, if given) is set, both layers are off (no holes, no commands): only `on_frame`
    is called, e.g. while calibrating. Setting `reload_event` loads the calibration again (and records the
    background again); the loop clears it. `recorder` (a `recording.SensorRecorder`, if given) records every
    frame's estimate and every swipe.
    """

    def layers():
        detector = HoleDetector(CAL_CFG.npz_path, required=require_calibration)
        processor = GestureProcessor(detector.calibration_params) if detector.calibration_params is not None else None
        return detector, processor

    detector, processor = layers()
    pressed = frozenset()
    was_paused = False

    with open_serial(port, baudrate) as ser:
        reader = FrameReader(ser)
        reader.flush()  # Drop what accumulated before we started
        t_last_frame, warned = time.monotonic(), False
        while stop_event is None or not stop_event.is_set():
            frame = reader.read_frame()
            t = time.monotonic()

            if frame is None and not warned and t - t_last_frame > NO_FRAME_WARN_S:
                print(
                    f"No frame from {ser.port} for {NO_FRAME_WARN_S:g} s: is it the Teensy running TEENSY_FIRMWARE? "
                    f"Connected: {teensy_ports()}"
                )
                warned = True
            if reload_event is not None and reload_event.is_set():
                reload_event.clear()
                detector, processor = layers()
            if paused is not None and paused.is_set():
                if pressed:
                    pressed = frozenset()
                    on_holes(HoleState(t, pressed))
                was_paused = True
                if frame is not None:
                    t_last_frame, warned = t, False
                    if on_frame is not None:
                        on_frame(t, frame, detector)
                continue
            if was_paused:  # A touch from before the pause must not end as a swipe
                was_paused = False
                processor = GestureProcessor(detector.calibration_params) if processor is not None else None

            if frame is not None:
                t_last_frame, warned = t, False
                new_pressed = detector.update(frame)
                if recorder is not None:
                    recorder.frame(t, detector, new_pressed)
                if on_frame is not None:
                    on_frame(t, frame, detector)

                if new_pressed != pressed:
                    pressed = new_pressed
                    on_holes(HoleState(t, pressed, detector.last_result["distance_cm"] if pressed else None))

            # Every iteration, even without a frame: the swipe ends after a time without holes
            swipe = processor.update(t, pressed) if processor is not None else None
            if swipe is not None:
                if recorder is not None:
                    recorder.swipe(swipe)
                if on_swipe is not None:
                    on_swipe(swipe)
                if swipe.command is not None:
                    on_command(swipe.command)


def _process_main(port: str, baudrate: int, commands: mp.Queue, stop_event, plot: bool, samples, run_dir) -> None:
    # Ctrl+C reaches the whole process group: the parent handles it and stops us with stop_event. (In the
    # plot, a KeyboardInterrupt would also be raised inside a Qt callback, which aborts the process.)
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    recorder = recording.SensorRecorder(recording.QueueLog(samples), run_dir) if samples is not None else None
    run = run_with_live_plot if plot else run_sensor_loop
    run(port, baudrate, on_holes=lambda state: None, on_command=commands.put, stop_event=stop_event, recorder=recorder)


class SensorProcess:
    """Runs `run_sensor_loop` in a separate process and hands its commands over a queue.

    With `plot`, the process also shows the live plot (`run_with_live_plot`); closing its window ends the process.
    With `logger` (a started `ExperimentLogger`), the process records what it sees (`recording.SensorRecorder`)
    into it, through a queue drained by this process.
    """

    def __init__(self, port: str = DEFAULT_PORT, baudrate: int = DEFAULT_BAUDRATE, plot: bool = False, logger=None):
        # Checked here, not only in the child: there it would just end the process
        port = check_port(port, baudrate)
        if not plot:  # With the plot, it can be calibrated from its panel
            load_calibration()
        # Spawn, not fork: the child must not inherit the parent's rclpy state and threads
        ctx = mp.get_context("spawn")
        self._commands = ctx.Queue()
        self._stop_event = ctx.Event()
        samples = ctx.Queue() if logger is not None else None
        self._drain = recording.QueueDrain(samples, logger) if logger is not None else None
        self._process = ctx.Process(
            target=_process_main,
            args=(port, baudrate, self._commands, self._stop_event, plot, samples, logger and logger.run_dir),
            name="waveguide_sensor",
            daemon=True,
        )

    def start(self) -> None:
        if self._drain is not None:
            self._drain.start()
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
        if self._drain is not None:
            self._drain.stop()  # What the process recorded until the end


# --- Calibration ---
def record_calibration(port: str, baudrate: int, config: CalibrationConfig) -> None:
    """Guided single-touch calibration recording, in the SAVE_TO_SQLITE.py format `wg_helpers.calibrate` reads.

    Each pass records nothing covered (key [0]), then holes 1..n_holes one at a time
    (wg_helpers.CYCLES_PER_PRESS frames each).
    """
    n_frames = wg_helpers.CYCLES_PER_PRESS
    with closing(sqlite3.connect(config.db_path)) as conn, open_serial(port, baudrate) as ser:
        live_calibration.create_signals_table(conn)
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


def run_metadata() -> dict:
    """The sensor configs, for the metadata of a recording (`--record`)."""
    configs = {"detection": DETECTION_CFG, "gesture": GESTURE_CFG, "calibration": CAL_CFG}
    return {name: json.loads(json.dumps(asdict(cfg), default=str)) for name, cfg in configs.items()}


# --- Standalone modes ---
def print_frames_and_holes(port: str, baudrate: int, plot: bool = False, recorder=None) -> None:
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
        run_with_live_plot(
            port, baudrate, on_holes=on_holes, on_command=on_command, on_frame=on_frame, recorder=recorder
        )
    else:
        run_sensor_loop(
            port,
            baudrate,
            on_holes=on_holes,
            on_command=on_command,
            on_frame=on_frame,
            require_calibration=False,
            recorder=recorder,
        )


@dataclass(frozen=True)
class PlotStyle:
    """Look of the live plot (`run_with_live_plot`), made for recordings (e.g. side by side with the robot video).

    Colors: Okabe-Ito palette (colorblind-safe), used with the same meaning in every panel: blue the signal,
    orange the background and z, vermillion a touch, green a command, purple y.
    """

    theme: str = "light"  # "light" (paper) or "dark"
    window_size: tuple[int, int] = (960, 1080)  # Half of a 1080p side-by-side video
    font_family: str = "Arial"
    font_pt: int = 13  # Axes and labels
    title_pt: int = 15
    status_pt: int = 22  # Covered hole / no touch, above the waveguide
    line_width: float = 2.0
    # Antialiasing the 1500-sample traces makes each redraw ~35 ms instead of ~6: the plot then cannot keep up, and
    # (GIL) neither can the sensor loop. The text, holes and arrows are always antialiased
    antialias_traces: bool = False
    hole_label_every: int = 5  # Hole numbers shown under the waveguide: the first, then every n-th

    def colors(self) -> dict:
        light = self.theme == "light"
        return dict(
            background="#FFFFFF" if light else "#121212",
            foreground="#222222" if light else "#E6E6E6",
            muted="#8C8C8C" if light else "#9E9E9E",
            signal="#0072B2" if light else "#56B4E9",
            reference="#E69F00",  # Background frame, z
            touch="#D55E00",
            command="#009E73",
            y="#CC79A7",
            hole_idle="#E3E3E3" if light else "#3A3A3A",
            hole_edge="#7F7F7F",
            hole_covered_edge="#000000" if light else "#FFFFFF",
            shade=(0, 0, 0, 14) if light else (255, 255, 255, 18),  # Echo search window
        )


PLOT_STYLE = PlotStyle()


class _PlotTools:
    """The look shared by the sensor plots (`PlotStyle`): Qt app, colors, fonts, pens, titles and axes."""

    def __init__(self, style: PlotStyle, app_name: str):
        import pyqtgraph as pg  # Only needed for the plots
        from pyqtgraph.Qt import QtCore, QtGui

        self.pg, self.QtCore, self.QtGui, self.style = pg, QtCore, QtGui, style
        self.c = c = style.colors()
        pg.setConfigOptions(antialias=True, background=c["background"], foreground=c["foreground"])
        self.app = pg.mkQApp(app_name)
        self.font = QtGui.QFont(style.font_family, style.font_pt)
        self.small_font = QtGui.QFont(style.font_family, style.font_pt - 3)
        self.label_style = {
            "font-size": f"{style.font_pt}pt",
            "font-family": style.font_family,
            "color": c["foreground"],
        }

    def rgb(self, color: str) -> np.ndarray:
        return np.array(self.QtGui.QColor(color).getRgb()[:3])

    def tint(self, color: str, alpha: int) -> tuple:
        return (*self.rgb(color), alpha)

    def pen(self, color, width=None, dashed=False):
        Qt = self.QtCore.Qt
        return self.pg.mkPen(
            color,
            width=self.style.line_width if width is None else width,
            style=Qt.PenStyle.DashLine if dashed else Qt.PenStyle.SolidLine,
        )

    def text_item(self, text, color, anchor, font_=None):
        item = self.pg.TextItem(text, anchor=anchor, color=color)
        item.setFont(self.font if font_ is None else font_)
        return item

    def set_title(self, plot, text, color=None, size=None):
        size = self.style.title_pt if size is None else size
        plot.setTitle(text, size=f"{size}pt", color=self.c["foreground"] if color is None else color)

    def style_axes(self, plot, x_label=None, y_label=None):
        for name in ("left", "bottom"):
            axis = plot.getAxis(name)
            axis.setTickFont(self.font)
            axis.setPen(self.pen(self.c["muted"], width=1))
            axis.setTextPen(self.c["foreground"])
        if x_label:
            plot.setLabel("bottom", x_label, **self.label_style)
        if y_label:
            plot.setLabel("left", y_label, **self.label_style)
        plot.getAxis("left").setWidth(80)  # Same width in every panel, so their x axes line up
        plot.showGrid(x=True, y=True, alpha=0.15)
        plot.setMenuEnabled(False)
        plot.hideButtons()


def run_with_live_plot(
    port: str,
    baudrate: int,
    on_holes,
    on_command,
    on_frame=None,
    trail_s: float = 2.0,
    swipe_display_s: float = 3.0,
    stop_event=None,
    style: PlotStyle = PLOT_STYLE,
    recorder=None,
) -> None:
    """`run_sensor_loop` with a live view of the processing, refreshed at ~60 Hz (look: `PlotStyle`):

    1. Echo: the latest frame (blue) and the background (orange) against the echo time, the echo search window
       (shaded) and the detected echo (vermillion line)
    2. Echo envelope: envelope of the averaged frame minus the background, as `wg_helpers.estimate` computes it,
       against the detection threshold (dashed)
    3. Waveguide: the holes (`CalibrationConfig` layout), with the covered hole and its distance above. The
       covered hole is vermillion; released holes fade back over `trail_s` [s], showing the swipe. The last
       touch is an arrow (swipe: along the waveguide, z hold: up or down) with its command (green) or why it
       was ignored (grey), fading out over `swipe_display_s` [s]. The segments are tinted: x (blue), the z zone
       (orange, +z and -z holes) and y (purple); the bend (`GestureConfig.hole_idx`) is the dashed line.

    Calibration panel (the ⚙ tab on the right, or C): number of holes (evenly spaced, plus the z holes) and of
    passes, then Start. The waveguide shows the hole to press (green) and moves on by itself
    (`live_calibration.CalibrationRecorder`); Space records the current step now, Esc cancels. Detection and
    commands are paused meanwhile. The new calibration is saved (`CalibrationConfig.npz_path`, the previous one
    kept as a backup) and used right away; the background is then recorded again (don't touch).

    The loop runs in a background thread (Qt needs the main thread), so the plot shows what the processing
    sees. The callbacks are passed on to the loop, and called from its thread. Runs until the window is
    closed or `stop_event` (if given) is set. Without a calibration, it runs with the detection off.
    """
    lc = live_calibration
    port = check_port(port, baudrate)
    calibration = load_calibration(CAL_CFG.npz_path) if CAL_CFG.npz_path.exists() else None
    import pyqtgraph as pg  # Only needed here
    from pyqtgraph.Qt import QtCore, QtGui, QtWidgets

    # Written by the sensor loop thread, read by the plot
    latest = {"frame": None, "detector": None, "n_frames": 0, "swipe": None}
    # Calibration from the panel: recorder (fed by the sensor thread), then the fit (its own thread)
    session = {"recorder": None, "fit_thread": None, "result": None, "error": None}
    paused, reload_event = threading.Event(), threading.Event()

    def plot_on_swipe(swipe):
        latest["swipe"] = swipe

    errors = []
    if stop_event is None:
        stop_event = threading.Event()

    # Holes drawn from the CalibrationConfig layout: also those not (yet) calibrated
    hole_ids = np.arange(1, CAL_CFG.n_holes + 1)
    hole_cm = CAL_CFG.first_hole_cm + CAL_CFG.hole_spacing_cm * (hole_ids - 1)
    hole_index = {int(hole): i for i, hole in enumerate(hole_ids)}
    last_covered = np.full(len(hole_ids), -np.inf)  # Last time each hole was covered (CLOCK_MONOTONIC) [s]

    def plot_on_frame(t, frame, detector):
        latest["frame"], latest["detector"] = frame, detector
        latest["n_frames"] += 1
        recorder = session["recorder"]
        if recorder is not None:
            recorder.update(t, frame)
        # Every frame, not every redraw: a fast swipe can cover a hole for less than a redraw period
        result = detector.last_result
        if not paused.is_set() and result is not None and result["detected"] and result["hole"] in hole_index:
            last_covered[hole_index[result["hole"]]] = t
        if on_frame is not None:
            on_frame(t, frame, detector)

    def sensor_loop():
        try:
            run_sensor_loop(
                port,
                baudrate,
                on_holes=on_holes,
                on_command=on_command,
                stop_event=stop_event,
                on_frame=plot_on_frame,
                require_calibration=False,
                on_swipe=plot_on_swipe,
                paused=paused,
                reload_event=reload_event,
                recorder=recorder,
            )
        except Exception as e:
            errors.append(e)

    time_ms = np.arange(FRAME_SIZE) / (wg_helpers.SAMPLING_RATE_HZ / 1000)  # x axis of the echo plots
    view = {"stop": FRAME_SIZE}  # Calibration-dependent: envelope samples shown (search window stop)

    # --- Style
    tools = _PlotTools(style, "Waveguide sensor")
    c, app, small_font = tools.c, tools.app, tools.small_font
    rgb, tint, pen, text_item, set_title, style_axes = (
        tools.rgb,
        tools.tint,
        tools.pen,
        tools.text_item,
        tools.set_title,
        tools.style_axes,
    )

    window = QtWidgets.QWidget()
    window.setWindowTitle("Waveguide sensor")
    window.setStyleSheet(f"background-color: {c['background']}; color: {c['foreground']};")
    window_layout = QtWidgets.QHBoxLayout(window)
    window_layout.setContentsMargins(0, 0, 0, 0)
    window_layout.setSpacing(0)
    win = pg.GraphicsLayoutWidget()
    window_layout.addWidget(win, 1)
    win.ci.setContentsMargins(16, 12, 24, 12)
    win.ci.setSpacing(18)

    def echo_line(color):
        line = pg.InfiniteLine(angle=90, pen=pen(color))
        line.hide()
        return line

    search_windows = []

    def search_window():
        region = pg.LinearRegionItem(values=(0, 1), movable=False, brush=c["shade"], pen=pg.mkPen(None))
        region.setZValue(-10)
        search_windows.append(region)
        return region

    # 1. Echo (raw frame)
    raw = win.addPlot()
    set_title(raw, "Echo")
    style_axes(raw, y_label="ADC counts")
    raw.setYRange(0, 4096)
    raw.setXRange(0, time_ms[-1], padding=0)
    raw.setMouseEnabled(x=False, y=False)
    legend = raw.addLegend(offset=(-10, 10), labelTextSize=f"{style.font_pt - 2}pt", labelTextColor=c["foreground"])
    legend.setBrush(tint(c["background"], 220))
    raw.addItem(search_window())
    trace = dict(antialias=style.antialias_traces, autoDownsample=True, downsampleMethod="peak", clipToView=True)
    background_curve = raw.plot(pen=pen(c["reference"], width=style.line_width * 0.6), name="background", **trace)
    frame_curve = raw.plot(pen=pen(c["signal"], width=style.line_width * 0.75), name="echo", **trace)
    background_curve.setZValue(1)  # On top: the echo hides it otherwise
    raw_echo = echo_line(c["touch"])
    raw.addItem(raw_echo)

    # 2. Echo envelope (detection signal)
    win.nextRow()
    signal = win.addPlot()
    set_title(signal, "Echo envelope")
    style_axes(signal, x_label="Echo time (ms)", y_label="Envelope (ADC)")
    signal.setXLink(raw)
    signal.setMouseEnabled(x=False, y=False)
    signal.addItem(search_window())
    threshold_line = signal.addLine(y=0, pen=pen(c["touch"], width=1.5, dashed=True))
    threshold_label = text_item("threshold", c["touch"], anchor=(1.0, 1.0), font_=small_font)
    signal.addItem(threshold_label)
    envelope_curve = signal.plot(pen=pen(c["signal"]), **trace)
    signal_echo = echo_line(c["touch"])
    signal.addItem(signal_echo)

    def set_threshold(value: float | None) -> None:
        threshold_line.setVisible(value is not None)
        threshold_label.setVisible(value is not None)
        if value is not None:
            threshold_line.setPos(value)
            threshold_label.setPos(time_ms[-1], value)

    def apply_calibration(cal: dict | None) -> None:
        """Search window and threshold of `cal` (None: no calibration, hidden)."""
        if cal is None:
            view["stop"] = FRAME_SIZE
            for region in search_windows:
                region.hide()
            set_threshold(None)
            return
        start, stop = int(cal["search_start"]), int(cal["search_stop"])
        view["stop"] = stop
        for region in search_windows:
            region.setRegion((time_ms[start], time_ms[min(stop, FRAME_SIZE - 1)]))
            region.show()
        set_threshold(float(cal["threshold"]))

    apply_calibration(calibration)

    # 3. Waveguide
    win.nextRow()
    guide = win.addPlot()
    style_axes(guide, x_label="Distance along the waveguide (cm)")
    guide.hideAxis("left")
    guide.showGrid(x=False, y=False)
    guide.setMouseEnabled(x=False, y=False)

    def set_status(text, color):
        set_title(guide, text, color=color, size=style.status_pt)

    set_status("Recording the background: don't touch", c["reference"])
    guide.setYRange(-1.0, 1.0)
    margin = 2 * float(np.min(np.diff(hole_cm))) if len(hole_cm) > 1 else 1.0
    x_min, x_max = hole_cm.min() - margin, hole_cm.max() + margin
    guide.setXRange(x_min, x_max, padding=0)
    # Holes are shown by recolouring their spots: the spots never move or resize, as removing a spot
    # (ScatterPlotItem.clear/setData) does not repaint where it was and leaves it on screen
    idle_color, covered_color = rgb(c["hole_idle"]), rgb(c["touch"])
    idle_pen, covered_pen = pg.mkPen(c["hole_edge"], width=1.2), pg.mkPen(c["hole_covered_edge"], width=2.5)
    holes = pg.ScatterPlotItem(
        x=hole_cm, y=np.zeros(len(hole_cm)), size=20, brush=pg.mkBrush(*idle_color), pen=idle_pen
    )
    holes.setZValue(5)
    guide.addItem(holes)
    for i, (hole, x) in enumerate(zip(hole_ids, hole_cm)):
        if i > 0 and hole % style.hole_label_every:
            continue
        label = text_item(str(hole), c["muted"], anchor=(0.5, 0.0), font_=small_font)
        label.setPos(x, -0.22)
        guide.addItem(label)
    shown_holes = None  # What the hole spots show now, to skip identical redraws

    def show_holes(colors: np.ndarray, outlined: int | None) -> None:
        nonlocal shown_holes
        if shown_holes is not None and outlined == shown_holes[0] and np.array_equal(colors, shown_holes[1]):
            return  # Nothing visibly changed
        holes.setBrush([pg.mkBrush(*color) for color in colors])
        holes.setPen([covered_pen if hole == outlined else idle_pen for hole in hole_ids])
        shown_holes = (outlined, colors)

    def show_trail(covered_hole: int | None) -> None:
        """Covered hole in the touch color; the others from it (just released) back to idle over trail_s."""
        level = np.clip(1 - (time.monotonic() - last_covered) / trail_s, 0, 1)
        show_holes(np.round(idle_color + level[:, None] * (covered_color - idle_color)).astype(int), covered_hole)

    # Segments (GestureConfig): x (blue) up to the z zone (orange, +z and -z holes), y (purple) after it; the bend
    # (hole_idx) is the dashed line
    cfg = GESTURE_CFG
    hole_cm_of = dict(zip(hole_ids.astype(int).tolist(), hole_cm.tolist()))

    def boundary(before, after):
        """Middle between the last hole <= `before` and the first hole >= `after` (or the plot edge)."""
        lo, hi = hole_cm[hole_ids <= before], hole_cm[hole_ids >= after]
        return (lo.max() + hi.min()) / 2 if len(lo) and len(hi) else (x_min if not len(lo) else x_max)

    bend_x = boundary(cfg.hole_idx, cfg.hole_idx + 1)
    zone_lo, zone_hi = boundary(cfg.z_zone.start - 1, cfg.z_zone.start), boundary(cfg.z_zone.stop - 1, cfg.z_zone.stop)
    for (lo, hi), color in (
        ((x_min, zone_lo), c["signal"]),
        ((zone_lo, zone_hi), c["reference"]),
        ((zone_hi, x_max), c["y"]),
    ):
        segment = pg.LinearRegionItem(values=(lo, hi), movable=False, brush=tint(color, 30), pen=pg.mkPen(None))
        segment.setZValue(-10)  # Behind the holes
        guide.addItem(segment)
    guide.addItem(pg.InfiniteLine(pos=bend_x, angle=90, pen=pen(c["muted"], width=1.5, dashed=True)))
    z_up_x = float(np.mean([hole_cm_of[h] for h in cfg.z_up_holes if h in hole_cm_of] or [bend_x]))
    z_down_x = float(np.mean([hole_cm_of[h] for h in cfg.z_down_holes if h in hole_cm_of] or [bend_x]))
    for text, x, anchor, color in (
        (f"◀ {cfg.segment_axes[0]}", zone_lo - 0.3, (1.0, 0.0), c["signal"]),
        ("+z", z_up_x, (0.5, 0.0), c["reference"]),
        ("−z", z_down_x, (0.5, 0.0), c["reference"]),
        (f"{cfg.segment_axes[1]} ▶", zone_hi + 0.3, (0.0, 0.0), c["y"]),
    ):
        label = text_item(text, color, anchor=anchor)
        label.setPos(x, -0.55)
        guide.addItem(label)

    distance_line = echo_line(c["touch"])
    guide.addItem(distance_line)

    # Last touch: an arrow over its holes (swipe: along the waveguide, z hold: up or down), with its command
    # (green) or why it was ignored (grey); fades out
    swipe_y = 0.5
    swipe_line = pg.PlotCurveItem()
    swipe_head = pg.ArrowItem(headLen=20, tipAngle=40, pen=None)
    swipe_label = text_item("", c["command"], anchor=(0.5, 1.0))
    swipe_items = (swipe_line, swipe_head, swipe_label)
    for item in swipe_items:
        item.hide()
        guide.addItem(item)
    shown_swipe = None

    def show_swipe():
        nonlocal shown_swipe
        swipe = latest["swipe"]
        if swipe is None:
            return
        if swipe is not shown_swipe:
            command = swipe.command
            color = c["command"] if command is not None else c["muted"]
            if command is not None:
                text = f"{command.axis} {command.distance * 100:+.1f} cm  @ {command.speed * 100:.1f} cm/s"
            else:
                text = f"ignored: {swipe.ignored_reason}"

            if swipe.kind == "z":  # Vertical arrow over the z holes, up or down
                up = command is None or command.distance > 0
                x = (z_up_x if up else z_down_x) if command is not None else bend_x
                y0, y1 = (0.2, 0.85) if up else (0.85, 0.2)
                swipe_line.setData([x, x], [y0, y1], pen=pen(color, width=4))
                swipe_head.setStyle(angle=90 if up else -90, brush=pg.mkBrush(color))
                swipe_head.setPos(x, y1)
                swipe_head.setVisible(command is not None)
                swipe_label.setAnchor((0.0, 0.5))
                swipe_label.setPos(x + 0.4, 0.55)
            else:  # Arrow along the waveguide, from the start to the end hole
                x0, x1 = hole_cm_of.get(swipe.start_hole, bend_x), hole_cm_of.get(swipe.end_hole, bend_x)
                swipe_line.setData([x0, x1], [swipe_y, swipe_y], pen=pen(color, width=4))
                swipe_head.setStyle(angle=180 if x1 > x0 else 0, brush=pg.mkBrush(color))  # angle 0 points left
                swipe_head.setPos(x1, swipe_y)
                swipe_head.setVisible(x1 != x0)
                swipe_label.setAnchor((0.5, 1.0))
                swipe_label.setPos((x0 + x1) / 2, swipe_y + 0.15)
            swipe_label.setText(text, color=color)
            swipe_line.show()
            swipe_label.show()
            shown_swipe = swipe
        opacity = float(np.clip(1 - (time.monotonic() - swipe.t) / swipe_display_s, 0, 1))
        for item in swipe_items:
            item.setOpacity(opacity)

    win.ci.layout.setRowStretchFactor(0, 3)
    win.ci.layout.setRowStretchFactor(1, 2)
    win.ci.layout.setRowStretchFactor(2, 2)

    # --- Calibration panel: collapsed to the ⚙ tab (out of the recordings), C toggles it
    panel_width = 270
    panel = QtWidgets.QWidget()
    panel_layout = QtWidgets.QHBoxLayout(panel)
    panel_layout.setContentsMargins(0, 8, 4, 8)
    panel_layout.setSpacing(4)
    toggle = QtWidgets.QToolButton()
    toggle.setText("⚙")
    toggle.setToolTip("Calibration (C)")
    toggle.setCheckable(True)
    toggle.setAutoRaise(True)
    toggle.setFocusPolicy(QtCore.Qt.FocusPolicy.NoFocus)
    toggle.setStyleSheet(f"color: {c['muted']}; font-size: {style.font_pt}pt;")
    toggle_column = QtWidgets.QVBoxLayout()
    toggle_column.addWidget(toggle)
    toggle_column.addStretch(1)
    panel_layout.addLayout(toggle_column)

    content = QtWidgets.QWidget()
    content.setFixedWidth(panel_width)
    content.setStyleSheet(
        f"QWidget {{ font-family: {style.font_family}; font-size: {style.font_pt - 2}pt; }}"
        f"QPushButton, QSpinBox {{ border: 1px solid {c['muted']}; border-radius: 4px; padding: 4px 8px; }}"
        f"QPushButton:disabled {{ color: {c['muted']}; }}"
    )
    form = QtWidgets.QVBoxLayout(content)
    title = QtWidgets.QLabel("Calibration")
    title.setStyleSheet(f"font-size: {style.title_pt}pt; font-weight: bold;")
    form.addWidget(title)
    fields = QtWidgets.QFormLayout()
    holes_spin = QtWidgets.QSpinBox()
    holes_spin.setRange(2, CAL_CFG.n_holes)
    holes_spin.setValue(min(11, CAL_CFG.n_holes))
    passes_spin = QtWidgets.QSpinBox()
    passes_spin.setRange(2, 5)  # >= 2: the detection threshold comes from the spread of the backgrounds
    passes_spin.setValue(2)
    fields.addRow("Holes", holes_spin)
    fields.addRow("Passes", passes_spin)
    form.addLayout(fields)
    plan_label = QtWidgets.QLabel()
    plan_label.setWordWrap(True)
    plan_label.setStyleSheet(f"color: {c['muted']};")
    form.addWidget(plan_label)
    start_button = QtWidgets.QPushButton("Start")
    cancel_button = QtWidgets.QPushButton("Cancel (Esc)")
    for button in (start_button, cancel_button):
        button.setFocusPolicy(QtCore.Qt.FocusPolicy.NoFocus)  # Space is for recording a step
        form.addWidget(button)
    cancel_button.setEnabled(False)
    info_label = QtWidgets.QLabel(
        "Each step moves on by itself: hold the green hole until it turns, then release. Space records the "
        "current step now (e.g. a weak hole)."
    )
    info_label.setWordWrap(True)
    form.addWidget(info_label)
    form.addStretch(1)
    content.hide()
    panel_layout.addWidget(content)
    window_layout.addWidget(panel)

    def plan() -> tuple[int, ...]:
        return lc.calibration_holes(CAL_CFG.n_holes, holes_spin.value(), always=cfg.z_up_holes + cfg.z_down_holes)

    def update_plan() -> None:
        holes_ = plan()
        steps = passes_spin.value() * (len(holes_) + 1)
        plan_label.setText(
            f"{len(holes_)} holes (evenly spaced + the z holes): {', '.join(map(str, holes_))}. {steps} steps."
        )

    holes_spin.valueChanged.connect(update_plan)
    passes_spin.valueChanged.connect(update_plan)
    update_plan()

    def set_panel(visible: bool) -> None:
        if visible == content.isVisible():
            return
        content.setVisible(visible)
        toggle.setChecked(visible)
        # The plot keeps its size: the window grows by the panel
        window.resize(window.width() + (1 if visible else -1) * (panel_width + 4), window.height())

    toggle.toggled.connect(set_panel)

    def start_calibration() -> None:
        session.update(recorder=lc.CalibrationRecorder(plan(), passes_spin.value()), result=None, error=None)
        paused.set()
        start_button.setEnabled(False)
        cancel_button.setEnabled(True)
        holes_spin.setEnabled(False)
        passes_spin.setEnabled(False)
        info_label.setText("Recording... Space: record the current step now. Esc: cancel.")
        win.setFocus()

    def end_calibration(message: str) -> None:
        session["recorder"] = None
        paused.clear()
        start_button.setEnabled(True)
        cancel_button.setEnabled(False)
        holes_spin.setEnabled(True)
        passes_spin.setEnabled(True)
        info_label.setText(message)

    def cancel_calibration() -> None:
        if session["recorder"] is not None and session["fit_thread"] is None:
            end_calibration("Cancelled: the previous calibration is still used.")

    def record_now() -> None:
        if session["recorder"] is not None:
            session["recorder"].record_now()

    start_button.clicked.connect(start_calibration)
    cancel_button.clicked.connect(cancel_calibration)
    shortcuts = []
    for key, action in (
        ("C", lambda: set_panel(not content.isVisible())),
        ("Space", record_now),
        ("Esc", cancel_calibration),
    ):
        shortcut = QtWidgets.QShortcut(QtGui.QKeySequence(key), window)
        shortcut.activated.connect(action)
        shortcuts.append(shortcut)

    def fit(presses) -> None:
        session["had_previous"] = CAL_CFG.npz_path.exists()
        try:
            session["result"] = lc.save_and_fit(
                presses, CAL_CFG.db_path, CAL_CFG.npz_path, CAL_CFG.first_hole_cm, CAL_CFG.hole_spacing_cm
            )
        except Exception as e:  # Shown in the panel; the previous calibration is kept
            session["error"] = e

    def show_calibration(recorder) -> None:
        """The hole to press (green, outlined), those recorded this pass (light green), the step in the status."""
        if session["fit_thread"] is not None:
            set_status("Fitting the calibration...", c["muted"])
            return
        plan_holes, recorded = set(recorder.holes), recorder.recorded_holes()
        green = rgb(c["command"])
        colors = (
            np.array(
                [
                    green
                    if h == recorder.key and recorder.phase in ("waiting", "recording")
                    else idle_color + 0.55 * (green - idle_color)
                    if h in recorded
                    else idle_color + 0.2 * (green - idle_color)
                    if h in plan_holes
                    else idle_color
                    for h in hole_ids
                ]
            )
            .round()
            .astype(int)
        )
        show_holes(colors, recorder.key if recorder.key > 0 else None)
        step = f"pass {recorder.pass_idx + 1}/{recorder.n_passes} · {recorder.step_idx + 1}/{recorder.n_steps}"
        if recorder.phase == "quiet" or (recorder.key == 0 and recorder.phase == "recording"):
            set_status(f"Don't touch: background  ({step})", c["reference"])
        elif recorder.phase == "waiting":
            set_status(f"Press hole {recorder.key}  ({step})", c["command"])
        elif recorder.phase == "recording":
            set_status(f"Hold hole {recorder.key}...", c["command"])
        elif recorder.phase == "release":
            set_status("Release", c["muted"])

    window.resize(*style.window_size)
    window.show()

    n_frames_last, t_last = 0, time.monotonic()
    shown_background = shown_frame = None

    def update():
        """Every ~16 ms: the trail and the arrow fade smoothly; the traces and status only change with a new frame."""
        nonlocal n_frames_last, t_last, shown_background, shown_frame
        if errors or stop_event.is_set():
            app.quit()
            return

        # Calibration: recording, fitting, done
        recorder = session["recorder"]
        if recorder is not None and recorder.done and session["fit_thread"] is None:
            session["fit_thread"] = threading.Thread(target=fit, args=(list(recorder.presses),), daemon=True)
            session["fit_thread"].start()
        if session["fit_thread"] is not None and not session["fit_thread"].is_alive():
            session["fit_thread"] = None
            if session["error"] is not None:
                end_calibration(f"Fit failed: {session['error']!r}. The previous calibration is still used.")
            else:
                new = load_calibration(CAL_CFG.npz_path)
                apply_calibration(new)
                reload_event.set()  # The sensor loop loads it and records the background again
                backup = " (the previous one kept as a backup)" if session.get("had_previous") else ""
                end_calibration(
                    f"Calibrated: {len(new['hole_ids'])} holes, threshold {new['threshold']:.0f}. "
                    f"Saved to {CAL_CFG.npz_path.name}{backup}."
                )
            recorder = None

        detector = latest["detector"]
        frame = latest["frame"]
        new_frame = frame is not None and frame is not shown_frame
        if new_frame:
            frame_curve.setData(time_ms, frame)  # Only the latest frame
            shown_frame = frame

        if recorder is not None:  # Calibrating: the change from the pass background, and the hole to press
            for item in (raw_echo, signal_echo, distance_line, *swipe_items):
                item.hide()
            show_calibration(recorder)
            if recorder.pass_background is not None and recorder.pass_background is not shown_background:
                background_curve.setData(time_ms, recorder.pass_background)
                shown_background = recorder.pass_background
            if new_frame and recorder.pass_background is not None:
                envelope = wg_helpers.ema(np.abs(np.asarray(frame, dtype=float) - recorder.pass_background))
                envelope_curve.setData(time_ms, envelope)
                set_threshold(recorder.threshold)
                set_title(signal, f"Change from the background  ·  {recorder.last_change:.0f}")
            return

        result = detector.last_result if detector is not None else None
        show_trail(result["hole"] if result is not None and result["detected"] else None)
        show_swipe()

        if detector is not None and detector.background is not None and detector.background is not shown_background:
            background_curve.setData(time_ms, detector.background)
            shown_background = detector.background

        if detector is None or detector.calibration_params is None:
            set_status("No calibration: open the ⚙ tab (C)", c["muted"])
        elif detector.background is None:
            set_status("Recording the background: don't touch", c["reference"])
        elif new_frame and result is not None:
            stop = view["stop"]
            averaged = detector.last_averaged
            envelope = wg_helpers.ema(np.abs(np.asarray(averaged[:stop], dtype=float) - detector.background[:stop]))
            envelope_curve.setData(time_ms[: len(envelope)], envelope)
            set_threshold(float(detector.calibration_params["threshold"]))
            set_title(signal, f"Echo envelope  ·  amplitude {result['amplitude']:.0f}")

            if result["detected"]:
                set_status(f"Hole {result['hole']}  ·  {result['distance_cm']:.1f} cm", c["touch"])
                distance_line.setPos(result["distance_cm"])
                distance_line.show()
                if result["echo_time_ms"] is not None:  # None for template matching
                    for line in (raw_echo, signal_echo):
                        line.setPos(result["echo_time_ms"])
                        line.show()
            else:
                set_status("No touch", c["muted"])
                for line in (raw_echo, signal_echo, distance_line):
                    line.hide()

        t = time.monotonic()
        if t - t_last >= 1.0:
            rate = (latest["n_frames"] - n_frames_last) / (t - t_last)
            window.setWindowTitle(f"Waveguide sensor ({rate:.0f} frames/s)")
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


# --- Live match: reproduce the same initial conditions (e.g. sensor position) as a saved target trace
MATCH_TARGETS = "reference_*.csv"  # In CAL_CFG.directory; --live-match without a file uses the newest


def latest_match_target(directory: Path = CAL_CFG.directory) -> Path:
    """The newest (modification time) reference trace in `directory`."""
    targets = sorted(directory.glob(MATCH_TARGETS), key=lambda path: path.stat().st_mtime)
    if not targets:
        raise FileNotFoundError(f"No {MATCH_TARGETS} in {directory}: give the target CSV (--live-match TARGET_CSV)")
    return targets[-1]


def load_target(path: Path) -> np.ndarray:
    """Trace values from a CSV: the y column of an x/y export (the first y if there are several curves), or the
    only column. Rows that are not all numbers (headers) are skipped. Samples missing from an x/y export are nan."""
    rows = []
    with open(path) as file:
        for line in file:
            cells = line.replace(";", ",").replace("\t", ",").split(",")
            try:
                rows.append([float(cell) for cell in cells if cell.strip()])
            except ValueError:
                continue
    rows = [row for row in rows if row]
    if not rows:
        raise ValueError(f"No numeric rows in {path}")
    width = min(len(row) for row in rows)
    data = np.array([row[:width] for row in rows])
    if width == 1:
        return data[:, 0]
    x, y = data[:, 0], data[:, 1]  # x/y pairs: place the y values at their sample index
    trace = np.full(int(x.max()) + 1, np.nan)
    trace[x.astype(int)] = y
    return trace


def save_target(trace: np.ndarray, directory: Path = CAL_CFG.directory) -> Path:
    """Write a trace in the x/y format of the pyqtgraph CSV export, as reference_<date>_<time>.csv: the newest
    target for the next --live-match."""
    path = directory / f"reference_{time.strftime('%Y_%m_%d_%H%M%S')}.csv"
    np.savetxt(
        path,
        np.column_stack([np.arange(len(trace)), trace]),
        delimiter=",",
        header="x0000,y0000",
        comments="",
        fmt=["%d", "%.2f"],
    )
    return path


def run_live_match(
    port: str, baudrate: int, target_path: Path, n_average: int = 5, style: PlotStyle = PLOT_STYLE
) -> None:
    """Live trace overlaid on a saved target trace (`load_target`), to put the sensor back as it was (e.g. after
    a reset) before recording.

    Top: the target (orange), the latest frame (faint blue) and the average of the last `n_average` frames (blue).
    Bottom: averaged live minus target. Drag the shaded region to choose the samples compared; the title shows
    their RMS difference and correlation. Press S to save the averaged trace as a new target (`save_target`).
    """
    target = load_target(target_path)
    port = check_port(port, baudrate)
    if len(target) != FRAME_SIZE:
        print(f"Note: the target has {len(target)} samples, the frames {FRAME_SIZE}: comparing the overlap only")
    n_compare = min(len(target), FRAME_SIZE)

    tools = _PlotTools(style, "Waveguide match")
    pg, QtCore, c = tools.pg, tools.QtCore, tools.c
    latest = {"frame": None, "n_frames": 0}
    stop_event, errors = threading.Event(), []

    def reader_loop():
        try:
            with open_serial(port, baudrate) as ser:
                reader = FrameReader(ser)
                reader.flush()
                while not stop_event.is_set():
                    frame = reader.read_frame()
                    if frame is not None:
                        latest["frame"] = frame.astype(float)
                        latest["n_frames"] += 1
        except Exception as e:
            errors.append(e)

    win = pg.GraphicsLayoutWidget(show=True, title="Waveguide match")
    win.resize(*style.window_size)
    win.ci.setContentsMargins(16, 12, 24, 12)
    win.ci.setSpacing(18)
    trace = dict(antialias=style.antialias_traces, autoDownsample=True, downsampleMethod="peak", clipToView=True)

    top = win.addPlot()
    tools.style_axes(top, y_label="ADC counts")
    tools.set_title(top, f"Live vs target ({target_path.name})")
    top.setYRange(0, 4096)
    top.setXRange(0, FRAME_SIZE, padding=0)
    legend = top.addLegend(offset=(-10, 10), labelTextSize=f"{style.font_pt - 2}pt", labelTextColor=c["foreground"])
    legend.setBrush(tools.tint(c["background"], 220))
    top.plot(target, pen=tools.pen(c["reference"]), name="target", **trace)
    latest_curve = top.plot(pen=tools.pen(tools.tint(c["signal"], 70), width=1), name="latest frame", **trace)
    average_curve = top.plot(pen=tools.pen(c["signal"]), name=f"live, average of {n_average}", **trace)
    # Samples compared (draggable); the whole trace by default
    region = pg.LinearRegionItem(values=(0, n_compare - 1), brush=c["shade"], pen=tools.pen(c["muted"], width=1))
    top.addItem(region)

    win.nextRow()
    bottom = win.addPlot()
    tools.style_axes(bottom, x_label="Sample index", y_label="Difference (ADC)")
    tools.set_title(bottom, "Live (averaged) minus target")
    bottom.setXLink(top)
    bottom.addLine(y=0, pen=tools.pen(c["muted"], width=1))
    difference_curve = bottom.plot(pen=tools.pen(c["y"]), **trace)

    win.nextRow()
    status = win.addLabel("Waiting for frames...", size=f"{style.title_pt}pt", color=c["muted"])
    win.ci.layout.setRowStretchFactor(0, 3)
    win.ci.layout.setRowStretchFactor(1, 2)

    recent = deque(maxlen=max(1, n_average))
    shown_frame = None

    def update():
        nonlocal shown_frame
        if errors:
            app.quit()
            return
        frame = latest["frame"]
        if frame is None or frame is shown_frame:
            return
        shown_frame = frame
        recent.append(frame)
        average = np.mean(recent, axis=0)
        latest_curve.setData(frame)
        average_curve.setData(average)
        difference_curve.setData(average[:n_compare] - target[:n_compare])

        # Match within the selected samples (target samples missing from the CSV are skipped)
        lo, hi = (int(round(v)) for v in region.getRegion())
        lo, hi = max(0, lo), min(n_compare, hi + 1)
        live_part, target_part = average[lo:hi], target[lo:hi]
        valid = ~np.isnan(target_part)
        if valid.sum() > 2:
            rms = np.sqrt(np.mean((live_part[valid] - target_part[valid]) ** 2))
            corr = np.corrcoef(live_part[valid], target_part[valid])[0, 1]
            status.setText(
                f"Samples {lo}-{hi - 1}:  RMS difference {rms:.1f} ADC  ·  correlation {corr:.4f}    (S: save as target)",
                color=c["foreground"],
            )

    def key_pressed(event):
        if event.key() == QtCore.Qt.Key.Key_S and recent:
            print(f"Saved the averaged live trace to {save_target(np.mean(recent, axis=0))}")

    win.keyPressEvent = key_pressed
    app = tools.app
    thread = threading.Thread(target=reader_loop, name="waveguide_match_reader", daemon=True)
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
    mode.add_argument(
        "--live-match",
        nargs="?",
        const="latest",
        metavar="TARGET_CSV",
        help=f"Live trace vs a saved target, to put the sensor back as it was (default: the newest {MATCH_TARGETS} in "
        f"{CAL_CFG.directory.name}/)",
    )
    parser.add_argument("--average", type=int, default=5, help="--live-match: frames averaged (the recordings use 5)")
    parser.add_argument(
        "--record",
        action="store_true",
        help=f"Record the run to {recording.DATA_DIR / 'sensor'}/<date-time> (see recording.py)",
    )
    parser.add_argument("--notes", default="", help="--record: purpose of the run, saved in its metadata")
    args = parser.parse_args()
    if args.record and (args.calibrate or args.live_match is not None):
        parser.error("--record is for the default and --live-plot modes")

    logger = recorder = None
    if args.record:
        logger = recording.start_logger("sensor", args.notes, {"args": vars(args), **run_metadata()})
        recorder = recording.SensorRecorder(logger.log_sample, logger.run_dir)

    try:
        if args.calibrate:
            print("---- Calibration mode ----")
            calibrate(args.port, args.baudrate)
        elif args.live_match is not None:
            print("---- Live match mode ----")
            target = latest_match_target() if args.live_match == "latest" else Path(args.live_match)
            print(f"Target: {target}")
            run_live_match(args.port, args.baudrate, target, args.average)
        elif args.live_plot:
            print("---- Live plot mode ----")
            print_frames_and_holes(args.port, args.baudrate, plot=True, recorder=recorder)
        else:
            print("---- Default mode ----")
            print_frames_and_holes(args.port, args.baudrate, recorder=recorder)

    except (FileNotFoundError, ValueError, serial.SerialException) as e:
        parser.exit(1, f"Error: {e}\n")
    except KeyboardInterrupt:
        print()
    finally:
        if logger is not None:
            logger.stop()
            print(f"Recorded to {logger.run_dir}")


if __name__ == "__main__":
    main()
