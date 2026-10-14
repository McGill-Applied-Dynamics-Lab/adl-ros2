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
import time
from contextlib import closing
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import serial
import wg_helpers
from commands import MoveCommand


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


CALIBRATION = CalibrationConfig()

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
    return serial.Serial(port, baudrate, timeout=SERIAL_TIMEOUT)


@dataclass(frozen=True)
class HoleState:
    """Holes pressed at time `t` (CLOCK_MONOTONIC) [s]."""

    t: float
    pressed: frozenset[int]


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

    def read_available(self) -> list[np.ndarray]:
        """Complete frames among the bytes already received (non-blocking)."""
        if self.ser.in_waiting:
            self._buffer.extend(self.ser.read(self.ser.in_waiting))
        frames = []
        while (frame := self._pop_frame()) is not None:
            frames.append(frame)
        return frames

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
    """Detect the pressed holes from one frame.

    TODO: stub, detects nothing; use the calibration (wg_helpers.estimate).
    """

    def update(self, frame: np.ndarray) -> frozenset[int]:
        return frozenset()


# --- Layer 2: gesture processing ---
class GestureProcessor:
    """Turn the pressed-hole changes into teleop commands.

    TODO: stub, emits nothing; replace with the swipe -> (axis, distance, speed) mapping.
    """

    def update(self, state: HoleState) -> MoveCommand | None:
        return None


# --- Process ---
def run_sensor_loop(port: str, baudrate: int, on_holes, on_command, stop_event=None, on_frame=None) -> None:
    """Read the serial port and run both layers until `stop_event` is set (or forever).

    `on_holes(HoleState)` is called when the pressed holes change, `on_command(MoveCommand)` for
    each command, and `on_frame(t, frame)` (if given) for every frame.
    """
    detector = HoleDetector()
    processor = GestureProcessor()
    pressed = frozenset()

    with open_serial(port, baudrate) as ser:
        reader = FrameReader(ser)
        reader.flush()  # Drop what accumulated before we started
        while stop_event is None or not stop_event.is_set():
            frame = reader.read_frame()
            t = time.monotonic()
            if frame is None:
                continue
            if on_frame is not None:
                on_frame(t, frame)

            new_pressed = detector.update(frame)
            if new_pressed == pressed:
                continue
            pressed = new_pressed

            state = HoleState(t, pressed)
            on_holes(state)
            command = processor.update(state)
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
        check_port(port)  # Here, not only in the child: there it would just end the process
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


def calibrate(port: str, baudrate: int, config: CalibrationConfig = CALIBRATION) -> None:
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
def print_frames_and_holes(port: str, baudrate: int) -> None:
    n_frames, t_last = 0, time.monotonic()

    def on_frame(t, frame):
        nonlocal n_frames, t_last
        n_frames += 1
        if t - t_last >= 1.0:
            print(f"{n_frames / (t - t_last):.0f} frames/s")
            n_frames, t_last = 0, t

    run_sensor_loop(
        port,
        baudrate,
        on_holes=lambda state: print(f"{state.t:.3f}  holes: {sorted(state.pressed)}"),
        on_command=lambda command: print(f"{command.t_created:.3f}  command: {command}"),
        on_frame=on_frame,
    )


def live_plot(port: str, baudrate: int) -> None:
    """Plot the latest raw frame at ~60 Hz (from sensor_live_plot.py)."""
    check_port(port)
    import pyqtgraph as pg  # Only needed here
    from pyqtgraph.Qt import QtCore

    with open_serial(port, baudrate) as ser:
        reader = FrameReader(ser)
        reader.flush()

        app = pg.mkQApp("Waveguide raw frames")
        win = pg.GraphicsLayoutWidget(show=True, title="Waveguide raw frames")
        win.resize(1200, 450)
        plot = win.addPlot(title="Echo (time-locked to the PW pin)")
        plot.setYRange(0, 4096)
        plot.setXRange(0, FRAME_SIZE)
        plot.setLabel("bottom", "Sample index (distance = index * c / (2 * fs))")
        plot.setLabel("left", "ADC raw amplitude (12-bit)")
        plot.showGrid(x=True, y=True, alpha=0.3)
        curve = plot.plot(pen=pg.mkPen("c", width=1.5))

        n_frames, t_last = 0, time.monotonic()

        def update():
            nonlocal n_frames, t_last
            frames = reader.read_available()
            if frames:
                curve.setData(frames[-1])  # Only the latest; the plot cannot keep up with every frame
            n_frames += len(frames)
            t = time.monotonic()
            if t - t_last >= 1.0:
                win.setWindowTitle(f"Waveguide raw frames ({n_frames / (t - t_last):.0f} frames/s)")
                n_frames, t_last = 0, t

        timer = QtCore.QTimer()
        timer.timeout.connect(update)
        timer.start(16)
        app.exec()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default=DEFAULT_PORT, help=f"Serial port of the Teensy (default: {DEFAULT_PORT})")
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--calibrate", action="store_true", help="Record a calibration and fit it (CalibrationConfig)")
    mode.add_argument("--live-plot", action="store_true", help="Plot the raw frames")
    args = parser.parse_args()

    try:
        if args.calibrate:
            calibrate(args.port, args.baudrate)
        elif args.live_plot:
            live_plot(args.port, args.baudrate)
        else:
            print_frames_and_holes(args.port, args.baudrate)
    except (FileNotFoundError, serial.SerialException) as e:
        parser.exit(1, f"Error: {e}\n")
    except KeyboardInterrupt:
        print()


if __name__ == "__main__":
    main()
