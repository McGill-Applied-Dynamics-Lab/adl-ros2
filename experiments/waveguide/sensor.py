"""Waveguide sensor processing.

Reads the Teensy over serial and turns the echo frames into teleop commands, in two layers:
1. Hole detection (on every frame, > 500 Hz): frame -> set of pressed holes
2. Gesture processing: changes of the pressed holes -> `MoveCommand` (axis, distance, speed)

Runs in its own process (`SensorProcess`), so the robot (rclpy, the GIL) never stalls the fast
loop. Imports no ROS: it can be developed and tested without the robot.

Standalone, prints the frame rate, the pressed holes and the commands:
    pixi run -e wg python experiments/waveguide/sensor.py --port /dev/ttyACM0

Calibration (guided, in the terminal), saved to data/waveguide/calibration/:
    pixi run -e wg python experiments/waveguide/sensor.py --port /dev/ttyACM0 --calibrate --n-holes 10
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

DEFAULT_BAUDRATE = 3_000_000
SERIAL_TIMEOUT = 0.1  # s, so the loop checks the stop event even without data

# Teensy packet: 3-byte header + FRAME_SIZE uint16 samples (little-endian, 12-bit ADC)
FRAME_SIZE = 1500
HEADER = b"\xaa\x55\x00"
PACKET_SIZE = len(HEADER) + 2 * FRAME_SIZE

CALIBRATION_DIR = Path(__file__).resolve().parents[2] / "data" / "waveguide" / "calibration"


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
        while True:
            idx = self._buffer.find(HEADER)
            if idx < 0:
                del self._buffer[: -(len(HEADER) - 1)]  # Keep a possible partial header
            else:
                del self._buffer[:idx]
                if len(self._buffer) >= PACKET_SIZE:
                    frame = np.frombuffer(self._buffer, dtype="<u2", count=FRAME_SIZE, offset=len(HEADER)).copy()
                    del self._buffer[:PACKET_SIZE]
                    return frame

            chunk = self.ser.read(max(self.ser.in_waiting, 1))
            if not chunk:
                return None
            self._buffer.extend(chunk)


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

    with serial.Serial(port, baudrate, timeout=SERIAL_TIMEOUT) as ser:
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

    def __init__(self, port: str, baudrate: int = DEFAULT_BAUDRATE):
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
def record_calibration(port: str, baudrate: int, n_holes: int, n_passes: int, db_path: Path) -> None:
    """Guided single-touch calibration recording, in the SAVE_TO_SQLITE.py format `wg_helpers.calibrate` reads.

    Each pass records nothing covered (key [0]), then holes 1..n_holes one at a time
    (wg_helpers.CYCLES_PER_PRESS frames each). At least 2 passes are needed for a detection threshold.
    """
    n_frames = wg_helpers.CYCLES_PER_PRESS
    with closing(sqlite3.connect(db_path)) as conn, serial.Serial(port, baudrate, timeout=SERIAL_TIMEOUT) as ser:
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
        for pass_idx in range(n_passes):
            print(f"\n--- Pass {pass_idx + 1}/{n_passes}")
            for key in range(n_holes + 1):
                what = "Uncover ALL holes" if key == 0 else f"Cover hole {key} only"
                input(f"[{key}/{n_holes}] {what}, then press Enter ")

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


def calibrate(args) -> None:
    CALIBRATION_DIR.mkdir(parents=True, exist_ok=True)
    stem = CALIBRATION_DIR / time.strftime("calibration_%Y%m%d_%H%M%S")
    db_path, npz_path = stem.with_suffix(".db"), stem.with_suffix(".npz")

    print(f"Calibration: {args.n_holes} holes, {args.passes} passes, recording to {db_path}")
    try:
        record_calibration(args.port, args.baudrate, args.n_holes, args.passes, db_path)
    except BaseException:
        db_path.unlink(missing_ok=True)  # An incomplete recording does not calibrate
        raise

    print()
    wg_helpers.calibrate(db_path, args.first_hole_cm, args.hole_spacing_cm, save_path=npz_path)


def print_frames_and_holes(args) -> None:
    n_frames, t_last = 0, time.monotonic()

    def on_frame(t, frame):
        nonlocal n_frames, t_last
        n_frames += 1
        if t - t_last >= 1.0:
            print(f"{n_frames / (t - t_last):.0f} frames/s")
            n_frames, t_last = 0, t

    run_sensor_loop(
        args.port,
        args.baudrate,
        on_holes=lambda state: print(f"{state.t:.3f}  holes: {sorted(state.pressed)}"),
        on_command=lambda command: print(f"{command.t_created:.3f}  command: {command}"),
        on_frame=on_frame,
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", required=True, help="Serial port of the Teensy, e.g. /dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)

    calibration = parser.add_argument_group("calibration")
    calibration.add_argument("--calibrate", action="store_true", help="Record a calibration and fit it")
    calibration.add_argument("--n-holes", type=int, help="Number of holes (required with --calibrate)")
    calibration.add_argument("--passes", type=int, default=2, help="Recordings of every hole (>= 2 for a threshold)")
    calibration.add_argument("--first-hole-cm", type=float, default=25.0, help="Distance to hole 1 [cm]")
    calibration.add_argument("--hole-spacing-cm", type=float, default=1.0, help="Distance between holes [cm]")
    args = parser.parse_args()

    try:
        if args.calibrate:
            if args.n_holes is None:
                parser.error("--n-holes is required with --calibrate")
            calibrate(args)
        else:
            print_frames_and_holes(args)
    except KeyboardInterrupt:
        print()


if __name__ == "__main__":
    main()
