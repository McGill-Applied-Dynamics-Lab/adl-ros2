"""Waveguide sensor processing.

Reads the Teensy over serial and turns the measurements into teleop commands, in two layers:
1. Hole detection (as fast as the serial comes, > 500 Hz): measurement -> set of pressed holes
2. Gesture processing: changes of the pressed holes -> `MoveCommand` (axis, distance, speed)

Runs in its own process (`SensorProcess`), so the robot (rclpy, the GIL) never stalls the fast
loop. Imports no ROS: it can be developed and tested without the robot.

Standalone, prints the pressed holes and the commands:
    pixi run -e wg python experiments/waveguide/sensor.py --port /dev/ttyACM0
"""

import argparse
import multiprocessing as mp
import queue
import time
from dataclasses import dataclass

import serial
from commands import MoveCommand

DEFAULT_BAUDRATE = 115200
SERIAL_TIMEOUT = 0.1  # s, so the loop checks the stop event even without data


@dataclass(frozen=True)
class HoleState:
    """Holes pressed at time `t` (CLOCK_MONOTONIC) [s]."""

    t: float
    pressed: frozenset[int]


# --- Layer 1: hole detection ---
def parse_line(line: bytes) -> list[float]:
    """Parse one serial line into a measurement.

    TODO: stub. Assumes comma-separated values, one per hole; replace with the Teensy format.
    """
    return [float(v) for v in line.decode("ascii").strip().split(",")]


class HoleDetector:
    """Detect the pressed holes from one measurement.

    TODO: stub thresholding each value; replace with the detection from the waveguide code.
    """

    def __init__(self, threshold: float = 0.5):
        self.threshold = threshold

    def update(self, measurement: list[float]) -> frozenset[int]:
        return frozenset(i for i, v in enumerate(measurement) if v > self.threshold)


# --- Layer 2: gesture processing ---
class GestureProcessor:
    """Turn the pressed-hole changes into teleop commands.

    TODO: stub, emits nothing; replace with the swipe -> (axis, distance, speed) mapping.
    """

    def update(self, state: HoleState) -> MoveCommand | None:
        return None


# --- Process ---
def run_sensor_loop(port: str, baudrate: int, on_holes, on_command, stop_event=None) -> None:
    """Read the serial port and run both layers until `stop_event` is set (or forever).

    `on_holes(HoleState)` is called when the pressed holes change, `on_command(MoveCommand)` for
    each command.
    """
    detector = HoleDetector()
    processor = GestureProcessor()
    pressed = frozenset()

    with serial.Serial(port, baudrate, timeout=SERIAL_TIMEOUT) as ser:
        ser.reset_input_buffer()  # Drop what accumulated before we started
        while stop_event is None or not stop_event.is_set():
            line = ser.readline()
            t = time.monotonic()
            if not line:
                continue
            try:
                measurement = parse_line(line)
            except ValueError:
                continue  # Partial or corrupted line

            new_pressed = detector.update(measurement)
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


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", required=True, help="Serial port of the Teensy, e.g. /dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)
    args = parser.parse_args()

    try:
        run_sensor_loop(
            args.port,
            args.baudrate,
            on_holes=lambda state: print(f"{state.t:.3f}  holes: {sorted(state.pressed)}"),
            on_command=lambda command: print(f"{command.t_created:.3f}  command: {command}"),
        )
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
