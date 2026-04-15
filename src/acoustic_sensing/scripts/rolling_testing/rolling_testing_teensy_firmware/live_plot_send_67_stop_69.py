#!/usr/bin/env python3
from __future__ import annotations

import argparse
import time

import numpy as np
import serial

SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 3_000_000
SERIAL_TIMEOUT_SEC = 10.0
SAMPLE_RATE = 178804.2
SPEED_OF_SOUND_M_S = 343.0
RANGING_DISTANCE_M = 0.6
EXPECTED_SAMPLES = int((SAMPLE_RATE / SPEED_OF_SOUND_M_S) * 2.0 * RANGING_DISTANCE_M * 1.2)
DEFAULT_YMIN = 0.0
DEFAULT_YMAX = 4095.0
CHANNEL_MARKERS = ("S0", "S1", "S2", "S3")
CMD_START = b"C"  # 67
CMD_STOP = b"E"   # 69
STREAM_END = "STREAM_END"

pg = None
QtCore = None
QtWidgets = None


def load_qt() -> None:
    global pg, QtCore, QtWidgets
    try:
        import pyqtgraph as _pg
        from pyqtgraph.Qt import QtCore as _QtCore
        from pyqtgraph.Qt import QtWidgets as _QtWidgets
    except ImportError as exc:
        raise SystemExit(
            "This viewer requires pyqtgraph and a Qt binding. "
            "Install pyqtgraph plus PyQt5/PyQt6/PySide6 in the active environment."
        ) from exc

    pg = _pg
    QtCore = _QtCore
    QtWidgets = _QtWidgets


class FrameParser:
    def __init__(self) -> None:
        self._line_buffer = bytearray()
        self._current_channel: str | None = None
        self._current_samples: list[int] = []
        self._current_frame: dict[str, list[int]] = {}
        self.latest_frame: list[np.ndarray] | None = None
        self.frames_seen = 0
        self.bad_frames_seen = 0

    def feed(self, chunk: bytes) -> None:
        self._line_buffer.extend(chunk)
        while True:
            try:
                newline_idx = self._line_buffer.index(ord("\n"))
            except ValueError:
                return

            raw_line = bytes(self._line_buffer[:newline_idx])
            del self._line_buffer[: newline_idx + 1]
            self._handle_line(raw_line)

    def _handle_line(self, raw_line: bytes) -> None:
        line = raw_line.decode("ascii", errors="ignore").strip()
        if not line:
            return
        if line == STREAM_END:
            return
        if line in CHANNEL_MARKERS:
            self._current_channel = line
            self._current_samples = []
            return
        if line == "T":
            if self._current_channel is None:
                return
            self._current_frame[self._current_channel] = self._current_samples
            self._current_channel = None
            if len(self._current_frame) == len(CHANNEL_MARKERS):
                if any(len(self._current_frame[channel]) != EXPECTED_SAMPLES for channel in CHANNEL_MARKERS):
                    self.bad_frames_seen += 1
                    self._current_frame = {}
                    return
                self.latest_frame = [
                    np.asarray(self._current_frame[channel], dtype=float)
                    for channel in CHANNEL_MARKERS
                ]
                self.frames_seen += 1
                self._current_frame = {}
            return
        if self._current_channel is None:
            return
        try:
            self._current_samples.append(int(line))
        except ValueError:
            return


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Send 67 to Teensy, show a Qt live plot, send 69 when the window closes."
    )
    parser.add_argument("--port", default=SERIAL_PORT, help=f"Serial port (default: {SERIAL_PORT})")
    parser.add_argument("--baud", type=int, default=BAUD_RATE, help=f"Baud rate (default: {BAUD_RATE})")
    parser.add_argument("--chunk-size", type=int, default=65536, help="Max bytes read per GUI tick")
    parser.add_argument("--update-ms", type=int, default=20, help="Serial poll interval in ms")
    parser.add_argument("--ymin", type=float, default=DEFAULT_YMIN, help=f"Fixed plot y-min (default: {DEFAULT_YMIN})")
    parser.add_argument("--ymax", type=float, default=DEFAULT_YMAX, help=f"Fixed plot y-max (default: {DEFAULT_YMAX})")
    return parser.parse_args()


def run_viewer(port: str, baud: int, chunk_size: int, update_ms: int, ymin: float, ymax: float) -> None:
    load_qt()

    class TeensyLiveViewer(QtWidgets.QMainWindow):
        def __init__(self, ser: serial.Serial) -> None:
            super().__init__()
            self.ser = ser
            self.parser = FrameParser()
            self.last_rendered_frame = 0
            self.start_time = time.monotonic()

            self.setWindowTitle("Teensy Live Plot 67/69")
            self.widget = pg.GraphicsLayoutWidget()
            self.setCentralWidget(self.widget)

            self.curves = []
            for idx, channel in enumerate(CHANNEL_MARKERS):
                plot = self.widget.addPlot(row=idx, col=0, title=channel)
                plot.setLabel("left", "ADC")
                plot.setLabel("bottom", "Sample")
                plot.showGrid(x=True, y=True, alpha=0.3)
                plot.setYRange(ymin, ymax, padding=0.0)
                curve = plot.plot(pen=pg.intColor(idx, hues=len(CHANNEL_MARKERS)))
                self.curves.append(curve)

            self.statusBar().showMessage("Sending 67 and waiting for RF frames...")
            self.ser.reset_input_buffer()
            self.ser.write(CMD_START)
            self.ser.flush()

            self.timer = QtCore.QTimer(self)
            self.timer.timeout.connect(self.poll_serial)
            self.timer.start(update_ms)

        def poll_serial(self) -> None:
            waiting = self.ser.in_waiting
            if waiting:
                chunk = self.ser.read(min(waiting, chunk_size))
                if chunk:
                    self.parser.feed(chunk)

            if self.parser.latest_frame is None:
                return
            if self.parser.frames_seen == self.last_rendered_frame:
                return

            self.last_rendered_frame = self.parser.frames_seen
            for curve, samples in zip(self.curves, self.parser.latest_frame, strict=True):
                curve.setData(samples)

            elapsed = max(time.monotonic() - self.start_time, 1e-6)
            fps = self.parser.frames_seen / elapsed
            self.statusBar().showMessage(
                f"Frames: {self.parser.frames_seen} | Bad frames: {self.parser.bad_frames_seen} | "
                f"Avg rate: {fps:.2f} frames/s | "
                "close window to send 69"
            )

        def closeEvent(self, event) -> None:  # noqa: N802
            self.timer.stop()
            try:
                self.ser.write(CMD_STOP)
                self.ser.flush()
                print("Sent 69 to Teensy.")
                time.sleep(0.05)
            finally:
                self.ser.close()
            event.accept()

    app = QtWidgets.QApplication([])
    ser = serial.Serial(
        port,
        baud,
        timeout=0,
        write_timeout=SERIAL_TIMEOUT_SEC,
    )
    window = TeensyLiveViewer(ser)
    window.resize(1100, 800)
    window.show()
    print(f"Sent 67 to Teensy on {port}. Close the Qt window to stop.")
    app_exec = getattr(app, "exec", None)
    if app_exec is None:
        app_exec = app.exec_
    app_exec()


def main() -> None:
    args = parse_args()
    run_viewer(args.port, args.baud, args.chunk_size, args.update_ms, args.ymin, args.ymax)


if __name__ == "__main__":
    main()
