#!/usr/bin/env python3
"""Qt live plotter for Teensy 4RF TEST mode."""

from __future__ import annotations

import argparse
import time

import numpy as np
import serial


DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUD = 3000000
CHANNEL_MARKERS = ("S0", "S1", "S2", "S3")
pg = None
QtCore = None
QtWidgets = None


def load_qt() -> None:
    global pg, QtCore, QtWidgets

    try:
        import pyqtgraph as _pg
        from pyqtgraph.Qt import QtCore as _QtCore
        from pyqtgraph.Qt import QtWidgets as _QtWidgets
    except ImportError as exc:  # pragma: no cover - depends on local GUI env
        raise SystemExit(
            "This live plotter requires pyqtgraph and a Qt binding. Install pyqtgraph "
            "plus PyQt5/PyQt6/PySide6 in the environment used to run this script."
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


def run_plotter(port: str, baud: int, chunk_size: int, update_ms: int) -> None:
    load_qt()

    class TeensyLivePlotter(QtWidgets.QMainWindow):
        def __init__(self, ser: serial.Serial) -> None:
            super().__init__()
            self.ser = ser
            self.parser = FrameParser()
            self.last_rendered_frame = 0
            self.start_time = time.monotonic()

            self.setWindowTitle("Teensy 4RF TEST Live Plot")
            self.widget = pg.GraphicsLayoutWidget()
            self.setCentralWidget(self.widget)

            self.curves = []
            for idx, channel in enumerate(CHANNEL_MARKERS):
                plot = self.widget.addPlot(row=idx, col=0, title=channel)
                plot.setLabel("left", "ADC")
                plot.setLabel("bottom", "Sample")
                plot.showGrid(x=True, y=True, alpha=0.3)
                curve = plot.plot(pen=pg.intColor(idx, hues=len(CHANNEL_MARKERS)))
                self.curves.append(curve)

            self.statusBar().showMessage("Sending TEST and waiting for frames...")
            self.ser.reset_input_buffer()
            self.ser.write(b"TEST")
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
                f"Frames: {self.parser.frames_seen} | Avg rate: {fps:.2f} frames/s | "
                "close window to send TESTEND"
            )

        def closeEvent(self, event) -> None:  # noqa: N802 - Qt API name
            self.timer.stop()
            try:
                self.ser.write(b"TESTEND")
                self.ser.flush()
                time.sleep(0.05)
            finally:
                self.ser.close()
            event.accept()

    app = QtWidgets.QApplication([])
    with serial.Serial(port, baud, timeout=0) as ser:
        window = TeensyLivePlotter(ser)
        window.resize(1100, 800)
        window.show()
        app_exec = getattr(app, "exec", None)
        if app_exec is None:
            app_exec = app.exec_
        app_exec()


def main() -> None:
    parser = argparse.ArgumentParser(description="Qt live plotter for Teensy 4RF TEST mode.")
    parser.add_argument("--port", default=DEFAULT_PORT, help=f"serial port, default {DEFAULT_PORT}")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help=f"baud rate, default {DEFAULT_BAUD}")
    parser.add_argument("--chunk-size", type=int, default=65536, help="max bytes read per GUI tick")
    parser.add_argument("--update-ms", type=int, default=20, help="serial poll interval in milliseconds")
    args = parser.parse_args()

    run_plotter(args.port, args.baud, args.chunk_size, args.update_ms)


if __name__ == "__main__":
    main()
