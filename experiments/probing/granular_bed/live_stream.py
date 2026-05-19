import sys
import struct
import serial
import numpy as np
import pyqtgraph as pg
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QPushButton,
    QVBoxLayout,
    QWidget,
    QHBoxLayout,
    QLabel,
    QLineEdit,
)
from PyQt5.QtCore import QThread, QTimer, pyqtSignal, pyqtSlot


class SerialWorker(QThread):
    # Signal to send (ch1, ch2, ch3, ch4, chunk_rate_hz, dt_since_last_us) to the GUI.
    # See quad_tx.ino for the wire format of each 'C' chunk:
    #   uint32 chunk_size + chunk_size*uint16 ch1 + chunk_size*uint16 ch2
    #     + chunk_size*uint16 ch3 + chunk_size*uint16 ch4
    #     + float32 chunk_rate_hz + uint32 dt_since_last_us
    data_ready = pyqtSignal(np.ndarray, np.ndarray, np.ndarray, np.ndarray, float, int)

    def __init__(self, port, baudrate=3000000):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.running = False
        self.stop_sent = False  # True only after stop() writes b"2".
        self.ser = None
        self.chunks_received = 0

    def read_exact(self, num_bytes):
        data = bytearray()
        while len(data) < num_bytes and self.running:
            chunk = self.ser.read(num_bytes - len(data))
            if chunk:
                data.extend(chunk)
        return bytes(data)

    def run(self):
        import traceback

        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.running = True

            self.ser.write(b"1")
            print("Sent START command ('1') to Teensy.")

            while self.running:
                # Blocking read with the serial timeout — no busy-wait spin.
                marker = self.ser.read(1)
                if not marker:
                    continue

                if marker == b"B":
                    print("Start marker ('B') received. Streaming...")

                elif marker == b"C":
                    size_bytes = self.read_exact(4)
                    if len(size_bytes) != 4:
                        continue
                    chunk_size = struct.unpack("<I", size_bytes)[0]

                    ch1_bytes = self.read_exact(chunk_size * 2)
                    ch2_bytes = self.read_exact(chunk_size * 2)
                    ch3_bytes = self.read_exact(chunk_size * 2)
                    ch4_bytes = self.read_exact(chunk_size * 2)
                    rate_bytes = self.read_exact(4)
                    dt_bytes = self.read_exact(4)

                    if (
                        len(ch1_bytes) == chunk_size * 2
                        and len(ch2_bytes) == chunk_size * 2
                        and len(ch3_bytes) == chunk_size * 2
                        and len(ch4_bytes) == chunk_size * 2
                        and len(rate_bytes) == 4
                        and len(dt_bytes) == 4
                    ):
                        # np.frombuffer returns a read-only view into the
                        # bytes object; copy so the array survives if the
                        # source bytes get GC'd before the slot runs.
                        ch1_data = np.frombuffer(ch1_bytes, dtype=np.uint16).copy()
                        ch2_data = np.frombuffer(ch2_bytes, dtype=np.uint16).copy()
                        ch3_data = np.frombuffer(ch3_bytes, dtype=np.uint16).copy()
                        ch4_data = np.frombuffer(ch4_bytes, dtype=np.uint16).copy()
                        chunk_rate_hz = struct.unpack("<f", rate_bytes)[0]
                        dt_since_last_us = struct.unpack("<I", dt_bytes)[0]

                        self.chunks_received += 1
                        # Periodic heartbeat — proves the worker is still alive.
                        if self.chunks_received % 200 == 0:
                            print(
                                f"[worker] {self.chunks_received} chunks, "
                                f"last rate={chunk_rate_hz:.1f} Hz, "
                                f"dt={dt_since_last_us / 1000:.1f} ms"
                            )

                        self.data_ready.emit(
                            ch1_data,
                            ch2_data,
                            ch3_data,
                            ch4_data,
                            float(chunk_rate_hz),
                            int(dt_since_last_us),
                        )

                elif marker == b"E":
                    # Only honour 'E' if WE sent the stop command. A spurious
                    # 0x45 byte from a wire desync would otherwise kill the
                    # stream silently. With 4 channels × ~1000 samples × 2
                    # bytes per chunk, the probability of an ADC byte equalling
                    # 'E' is high — guarding here is essential.
                    if not self.stop_sent:
                        print(
                            "[worker] saw 'E' but never sent STOP — likely a "
                            "wire desync; ignoring and continuing."
                        )
                        continue
                    total_bytes = self.read_exact(4)
                    if len(total_bytes) == 4:
                        total = struct.unpack("<I", total_bytes)[0]
                        print(
                            f"End marker ('E') received. Total samples recorded: {total}"
                        )
                    break

                # Any other byte (e.g. the 'Ready' string at boot, or junk after
                # a desync) is silently consumed.

        except Exception as e:
            print(f"Serial Error: {e}")
            traceback.print_exc()
        finally:
            print(f"[worker] exiting (chunks={self.chunks_received}).")
            if self.ser and self.ser.is_open:
                self.ser.close()

    def stop(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.stop_sent = True
            self.ser.write(b"2")
            print("Sent STOP command ('2') to Teensy.")
        self.wait()


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Teensy 40kHz Four-Channel Plotter")
        self.resize(1000, 400)

        # Buffer settings — 40 kHz nominal, 0.2 s of rolling history.
        self.sample_rate = 40000
        self.window_size = int(self.sample_rate * 1.00)
        self.y1_data = np.zeros(self.window_size)
        self.y2_data = np.zeros(self.window_size)
        self.y3_data = np.zeros(self.window_size)
        self.y4_data = np.zeros(self.window_size)
        self.dirty = False  # set on every chunk; the redraw timer consumes it

        # Plot Y-range (post-calibration). Plotted values are baseline-subtracted
        # ADC counts, so set to (0, limit) to show only positive deflections.
        self.y_min = 0
        self.y_max = 100

        # Calibration settings
        self.calibration_seconds = 2.0
        self.calib_samples_target = int(self.sample_rate * self.calibration_seconds)
        self.calibrating = False

        # Plot-rendering throttle. Chunks arrive at ~80 Hz, but redrawing 4×
        # 8 k-point curves that fast is what causes the lag — render at PLOT_HZ
        # with at most PLOT_MAX_POINTS samples per curve (stride-decimated).
        self.plot_hz = 30
        self.plot_max_points = 2000

        self.init_ui()
        self.worker = None

        self.redraw_timer = QTimer(self)
        self.redraw_timer.timeout.connect(self._redraw)
        self.redraw_timer.start(int(1000 / self.plot_hz))

    def init_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QVBoxLayout(main_widget)

        control_layout = QHBoxLayout()
        self.port_input = QLineEdit("/dev/ttyACM0")  # Adjusted default for Linux/ROS
        self.port_input.setFixedWidth(150)

        self.btn_start = QPushButton("Start Stream")
        self.btn_start.clicked.connect(self.start_stream)

        self.btn_stop = QPushButton("Stop Stream")
        self.btn_stop.clicked.connect(self.stop_stream)
        self.btn_stop.setEnabled(False)

        control_layout.addWidget(QLabel("Port:"))
        control_layout.addWidget(self.port_input)
        control_layout.addWidget(self.btn_start)
        control_layout.addWidget(self.btn_stop)
        control_layout.addStretch()
        self.rate_label = QLabel("Rate: — Hz")
        control_layout.addWidget(self.rate_label)
        layout.addLayout(control_layout)

        pg.setConfigOptions(antialias=True)
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setLabel("left", "Relative Value")
        self.plot_widget.setLabel("bottom", "Samples")
        self.plot_widget.addLegend()
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        layout.addWidget(self.plot_widget)

        self.curve1 = self.plot_widget.plot(
            pen=pg.mkPen("cyan", width=1.5), name="Channel 1 (pin 14)"
        )
        self.curve2 = self.plot_widget.plot(
            pen=pg.mkPen("magenta", width=1.5), name="Channel 2 (pin 15)"
        )
        self.curve3 = self.plot_widget.plot(
            pen=pg.mkPen("yellow", width=1.5), name="Channel 3 (pin 16)"
        )
        self.curve4 = self.plot_widget.plot(
            pen=pg.mkPen("lime", width=1.5), name="Channel 4 (pin 17)"
        )
        # Skip the per-sample isfinite() guard in pyqtgraph's path generation.
        for curve in (self.curve1, self.curve2, self.curve3, self.curve4):
            if hasattr(curve, "setSkipFiniteCheck"):
                curve.setSkipFiniteCheck(True)

    def start_stream(self):
        port = self.port_input.text().strip()
        if not port:
            return

        # 1. Reset buffers and calibration state
        self.y1_data = np.zeros(self.window_size)
        self.y2_data = np.zeros(self.window_size)
        self.y3_data = np.zeros(self.window_size)
        self.y4_data = np.zeros(self.window_size)

        self.calibrating = True
        self.calib_samples_collected = 0
        self.calib_sum_ch1 = 0.0
        self.calib_sum_ch2 = 0.0
        self.calib_sum_ch3 = 0.0
        self.calib_sum_ch4 = 0.0
        self.baseline_ch1 = 0.0
        self.baseline_ch2 = 0.0
        self.baseline_ch3 = 0.0
        self.baseline_ch4 = 0.0

        # Set plot to absolute ADC range during calibration
        self.plot_widget.setYRange(0, 4096)
        print(f"Calibrating for {self.calibration_seconds} seconds...")

        self.worker = SerialWorker(port)
        self.worker.data_ready.connect(self.update_plot)
        self.worker.finished.connect(self._on_worker_finished)
        self.worker.start()

        self.btn_start.setEnabled(False)
        self.btn_stop.setEnabled(True)
        self.port_input.setEnabled(False)

    def stop_stream(self):
        if self.worker:
            self.worker.stop()

        self.btn_start.setEnabled(True)
        self.btn_stop.setEnabled(False)
        self.port_input.setEnabled(True)

    @pyqtSlot(np.ndarray, np.ndarray, np.ndarray, np.ndarray, float, int)
    def update_plot(
        self, new_ch1, new_ch2, new_ch3, new_ch4, chunk_rate_hz, dt_since_last_us
    ):
        chunk_len = len(new_ch1)

        # Show the firmware's per-chunk rate + inter-chunk gap so it's obvious
        # the new packet format is being parsed; use last-known rate if this
        # chunk had too few samples to compute one (chunk_rate_hz reported as 0).
        if chunk_rate_hz > 0:
            self.last_rate_hz = chunk_rate_hz
        if getattr(self, "last_rate_hz", 0) > 0:
            self.rate_label.setText(
                f"Rate: {self.last_rate_hz:.1f} Hz  (dt={dt_since_last_us / 1000:.1f} ms)"
            )

        # 2. Calibration Phase Logic
        if self.calibrating:
            self.calib_sum_ch1 += np.sum(new_ch1)
            self.calib_sum_ch2 += np.sum(new_ch2)
            self.calib_sum_ch3 += np.sum(new_ch3)
            self.calib_sum_ch4 += np.sum(new_ch4)
            self.calib_samples_collected += chunk_len

            if self.calib_samples_collected >= self.calib_samples_target:
                # Calculate mean baselines
                self.baseline_ch1 = self.calib_sum_ch1 / self.calib_samples_collected
                self.baseline_ch2 = self.calib_sum_ch2 / self.calib_samples_collected
                self.baseline_ch3 = self.calib_sum_ch3 / self.calib_samples_collected
                self.baseline_ch4 = self.calib_sum_ch4 / self.calib_samples_collected
                self.calibrating = False

                print(
                    f"Calibration complete! Baselines: CH1={self.baseline_ch1:.1f}, "
                    f"CH2={self.baseline_ch2:.1f}, CH3={self.baseline_ch3:.1f}, "
                    f"CH4={self.baseline_ch4:.1f}"
                )

                # Switch plot to the configured positive Y-range
                self.plot_widget.setYRange(self.y_min, self.y_max)

        # 3. Vectorized Subtraction (with safety casting)
        if not self.calibrating:
            # Cast to float32 to prevent uint16 underflow, then subtract
            plot_ch1 = new_ch1.astype(np.float32) - self.baseline_ch1
            plot_ch2 = new_ch2.astype(np.float32) - self.baseline_ch2
            plot_ch3 = new_ch3.astype(np.float32) - self.baseline_ch3
            plot_ch4 = new_ch4.astype(np.float32) - self.baseline_ch4
        else:
            # Still calibrating, plot raw absolute values
            plot_ch1 = new_ch1.astype(np.float32)
            plot_ch2 = new_ch2.astype(np.float32)
            plot_ch3 = new_ch3.astype(np.float32)
            plot_ch4 = new_ch4.astype(np.float32)

        # 4. Roll arrays — rendering is deferred to _redraw on the QTimer.
        if chunk_len >= self.window_size:
            # Chunk larger than the buffer (only possible if CHUNK_SIZE in the
            # firmware exceeds window_size). Keep the most recent tail.
            self.y1_data[:] = plot_ch1[-self.window_size :]
            self.y2_data[:] = plot_ch2[-self.window_size :]
            self.y3_data[:] = plot_ch3[-self.window_size :]
            self.y4_data[:] = plot_ch4[-self.window_size :]
        else:
            self.y1_data[:-chunk_len] = self.y1_data[chunk_len:]
            self.y2_data[:-chunk_len] = self.y2_data[chunk_len:]
            self.y3_data[:-chunk_len] = self.y3_data[chunk_len:]
            self.y4_data[:-chunk_len] = self.y4_data[chunk_len:]
            self.y1_data[-chunk_len:] = plot_ch1
            self.y2_data[-chunk_len:] = plot_ch2
            self.y3_data[-chunk_len:] = plot_ch3
            self.y4_data[-chunk_len:] = plot_ch4
        self.dirty = True

    def _redraw(self):
        """Decimate buffers to plot_max_points and push to the curves.

        Runs on a QTimer at plot_hz; skips work when no new data arrived since
        the last redraw. Stride-decimation keeps render time bounded regardless
        of buffer size.
        """
        if not self.dirty:
            return
        self.dirty = False
        stride = max(1, self.window_size // self.plot_max_points)
        self.curve1.setData(self.y1_data[::stride])
        self.curve2.setData(self.y2_data[::stride])
        self.curve3.setData(self.y3_data[::stride])
        self.curve4.setData(self.y4_data[::stride])

    def _on_worker_finished(self):
        """Called on the GUI thread when the SerialWorker exits, intended or not.

        Re-enables the UI controls so the user can see streaming has stopped
        and can restart it. A printed `[worker] exiting` already appears on
        stdout with the chunk count and any traceback.
        """
        if self.worker is None or self.worker.stop_sent:
            return
        print("[gui] worker exited unexpectedly (no STOP was sent).")
        self.rate_label.setText("Rate: STOPPED (worker exited)")
        self.btn_start.setEnabled(True)
        self.btn_stop.setEnabled(False)
        self.port_input.setEnabled(True)

    def closeEvent(self, event):
        self.stop_stream()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
