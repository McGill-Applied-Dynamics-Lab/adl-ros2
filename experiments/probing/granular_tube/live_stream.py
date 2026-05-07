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
from PyQt5.QtCore import QThread, pyqtSignal, pyqtSlot


class SerialWorker(QThread):
    # Signal to send (chunk1, chunk2, chunk_rate_hz, dt_since_last_us) to the GUI thread.
    # See teensy_tx.ino for the wire format of each 'C' chunk:
    #   uint32 chunk_size + chunk_size*uint16 ch1 + chunk_size*uint16 ch2
    #     + float32 chunk_rate_hz + uint32 dt_since_last_us
    data_ready = pyqtSignal(np.ndarray, np.ndarray, float, int)

    def __init__(self, port, baudrate=3000000):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.running = False
        self.ser = None

    def read_exact(self, num_bytes):
        data = bytearray()
        while len(data) < num_bytes and self.running:
            chunk = self.ser.read(num_bytes - len(data))
            if chunk:
                data.extend(chunk)
        return bytes(data)

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.running = True

            self.ser.write(b"1")
            print("Sent START command ('1') to Teensy.")

            while self.running:
                if self.ser.in_waiting > 0:
                    marker = self.ser.read(1)

                    if marker == b"B":
                        print("Start marker ('B') received. Streaming...")

                    elif marker == b"C":
                        size_bytes = self.read_exact(4)
                        if len(size_bytes) != 4:
                            continue
                        chunk_size = struct.unpack("<I", size_bytes)[0]

                        ch1_bytes = self.read_exact(chunk_size * 2)
                        ch2_bytes = self.read_exact(chunk_size * 2)
                        rate_bytes = self.read_exact(4)
                        dt_bytes = self.read_exact(4)

                        if (
                            len(ch1_bytes) == chunk_size * 2
                            and len(ch2_bytes) == chunk_size * 2
                            and len(rate_bytes) == 4
                            and len(dt_bytes) == 4
                        ):
                            ch1_data = np.frombuffer(ch1_bytes, dtype=np.uint16)
                            ch2_data = np.frombuffer(ch2_bytes, dtype=np.uint16)
                            chunk_rate_hz = struct.unpack("<f", rate_bytes)[0]
                            dt_since_last_us = struct.unpack("<I", dt_bytes)[0]

                            self.data_ready.emit(
                                ch1_data,
                                ch2_data,
                                float(chunk_rate_hz),
                                int(dt_since_last_us),
                            )

                    elif marker == b"E":
                        total_bytes = self.read_exact(4)
                        if len(total_bytes) == 4:
                            total = struct.unpack("<I", total_bytes)[0]
                            print(
                                f"End marker ('E') received. Total samples recorded: {total}"
                            )
                        break

        except Exception as e:
            print(f"Serial Error: {e}")
        finally:
            if self.ser and self.ser.is_open:
                self.ser.close()

    def stop(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.write(b"2")
            print("Sent STOP command ('2') to Teensy.")
        self.wait()


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Teensy 10kHz Dual-Channel Plotter")
        self.resize(1000, 400)

        # Buffer settings
        self.sample_rate = 10000
        self.window_size = int(self.sample_rate * 1.00)  # 2 seconds of history
        self.y1_data = np.zeros(self.window_size)
        self.y2_data = np.zeros(self.window_size)

        # Plot Y-range (post-calibration). Plotted values are baseline-subtracted
        # ADC counts, so set to (0, limit) to show only positive deflections.
        self.y_min = 0
        self.y_max = 300

        # Calibration settings
        self.calibration_seconds = 2.0
        self.calib_samples_target = int(self.sample_rate * self.calibration_seconds)
        self.calibrating = False

        self.init_ui()
        self.worker = None

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
            self.y1_data, pen=pg.mkPen("cyan", width=1.5), name="Channel 1 (A7)"
        )
        self.curve2 = self.plot_widget.plot(
            self.y2_data, pen=pg.mkPen("magenta", width=1.5), name="Channel 2 (A5)"
        )

    def start_stream(self):
        port = self.port_input.text().strip()
        if not port:
            return

        # 1. Reset buffers and calibration state
        self.y1_data = np.zeros(self.window_size)
        self.y2_data = np.zeros(self.window_size)

        self.calibrating = True
        self.calib_samples_collected = 0
        self.calib_sum_ch1 = 0.0
        self.calib_sum_ch2 = 0.0
        self.baseline_ch1 = 0.0
        self.baseline_ch2 = 0.0

        # Set plot to absolute ADC range during calibration
        self.plot_widget.setYRange(0, 4096)
        print(f"Calibrating for {self.calibration_seconds} seconds...")

        self.worker = SerialWorker(port)
        self.worker.data_ready.connect(self.update_plot)
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

    @pyqtSlot(np.ndarray, np.ndarray, float, int)
    def update_plot(self, new_ch1, new_ch2, chunk_rate_hz, dt_since_last_us):
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
            self.calib_samples_collected += chunk_len

            if self.calib_samples_collected >= self.calib_samples_target:
                # Calculate mean baselines
                self.baseline_ch1 = self.calib_sum_ch1 / self.calib_samples_collected
                self.baseline_ch2 = self.calib_sum_ch2 / self.calib_samples_collected
                self.calibrating = False

                print(
                    f"Calibration complete! Baselines: CH1={self.baseline_ch1:.1f}, CH2={self.baseline_ch2:.1f}"
                )

                # Switch plot to the configured positive Y-range
                self.plot_widget.setYRange(self.y_min, self.y_max)

        # 3. Vectorized Subtraction (with safety casting)
        if not self.calibrating:
            # Cast to float32 to prevent uint16 underflow, then subtract
            plot_ch1 = new_ch1.astype(np.float32) - self.baseline_ch1
            plot_ch2 = new_ch2.astype(np.float32) - self.baseline_ch2
        else:
            # Still calibrating, plot raw absolute values
            plot_ch1 = new_ch1.astype(np.float32)
            plot_ch2 = new_ch2.astype(np.float32)

        # 4. Roll arrays and update visuals
        self.y1_data = np.roll(self.y1_data, -chunk_len)
        self.y2_data = np.roll(self.y2_data, -chunk_len)

        self.y1_data[-chunk_len:] = plot_ch1
        self.y2_data[-chunk_len:] = plot_ch2

        self.curve1.setData(self.y1_data)
        self.curve2.setData(self.y2_data)

    def closeEvent(self, event):
        self.stop_stream()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
