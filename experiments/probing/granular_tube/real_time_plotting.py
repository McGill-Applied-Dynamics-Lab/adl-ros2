import sys
import time
import numpy as np
import serial
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg

# Change this to your Teensy's port!
SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 3000000
BUFFER_SIZE = 100000  # Must match the Teensy's BUFFER_SIZE


class SerialWorker(QtCore.QThread):
    # Signal to send the parsed NumPy arrays and elapsed time back to the GUI
    data_ready = QtCore.pyqtSignal(np.ndarray, np.ndarray, float)

    def __init__(self, port, baudrate, buffer_size):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.buffer_size = buffer_size
        self.running = True

    def run(self):
        try:
            ser = serial.Serial(self.port, self.baudrate, timeout=1)
            time.sleep(2)  # Give the serial connection a moment to initialize

            while self.running:
                # 1. Trigger Recording
                ser.write(b"1")

                # Wait for "Data Read" confirmation
                while self.running:
                    line = ser.readline().decode("utf-8", errors="ignore").strip()
                    if line == "Data Read":
                        break

                # 2. Trigger Data Dump
                ser.write(b"2")

                # Wait for the Start ('S') signal
                while self.running:
                    line = ser.readline().decode("utf-8", errors="ignore").strip()
                    if line == "S":
                        break

                # 3. Read Data Chunk
                data_list = []
                time_micros = 0.0

                # Read continuously until the stop signal 'T:' is received
                while self.running:
                    line = ser.readline().decode("utf-8", errors="ignore").strip()

                    if line.startswith("T:"):
                        try:
                            time_micros = float(line.split(":")[1])
                        except ValueError:
                            pass
                        break

                    if line:
                        try:
                            data_list.append(int(line))
                        except ValueError:
                            pass

                # 4. Parse and Emit
                if len(data_list) > 0:
                    # Ensure we have an even number of elements to avoid splitting errors
                    if len(data_list) % 2 != 0:
                        data_list.pop()

                    # Convert to a fast numpy array
                    data_arr = np.array(data_list, dtype=np.uint16)

                    # Since Teensy prints buffer1[i] then buffer2[i],
                    # evens are buffer1, odds are buffer2
                    buf1 = data_arr[0::2]
                    buf2 = data_arr[1::2]

                    self.data_ready.emit(buf1, buf2, time_micros)

        except Exception as e:
            print(f"Serial Error: {e}")

    def stop(self):
        self.running = False
        self.wait()


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, port):
        super().__init__()
        self.setWindowTitle("Teensy Real-Time Pressure Sensor Data")
        self.resize(900, 600)

        # 1. Setup the PyQtGraph widget
        self.graphWidget = pg.PlotWidget()
        self.setCentralWidget(self.graphWidget)
        self.graphWidget.setBackground("w")
        self.graphWidget.setTitle("Awaiting Data...", color="k", size="14pt")
        self.graphWidget.setLabel("left", "ADC Value (12-bit)", color="k")
        self.graphWidget.setLabel("bottom", "Sample Index", color="k")
        self.graphWidget.showGrid(x=True, y=True)
        self.graphWidget.addLegend()

        # 2. Initialize the plot curves
        # Using distinct colors for the two pressure sensors
        self.curve1 = self.graphWidget.plot(
            pen=pg.mkPen("b", width=1.5), name="A7 (Buffer 1)"
        )
        self.curve2 = self.graphWidget.plot(
            pen=pg.mkPen("r", width=1.5), name="A5 (Buffer 2)"
        )

        # 3. Start the Serial Worker Thread
        self.worker = SerialWorker(port, BAUD_RATE, BUFFER_SIZE)
        self.worker.data_ready.connect(self.update_plot)
        self.worker.start()

    def update_plot(self, buf1, buf2, time_micros):
        # Update the graph lines with the new chunk
        self.curve1.setData(buf1)
        self.curve2.setData(buf2)

        # Calculate and display the sampling rate based on the Teensy's timing
        if time_micros > 0:
            sample_rate_hz = (len(buf1) / time_micros) * 1000000.0
            self.graphWidget.setTitle(
                f"Sensors Data | Chunk Size: {len(buf1)} | Teensy Sample Rate: {sample_rate_hz:.2f} Hz",
                color="k",
            )

    def closeEvent(self, event):
        # Ensure the serial port and thread close cleanly when exiting
        self.worker.stop()
        event.accept()


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow(SERIAL_PORT)
    window.show()
    sys.exit(app.exec_())
