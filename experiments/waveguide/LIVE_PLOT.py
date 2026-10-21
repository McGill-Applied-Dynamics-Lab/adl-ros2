import sys
import time
import serial
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore

BUFFER_SIZE = 1500

# 3-byte header (0xAA, 0x55, 0x00) + (1042 * 2 bytes) = 2087 bytes total
PACKET_SIZE = 3 + (BUFFER_SIZE * 2)

ser = serial.Serial("/dev/ttyACM0", 3000000, timeout=0.01)

app = QtWidgets.QApplication(sys.argv)
win = pg.GraphicsLayoutWidget(show=True, title="Hardware-Synced Ultrasonic Echo")
win.resize(1200, 450)

p = win.addPlot(title="Sensor 0 Echo (Time-Locked to PW Pin)")
p.setYRange(0, 4096)
p.setXRange(0, BUFFER_SIZE)
p.setLabel("bottom", "Sample Index (Distance = Index * C / (2 * Fs))")
p.setLabel("left", "ADC Raw Amplitude (12-bit)")
p.showGrid(x=True, y=True, alpha=0.3)

curve = p.plot(pen=pg.mkPen("c", width=1.5))
raw_buffer = bytearray()

# Ping rate, printed once a second: frames received (every complete packet) and frames drawn (one per timer tick at
# most, so a frame is skipped when two arrive between ticks)
RATE_INTERVAL_S = 1.0
rate_start = time.monotonic()
frames_received = frames_plotted = 0


def update_plot():
    global raw_buffer, rate_start, frames_received, frames_plotted

    # Pull available incoming bytes without blocking Qt
    waiting = ser.in_waiting
    if waiting:
        raw_buffer.extend(ser.read(waiting))

    latest_frame = None

    # Parse packets, keeping only the most recent complete capture
    while len(raw_buffer) >= PACKET_SIZE:
        idx = raw_buffer.find(b"\xaa\x55\x00")
        if idx == -1:
            raw_buffer = raw_buffer[-2:]
            break

        if len(raw_buffer) < idx + PACKET_SIZE:
            break

        payload = raw_buffer[idx + 3 : idx + PACKET_SIZE]
        raw_buffer = raw_buffer[idx + PACKET_SIZE :]
        latest_frame = np.frombuffer(payload, dtype=np.uint16)
        frames_received += 1

    # Plot directly to keep time reference fixed
    if latest_frame is not None:
        curve.setData(latest_frame)
        frames_plotted += 1

    elapsed = time.monotonic() - rate_start
    if elapsed >= RATE_INTERVAL_S:
        print(f"\rPing rate: {frames_received / elapsed:5.1f} Hz | plotted {frames_plotted / elapsed:5.1f} Hz | "
              f"skipped {frames_received - frames_plotted:3d}", end="", flush=True)
        rate_start += elapsed
        frames_received = frames_plotted = 0


timer = QtCore.QTimer()
timer.timeout.connect(update_plot)
timer.start(16)  # ~60 FPS update rate

if __name__ == "__main__":
    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
