"""Live trace overlaid on a saved target trace, to reproduce the same initial conditions before a recording.

Usage:
    python LIVE_PLOT_MATCH.py target.csv [--port /dev/ttyACM0] [--average 5]

The target is a CSV of one trace, e.g. exported from the LIVE_PLOT.py window (right click -> Export -> CSV, which
writes x/y column pairs: sample index, ADC value), or a plain file with one value per row. Header lines are skipped.

Top: the target (orange), the latest frame (faint cyan) and the average of the last --average frames (cyan).
Bottom: averaged live minus target. Drag the shaded region to choose the samples compared; the title shows the RMS
difference and the correlation there. Press S to save the current averaged trace as a new target CSV.
"""

import argparse
import sys
from collections import deque
from datetime import datetime

import numpy as np
import pyqtgraph as pg
import serial
from pyqtgraph.Qt import QtCore, QtWidgets

BUFFER_SIZE = 1500

# 3-byte header (0xAA, 0x55, 0x00) + (BUFFER_SIZE * 2 bytes)
PACKET_SIZE = 3 + (BUFFER_SIZE * 2)


def load_target(path):
    """Trace values from a CSV: the y column of an x/y export (the first y if there are several curves), or the
    only column. Rows that are not all numbers (headers) are skipped."""
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
    # x/y pairs: place the y values at their sample index
    x, y = data[:, 0], data[:, 1]
    trace = np.full(int(x.max()) + 1, np.nan)
    trace[x.astype(int)] = y
    return trace


def save_trace(trace):
    """Write a trace in the same x/y format as the pyqtgraph CSV export."""
    path = f"target_{datetime.now():%Y%m%d_%H%M%S}.csv"
    np.savetxt(path, np.column_stack([np.arange(len(trace)), trace]), delimiter=",", header="x0000,y0000",
               comments="", fmt=["%d", "%.2f"])
    print(f"Saved the current averaged trace to {path}")


parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("target", help="CSV file with the target trace")
parser.add_argument("--port", default="/dev/ttyACM0")
parser.add_argument("--average", type=int, default=5,
                    help="frames averaged for the live trace and the comparison (the recordings use 5)")
args = parser.parse_args()

target = load_target(args.target)
if len(target) != BUFFER_SIZE:
    print(f"Note: the target has {len(target)} samples, the live trace {BUFFER_SIZE}; comparing the overlap only.")
n_compare = min(len(target), BUFFER_SIZE)

ser = serial.Serial(args.port, 3000000, timeout=0.01)

app = QtWidgets.QApplication(sys.argv)
win = pg.GraphicsLayoutWidget(show=True, title="Live trace vs target")
win.resize(1200, 700)

p = win.addPlot(title="Live vs target")
p.setYRange(0, 4096)
p.setXRange(0, BUFFER_SIZE)
p.setLabel("left", "ADC Raw Amplitude (12-bit)")
p.showGrid(x=True, y=True, alpha=0.3)
p.addLegend()
target_curve = p.plot(target, pen=pg.mkPen((255, 150, 0), width=1.5), name="target")
latest_curve = p.plot(pen=pg.mkPen((0, 255, 255, 70), width=1), name="latest frame")
average_curve = p.plot(pen=pg.mkPen("c", width=1.5), name=f"live, average of {args.average}")

# Samples compared (draggable); the whole trace by default
region = pg.LinearRegionItem(values=(0, n_compare - 1), brush=(255, 255, 255, 20))
p.addItem(region)

win.nextRow()
d = win.addPlot(title="Live (averaged) minus target")
d.setXLink(p)
d.setLabel("bottom", "Sample Index (Distance = Index * C / (2 * Fs))")
d.setLabel("left", "Difference (ADC)")
d.showGrid(x=True, y=True, alpha=0.3)
d.addLine(y=0, pen=pg.mkPen((150, 150, 150), width=1))
difference_curve = d.plot(pen=pg.mkPen("m", width=1))
win.ci.layout.setRowStretchFactor(0, 3)
win.ci.layout.setRowStretchFactor(1, 2)

raw_buffer = bytearray()
recent = deque(maxlen=max(1, args.average))


def update_plot():
    global raw_buffer

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
        latest_frame = np.frombuffer(payload, dtype=np.uint16).astype(float)

    if latest_frame is None:
        return

    recent.append(latest_frame)
    average = np.mean(recent, axis=0)
    latest_curve.setData(latest_frame)
    average_curve.setData(average)

    difference = average[:n_compare] - target[:n_compare]
    difference_curve.setData(difference)

    # Match within the selected samples (target samples missing from the CSV are skipped)
    lo, hi = (int(round(v)) for v in region.getRegion())
    lo, hi = max(0, lo), min(n_compare, hi + 1)
    live_part, target_part = average[lo:hi], target[lo:hi]
    valid = ~np.isnan(target_part)
    if valid.sum() > 2:
        rms = np.sqrt(np.mean((live_part[valid] - target_part[valid]) ** 2))
        corr = np.corrcoef(live_part[valid], target_part[valid])[0, 1]
        p.setTitle(f"Live vs target, samples {lo}-{hi - 1}: RMS difference {rms:.1f} ADC, correlation {corr:.4f}"
                   f" (press S to save the live trace as a target)")


def key_pressed(event):
    if event.key() == QtCore.Qt.Key.Key_S and recent:
        save_trace(np.mean(recent, axis=0))


win.keyPressEvent = key_pressed

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
