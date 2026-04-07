#!/usr/bin/env python3
import numpy as np
import matplotlib
matplotlib.use("TkAgg")  # GUI backend for devcontainer w/ X11
import matplotlib.pyplot as plt

from scipy.signal import butter, filtfilt

import rclpy
from rclpy.node import Node
from acoustic_sensing.msg import AcousticPacket


# ---------------- Parameters ----------------
SAMPLES_PER_CYCLE = 4000
FS = 2e5

CUTOFF = 100.0
LP_ORDER = 4
LP_B, LP_A = butter(LP_ORDER, CUTOFF / (0.5 * FS), btype='low')

PRE_SAMPLES = 50
PLOT_LEN = 4000
THRESHOLD_FAC = 100.0

UPDATE_INTERVAL_SEC = 0.02


class TwoWindowPlotNode(Node):
    def __init__(self):
        super().__init__('two_window_plot_node')

        self.sub = self.create_subscription(
            AcousticPacket,
            'acoustic/raw',
            self.packet_callback,
            10
        )
        self.get_logger().info("Subscribed to /acoustic/raw (expecting rf_id = 1 or 2)")

        # Buffers
        self.have_rf1 = False
        self.have_rf2 = False
        self.rf1_data = np.zeros(PLOT_LEN)
        self.rf2_data = np.zeros(PLOT_LEN)

        # --------- Window 1: RF1 ----------
        self.fig1, self.ax1 = plt.subplots(figsize=(8, 4))
        x = np.arange(PLOT_LEN)
        (self.line1,) = self.ax1.plot(x, np.zeros(PLOT_LEN), lw=1)
        self.ax1.axvline(PRE_SAMPLES, color='gray', linestyle='--')
        self.ax1.set_title("RF1 (rf_id=1)")
        self.ax1.set_ylim([-10, 4000])
        self.fig1.canvas.manager.set_window_title("RF1")

        # --------- Window 2: RF2 ----------
        self.fig2, self.ax2 = plt.subplots(figsize=(8, 4))
        (self.line2,) = self.ax2.plot(x, np.zeros(PLOT_LEN), lw=1)
        self.ax2.axvline(PRE_SAMPLES, color='gray', linestyle='--')
        self.ax2.set_title("RF2 (rf_id=2)")
        self.ax2.set_ylim([-10, 4000])
        self.fig2.canvas.manager.set_window_title("RF2")

        plt.show(block=False)

        # Timer to refresh
        self.timer = self.create_timer(UPDATE_INTERVAL_SEC, self.update_plots)

    # ----------------------------------
    def process_waveform(self, raw):
        dc_est = filtfilt(LP_B, LP_A, raw)
        dc_removed = raw - dc_est

        rect = np.abs(dc_removed)
        b_len = max(int(0.1 * len(rect)), 1)
        mean0, std0 = rect[:b_len].mean(), rect[:b_len].std()
        thresh = mean0 + THRESHOLD_FAC * std0
        over = np.nonzero(rect > thresh)[0]
        pulse_idx = int(over[0]) if len(over) else 0

        shift = PRE_SAMPLES - pulse_idx
        aligned = np.roll(dc_removed, shift)

        out = np.zeros(PLOT_LEN)
        out[: min(PLOT_LEN, len(aligned))] = aligned[:PLOT_LEN]

        return out

    # ----------------------------------
    def packet_callback(self, msg: AcousticPacket):
        raw = np.array(msg.samples, dtype=float)
        processed = self.process_waveform(raw)

        if msg.rf_id == 1:
            self.rf1_data = processed
            self.have_rf1 = True
        elif msg.rf_id == 2:
            self.rf2_data = processed
            self.have_rf2 = True

    # ----------------------------------
    def update_plots(self):
        updated = False

        if self.have_rf1:
            self.line1.set_ydata(np.abs(self.rf1_data))
            self.fig1.canvas.draw()
            updated = True

        if self.have_rf2:
            self.line2.set_ydata(np.abs(self.rf2_data))
            self.fig2.canvas.draw()
            updated = True

        if updated:
            plt.pause(0.001)


def main(args=None):
    rclpy.init(args=args)
    node = TwoWindowPlotNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
