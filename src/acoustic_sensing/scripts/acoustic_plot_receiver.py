#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt
import matplotlib
matplotlib.use("TkAgg")


from acoustic_sensing.msg import AcousticPacket
# === Parameters (mirroring your TCP script) ==================================

SAMPLES_PER_CYCLE = 4000      # buf_size on BBB / in AcousticPacket.samples
FS                = 2e5       # ADC sampling rate (Hz)

# DC-removal filter
CUTOFF   = 100.0              # low-pass cutoff (Hz)
LP_ORDER = 4
LP_B, LP_A = butter(LP_ORDER, CUTOFF / (0.5 * FS), btype="low")

# Alignment & plotting
PRE_SAMPLES   = 50            # desired pulse index in the aligned trace
PLOT_LEN      = 4000          # number of samples to display
THRESHOLD_FAC = 100           # for rectified pulse detection

UPDATE_INTERVAL_SEC = 0.01    # plot update interval


class AcousticPlotNode(Node):
    def __init__(self):
        super().__init__("acoustic_plot_node")

        # Subscriber for AcousticPacket
        self.sub_ = self.create_subscription(
            AcousticPacket,
            "/acoustic/raw",
            self.packet_callback,
            10
        )

        self.get_logger().info("AcousticPlotNode subscribed to /acoustic/raw")

        # Shared state for plotting
        self.latest_waveform = np.zeros(PLOT_LEN, dtype=float)
        self.latest_rf_id = 0
        self.latest_time_offset = 0
        self.have_data = False

        # Set up matplotlib figure
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot(np.zeros(PLOT_LEN), lw=1)
        self.ax.axvline(PRE_SAMPLES, color="gray", linestyle="--", label="Pulse index")
        self.ax.set_xlabel("Sample (aligned)")
        self.ax.set_ylabel("Amplitude (DC-removed, abs)")
        self.ax.set_title("Real-time Acoustic Trace (waiting for data...)")
        self.ax.legend()
        self.ax.set_ylim([-10, 4000])

        # interactive / non-blocking
        plt.ion()
        plt.show(block=False)

    # -------------------------------------------------------------------------
    # ROS callback: receive AcousticPacket and process waveform
    # -------------------------------------------------------------------------
    def packet_callback(self, msg: AcousticPacket):
        """
        Called every time a new AcousticPacket arrives on /acoustic/raw.
        We:
          - convert samples to float
          - DC-remove via low-pass (filtfilt)
          - detect pulse
          - align pulse to PRE_SAMPLES
          - store an aligned segment for plotting
        """
        # Convert uint16 samples to float (and cast to int16-like range if needed)
        raw = np.array(msg.samples, dtype=np.int16).astype(float)

        if raw.size == 0:
            self.get_logger().warn("Received empty samples array.")
            return

        # 1) DC removal
        try:
            dc_est = filtfilt(LP_B, LP_A, raw)
            dc_removed = raw - dc_est
        except Exception as e:
            self.get_logger().warn(f"filtfilt failed: {e}")
            dc_removed = raw - np.mean(raw)

        # 2) Pulse detection on rectified signal
        rect = np.abs(dc_removed)

        b_len = max(int(0.1 * rect.size), 1)  # first 10% as baseline
        baseline = rect[:b_len]
        mean0 = baseline.mean()
        std0 = baseline.std()
        thresh = mean0 + THRESHOLD_FAC * std0

        over = np.nonzero(rect > thresh)[0]
        pulse_idx = int(over[0]) if over.size > 0 else 0

        # 3) Align: roll so pulse lands at PRE_SAMPLES
        shift = PRE_SAMPLES - pulse_idx
        aligned = np.roll(dc_removed, shift)

        # 4) Prepare data for plotting
        if aligned.size >= PLOT_LEN:
            to_plot = aligned[:PLOT_LEN]
        else:
            to_plot = np.zeros(PLOT_LEN)
            to_plot[:aligned.size] = aligned

        # 5) Store in shared state (abs value, like your script)
        self.latest_waveform = np.abs(to_plot)
        self.latest_rf_id = msg.rf_id
        self.latest_time_offset = msg.time_offset_ms
        self.have_data = True

    # -------------------------------------------------------------------------
    # Plot update: called from main loop
    # -------------------------------------------------------------------------
    def update_plot(self):
        if not self.have_data:
            return

        self.line.set_ydata(self.latest_waveform)
        # Fixed y-limits like your script
        self.ax.set_ylim([-10, 4000])

        title = f"Real-time Acoustic Trace | rf_id={self.latest_rf_id} | t_offset={self.latest_time_offset} ms"
        self.ax.set_title(title)

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


def main():
    rclpy.init()

    node = AcousticPlotNode()

    try:
        # We don't use rclpy.spin(node) because that would block matplotlib.
        # Instead we spin "manually" and update the plot in the same thread.
        last_update = 0.0

        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)

            # Update plot at ~100 Hz max
            node.update_plot()
            plt.pause(UPDATE_INTERVAL_SEC)

    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt - shutting down plot node.")

    finally:
        node.destroy_node()
        rclpy.shutdown()
        plt.close("all")


if __name__ == "__main__":
    main()
