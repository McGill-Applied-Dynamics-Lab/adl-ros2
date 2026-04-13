#!/usr/bin/env python3
import os
import socket
import struct
import threading
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from scipy.signal import butter, filtfilt

# Matplotlib backend: GUI if available, otherwise run headless (no window)
import matplotlib
if "DISPLAY" in os.environ and os.environ["DISPLAY"]:
    matplotlib.use("TkAgg")
else:
    matplotlib.use("Agg")
import matplotlib.pyplot as plt

from acoustic_sensing.msg import AcousticPacket


# ---------------- TCP / Data params ----------------
HOST = "0.0.0.0"
PORT = 1234

SAMPLES_PER_CYCLE = 4000
BYTES_TIMESTAMP = 2
BYTES_SAMPLES = SAMPLES_PER_CYCLE * 2  # uint16


# ---------------- Processing / Plot params ----------------
FS = 2e5
CUTOFF = 100.0
LP_ORDER = 4
LP_B, LP_A = butter(LP_ORDER, CUTOFF / (0.5 * FS), btype="low")

PRE_SAMPLES = 50
PLOT_LEN = 4000
THRESHOLD_FAC = 100.0

PLOT_UPDATE_HZ = 30.0  # redraw rate


def recvall(conn: socket.socket, n: int) -> bytes:
    """Receive exactly n bytes or return b'' if the connection closes."""
    data = bytearray()
    while len(data) < n:
        chunk = conn.recv(n - len(data))
        if not chunk:
            return b""
        data.extend(chunk)
    return bytes(data)


class TcpReceiverPlotPublisher(Node):
    """
    Receives BBB TCP stream:
      loop:
        uint16 timeDiff (2 bytes)
        uint16 samples[4000] (8000 bytes)
    Publishes AcousticPacket on /acoustic/raw and plots processed waveform.
    """

    def __init__(self):
        super().__init__("tcp_receiver_plot_publisher")

        # Parameters
        self.declare_parameter("rf_id_constant", 1)
        self.rf_id_constant = int(self.get_parameter("rf_id_constant").value)

        self.declare_parameter("plot_abs", True)
        self.plot_abs = bool(self.get_parameter("plot_abs").value)

        # ROS publisher
        self.pub = self.create_publisher(AcousticPacket, "/acoustic/raw", qos_profile_sensor_data)

        # Shared state for plotting
        self._lock = threading.Lock()
        self.have_data = False
        self.latest_waveform = np.zeros(PLOT_LEN, dtype=float)
        self.latest_time_ms = 0

        # Plot setup (only if GUI backend)
        self.gui_enabled = matplotlib.get_backend().lower() != "agg"
        if self.gui_enabled:
            self.fig, self.ax = plt.subplots(figsize=(9, 4))
            x = np.arange(PLOT_LEN)
            (self.line,) = self.ax.plot(x, np.zeros(PLOT_LEN), lw=1)
            self.ax.axvline(PRE_SAMPLES, color="gray", linestyle="--")
            self.ax.set_ylim([-10, 4000])
            self.ax.set_xlabel("Sample (aligned)")
            self.ax.set_ylabel("Amplitude" + (" (abs)" if self.plot_abs else ""))
            self.ax.set_title("Waiting for data...")
            plt.ion()
            plt.show(block=False)

            self.plot_timer = self.create_timer(1.0 / PLOT_UPDATE_HZ, self._update_plot)
            self.get_logger().info("GUI display detected; plotting enabled.")
        else:
            self.get_logger().warn("No DISPLAY detected (headless). Plot window will not open. Publishing still works.")

        # TCP server
        self._stop = threading.Event()
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((HOST, PORT))
        self.server.listen(1)
        self.get_logger().info(f"TCP server listening on {HOST}:{PORT} (waiting for BBB client)")

        self.tcp_thread = threading.Thread(target=self._accept_loop, daemon=True)
        self.tcp_thread.start()

    def _accept_loop(self):
        while rclpy.ok() and not self._stop.is_set():
            try:
                conn, addr = self.server.accept()
            except OSError:
                break

            self.get_logger().info(f"Accepted connection from {addr}")
            try:
                self._recv_loop(conn)
            except Exception as e:
                self.get_logger().error(f"TCP receive loop crashed: {e}")
            finally:
                try:
                    conn.close()
                except Exception:
                    pass
                self.get_logger().info("Client disconnected.")

    def _process_waveform(self, samples_u16: np.ndarray) -> np.ndarray:
        # Convert to int16-like then float
        raw = samples_u16.astype(np.int16).astype(float)

        # DC removal
        try:
            dc_est = filtfilt(LP_B, LP_A, raw)
            dc_removed = raw - dc_est
        except Exception:
            dc_removed = raw - np.mean(raw)

        # Pulse detection
        rect = np.abs(dc_removed)
        b_len = max(int(0.1 * len(rect)), 1)
        mean0, std0 = rect[:b_len].mean(), rect[:b_len].std()
        thresh = mean0 + THRESHOLD_FAC * std0
        over = np.nonzero(rect > thresh)[0]
        pulse_idx = int(over[0]) if len(over) else 0

        # Align
        shift = PRE_SAMPLES - pulse_idx
        aligned = np.roll(dc_removed, shift)

        out = np.zeros(PLOT_LEN, dtype=float)
        out[: min(PLOT_LEN, len(aligned))] = aligned[:PLOT_LEN]
        return out

    def _recv_loop(self, conn: socket.socket):
        while rclpy.ok() and not self._stop.is_set():
            # 1) timestamp
            t_b = recvall(conn, BYTES_TIMESTAMP)
            if not t_b:
                break
            time_offset_ms = struct.unpack("=H", t_b)[0]

            # 2) samples
            s_b = recvall(conn, BYTES_SAMPLES)
            if not s_b:
                break
            samples_u16 = np.frombuffer(s_b, dtype=np.uint16)

            # Publish ROS message
            msg = AcousticPacket()
            msg.rf_id = int(self.rf_id_constant)
            msg.time_offset_ms = int(time_offset_ms)
            msg.samples = samples_u16.tolist()
            self.pub.publish(msg)

            # Prepare waveform for plotting
            processed = self._process_waveform(samples_u16)
            y = np.abs(processed) if self.plot_abs else processed

            with self._lock:
                self.latest_waveform = y
                self.latest_time_ms = int(time_offset_ms)
                self.have_data = True

    def _update_plot(self):
        if not self.gui_enabled:
            return

        with self._lock:
            if not self.have_data:
                return
            y = self.latest_waveform.copy()
            t_ms = self.latest_time_ms

        self.line.set_ydata(y)
        self.ax.set_ylim([-10, 4000])
        self.ax.set_title(f"RF={self.rf_id_constant} | t_offset={t_ms} ms")
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
        plt.pause(0.001)

    def destroy_node(self):
        self._stop.set()
        try:
            self.server.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = TcpReceiverPlotPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        try:
            plt.close("all")
        except Exception:
            pass


if __name__ == "__main__":
    main()
