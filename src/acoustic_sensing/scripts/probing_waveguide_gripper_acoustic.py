#!/usr/bin/env python3
import pickle

# -------------------- ROS/acoustic imports --------------------
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
from arm_client.robot import Pose, Robot, Twist
from rclpy.context import Context
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.signal import butter, filtfilt
from scipy.spatial.transform import Rotation as R
from waveguide_gripper_grid_generator import fetch_landmarks

from acoustic_sensing.msg import AcousticPacket
from arm_client import CONFIG_DIR

# -------------------------------------------------------------

# Keep only ONE copy of these (your pasted code had duplicates)
SETTLE_SEC = 1.00  # wait time after moves (s)
TRAJ_FREQ = 10.0  # Hz


# -------------------- Acoustic processing params --------------------
SAMPLES_PER_CYCLE = 4000
FS = 2e5
CUTOFF = 100.0
LP_ORDER = 4
LP_B, LP_A = butter(LP_ORDER, CUTOFF / (0.5 * FS), btype="low")

PRE_SAMPLES = 50
PLOT_LEN = 4000
THRESHOLD_FAC = 100
# -------------------------------------------------------------------


def ensure_ros_context():
    ctx = Context()
    if not ctx.ok():
        rclpy.init(context=ctx)
    return ctx


class AcousticRecorderNode(Node):
    """
    Subscribes to /acoustic/raw (AcousticPacket) and buffers packets.
    start_segment(t0_perf) / stop_segment() returns only packets in that window,
    aligned to t0_perf (same perf_counter anchor used by probe()).

    Receiver detection:
      - wait_for_first_packet(timeout)
      - last_rx_age_sec()
    """

    def __init__(self, topic="/acoustic/raw", max_packets=200000, context=None, store_raw=True):
        super().__init__("acoustic_recorder_node", context=context)
        # IMPORTANT: match publisher QoS (sensor data is typically BEST_EFFORT)
        self.sub_ = self.create_subscription(AcousticPacket, topic, self._cb, qos_profile_sensor_data)

        self.store_raw = bool(store_raw)

        self._lock = threading.Lock()
        self._buf = deque(maxlen=max_packets)

        # Receiver detection
        self._rx_count = 0
        self._rx_last_perf = None
        self._rx_evt = threading.Event()

        # Segment control
        self._seg_active = False
        self._seg_t0 = None
        self._seg_start_len = 0

    def _process_packet(self, samples_u16: np.ndarray) -> np.ndarray:
        raw = samples_u16.astype(np.int16).astype(np.float32)

        # DC removal
        try:
            dc_est = filtfilt(LP_B, LP_A, raw)
            dc_removed = raw - dc_est
        except Exception:
            dc_removed = raw - raw.mean()

        # Pulse detect
        rect = np.abs(dc_removed)
        b_len = max(int(0.1 * rect.size), 1)
        baseline = rect[:b_len]
        thresh = baseline.mean() + THRESHOLD_FAC * baseline.std()

        over = np.nonzero(rect > thresh)[0]
        pulse_idx = int(over[0]) if over.size > 0 else 0

        # Align pulse to PRE_SAMPLES
        shift = PRE_SAMPLES - pulse_idx
        aligned = np.roll(dc_removed, shift)

        out = np.zeros(PLOT_LEN, dtype=np.float32)
        n = min(PLOT_LEN, aligned.size)
        out[:n] = aligned[:n]
        return out

    def _cb(self, msg: AcousticPacket):
        t_abs = time.perf_counter()

        samples_u16 = np.asarray(msg.samples, dtype=np.uint16)
        wf = self._process_packet(samples_u16)

        pkt = {
            "t_abs_perf": float(t_abs),
            "rf_id": int(msg.rf_id),
            "time_offset_ms": int(msg.time_offset_ms),
            "waveform": wf,  # float32 (PLOT_LEN,)
        }
        if self.store_raw:
            pkt["samples_u16"] = samples_u16.copy()  # uint16 (4000,)

        with self._lock:
            self._buf.append(pkt)

            # Receiver detection bookkeeping
            self._rx_count += 1
            self._rx_last_perf = float(t_abs)
            self._rx_evt.set()

    def wait_for_first_packet(self, timeout_sec: float = 5.0) -> bool:
        with self._lock:
            if self._rx_count > 0:
                return True
        return self._rx_evt.wait(timeout=timeout_sec)

    def last_rx_age_sec(self) -> float:
        with self._lock:
            if self._rx_last_perf is None:
                return float("inf")
            return float(time.perf_counter() - self._rx_last_perf)

    def start_segment(self, t0_perf: float):
        with self._lock:
            self._seg_active = True
            self._seg_t0 = float(t0_perf)
            self._seg_start_len = len(self._buf)

    def stop_segment(self):
        with self._lock:
            if not self._seg_active:
                return {"t0_perf": None, "bbb0_ms": None, "packets": []}

            pkts = list(self._buf)[self._seg_start_len :]
            t0 = float(self._seg_t0)

            bbb0 = int(pkts[0]["time_offset_ms"]) if len(pkts) else None

            for p in pkts:
                p["t_rel_perf"] = float(p["t_abs_perf"] - t0)  # seconds since probe start
                if bbb0 is not None:
                    p["t_rel_bbb"] = float((p["time_offset_ms"] - bbb0) / 1000.0)
                else:
                    p["t_rel_bbb"] = None

            self._seg_active = False
            self._seg_t0 = None
            self._seg_start_len = 0

        return {"t0_perf": t0, "bbb0_ms": bbb0, "packets": pkts}


def start_spin_thread(node: Node, ctx: Context):
    stop_evt = threading.Event()

    def _spin():
        while ctx.ok() and not stop_evt.is_set():
            rclpy.spin_once(node, timeout_sec=0.01)

    th = threading.Thread(target=_spin, daemon=True)
    th.start()
    return stop_evt, th


# Helper functions
def probe(
    robot: Robot,
    start_xyz: np.ndarray,
    depth: float,
    probe_time: float,
    traj_freq: float = 5.0,
    fixed_ori: R | None = None,
    probe_func: str = "cos",
    t0_perf: float | None = None,
):
    """
    Complete plunge and retract motion.
    """
    cur = robot.end_effector_pose.copy()
    if fixed_ori is None:
        fixed_ori = cur.orientation
    target_pose = cur.copy()
    target_pose.position = start_xyz.astype(float)

    robot.set_target(pose=target_pose)
    time.sleep(SETTLE_SEC)

    z_init = float(start_xyz[2])
    N = max(1, int(probe_time * traj_freq))
    dt = 1.0 / traj_freq

    waypoints = []
    time_from_start = []

    for k in range(N + 1):
        s = k / N
        if probe_func == "cos":
            z = z_init + depth / 2 * (np.cos(2 * np.pi * s) - 1)
        elif probe_func == "linear":
            z = z_init + depth * (np.abs(2 * s - 1) - 1)
        t = k * dt

        target_position = np.array([start_xyz[0], start_xyz[1], z], dtype=float)
        target_pose = Pose(target_position, fixed_ori)
        twist = Twist(np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0]))

        waypoints.append((target_pose, twist))
        time_from_start.append(t)

    ee_forces = []
    ee_poses = []
    ts = []

    robot.execute_trajectory(waypoints, time_from_start)

    if t0_perf is None:
        t0_perf = time.perf_counter()
    t0 = t0_perf

    t_min = 0.0
    z_min = z_init
    while robot.wait_for_trajectory_completion(probe_time, timeout_margin=0.5):
        ee_force = robot.end_effector_external_wrench["force"]
        ee_pose = robot.end_effector_pose
        time_stamp = time.perf_counter() - t0

        if ee_pose.position[2] < z_min:
            z_min = ee_pose.position[2]
            t_min = time.perf_counter()

        ee_poses.append(ee_pose.copy())
        ee_forces.append(ee_force.copy())
        ts.append(time_stamp)

        time.sleep(0.01)

    return ts, ee_poses, ee_forces, t_min


# Probing parameters
Z_OFFSET = 0.0250
PROBE_DEPTH = 0.0200
PROBE_TIME = 2.0
BASE_ORI = R.from_euler("xyz", [-270, 0, 0], degrees=True)


def main():
    robot = Robot(namespace="fr3")
    robot.wait_until_ready()

    robot.controller_switcher_client.switch_controller("fr3_pose_controller")
    robot.fr3_pose_controller_parameters_client.load_param_config(
        file_path=CONFIG_DIR / "controllers" / "fr3_pose" / "default.yaml"
    )

    # ROS acoustic subscriber
    ctx = ensure_ros_context()
    ac_node = AcousticRecorderNode(topic="/acoustic/raw", context=ctx, store_raw=True)
    stop_evt, spin_th = start_spin_thread(ac_node, ctx)

    # Fail fast if nothing is arriving
    if not ac_node.wait_for_first_packet(timeout_sec=5.0):
        stop_evt.set()
        spin_th.join(timeout=1.0)
        try:
            ac_node.destroy_node()
        except Exception:
            pass
        try:
            if ctx.ok():
                rclpy.shutdown(context=ctx)
        except Exception:
            pass
        raise RuntimeError(
            "No acoustic data received on /acoustic/raw within 5s.\n"
            "Check:\n"
            "  1) TCP receiver/publisher node is running\n"
            "  2) BBB client is connected and streaming\n"
            "  3) Topic name matches (/acoustic/raw)\n"
            "  4) QoS matches (subscriber uses qos_profile_sensor_data)\n"
        )

    try:
        PROJECT_ROOT = Path(__file__).resolve().parent

        landmark_file = PROJECT_ROOT / "results" / "grids" / "landmarks.txt"
        if not landmark_file.exists():
            raise FileNotFoundError(
                f"Landmark file not found: {landmark_file}. Please run the landmark detection script first."
            )

        landmarks = fetch_landmarks(landmark_file, ["x", "y", "z"])

        z_surface = landmarks["z"] + Z_OFFSET
        home_position = np.array([landmarks["x"], landmarks["y"], z_surface])
        home_pose = Pose(home_position, BASE_ORI)

        grid_file = PROJECT_ROOT / "results" / "grids" / "grids.pkl"
        if not grid_file.exists():
            raise FileNotFoundError(f"Grid file not found: {grid_file}. Please run grid_generator.py first.")

        with open(grid_file, "rb") as f:
            grids = pickle.load(f)

        set_name = input("Select grid set (train/test) [default: test]: ").strip().lower()
        if set_name not in ["train", "test"]:
            set_name = "test"
            print(f"Invalid selection. Using default: {set_name}")

        grid_xy_world = grids["WORLD_FRAME"][set_name]
        grid_xy_gripper = grids["GRIPPER_FRAME"][set_name]

        exp_dict = {
            "ts": [],
            "grid_positions": [],
            "ee_poses": [],
            "ee_forces": [],
            "set_name": set_name,
            "landmarks": landmarks,
            "z_offset": Z_OFFSET,
            "acoustic": [],  # ADDED
        }

        robot.set_target(pose=home_pose)
        time.sleep(SETTLE_SEC)

        input("Press Enter to start probing...")
        for i, loc in enumerate(grid_xy_world):
            x, y = loc
            x_gripper, y_gripper = grid_xy_gripper[i]
            exp_dict["grid_positions"].append([x_gripper, y_gripper])
            print(f"\n Probe {i + 1}/{len(grid_xy_world)}")

            print("\tMoving to probe location...")
            approach_xy = np.array([x, y, z_surface], dtype=float)
            robot.set_target(position=approach_xy)
            time.sleep(SETTLE_SEC)

            print("\tStarting probe...")

            # Acoustic segment anchor
            t0_perf = time.perf_counter()
            ac_node.start_segment(t0_perf)

            ts, ee_poses, ee_forces, _ = probe(
                robot,
                start_xyz=approach_xy,
                depth=Z_OFFSET + PROBE_DEPTH,
                probe_time=PROBE_TIME,
                traj_freq=TRAJ_FREQ,
                fixed_ori=BASE_ORI,
                probe_func="linear",
                t0_perf=t0_perf,
            )

            acoustic_segment = ac_node.stop_segment()
            if len(acoustic_segment.get("packets", [])) == 0:
                raise RuntimeError(
                    f"Probe {i + 1}: no acoustic packets received during probe window.\n"
                    f"Last RX age = {ac_node.last_rx_age_sec():.3f}s\n"
                    "Check: TCP connection, BBB streaming loop, /acoustic/raw publisher health."
                )

            exp_dict["acoustic"].append(acoustic_segment)

            ee_positions = [pose.position for pose in ee_poses]
            ee_orientations = [pose.orientation.as_quat() for pose in ee_poses]

            exp_dict["ts"].append(ts)
            exp_dict["ee_poses"].append({"positions": ee_positions, "orientations": ee_orientations})
            exp_dict["ee_forces"].append(ee_forces)
            print("\tProbe complete.")

        print("\nReturning home...")
        robot.set_target(pose=home_pose)
        time.sleep(SETTLE_SEC)

        robot.shutdown()
        print("Done.")

        results_dir = PROJECT_ROOT / "results"
        results_dir.mkdir(parents=True, exist_ok=True)

        base_filename = f"{len(grid_xy_world)}_grid_{set_name.upper()}"
        counter = 0
        filename = f"{base_filename}_{counter:02d}.pkl"
        full_path = results_dir / filename
        while full_path.exists():
            counter += 1
            filename = f"{base_filename}_{counter:02d}.pkl"
            full_path = results_dir / filename

        with open(full_path, "wb") as f:
            pickle.dump(exp_dict, f)
        print(f"Results saved to: {full_path}")

    finally:
        # ROS cleanup
        try:
            stop_evt.set()
        except Exception:
            pass
        try:
            spin_th.join(timeout=1.0)
        except Exception:
            pass
        try:
            ac_node.destroy_node()
        except Exception:
            pass
        try:
            if ctx.ok():
                rclpy.shutdown(context=ctx)
        except Exception:
            pass


if __name__ == "__main__":
    main()
