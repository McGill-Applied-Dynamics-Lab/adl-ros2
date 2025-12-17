import time
import numpy as np
import threading
from collections import deque
from pathlib import Path
import pickle

import rclpy
from rclpy.node import Node
from scipy.signal import butter, filtfilt
from msg import AcousticPacket

from scipy.spatial.transform import Rotation as R
from arm_client.robot import Robot, Pose, Twist
from arm_client import CONFIG_DIR
from waveguide_gripper_grid_generator import fetch_landmarks


# ----------------------- Robot probing params -----------------------
SETTLE_SEC = 2.00
TRAJ_FREQ = 10.0  # waypoint rate (Hz) for trajectory generation

Z_OFFSET = 0.0250
PROBE_DEPTH = 0.0200
PROBE_TIME = 2.0
BASE_ORI = R.from_euler("xyz", [-270, 0, 0], degrees=True)

# Robot logging rate during probe (independent from TRAJ_FREQ)
ROBOT_LOG_HZ = 100.0
ROBOT_LOG_DT = 1.0 / ROBOT_LOG_HZ

# ----------------------- Acoustic processing params -----------------------
SAMPLES_PER_CYCLE = 4000
FS = 2e5
CUTOFF = 100.0
LP_ORDER = 4
LP_B, LP_A = butter(LP_ORDER, CUTOFF / (0.5 * FS), btype="low")

PRE_SAMPLES = 50
PLOT_LEN = 4000
THRESHOLD_FAC = 100


class AcousticRecorderNode(Node):
    """
    Subscribes to /acoustic/raw, processes each packet (DC remove + pulse align),
    and buffers packets with perf_counter timestamps so they can align with robot samples.
    """
    def __init__(self, topic="/acoustic/raw", max_packets=20000):
        super().__init__("acoustic_recorder_node")
        self.sub_ = self.create_subscription(AcousticPacket, topic, self._cb, 10)
        self.get_logger().info(f"AcousticRecorderNode subscribed to {topic}")

        self._lock = threading.Lock()
        self._buf = deque(maxlen=max_packets)

        # segmentation state
        self._seg_active = False
        self._seg_t0 = None
        self._seg_start_len = 0

    def _process_packet(self, msg: AcousticPacket) -> np.ndarray:
        raw = np.array(msg.samples, dtype=np.int16).astype(float)
        if raw.size == 0:
            return np.zeros(PLOT_LEN, dtype=np.float32)

        # DC removal
        try:
            dc_est = filtfilt(LP_B, LP_A, raw)
            dc_removed = raw - dc_est
        except Exception:
            dc_removed = raw - np.mean(raw)

        # Pulse detection
        rect = np.abs(dc_removed)
        b_len = max(int(0.1 * rect.size), 1)
        baseline = rect[:b_len]
        mean0 = baseline.mean()
        std0 = baseline.std()
        thresh = mean0 + THRESHOLD_FAC * std0

        over = np.nonzero(rect > thresh)[0]
        pulse_idx = int(over[0]) if over.size > 0 else 0

        # Align pulse to PRE_SAMPLES
        shift = PRE_SAMPLES - pulse_idx
        aligned = np.roll(dc_removed, shift)

        # Normalize length
        if aligned.size >= PLOT_LEN:
            to_plot = aligned[:PLOT_LEN]
        else:
            to_plot = np.zeros(PLOT_LEN, dtype=float)
            to_plot[:aligned.size] = aligned

        return np.abs(to_plot).astype(np.float32)

    def _cb(self, msg: AcousticPacket):
        t_abs = time.perf_counter()
        wf = self._process_packet(msg)

        pkt = {
            "t_abs_perf": float(t_abs),
            "rf_id": int(msg.rf_id),
            "time_offset_ms": int(msg.time_offset_ms),
            "waveform": wf,  # float32, length PLOT_LEN
        }

        with self._lock:
            self._buf.append(pkt)

    def start_segment(self, t0_perf: float):
        with self._lock:
            self._seg_active = True
            self._seg_t0 = float(t0_perf)
            self._seg_start_len = len(self._buf)

    def stop_segment(self):
        with self._lock:
            if not self._seg_active:
                return {"t0_perf": None, "packets": []}

            pkts = list(self._buf)[self._seg_start_len:]
            t0 = float(self._seg_t0)

            for p in pkts:
                p["t_rel"] = p["t_abs_perf"] - t0

            self._seg_active = False
            self._seg_t0 = None
            self._seg_start_len = 0

        return {"t0_perf": t0, "packets": pkts}


def start_spin_thread(node: Node):
    stop_evt = threading.Event()

    def _spin():
        while rclpy.ok() and not stop_evt.is_set():
            rclpy.spin_once(node, timeout_sec=0.01)

    th = threading.Thread(target=_spin, daemon=True)
    th.start()
    return stop_evt, th


def probe(
    robot: Robot,
    start_xyz: np.ndarray,
    depth: float,
    probe_time: float,
    t0_perf: float,
    traj_freq: float = 10.0,
    fixed_ori: R | None = None,
    probe_func: str = "linear",
):
    """
    Execute plunge/retract trajectory and log robot data with timestamps aligned to t0_perf.
    Returns dict with:
      - t0_perf
      - robot samples: t_rel, t_abs_perf, positions, orientations, forces
      - t_min_rel: time (rel) of minimum z
    """
    # Move to input start location (surface point)
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

    # Build waypoints
    waypoints = []
    time_from_start = []
    for k in range(N + 1):
        s = k / N
        if probe_func == "cos":
            z = z_init + depth / 2 * (np.cos(2 * np.pi * s) - 1)
        elif probe_func == "linear":
            z = z_init + depth * (np.abs(2 * s - 1) - 1)
        else:
            raise ValueError(f"Unknown probe_func: {probe_func}")

        t = k * dt
        target_position = np.array([start_xyz[0], start_xyz[1], z], dtype=float)
        target_orientation = fixed_ori
        target_pose = Pose(target_position, target_orientation)
        twist = Twist(np.zeros(3), np.zeros(3))

        waypoints.append((target_pose, twist))
        time_from_start.append(t)

    # Execute
    robot.execute_trajectory(waypoints, time_from_start)

    # Log robot samples for the duration (time-based, robust)
    t_end = t0_perf + probe_time

    t_rel = []
    t_abs = []
    positions = []
    orientations = []
    forces = []

    z_min = z_init
    t_min_rel = 0.0

    while True:
        now = time.perf_counter()
        if now > t_end:
            break

        ee_force = robot.end_effector_wrench["force"].copy()
        ee_pose = robot.end_effector_pose.copy()

        trel = now - t0_perf

        # record
        t_abs.append(float(now))
        t_rel.append(float(trel))
        positions.append(ee_pose.position.copy())
        orientations.append(ee_pose.orientation.as_quat().copy())
        forces.append(ee_force)

        if ee_pose.position[2] < z_min:
            z_min = float(ee_pose.position[2])
            t_min_rel = float(trel)

        time.sleep(ROBOT_LOG_DT)

    # Make sure trajectory is actually done (optional but nice)
    try:
        robot.wait_for_trajectory_completion(probe_time, timeout_margin=0.5)
    except Exception:
        pass

    return {
        "t0_perf": float(t0_perf),
        "t_rel": t_rel,
        "t_abs_perf": t_abs,
        "positions": positions,
        "orientations": orientations,
        "forces": forces,
        "t_min_rel": float(t_min_rel),
        "meta": {
            "traj_freq": float(traj_freq),
            "probe_time": float(probe_time),
            "probe_func": probe_func,
            "depth": float(depth),
            "z_init": float(z_init),
        },
    }


def main():
    PROJECT_ROOT = Path(__file__).resolve().parent

    # Load landmarks
    landmark_file = PROJECT_ROOT / "results" / "grids" / "landmarks.txt"
    if not landmark_file.exists():
        raise FileNotFoundError(f"Landmark file not found: {landmark_file}")

    landmarks = fetch_landmarks(landmark_file, ["x", "y", "z"])
    z_surface = landmarks["z"] + Z_OFFSET
    home_position = np.array([landmarks["x"], landmarks["y"], z_surface])
    home_pose = Pose(home_position, BASE_ORI)

    # Load grids
    grid_file = PROJECT_ROOT / "results" / "grids" / "grids.pkl"
    if not grid_file.exists():
        raise FileNotFoundError(f"Grid file not found: {grid_file}")

    with open(grid_file, "rb") as f:
        grids = pickle.load(f)

    set_name = input("Select grid set (train/test) [default: test]: ").strip().lower()
    if set_name not in ["train", "test"]:
        set_name = "test"
        print(f"Invalid selection. Using default: {set_name}")

    grid_xy_world = grids["WORLD_FRAME"][set_name]
    grid_xy_gripper = grids["GRIPPER_FRAME"][set_name]

    # Init robot + ROS
    robot = Robot(namespace="fr3")
    rclpy.init()
    ac_node = AcousticRecorderNode(topic="/acoustic/raw")
    stop_evt, spin_th = start_spin_thread(ac_node)

    exp_dict = {
        "set_name": set_name,
        "landmarks": landmarks,
        "z_offset": float(Z_OFFSET),
        "probe_depth": float(PROBE_DEPTH),
        "probe_time": float(PROBE_TIME),
        "traj_freq": float(TRAJ_FREQ),
        "robot_log_hz": float(ROBOT_LOG_HZ),
        "acoustic_params": {
            "fs": float(FS),
            "cutoff": float(CUTOFF),
            "lp_order": int(LP_ORDER),
            "pre_samples": int(PRE_SAMPLES),
            "plot_len": int(PLOT_LEN),
            "threshold_fac": float(THRESHOLD_FAC),
        },
        "probes": [],
    }

    try:
        # Robot setup
        robot.wait_until_ready()
        robot.controller_switcher_client.switch_controller("fr3_pose_controller")
        robot.fr3_pose_controller_parameters_client.load_param_config(
            file_path=CONFIG_DIR / "controllers" / "fr3_pose" / "default.yaml"
        )

        input("Press Enter to start probing...")

        for i, loc in enumerate(grid_xy_world):
            x, y = loc
            xg, yg = grid_xy_gripper[i]

            print(f"\nProbe {i + 1}/{len(grid_xy_world)}")
            print("\tMoving to probe location...")
            approach_xyz = np.array([x, y, z_surface], dtype=float)
            robot.set_target(position=approach_xyz)
            time.sleep(SETTLE_SEC)

            print("\tStarting probe...")
            # Shared anchor: both streams use this
            t0_perf = time.perf_counter()
            ac_node.start_segment(t0_perf)

            robot_data = probe(
                robot,
                start_xyz=approach_xyz,
                depth=Z_OFFSET + PROBE_DEPTH,
                probe_time=PROBE_TIME,
                t0_perf=t0_perf,
                traj_freq=TRAJ_FREQ,
                fixed_ori=BASE_ORI,
                probe_func="linear",
            )

            acoustic_data = ac_node.stop_segment()

            exp_dict["probes"].append({
                "grid_xy_gripper": [float(xg), float(yg)],
                "grid_xy_world": [float(x), float(y)],
                "t0_perf": float(t0_perf),
                "robot": robot_data,
                "acoustic": acoustic_data,
            })

            print(f"\tProbe complete. Robot samples: {len(robot_data['t_rel'])}, "
                  f"Acoustic packets: {len(acoustic_data['packets'])}")

        # Return home
        print("\nReturning home...")
        robot.set_target(pose=home_pose)
        time.sleep(SETTLE_SEC)

    finally:
        # Always clean up
        try:
            robot.shutdown()
        except Exception:
            pass

        stop_evt.set()
        try:
            ac_node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass

    # Save results
    results_dir = PROJECT_ROOT / "results"
    results_dir.mkdir(parents=True, exist_ok=True)

    base_filename = f"{len(grid_xy_world)}_grid_{set_name.upper()}"
    counter = 0
    full_path = results_dir / f"{base_filename}_{counter:02d}.pkl"
    while full_path.exists():
        counter += 1
        full_path = results_dir / f"{base_filename}_{counter:02d}.pkl"

    with open(full_path, "wb") as f:
        pickle.dump(exp_dict, f)
    print(f"Results saved to: {full_path}")


if __name__ == "__main__":
    main()
