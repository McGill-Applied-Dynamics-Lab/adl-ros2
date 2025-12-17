import time
import numpy as np
from scipy.spatial.transform import Rotation as R
from arm_client.robot import Robot, Pose, Twist
from arm_client import CONFIG_DIR
from pathlib import Path
import pickle
from waveguide_gripper_grid_generator import fetch_landmarks

# -------------------- ADDED: ROS/acoustic imports --------------------
import threading
from collections import deque
import rclpy
from rclpy.node import Node
from rclpy.context import Context
from scipy.signal import butter, filtfilt
from acoustic_sensing.msg import AcousticPacket
# --------------------------------------------------------------------

SETTLE_SEC = 1.00  # wait time after moves (s)
TRAJ_FREQ = 10.0  # Hz


# -------------------- ADDED: Acoustic processing params --------------------
SAMPLES_PER_CYCLE = 4000
FS = 2e5
CUTOFF = 100.0
LP_ORDER = 4
LP_B, LP_A = butter(LP_ORDER, CUTOFF / (0.5 * FS), btype="low")

PRE_SAMPLES = 50
PLOT_LEN = 4000
THRESHOLD_FAC = 100
# Logging rate during probe (keeps your original sleep(0.01) intent)
ROBOT_LOG_DT = 0.01
# --------------------------------------------------------------------------


# -------------------- ADDED: Safe ROS context init --------------------
def ensure_ros_context():
    ctx = Context()
    # Context.ok() is False until initialized; after init it becomes True.
    if not ctx.ok():
        rclpy.init(context=ctx)
    return ctx
# --------------------------------------------------------------------


# -------------------- ADDED: Acoustic recorder node --------------------
class AcousticRecorderNode(Node):
    def __init__(self, topic="/acoustic/raw", max_packets=20000, context=None):
        super().__init__("acoustic_recorder_node", context=context)
        self.sub_ = self.create_subscription(AcousticPacket, topic, self._cb, 10)

        self._lock = threading.Lock()
        self._buf = deque(maxlen=max_packets)

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

        # Pulse detect
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


def start_spin_thread(node: Node, ctx: Context):
    stop_evt = threading.Event()

    def _spin():
        while ctx.ok() and not stop_evt.is_set():
            rclpy.spin_once(node, timeout_sec=0.01)

    th = threading.Thread(target=_spin, daemon=True)
    th.start()
    return stop_evt, th
# --------------------------------------------------------------------


# Helper functions
def probe(
    robot: Robot,
    start_xyz: np.ndarray,
    depth: float,
    probe_time: float,
    traj_freq: float = 5.0,
    fixed_ori: R | None = None,
    probe_func: str = 'cos',
    t0_perf: float | None = None,  # ---------------- ADDED (optional) ----------------
):
    """
    Complete plunge and retract motion.

    Args:
        robot: Robot client (already ready & in Cartesian control).
        start_xyz: np.array([x, y, z_surface]) start point of plunge.
        depth: probe depth (positive indicates downwards).
        probe_time: seconds to complete the probe cycle (plunge + retract).
        traj_freq: Frequency of trajectory points per second.
    """
    # Define starting orientation
    cur = robot.end_effector_pose.copy()  # Pose(position, orientation)
    if fixed_ori is None:
        fixed_ori = cur.orientation  # maintain current orientation
    target_pose = cur.copy()
    target_pose.position = start_xyz.astype(float)

    # Move to input start location
    robot.set_target(pose=target_pose)
    time.sleep(SETTLE_SEC)

    # --- Compute probe trajectory ---
    z_init = float(start_xyz[2])
    N = max(1, int(probe_time * traj_freq))
    dt = 1.0 / traj_freq

    waypoints = []  # list of (Pose, Twist) tuples of the trajectory
    time_from_start = []  # matching time of the trajectory points

    # Include the endpoint (k = 0..N)
    for k in range(N + 1):
        s = k / N  # 0..1
        if probe_func == 'cos':
            z = z_init + depth / 2 * (np.cos(2 * np.pi * s) - 1)
        elif probe_func == 'linear':
            z = z_init + depth * (np.abs(2*s - 1) - 1)
        t = k * dt

        target_position = np.array([start_xyz[0], start_xyz[1], z], dtype=float)
        target_orientation = fixed_ori

        target_pose = Pose(target_position, target_orientation)
        twist = Twist(np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0]))

        waypoints.append((target_pose, twist))
        time_from_start.append(t)

    # --- Execute trajectory ---
    # Initialize save arrays
    ee_forces = []
    ee_poses = []
    ts = []

    # ---------------- ADDED: shared anchor for robot/acoustic alignment ----------------
    if t0_perf is None:
        t0_perf = time.perf_counter()
    # -------------------------------------------------------------------------------

    robot.execute_trajectory(waypoints, time_from_start)

    # ---------------- FIXED: robust time-based sampling (no reliance on return value) ----------------
    t_min = 0.0
    z_min = z_init
    t_end = t0_perf + probe_time

    while time.perf_counter() < t_end:
        ee_force = robot.end_effector_wrench["force"]
        ee_pose = robot.end_effector_pose
        now = time.perf_counter()
        time_stamp = now - t0_perf  # time since trajectory start (shared)

        # Time-stamp when z-displacement is max
        if ee_pose.position[2] < z_min:
            z_min = ee_pose.position[2]
            t_min = now  # absolute perf_counter time

        # Record data
        ee_poses.append(ee_pose.copy())
        ee_forces.append(ee_force.copy())
        ts.append(time_stamp)

        time.sleep(ROBOT_LOG_DT)

    # Optional: ensure trajectory completion once at the end
    try:
        robot.wait_for_trajectory_completion(probe_time, timeout_margin=0.5)
    except Exception:
        pass
    # -----------------------------------------------------------------------------------------------

    return ts, ee_poses, ee_forces, t_min


# Probing parameters
Z_OFFSET = 0.0250  # (m) offset from landmark z to surface
PROBE_DEPTH = 0.0200  # m (additional depth beyond z_offset)
PROBE_TIME = 2.0  # plunge and retract (s)
BASE_ORI = R.from_euler("xyz", [-270, 0, 0], degrees=True)


def main():
    # Setup
    robot = Robot(namespace="fr3")
    robot.wait_until_ready()

    robot.controller_switcher_client.switch_controller("fr3_pose_controller")
    robot.fr3_pose_controller_parameters_client.load_param_config(
        file_path=CONFIG_DIR / "controllers" / "fr3_pose" / "default.yaml"
    )

    # ---------------- ADDED: init ROS safely + start acoustic node ----------------
    ctx = ensure_ros_context()
    ac_node = AcousticRecorderNode(topic="/acoustic/raw", context=ctx)
    stop_evt, spin_th = start_spin_thread(ac_node, ctx)
    # ---------------------------------------------------------------------------

    # Load landmark file
    PROJECT_ROOT = Path(__file__).resolve().parent
    landmark_file = (
        PROJECT_ROOT / "results" / "grids" / "landmarks.txt"
    )

    # Check if landmark file exists
    if not landmark_file.exists():
        raise FileNotFoundError(
            f"Landmark file not found: {landmark_file}. Please run the landmark detection script first."
        )

    landmarks = fetch_landmarks(landmark_file, ["x", "y", "z"])

    # Parameters - use landmarks for home position
    z_surface = landmarks["z"] + Z_OFFSET  # (m) surface is offset from landmark
    home_position = np.array([landmarks["x"], landmarks["y"], z_surface])
    home_pose = Pose(home_position, BASE_ORI)

    # Load probe locations from grid file
    grid_file = PROJECT_ROOT / "results" / "grids" / "grids.pkl"

    # Check if grid file exists
    if not grid_file.exists():
        raise FileNotFoundError(
            f"Grid file not found: {grid_file}. Please run grid_generator.py first."
        )

    with open(grid_file, "rb") as f:
        grids = pickle.load(f)

    # Let user select train or test set
    set_name = input("Select grid set (train/test) [default: test]: ").strip().lower()
    if set_name not in ["train", "test"]:
        set_name = "test"
        print(f"Invalid selection. Using default: {set_name}")

    # Get world frame grid (N, 2) array for robot motion
    grid_xy_world = grids["WORLD_FRAME"][set_name]  # (N, 2) array in world/robot frame

    # Get gripper frame grid (N, 2) array for data saving
    grid_xy_gripper = grids["GRIPPER_FRAME"][set_name]  # (N, 2) array in gripper frame

    # Initialize results
    exp_dict = {
        "ts": [],
        "grid_positions": [],  # Store the (x, y) positions from the grid in GRIPPER_FRAME
        "ee_poses": [],
        "ee_forces": [],
        "set_name": set_name,  # Record which set was used
        "landmarks": landmarks,  # Store landmarks for reference
        "z_offset": Z_OFFSET,  # Store z_offset for reference
    }

    # ---------------- ADDED: store acoustic segments per probe ----------------
    exp_dict["acoustic"] = []
    # ------------------------------------------------------------------------

    try:
        # Iterate over probe locations
        input("Press Enter to start probing...")
        for i, loc in enumerate(grid_xy_world):
            x, y = loc
            # Save gripper frame coordinates
            x_gripper, y_gripper = grid_xy_gripper[i]
            exp_dict["grid_positions"].append([x_gripper, y_gripper])
            print(f"\n Probe {i + 1}/{len(grid_xy_world)}")

            # --- Move to probe location ---
            print("\tMoving to probe location...")
            approach_xy = np.array([x, y, z_surface], dtype=float)
            robot.set_target(position=approach_xy)
            time.sleep(SETTLE_SEC)

            # --- Probe cycle ---
            print("\tStarting probe...")

            # ---------------- ADDED: shared anchor & segment start ----------------
            t0_perf = time.perf_counter()
            ac_node.start_segment(t0_perf)
            # --------------------------------------------------------------------

            ts, ee_poses, ee_forces, _ = probe(
                robot,
                start_xyz=approach_xy,
                depth=Z_OFFSET + PROBE_DEPTH,  # Total depth from surface
                probe_time=PROBE_TIME,
                traj_freq=TRAJ_FREQ,
                fixed_ori=BASE_ORI,
                probe_func='linear',
                t0_perf=t0_perf,  # ---------------- ADDED ----------------
            )

            # ---------------- ADDED: segment stop + save ----------------
            acoustic_segment = ac_node.stop_segment()
            exp_dict["acoustic"].append(acoustic_segment)
            # -----------------------------------------------------------

            # Convert Pose objects to numpy arrays for saving
            ee_positions = [pose.position for pose in ee_poses]
            ee_orientations = [pose.orientation.as_quat() for pose in ee_poses]

            # --- Store results ---
            exp_dict["ts"].append(ts)
            # Store numpy arrays instead of Pose objects
            exp_dict["ee_poses"].append({
                "positions": ee_positions,
                "orientations": ee_orientations
            })
            exp_dict["ee_forces"].append(ee_forces)  # already numpy arrays
            print("\tProbe complete.")

        # Return home at the end
        print("\nReturning home...")
        robot.set_target(pose=home_pose)
        time.sleep(SETTLE_SEC)

        robot.shutdown()
        print("Done.")

    finally:
        # ---------------- ADDED: clean ROS shutdown always ----------------
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
        # ----------------------------------------------------------------

    # Save results
    results_dir = PROJECT_ROOT / "results"
    results_dir.mkdir(parents=True, exist_ok=True)

    # Generate filename based on set name and number of points
    # Add number suffix if file exists (starting from 00)
    base_filename = f"{len(grid_xy_world)}_grid_{set_name.upper()}"
    counter = 0
    filename = f"{base_filename}_{counter:02d}.pkl"
    full_path = results_dir / filename

    # Find the next available number
    while full_path.exists():
        counter += 1
        filename = f"{base_filename}_{counter:02d}.pkl"
        full_path = results_dir / filename

    # Save the data
    with open(full_path, "wb") as f:
        pickle.dump(exp_dict, f)
        print(f"Results saved to: {full_path}")


if __name__ == "__main__":
    main()
