from dataclasses import dataclass
import time
import threading
import queue
import numpy as np
import serial
from scipy.spatial.transform import Rotation as R
from arm_client.robot import Robot, Pose
from arm_client.planning.types import CartesianWaypoint, PlannedJointTrajectory
from arm_client.planning.waypoints import generate_linear_waypoints
from pathlib import Path
import pickle
import matplotlib

matplotlib.use("Agg")  # non-interactive backend — saves to file without requiring a display
import matplotlib.pyplot as plt

SETTLE_SEC = 1.0  # wait time after blocking moves (s)
MOVE_SPEED = 0.05  # m/s for traversal moves
QUEUE_SIZE = 3  # number of probe plans to pre-compute ahead of execution

# Trigger settings
Z_INIT = 0.15  # m
BUTTON_X = 0.681  # m
BUTTON_Y = -0.147  # m
TRIG_DEPTH = 0.0205  # m
TRIG_TIME = 2.0  # s

# Orientation/starting position
BASE_ORI = R.from_euler("xyz", [-180, 0, 0], degrees=True)
PROBE_START_Z = 0.1150  # start height for probing (m)

# Teensy configuration (quad_tx 4-channel firmware)
TEENSY = True  # set False to skip Teensy and run robot-only
DEBUG_PLOTS = True  # save per-probe diagnostic plots to <output_dir>/debug_plots/
SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 3000000
TEENSY_SAMPLE_RATE_HZ_FALLBACK = 40000.0
BASELINE_DURATION_SEC = 2.0
PLUNGE_SIMILARITY_WEIGHT = 0.1  # higher than default (0.001) to suppress IK branch-switching during plunge
PLUNGE_ORI_WEIGHT = 200.0  # higher than default (50.0) to keep orientation fixed throughout plunge
PLUNGE_POINTS_PER_SEC = 100  # IK solve density — keeps joint-space gaps constant regardless of plunge speed

# TODO: Estimate bed curvature1.


# ===================== Teensy serial helpers =====================
def read_exact_bytes(ser, num_bytes):
    """Read exactly num_bytes from serial port."""
    data = bytearray()
    while len(data) < num_bytes:
        chunk = ser.read(num_bytes - len(data))
        if chunk:
            data.extend(chunk)
    return bytes(data)


def collect_teensy_data_streaming(ser, max_duration=15.0):
    """
    Collect 4-channel Teensy data via streaming chunks (quad_tx wire format).

    Each 'C' chunk contains:
        uint32 chunk_size
        chunk_size * uint16 per channel (ch1..ch4)
        float32 chunk_rate_hz
        uint32 dt_since_last_us

    Returns:
        data (np.ndarray, shape (N, 4)): raw ADC uint16 samples per channel.
        timestamps (np.ndarray, shape (N,)): per-sample timestamps in seconds,
            anchored so the first sample is t=0.
    """
    all_ch = [[], [], [], []]
    all_timestamps = []
    cum_chunk_start_s = 0.0
    last_known_rate = None
    start_time = time.perf_counter()

    try:
        ser.write(b"1")
        time.sleep(0.1)

        while time.perf_counter() - start_time < max_duration:
            if ser.in_waiting > 0:
                marker = ser.read(1)

                if marker == b"B":
                    continue

                elif marker == b"C":
                    chunk_size = int.from_bytes(read_exact_bytes(ser, 4), byteorder="little")
                    ch_bytes = [read_exact_bytes(ser, chunk_size * 2) for _ in range(4)]
                    rate_bytes = read_exact_bytes(ser, 4)
                    dt_us = int.from_bytes(read_exact_bytes(ser, 4), byteorder="little")

                    samples = [np.frombuffer(cb, dtype=np.uint16) for cb in ch_bytes]
                    chunk_rate_hz = float(np.frombuffer(rate_bytes, dtype=np.float32)[0])

                    cum_chunk_start_s += dt_us * 1e-6

                    if chunk_rate_hz > 0:
                        last_known_rate = chunk_rate_hz
                    effective_rate = last_known_rate if last_known_rate is not None else TEENSY_SAMPLE_RATE_HZ_FALLBACK
                    period = 1.0 / effective_rate
                    chunk_ts = cum_chunk_start_s + np.arange(chunk_size) * period

                    for i in range(4):
                        all_ch[i].extend(samples[i])
                    all_timestamps.append(chunk_ts)

                elif marker == b"E":
                    total_count = int.from_bytes(read_exact_bytes(ser, 4), byteorder="little")
                    print(f"  Received {total_count} samples from Teensy")
                    break
            else:
                time.sleep(0.001)

        if len(all_ch[0]) > 0:
            data = np.column_stack([np.array(ch, dtype=np.uint16) for ch in all_ch])
            timestamps = np.concatenate(all_timestamps)
            timestamps = timestamps - timestamps[0]
            return data, timestamps
        else:
            return np.zeros((0, 4), dtype=np.uint16), np.array([])

    except Exception as e:
        print(f"Error in Teensy streaming: {e}")
        return np.zeros((0, 4), dtype=np.uint16), np.array([])


# ===================== Baseline acquisition =====================
def acquire_baseline(ser, duration_sec=BASELINE_DURATION_SEC):
    """
    Stream Teensy data for duration_sec with no robot motion to get a per-sensor
    mean baseline for later subtraction.

    Returns:
        baseline (np.ndarray): (N, 4) raw ADC samples.
        baseline_mean (np.ndarray): (4,) per-channel mean.
    """
    print(f"Acquiring {duration_sec:.1f}s baseline...")
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    holder = []

    def _worker():
        try:
            holder.append(collect_teensy_data_streaming(ser, max_duration=duration_sec + 5.0))
        except Exception as e:
            print(f"  Error in baseline thread: {e}")
            holder.append((np.zeros((0, 4), dtype=np.uint16), np.array([])))

    th = threading.Thread(target=_worker)
    th.start()

    time.sleep(0.1 + duration_sec)
    ser.write(b"2")
    time.sleep(0.1)
    th.join(timeout=10.0)

    baseline = holder[0][0] if holder else np.zeros((0, 4), dtype=np.uint16)

    if baseline.size == 0:
        print("  Warning: no baseline data received; using zero baseline.")
        return baseline, np.zeros(4, dtype=np.float64)

    baseline_mean = baseline.astype(np.float64).mean(axis=0)
    print(f"  Baseline: {baseline.shape[0]} samples, means = [{', '.join(f'{m:.1f}' for m in baseline_mean)}]")
    return baseline, baseline_mean


# ===================== Planning helpers =====================
@dataclass
class ProbePlan:
    probe_idx: int

    traversal_traj: PlannedJointTrajectory  # current_pos → [x,y,Z_INIT] → [x,y,PROBE_START_Z]


def plan_traversal_traj(robot: Robot, start_pos, target_xy, ori, speed, initial_joint_config):
    """
    Plan a single traversal from start_pos through [target_xy, Z_INIT] down to
    [target_xy, PROBE_START_Z] — combining what was previously three separate IK calls.
    """
    mid_pos = np.array([target_xy[0], target_xy[1], Z_INIT], dtype=float)
    end_pos = np.array([target_xy[0], target_xy[1], PROBE_START_Z], dtype=float)

    dist1 = np.linalg.norm(mid_pos - start_pos)
    dist2 = np.linalg.norm(end_pos - mid_pos)
    total_dist = dist1 + dist2
    duration = max(total_dist / speed, 0.5)
    s_mid = dist1 / total_dist if total_dist > 0 else 0.5

    waypoints = [
        CartesianWaypoint(position=start_pos.astype(float), orientation=ori, s=0.0),
        CartesianWaypoint(position=mid_pos, orientation=ori, s=float(s_mid)),
        CartesianWaypoint(position=end_pos, orientation=ori, s=1.0),
    ]
    n_pts = 10
    return robot.plan_joint_trajectory(
        waypoints=waypoints,
        duration=duration,
        n_points=n_pts,
        visualize=False,
        show_progress=False,
        initial_joint_config=initial_joint_config,
    )


def plan_linear_traj(robot: Robot, start_pos, end_pos, ori, speed, initial_joint_config):
    """Plan a linear IK trajectory between two Cartesian positions."""
    dist = np.linalg.norm(end_pos - start_pos)
    duration = max(dist / speed, 0.5)
    n_pts = max(10, int(duration * 10))
    n_pts = 2
    waypoints = generate_linear_waypoints(start_pos, ori, end_pos, ori, num_waypoints=n_pts)
    return robot.plan_joint_trajectory(
        waypoints=waypoints,
        duration=duration,
        n_points=n_pts,
        visualize=False,
        show_progress=False,
        initial_joint_config=initial_joint_config,
    )


def plan_plunge_traj(
    robot: Robot,
    start_xyz,
    depth,
    probe_time,
    ori,
    initial_joint_config,
    plunge_func="cos",
    similarity_weight=PLUNGE_SIMILARITY_WEIGHT,
    ori_weight=PLUNGE_ORI_WEIGHT,
):
    """Plan a plunge-and-retract IK trajectory."""
    z_init = float(start_xyz[2])
    n_waypoints = 50
    n_points = max(50, int(probe_time * PLUNGE_POINTS_PER_SEC))
    waypoints = []
    for k in range(n_waypoints + 1):
        s = k / n_waypoints
        if plunge_func == "cos":
            z = z_init + depth / 2 * (np.cos(2 * np.pi * s) - 1)
        else:
            z = z_init + depth * (np.abs(2 * s - 1) - 1)

        waypoints.append(
            CartesianWaypoint(
                position=np.array([start_xyz[0], start_xyz[1], z], dtype=float),
                orientation=ori,
                s=s,
            )
        )
    return robot.plan_joint_trajectory(
        waypoints=waypoints,
        duration=probe_time,
        n_points=n_points,
        visualize=False,
        show_progress=False,
        initial_joint_config=initial_joint_config,
        similarity_weight=similarity_weight,
        ori_weight=ori_weight,
        pin_start=True,
    )


# ===================== Background planner =====================
def planner_worker(
    robot,
    grid_xy_world,
    depths,
    plunge_times,
    start_idx,
    start_joint_cfg,
    plan_queue: queue.Queue,
    abort_event: threading.Event,
):
    """
    Pre-plan all trajectories for each probe in a background thread and push
    ProbePlan objects (traversal, plunge, retract) into plan_queue.

    Trajectories are chained via joint_positions[-1] so the IK solver uses a
    consistent warm start across the entire sequence.
    """
    last_cfg = start_joint_cfg.copy()
    current_pos = np.array([BUTTON_X, BUTTON_Y, Z_INIT], dtype=float)  # robot is at home when planner starts
    n = len(depths)

    for i in range(start_idx, n):
        # print(f"\t[planner] ({i + 1}/{n}) Started planning")
        plan_start_time = time.perf_counter()
        if abort_event.is_set():
            break

        x, y = grid_xy_world[i]
        depth = depths[i]
        plunge_time = plunge_times[i]

        try:
            # Traverse: current_pos → [x, y, Z_INIT] → [x, y, PROBE_START_Z]
            traversal_traj = plan_traversal_traj(robot, current_pos, np.array([x, y]), BASE_ORI, MOVE_SPEED, last_cfg)
            last_cfg = traversal_traj.joint_positions[-1]
            current_pos = np.array([x, y, PROBE_START_Z], dtype=float)

            probe_plan = ProbePlan(
                probe_idx=i,
                traversal_traj=traversal_traj,
            )

            while not abort_event.is_set():
                try:
                    plan_queue.put(probe_plan, timeout=2.0)
                    break
                except queue.Full:
                    continue
            print(f"\t[planner]({i + 1}/{n}) planning completed in {time.perf_counter() - plan_start_time:.2f} s")

        except Exception as e:
            plan_queue.put(e)
            return

    plan_queue.put(None)  # sentinel: all probes planned


# ===================== Probe execution (no planning) =====================
def execute_probe(robot: Robot, plunge_traj, start_xyz, probe_time, ser):
    """
    Execute a pre-planned plunge trajectory while recording ee state and Teensy data.
    The robot must already be positioned at start_xyz before calling this function.

    Returns:
        ts: List of perf_counter timestamps (0-referenced to trajectory start).
        ee_poses: List of Pose objects recorded during execution.
        ee_forces: List of force numpy arrays recorded during execution.
        t_min: Timestamp when z-displacement was maximum.
        teensy_data: (N, 4) uint16 array of raw ADC samples (empty if ser=None).
        teensy_timestamps: (N,) float array of sample timestamps in seconds.
    """
    teensy_holder = []
    teensy_thread = None
    teensy_max_duration = probe_time + 5.0

    if ser is not None:
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        def _collect():
            teensy_holder.append(collect_teensy_data_streaming(ser, max_duration=teensy_max_duration))

        teensy_thread = threading.Thread(target=_collect, daemon=True)
        teensy_thread.start()
        time.sleep(0.05)  # let stream start before trajectory begins

    ee_forces = []
    ee_poses = []
    ts = []
    t_min = 0.0
    z_min = float(start_xyz[2])

    t0 = time.perf_counter()  # anchor: t=0 is trajectory start
    robot.follow_joint_trajectory(plunge_traj, blocking=False)

    while robot.wait_for_trajectory_completion(probe_time, timeout_margin=0.5):
        ee_force = robot.end_effector_wrench["force"]
        ee_pose = robot.end_effector_pose
        time_stamp = time.perf_counter() - t0

        if ee_pose.position[2] < z_min:
            z_min = ee_pose.position[2]
            t_min = time_stamp

        ee_poses.append(ee_pose.copy())
        ee_forces.append(ee_force.copy())
        ts.append(time_stamp)
        time.sleep(0.01)

    if ser is not None and teensy_thread is not None:
        ser.write(b"2")
        time.sleep(0.1)
        teensy_thread.join(timeout=teensy_max_duration)

    teensy_data, teensy_timestamps = (
        teensy_holder[0] if teensy_holder else (np.zeros((0, 4), dtype=np.uint16), np.array([]))
    )

    return ts, ee_poses, ee_forces, t_min, teensy_data, teensy_timestamps


# ===================== Debug plots =====================
def plot_probe_debug(
    probe_idx: int,
    ts,
    ee_poses,
    ee_forces,
    teensy_data,
    teensy_timestamps,
    baseline_mean,
    output_dir: Path,
    target_xyz,
    depth: float,
    probe_time: float,
    plunge_func: str = "cos",
):
    """Save a 3-panel diagnostic figure for one probe to output_dir/debug_plots/."""
    plot_dir = output_dir / "debug_plots"
    plot_dir.mkdir(exist_ok=True)

    ts_arr = np.array(ts)
    positions = np.array([p.position for p in ee_poses])  # (N, 3)

    # Reference trajectory over the recording time window
    t_ref = np.linspace(0.0, probe_time, 500)
    s_ref = t_ref / probe_time
    z_init = float(target_xyz[2])
    if plunge_func == "cos":
        z_ref = z_init + depth / 2 * (np.cos(2 * np.pi * s_ref) - 1)
    else:
        z_ref = z_init + depth * (np.abs(2 * s_ref - 1) - 1)

    fig, axes = plt.subplots(3, 1, figsize=(10, 10))
    fig.suptitle(f"Probe {probe_idx + 1} diagnostics")

    # --- Panel 1: pressure (Teensy channels, baseline-subtracted) ---
    ax = axes[0]
    if teensy_data.size > 0:
        corrected = teensy_data.astype(np.float64) - baseline_mean[np.newaxis, :]
        for ch in range(teensy_data.shape[1]):
            ax.plot(teensy_timestamps, corrected[:, ch], label=f"ch{ch + 1}", linewidth=0.8)
        ax.set_ylabel("ADC counts (baseline-sub.)")
        ax.legend(loc="upper right", fontsize=8)
    else:
        ax.text(0.5, 0.5, "No Teensy data", ha="center", va="center", transform=ax.transAxes)
        ax.set_ylabel("ADC counts")
    ax.set_xlabel("Time (s)")
    ax.set_title("Pressure (4-channel)")
    ax.grid(True, linewidth=0.4)

    # --- Panel 2: EE x-y deviation from target (drift during plunge) ---
    ax = axes[1]
    if len(positions) > 0:
        ax.axhline(0.0, color="k", linewidth=0.8, linestyle="--", label="target")
        ax.plot(ts_arr, (positions[:, 0] - target_xyz[0]) * 1e3, label="Δx (mm)")
        ax.plot(ts_arr, (positions[:, 1] - target_xyz[1]) * 1e3, label="Δy (mm)")
        ax.set_ylabel("Deviation from target (mm)")
        ax.legend(loc="upper right", fontsize=8)
    else:
        ax.text(0.5, 0.5, "No EE data", ha="center", va="center", transform=ax.transAxes)
    ax.set_xlabel("Time (s)")
    ax.set_title("EE x-y deviation from target (should stay near 0)")
    ax.grid(True, linewidth=0.4)

    # --- Panel 3: EE z position vs reference ---
    ax = axes[2]
    if len(positions) > 0:
        ax.plot(t_ref, z_ref * 1e3, color="k", linewidth=1.2, linestyle="--", label="reference")
        ax.plot(ts_arr, positions[:, 2] * 1e3, label="actual")
        ax.set_ylabel("z (mm)")
        ax.legend(loc="upper right", fontsize=8)
    else:
        ax.text(0.5, 0.5, "No EE data", ha="center", va="center", transform=ax.transAxes)
    ax.set_xlabel("Time (s)")
    ax.set_title("EE z position vs reference")
    ax.grid(True, linewidth=0.4)

    fig.tight_layout()
    out_path = plot_dir / f"probe_{probe_idx + 1:04d}.png"
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"\tDebug plot saved: {out_path.name}")


# ===================== Main =====================
def main():
    PROJECT_ROOT = Path(__file__).resolve().parent

    # ---- Connect to Teensy ----
    ser = None
    if TEENSY:
        try:
            ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2)
            time.sleep(2)
            ser.write(b"2")  # reset if mid-stream from a previous crashed run
            time.sleep(0.5)
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            print(f"Connected to Teensy on {SERIAL_PORT}")
        except Exception as e:
            print(f"Warning: could not connect to Teensy ({e}). Continuing without sensor data.")
            ser = None

    # ---- Setup robot ----
    robot = Robot(namespace="fr3")
    robot.wait_until_ready()
    robot.controller_switcher_client.switch_controller("joint_trajectory_controller")

    home_position = np.array([BUTTON_X, BUTTON_Y, Z_INIT])
    home_pose = Pose(home_position, BASE_ORI)

    # ---- Load grid ----
    # grid_file = PROJECT_ROOT / "results" / "grids" / "GLYCERIN_GRID.pkl"
    grid_file = PROJECT_ROOT / "results" / "grids" / "TEST_GRID.pkl"

    if not grid_file.exists():
        raise FileNotFoundError(f"Grid file not found: {grid_file}. Please run a grid generator first.")

    with open(grid_file, "rb") as f:
        grids = pickle.load(f)

    set_name = input("Select grid set (train/test) [default: test]: ").strip().lower()
    if set_name not in ["train", "test"]:
        set_name = "test"
        print(f"Invalid selection. Using default: {set_name}")

    grid_xy_world = grids["WORLD_FRAME"][set_name]
    grid_xy_sensor = grids["SENSOR_FRAME"][set_name]
    depths = grids["DEPTHS"][set_name]
    plunge_times = grids["PLUNGE_TIMES"][set_name]
    N_POINTS = len(depths)

    if not (len(plunge_times) == N_POINTS and len(grid_xy_world) == N_POINTS and len(grid_xy_sensor) == N_POINTS):
        raise ValueError("Inconsistent grid data lengths.")

    # ---- Initialize / resume results ----
    exp_dict = {
        "ts": [],
        "grid_positions": [],
        "ee_poses": [],
        "ee_forces": [],
        "plunge_depths": [],
        "plunge_times": [],
        "set_name": set_name,
        # Teensy 4-channel acoustic data (columns: ch1, ch2, ch3, ch4)
        "teensy_data": [],
        "teensy_timestamps": [],
        # Per-probe baseline: raw samples and per-channel means
        "baseline_data": [],
        "baseline_mean": [],
    }

    results_dir = PROJECT_ROOT / "results"
    results_dir.mkdir(parents=True, exist_ok=True)
    filename = f"R_GLYCERIN_GRID_{set_name}.pkl"
    full_path = results_dir / filename

    start_idx = 0
    if not full_path.exists():
        with open(full_path, "wb") as f:
            pickle.dump(exp_dict, f)
            print(f"Created new results file: {full_path}")
    else:
        with open(full_path, "rb") as f:
            exp_dict = pickle.load(f)
            print(f"Loaded existing results file: {full_path}")
            start_idx = len(exp_dict["grid_positions"])
            # Back-fill keys if resuming from an older results file
            exp_dict.setdefault("teensy_data", [])
            exp_dict.setdefault("teensy_timestamps", [])
            exp_dict.setdefault("baseline_data", [])
            exp_dict.setdefault("baseline_mean", [])

    print(f"Resuming from probe number {start_idx + 1}")

    # Move home before starting
    print("Going to start position")
    robot.move_to(pose=home_pose, speed=MOVE_SPEED)
    time.sleep(SETTLE_SEC)

    input("Press Enter to start probing...")

    # ---- Start background planner ----
    abort_event = threading.Event()
    plan_queue = queue.Queue(maxsize=QUEUE_SIZE)
    planner_thread = threading.Thread(
        target=planner_worker,
        args=(robot, grid_xy_world, depths, plunge_times, start_idx, robot.q.copy(), plan_queue, abort_event),
        daemon=True,
    )
    planner_thread.start()

    # ---- Main probe loop ----
    try:
        for i in range(start_idx, N_POINTS):
            x, y = grid_xy_world[i]
            x_sensor, y_sensor = grid_xy_sensor[i]
            approach_xyz = np.array([x, y, PROBE_START_Z], dtype=float)

            print(f"\n Probe {i + 1} / {N_POINTS}")

            # Fetch pre-planned trajectory bundle
            item: ProbePlan = plan_queue.get(timeout=120.0)
            if isinstance(item, Exception):
                raise RuntimeError(f"Planner error on probe {i + 1}") from item
            if item is None:
                raise RuntimeError("Planner returned None")

            if item.probe_idx != i:
                raise RuntimeError(f"Trajectory index mismatch: expected {i}, got {item.probe_idx}")

            # Traverse to probe location and descend (single trajectory)
            print("\tTraversing to probe location...")
            robot.follow_joint_trajectory(item.traversal_traj, blocking=True)
            time.sleep(SETTLE_SEC)

            # Plan plunge concurrently with baseline, seeded from actual robot.q
            # after traversal so the IK warm-start matches the true starting config.
            plunge_holder: list = []
            plunge_error: list = []

            def _plan_plunge():
                try:
                    plunge_holder.append(
                        plan_plunge_traj(robot, approach_xyz, depths[i], plunge_times[i], BASE_ORI, robot.q.copy())
                    )
                except Exception as exc:
                    plunge_error.append(exc)

            plunge_plan_thread = threading.Thread(target=_plan_plunge, daemon=True)
            plunge_plan_thread.start()

            # Baseline runs concurrently with plunge planning
            print(f"\tAcquiring baseline for probe {i + 1}...")
            if ser is not None:
                baseline, baseline_mean = acquire_baseline(ser)
            else:
                baseline = np.zeros((0, 4), dtype=np.uint16)
                baseline_mean = np.zeros(4, dtype=np.float64)

            plunge_plan_thread.join(timeout=60.0)
            if plunge_error:
                raise RuntimeError(f"Plunge planning failed: {plunge_error[0]}") from plunge_error[0]
            if not plunge_holder:
                raise RuntimeError("Plunge planning timed out")

            # Execute plunge with data recording
            print("\tProbing...")
            ts, ee_poses, ee_forces, _, teensy_data, teensy_timestamps = execute_probe(
                robot, plunge_holder[0], approach_xyz, plunge_times[i], ser
            )

            # # Retract to safe height
            # print("\tRetracting in z...")
            # robot.follow_joint_trajectory(item.retract_traj, blocking=True)
            # time.sleep(SETTLE_SEC)
            print("\tPlunge complete.")

            ee_positions = [pose.position for pose in ee_poses]
            ee_orientations = [pose.orientation.as_quat() for pose in ee_poses]

            # Validate depth reached
            if np.any(np.asarray(ee_positions)[:, 2] <= (PROBE_START_Z - depths[i] + 2.5e-3)):
                exp_dict["ts"].append(ts)
                exp_dict["ee_poses"].append({"positions": ee_positions, "orientations": ee_orientations})
                exp_dict["ee_forces"].append(ee_forces)
                exp_dict["plunge_depths"].append(depths[i])
                exp_dict["plunge_times"].append(plunge_times[i])
                exp_dict["grid_positions"].append([x_sensor, y_sensor])
                exp_dict["teensy_data"].append(teensy_data)
                exp_dict["teensy_timestamps"].append(teensy_timestamps)
                exp_dict["baseline_data"].append(baseline)
                exp_dict["baseline_mean"].append(baseline_mean)

                with open(full_path, "wb") as f:
                    pickle.dump(exp_dict, f)
                    print(f"Results saved to: {full_path}")

                if DEBUG_PLOTS:
                    plot_probe_debug(
                        probe_idx=i,
                        ts=ts,
                        ee_poses=ee_poses,
                        ee_forces=ee_forces,
                        teensy_data=teensy_data,
                        teensy_timestamps=teensy_timestamps,
                        baseline_mean=baseline_mean,
                        output_dir=results_dir,
                        target_xyz=approach_xyz,
                        depth=depths[i],
                        probe_time=plunge_times[i],
                    )
            else:
                raise ValueError("Plunge depth not achieved. Check robot operation.")

    finally:
        abort_event.set()

    # ---- Finish ----
    print("\nReturning home...")
    robot.move_to(pose=home_pose, speed=MOVE_SPEED)
    time.sleep(SETTLE_SEC)

    if ser is not None:
        ser.close()
    robot.shutdown()
    print("Done.")


if __name__ == "__main__":
    main()
