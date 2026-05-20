import time
import threading
import numpy as np
import serial
from scipy.spatial.transform import Rotation as R
from arm_client.robot import Robot, Pose
from arm_client.planning.types import CartesianWaypoint
from pathlib import Path
import pickle

SETTLE_SEC = 0.2  # wait time after moves (s)
MOVE_SPEED = 0.05  # m/s for traversal moves

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
TEENSY = False  # set False to skip Teensy and run robot-only
SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 3000000
TEENSY_SAMPLE_RATE_HZ_FALLBACK = 40000.0


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


# ===================== Probe helper =====================
def probe(
    robot: Robot,
    start_xyz: np.ndarray,
    depth: float,
    probe_time: float,
    fixed_ori: R | None = None,
    plunge_func: str = "cos",
    initial_joint_config: np.ndarray | None = None,
    ser=None,
):
    """
    Plan and execute a plunge-and-retract cycle using the IK joint trajectory controller.

    The robot must already be positioned at start_xyz before calling this function.

    Args:
        robot: Robot client (joint_trajectory_controller active).
        start_xyz: [x, y, z_surface] start point of plunge.
        depth: Probe depth in metres (positive = downwards).
        probe_time: Duration of the full plunge+retract cycle (s).
        fixed_ori: End-effector orientation held throughout (defaults to current).
        plunge_func: 'cos' or 'linear' plunge profile.
        initial_joint_config: Seed joint config for IK (uses robot.q if None).
        ser: Open serial.Serial to the Teensy, or None to skip sensor recording.

    Returns:
        ts: List of perf_counter timestamps during execution.
        ee_poses: List of Pose objects recorded during execution.
        ee_forces: List of force numpy arrays recorded during execution.
        t_min: perf_counter timestamp when z-displacement was maximum.
        final_joint_cfg: Last joint config from the planned trajectory.
        teensy_data: (N, 4) uint16 array of raw ADC samples (zeros if ser=None).
        teensy_timestamps: (N,) float array of sample timestamps in seconds (empty if ser=None).
    """
    if fixed_ori is None:
        fixed_ori = robot.end_effector_pose.orientation

    # Build plunge waypoints
    z_init = float(start_xyz[2])
    N = max(2, int(probe_time * 10))  # ~10 waypoints/s

    waypoints = []
    for k in range(N + 1):
        s = k / N
        if plunge_func == "cos":
            z = z_init + depth / 2 * (np.cos(2 * np.pi * s) - 1)
        else:  # linear
            z = z_init + depth * (np.abs(2 * s - 1) - 1)
        waypoints.append(
            CartesianWaypoint(
                position=np.array([start_xyz[0], start_xyz[1], z], dtype=float),
                orientation=fixed_ori,
                s=s,
            )
        )

    joint_traj = robot.plan_joint_trajectory(
        waypoints=waypoints,
        duration=probe_time,
        n_points=N + 1,
        visualize=False,
        show_progress=False,
        initial_joint_config=initial_joint_config,
    )

    # Optionally start Teensy streaming before the trajectory begins
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

    # Execute trajectory and record ee state
    ee_forces = []
    ee_poses = []
    ts = []
    t_min = 0.0
    z_min = z_init

    robot.follow_joint_trajectory(joint_traj, blocking=False)

    while robot.wait_for_trajectory_completion(probe_time, timeout_margin=0.5):
        ee_force = robot.end_effector_external_wrench["force"]
        ee_pose = robot.end_effector_pose
        time_stamp = time.perf_counter()

        if ee_pose.position[2] < z_min:
            z_min = ee_pose.position[2]
            t_min = time_stamp

        ee_poses.append(ee_pose.copy())
        ee_forces.append(ee_force.copy())
        ts.append(time_stamp)
        time.sleep(0.01)

    # Stop Teensy stream
    if ser is not None and teensy_thread is not None:
        ser.write(b"2")
        time.sleep(0.1)
        teensy_thread.join(timeout=teensy_max_duration)

    teensy_data, teensy_timestamps = (
        teensy_holder[0] if teensy_holder else (np.zeros((0, 4), dtype=np.uint16), np.array([]))
    )

    final_joint_cfg = joint_traj.joint_positions[-1]
    return ts, ee_poses, ee_forces, t_min, final_joint_cfg, teensy_data, teensy_timestamps


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
            # Back-fill Teensy keys if resuming from a pre-Teensy results file
            exp_dict.setdefault("teensy_data", [])
            exp_dict.setdefault("teensy_timestamps", [])

    print(f"Resuming from probe number {start_idx + 1}")

    # Move home before starting
    robot.move_to(pose=home_pose, speed=MOVE_SPEED)
    time.sleep(SETTLE_SEC)

    input("Press Enter to start probing...")

    # ---- Main probe loop ----
    for i in range(start_idx, N_POINTS):
        x, y = grid_xy_world[i]
        x_sensor, y_sensor = grid_xy_sensor[i]

        print(f"\n Probe {i + 1} / {N_POINTS}")

        # Travel home
        robot.move_to(pose=home_pose, speed=MOVE_SPEED)
        time.sleep(SETTLE_SEC)

        # Press trigger (no Teensy — only used for t_ref time-referencing)
        # print("\tPressing trigger...")
        # _, _, _, t_ref, _, _, _ = probe(
        #     robot,
        #     start_xyz=home_position,
        #     depth=TRIG_DEPTH,
        #     probe_time=TRIG_TIME,
        #     fixed_ori=BASE_ORI,
        #     plunge_func="cos",
        #     initial_joint_config=robot.q.copy(),
        #     ser=None,
        # )

        # Move to probe location at safe height
        print("\tMoving to probe location...")
        robot.move_to(position=np.array([x, y, Z_INIT], dtype=float), speed=MOVE_SPEED)
        time.sleep(SETTLE_SEC)

        # Descend to probe start height
        print("\tDescending to probe start height...")
        approach_xyz = np.array([x, y, PROBE_START_Z], dtype=float)
        robot.move_to(position=approach_xyz, speed=MOVE_SPEED)
        time.sleep(SETTLE_SEC)

        # Calibration data

        # Probe cycle with Teensy
        print("\tStarting probe...")
        ts, ee_poses, ee_forces, _, _, teensy_data, teensy_timestamps = probe(
            robot,
            start_xyz=approach_xyz,
            depth=depths[i],
            probe_time=plunge_times[i],
            fixed_ori=BASE_ORI,
            initial_joint_config=robot.q.copy(),
            ser=ser,
        )

        # Retract in z
        print("\tRetracting in z...")
        robot.move_to(position=np.array([x, y, Z_INIT], dtype=float), speed=MOVE_SPEED)
        time.sleep(SETTLE_SEC)
        print("\tPlunge complete.")

        ee_positions = [pose.position for pose in ee_poses]
        ee_orientations = [pose.orientation.as_quat() for pose in ee_poses]

        # Validate depth reached (same check as before)
        if np.any(np.asarray(ee_positions)[:, 2] <= (PROBE_START_Z - depths[i] + 2.5e-3)):
            ts_adjusted = [t - t_ref for t in ts]
            exp_dict["ts"].append(ts_adjusted)
            exp_dict["ee_poses"].append(
                {
                    "positions": ee_positions,
                    "orientations": ee_orientations,
                }
            )
            exp_dict["ee_forces"].append(ee_forces)
            exp_dict["plunge_depths"].append(depths[i])
            exp_dict["plunge_times"].append(plunge_times[i])
            exp_dict["grid_positions"].append([x_sensor, y_sensor])
            exp_dict["teensy_data"].append(teensy_data)
            exp_dict["teensy_timestamps"].append(teensy_timestamps)

            with open(full_path, "wb") as f:
                pickle.dump(exp_dict, f)
                print(f"Results saved to: {full_path}")
        else:
            raise ValueError("Plunge depth not achieved. Check robot operation.")

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
