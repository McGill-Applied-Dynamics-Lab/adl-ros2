#!/usr/bin/env python3
import time
import threading
import numpy as np
import serial
from scipy.spatial.transform import Rotation as R
from arm_client.robot import Robot, Pose, Twist
from arm_client import CONFIG_DIR
from pathlib import Path
import pickle
from waveguide_gripper_grid_generator import fetch_landmarks

SETTLE_SEC = 2.00  # wait time after moves (s)
TRAJ_FREQ = 10.0  # Hz

# Probing parameters
Z_OFFSET = 0.0250  # (m) offset from landmark z to surface
PROBE_DEPTH = 0.0200  # m (additional depth beyond z_offset)
PROBE_TIME = 2.0  # plunge and retract (s)
BASE_ORI = R.from_euler("xyz", [-270, 0, 0], degrees=True)

# Serial / Teensy parameters
SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 3000000
SERIAL_TIMEOUT_SEC = 10

CMD_START = bytes([0x43])  # 'C' (67) - start continuous streaming
CMD_STOP  = bytes([0x45])  # 'E' (69) - stop streaming

EXPECTED_SAMPLES = 750
CHANNEL_MARKERS = ("S0", "S1", "S2", "S3")


# ── RF helper functions ──────────────────────────────────────────────────────

def read_line_or_raise(ser: serial.Serial) -> str:
    raw = ser.readline()
    if raw == b"":
        raise TimeoutError("Timed out waiting for serial data from Teensy")
    return raw.decode("ascii", errors="ignore").strip()


def _stream_reader(ser: serial.Serial, frames: list, stop_event: threading.Event) -> None:
    """
    Background thread: reads frames from the Teensy until "STREAM_END".

    Each frame is [ch0, ch1, ch2, ch3], each channel a list of EXPECTED_SAMPLES ints.
    Appends completed frames to `frames`. Stops on "STREAM_END" or stop_event.
    """
    current_channel = None
    current_samples = []
    current_frame = {}

    while not stop_event.is_set():
        try:
            raw = ser.readline()
        except serial.SerialException:
            break
        if raw == b"":
            continue

        line = raw.decode("ascii", errors="ignore").strip()

        if line == "STREAM_END":
            # Flush any completed frame in progress
            if len(current_frame) == len(CHANNEL_MARKERS):
                frames.append([current_frame[m] for m in CHANNEL_MARKERS])
            break

        if line in CHANNEL_MARKERS:
            current_channel = line
            current_samples = []
        elif line == "T":
            if current_channel is not None:
                current_frame[current_channel] = current_samples
                current_channel = None
                # A full frame is ready when all 4 channels are collected
                if len(current_frame) == len(CHANNEL_MARKERS):
                    frames.append([current_frame[m] for m in CHANNEL_MARKERS])
                    current_frame = {}
        else:
            try:
                current_samples.append(int(line))
            except ValueError:
                pass  # skip malformed lines


def rf_stream_start(ser: serial.Serial) -> tuple[list, threading.Event, threading.Thread]:
    """Send 'C' to begin streaming; return (frames_list, stop_event, reader_thread)."""
    frames: list = []
    stop_event = threading.Event()
    ser.reset_input_buffer()
    ser.write(CMD_START)
    t = threading.Thread(target=_stream_reader, args=(ser, frames, stop_event), daemon=True)
    t.start()
    return frames, stop_event, t


def rf_stream_stop(ser: serial.Serial, stop_event: threading.Event, thread: threading.Thread) -> None:
    """Send 'E' to stop streaming and wait for the reader thread to finish."""
    ser.write(CMD_STOP)
    thread.join(timeout=SERIAL_TIMEOUT_SEC)
    stop_event.set()  # in case thread didn't see STREAM_END


# ── Probe motion ─────────────────────────────────────────────────────────────

def probe(
    robot: Robot,
    start_xyz: np.ndarray,
    depth: float,
    probe_time: float,
    traj_freq: float = 5.0,
    fixed_ori: R | None = None,
    probe_func: str = 'cos'
):
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
        if probe_func == 'cos':
            z = z_init + depth / 2 * (np.cos(2 * np.pi * s) - 1)
        elif probe_func == 'linear':
            z = z_init + depth * (np.abs(2*s - 1) - 1)
        else:
            raise ValueError(f"Unsupported probe_func: {probe_func}")

        target_position = np.array([start_xyz[0], start_xyz[1], z], dtype=float)
        target_orientation = fixed_ori
        target_pose = Pose(target_position, target_orientation)
        twist = Twist(np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0]))

        waypoints.append((target_pose, twist))
        time_from_start.append(k * dt)

    ee_forces = []
    ee_poses = []
    ts = []

    robot.execute_trajectory(waypoints, time_from_start)

    t0 = time.perf_counter()
    t_min = 0.0
    z_min = z_init
    while robot.wait_for_trajectory_completion(probe_time, timeout_margin=0.5):
        ee_force = robot.end_effector_wrench["force"]
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


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    robot = Robot(namespace="fr3")
    robot.wait_until_ready()

    robot.controller_switcher_client.switch_controller("fr3_pose_controller")
    robot.fr3_pose_controller_parameters_client.load_param_config(
        file_path=CONFIG_DIR / "controllers" / "fr3_pose" / "probing.yaml"
    )

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
        raise FileNotFoundError(
            f"Grid file not found: {grid_file}. Please run grid_generator.py first."
        )

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
        "rf_data": [],
        "set_name": set_name,
        "landmarks": landmarks,
        "z_offset": Z_OFFSET,
    }

    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=SERIAL_TIMEOUT_SEC)

    try:
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

            # RF: send 'C' (0x43), start background reader thread
            print("\tStarting RF stream...")
            rf_frames, stop_event, rf_thread = rf_stream_start(ser)

            # Probe cycle
            print("\tStarting probe...")
            ts, ee_poses, ee_forces, _ = probe(
                robot,
                start_xyz=approach_xy,
                depth=Z_OFFSET + PROBE_DEPTH,
                probe_time=PROBE_TIME,
                traj_freq=TRAJ_FREQ,
                fixed_ori=BASE_ORI,
                probe_func='cos'
            )

            # RF: send 'E' (0x45), wait for reader thread to finish
            rf_stream_stop(ser, stop_event, rf_thread)
            print(f"\tRF stream stopped — {len(rf_frames)} frames collected.")

            ee_positions = [pose.position for pose in ee_poses]
            ee_orientations = [pose.orientation.as_quat() for pose in ee_poses]

            exp_dict["ts"].append(ts)
            exp_dict["ee_poses"].append({
                "positions": ee_positions,
                "orientations": ee_orientations
            })
            exp_dict["ee_forces"].append(ee_forces)
            exp_dict["rf_data"].append(rf_frames)
            print("\tProbe complete.")

        print("\nReturning home...")
        robot.set_target(pose=home_pose)
        time.sleep(SETTLE_SEC)
    finally:
        ser.close()
        robot.shutdown()

    print("Done.")

    results_dir = PROJECT_ROOT / "results"
    results_dir.mkdir(parents=True, exist_ok=True)

    base_filename = f"{len(grid_xy_world)}_grid_{set_name.upper()}_4RF"
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


if __name__ == "__main__":
    main()
