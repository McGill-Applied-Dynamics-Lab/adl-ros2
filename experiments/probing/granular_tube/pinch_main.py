from typing import Sequence
from pathlib import Path
import time
import threading
import queue
import pickle

import numpy as np
import serial
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

from arm_client.robot import Robot, Pose
from arm_client.gripper.franka_hand import Gripper, GripperConfig
from arm_client.planning.types import CartesianWaypoint

# ===================== Experiment Setup =====================
TUBE_CENTER_POS = np.array([0.4913, 0.0373])  # (x, y), m
OUTER_RADIUS = 0.025  # Distance from fingers center to tube center (m)
TUBE_LENGTH = 0.040  # set lower than the actual tube height (m)
TUBE_DIAMETER = 0.040  # m
GRIPPER_THICKNESS = 0.024  # probe thickness (m)
MOVE_SPEED = 0.01  # (m/s)
MOVE_SPEED_ROT = 0.2  # (rad/s)
Z_MIN = 0.19  # minimum sensor height from table (m)

SAFE_ORI = R.from_euler("xyz", [-180, 90, -90], degrees=True)
SAFE_POS = np.array([0.48, -0.15, Z_MIN + 0.05])

# ===================== Experiment Configurations =====================
# (Probe-by-probe parameters — azimuths, normalized heights, pinch depths,
# pinch speeds — are loaded from PARAMETERS_FILE; generate them with
# `python generate_pinch_parameters.py`.)

PINCH_TIME = 1.0  # Hold time after gripper reaches target (s)
SETTLE_SEC = 0.1  # Wait time after moves (s)

# Trajectory configurations
QUEUE_SIZE = 3
TRAJ_N_POINTS = 10

# Gripper recording rate while pinching (s between samples)
GRIPPER_POLL_DT = 0.005

# Baseline pressure acquisition (run once at startup)
BASELINE_DURATION_SEC = 3.0

# Teensy sample rate (Hz) — used to compute pressure timestamps for plotting
TEENSY_SAMPLE_RATE_HZ = 10000.0

gripper_cfg = GripperConfig(
    max_width=0.08,
    min_width=0.0,
    default_speed=0.01,
)

# ===================== Teensy Configuration =====================
SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 3000000

# ===================== File Paths =====================
PROJECT_ROOT = Path(__file__).resolve().parent
PARAMETERS_FILE = PROJECT_ROOT / "results" / "grids" / "pinch_params.pkl"
RESULTS_FILE = PROJECT_ROOT / "results" / "pinch_main.pkl"
PLOTS_DIR = PROJECT_ROOT / "results" / "plots_pinch_main"


# ===================== Teensy serial helpers (from twist_v2.py) =====================
def read_exact_bytes(ser, num_bytes):
    """Helper to ensure we get every single byte requested."""
    data = bytearray()
    while len(data) < num_bytes:
        chunk = ser.read(num_bytes - len(data))
        if chunk:
            data.extend(chunk)
    return bytes(data)


def collect_teensy_data_streaming(ser, max_duration=15.0):
    """
    Collect Teensy pressure sensor data via streaming chunks.

    Reads chunks of data as they arrive from the Teensy, avoiding buffer overflow.
    The main thread should call ser.write(b"2") to end collection.

    Returns:
        numpy array of shape (N, 2) with pressure readings.
    """
    all_samples_1 = []
    all_samples_2 = []
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
                    chunk_size_bytes = read_exact_bytes(ser, 4)
                    chunk_size = int.from_bytes(chunk_size_bytes, byteorder="little")

                    chunk_data_1 = read_exact_bytes(ser, chunk_size * 2)
                    chunk_data_2 = read_exact_bytes(ser, chunk_size * 2)

                    samples_1 = np.frombuffer(chunk_data_1, dtype=np.uint16)
                    samples_2 = np.frombuffer(chunk_data_2, dtype=np.uint16)

                    all_samples_1.extend(samples_1)
                    all_samples_2.extend(samples_2)

                elif marker == b"E":
                    total_bytes = read_exact_bytes(ser, 4)
                    total_count = int.from_bytes(total_bytes, byteorder="little")
                    print(f"  Received {total_count} samples from Teensy")
                    break
            else:
                time.sleep(0.001)

        if len(all_samples_1) > 0:
            return np.column_stack([all_samples_1, all_samples_2])
        else:
            return np.array([])

    except Exception as e:
        print(f"Error in streaming data collection: {e}")
        return np.array([])


# ===================== Geometry / planner (from pinch.py) =====================
def coord_to_pose(angle, height) -> Pose:
    """Convert (angle, height) to a workspace cartesian Pose."""
    x = TUBE_CENTER_POS[0] + (OUTER_RADIUS * np.sin(angle))
    y = TUBE_CENTER_POS[1] - (OUTER_RADIUS * np.cos(angle))
    z = height + Z_MIN
    return Pose(np.array([x, y, z]), R.from_euler("z", angle, degrees=False) * SAFE_ORI)


def move_waypoints(
    start_rad, end_rad, start_height, end_height
) -> tuple[float, Sequence[CartesianWaypoint]]:
    """Generate trajectory between consecutive pinch locations."""
    delta = (end_rad - start_rad + np.pi) % (2 * np.pi) - np.pi

    waypoints = []
    arc_length = OUTER_RADIUS * np.abs(delta)
    z_diff = end_height - start_height
    n_points = TRAJ_N_POINTS

    ds = np.sqrt(arc_length**2 + z_diff**2)
    dz = z_diff / n_points

    time_trans = ds / MOVE_SPEED
    time_rot = np.abs(delta) / MOVE_SPEED_ROT
    traj_duration = max(time_trans, time_rot, 0.1)

    for i in range(n_points + 1):
        current_theta = start_rad + (delta * i / n_points)
        current_height = start_height + (dz * i)
        waypoint = coord_to_pose(current_theta, current_height)
        waypoints.append(
            CartesianWaypoint(
                position=waypoint.position,
                orientation=waypoint.orientation,
                s=i / n_points,
            )
        )

    return traj_duration, waypoints


def planner_worker(
    robot,
    angles,
    heights,
    start_angle,
    start_height,
    start_joint_cfg,
    plan_queue,
    abort_event,
):
    """Background thread to pre-compute joint trajectories."""
    last_angle = start_angle
    last_height = start_height
    last_joint_cfg = start_joint_cfg

    for i, (target_angle, target_height) in enumerate(zip(angles, heights)):
        if abort_event.is_set():
            break

        traj_duration, waypoints = move_waypoints(
            last_angle, target_angle, last_height, target_height
        )

        try:
            joint_traj = robot.plan_joint_trajectory(
                waypoints=waypoints,
                duration=traj_duration,
                visualize=False,
                n_points=len(waypoints),
                show_progress=False,
                initial_joint_config=last_joint_cfg,
            )
        except Exception as e:
            print(f"\n[Planner] Error planning trajectory: {e}")
            try:
                plan_queue.put(e, timeout=1.0)
            except queue.Full:
                pass
            break

        while not abort_event.is_set():
            try:
                plan_queue.put(joint_traj, timeout=0.1)
                break
            except queue.Full:
                continue

        if abort_event.is_set():
            break

        last_height = target_height
        last_angle = target_angle
        last_joint_cfg = joint_traj.joint_positions[-1]


# ===================== Baseline acquisition + plotting =====================
def acquire_baseline(ser, duration_sec=BASELINE_DURATION_SEC):
    """
    Stream pressure data for `duration_sec` with no robot motion, so we
    can compute a per-sensor mean to subtract from later experiments.

    Returns:
        baseline (np.ndarray): (N, 2) raw ADC samples.
        baseline_mean (np.ndarray): (2,) per-sensor mean for baseline subtraction.
    """
    print(f"Acquiring {duration_sec:.1f}s of baseline pressure data...")
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    holder = []

    def worker():
        try:
            holder.append(
                collect_teensy_data_streaming(ser, max_duration=duration_sec + 5.0)
            )
        except Exception as e:
            print(f"  Error in baseline collection thread: {e}")
            holder.append(np.array([]))

    th = threading.Thread(target=worker)
    th.start()

    # The thread starts the stream; sleep for the requested baseline duration,
    # then ask the Teensy to stop.
    time.sleep(0.1 + duration_sec)
    ser.write(b"2")
    time.sleep(0.1)
    th.join(timeout=10.0)

    baseline = holder[0] if holder else np.array([])
    if baseline.size == 0:
        print("  Warning: no baseline data received; using zero baseline.")
        return baseline, np.zeros(2, dtype=np.float64)

    baseline_mean = baseline.astype(np.float64).mean(axis=0)
    print(
        f"  Baseline collected: {baseline.shape[0]} samples, "
        f"means = ({baseline_mean[0]:.2f}, {baseline_mean[1]:.2f})"
    )
    return baseline, baseline_mean


def generate_plot(probe_idx, angle, height, pinch_depth, pinch_speed,
                  ts_gripper, gripper_widths, pressures, baseline_mean):
    """Save a dual-axis plot: baseline-subtracted pressures + gripper width vs time."""
    fig, ax1 = plt.subplots(figsize=(10, 6))

    if pressures.size > 0:
        t_press = np.arange(pressures.shape[0]) / TEENSY_SAMPLE_RATE_HZ
        p1 = pressures[:, 0].astype(np.float64) - baseline_mean[0]
        p2 = pressures[:, 1].astype(np.float64) - baseline_mean[1]
        ax1.plot(t_press, p1, color="tab:blue", alpha=0.7,
                 label="Sensor 1 (baseline-subtracted)")
        ax1.plot(t_press, p2, color="tab:orange", alpha=0.7,
                 label="Sensor 2 (baseline-subtracted)")

    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Pressure ADC - baseline (12-bit counts)", color="k")
    ax1.tick_params(axis="y", labelcolor="k")
    ax1.grid(True, linestyle="--", alpha=0.5)

    ax2 = ax1.twinx()
    if len(ts_gripper) > 0:
        ax2.plot(np.array(ts_gripper), np.array(gripper_widths) * 1000.0,
                 color="tab:red", linewidth=2.5, label="Gripper width (mm)")
    ax2.set_ylabel("Gripper width (mm)", color="tab:red")
    ax2.tick_params(axis="y", labelcolor="tab:red")

    lines_1, labels_1 = ax1.get_legend_handles_labels()
    lines_2, labels_2 = ax2.get_legend_handles_labels()
    ax1.legend(lines_1 + lines_2, labels_1 + labels_2, loc="upper left")

    plt.title(
        f"Probe {probe_idx}: angle={np.degrees(angle):.1f}°, "
        f"height={height * 1000:.1f}mm, depth={pinch_depth * 1000:.1f}mm, "
        f"speed={pinch_speed * 1000:.1f}mm/s"
    )
    plt.tight_layout()

    PLOTS_DIR.mkdir(parents=True, exist_ok=True)
    filepath = PLOTS_DIR / f"probe_{probe_idx:03d}.png"
    plt.savefig(filepath, dpi=150)
    plt.close(fig)


# ===================== Pinch + serial recording =====================
def execute_pinch(gripper, pinch_depth, pinch_speed, ser):
    """
    Run a single pinch cycle (close -> hold -> open) while streaming
    pressure data from the Teensy and recording gripper width over time.

    Returns:
        ts (list[float]):              gripper sample timestamps (s, t0 = pinch start)
        gripper_widths (list[float]):  gripper widths at each ts
        pressures (np.ndarray):        (N, 2) pressure ADC samples covering the cycle
    """
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    ts_all = []
    gripper_widths_all = []

    pressure_list = []

    def collect_teensy_in_thread():
        try:
            pressure_list.append(collect_teensy_data_streaming(ser, max_duration=15.0))
        except Exception as e:
            print(f"  Error in Teensy collection thread: {e}")
            pressure_list.append(np.array([]))

    teensy_thread = threading.Thread(target=collect_teensy_in_thread)
    teensy_thread.start()
    time.sleep(0.1)  # let streaming actually start before we begin moving

    depth_target = TUBE_DIAMETER + 2 * (GRIPPER_THICKNESS - pinch_depth)

    close_result = {"success": False}

    def gripper_close_worker():
        try:
            close_result["success"] = gripper.set_target(depth_target, speed=pinch_speed)
        except Exception as e:
            close_result["error"] = e

    close_thread = threading.Thread(target=gripper_close_worker)

    t0 = time.perf_counter()
    close_thread.start()

    while close_thread.is_alive():
        try:
            ts_all.append(time.perf_counter() - t0)
            gripper_widths_all.append(gripper.value)
        except Exception:
            pass
        time.sleep(GRIPPER_POLL_DT)

    # Hold
    hold_start = time.perf_counter()
    while time.perf_counter() - hold_start < PINCH_TIME:
        try:
            ts_all.append(time.perf_counter() - t0)
            gripper_widths_all.append(gripper.value)
        except Exception:
            pass
        time.sleep(GRIPPER_POLL_DT)

    # Open (also recorded so we capture the release transient)
    open_result = {"success": False}

    def gripper_open_worker():
        try:
            open_result["success"] = gripper.open(speed=0.1)
        except Exception as e:
            open_result["error"] = e

    open_thread = threading.Thread(target=gripper_open_worker)
    open_thread.start()

    while open_thread.is_alive():
        try:
            ts_all.append(time.perf_counter() - t0)
            gripper_widths_all.append(gripper.value)
        except Exception:
            pass
        time.sleep(GRIPPER_POLL_DT)

    # Stop Teensy stream
    ser.write(b"2")
    time.sleep(0.1)

    teensy_thread.join(timeout=15.0)
    pressures = pressure_list[0] if len(pressure_list) > 0 else np.array([])

    time.sleep(SETTLE_SEC)

    if not close_result.get("success", False):
        raise RuntimeError("Gripper failed to close to target")

    return ts_all, gripper_widths_all, pressures


# ===================== Parameter / results persistence =====================
def load_parameters():
    """
    Load pre-saved pinch parameters generated by generate_pinch_parameters.py.

    Expected keys in the pickle:
        azimuths            (rad, 0..2*pi)
        normalized_heights  (unitless, 0..1; scaled by TUBE_LENGTH here)
        pinch_depths        (m)
        pinch_speeds        (m/s)
        num_points          (int)
    """
    if not PARAMETERS_FILE.exists():
        raise FileNotFoundError(
            f"Pinch parameters not found at {PARAMETERS_FILE}. "
            "Run `python generate_pinch_parameters.py` first."
        )

    with open(PARAMETERS_FILE, "rb") as f:
        params = pickle.load(f)

    azimuths = np.asarray(params["azimuths"], dtype=np.float64)
    normalized_heights = np.asarray(params["normalized_heights"], dtype=np.float64)
    pinch_depths = np.asarray(params["pinch_depths"], dtype=np.float64)
    pinch_speeds = np.asarray(params["pinch_speeds"], dtype=np.float64)
    num_points = int(params["num_points"])

    # Heights stored normalized; scale by tube height for use here.
    heights = normalized_heights * TUBE_LENGTH

    print(f"Loaded {num_points} parameters from {PARAMETERS_FILE}")
    return {
        "angles": azimuths,
        "heights": heights,
        "pinch_depths": pinch_depths,
        "pinch_speeds": pinch_speeds,
        "num_points": num_points,
    }


def load_or_init_results():
    RESULTS_FILE.parent.mkdir(parents=True, exist_ok=True)
    if RESULTS_FILE.exists():
        with open(RESULTS_FILE, "rb") as f:
            exp_dict = pickle.load(f)
        return exp_dict, len(exp_dict["target_angles"])

    exp_dict = {
        "target_angles": [],
        "target_heights": [],
        "target_pinch_depths": [],
        "target_pinch_speeds": [],
        "ts_gripper": [],
        "gripper_widths": [],
        "pressures": [],
        "baseline_pressures": np.array([]),
        "baseline_mean": np.zeros(2, dtype=np.float64),
    }
    return exp_dict, 0


# ===================== Main =====================
def main():
    # ---- Connect to Teensy ----
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2)
        time.sleep(2)

        # If a previous crashed run left the Teensy mid-stream, this breaks it out.
        ser.write(b"2")
        time.sleep(0.5)
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        print(f"Connected to Teensy on {SERIAL_PORT}")
    except Exception as e:
        print(f"Failed to connect to Teensy: {e}")
        return

    # ---- Parameters + previous results ----
    try:
        params = load_parameters()
    except FileNotFoundError as e:
        print(e)
        ser.close()
        return
    angles = params["angles"]
    heights = params["heights"]
    pinch_depths = params["pinch_depths"]
    pinch_speeds = params["pinch_speeds"]
    num_points = params["num_points"]

    exp_dict, start_idx = load_or_init_results()
    if start_idx >= num_points:
        print(f"All {num_points} probes already collected. Nothing to do.")
        ser.close()
        return
    if start_idx > 0:
        print(f"Resuming from probe {start_idx + 1} / {num_points}")

    # Acquire baseline pressure once at startup (or reuse saved baseline on resume).
    saved_baseline = exp_dict.get("baseline_pressures", np.array([]))
    if isinstance(saved_baseline, np.ndarray) and saved_baseline.size > 0:
        baseline_mean = np.asarray(
            exp_dict.get("baseline_mean", saved_baseline.astype(np.float64).mean(axis=0))
        )
        print(
            f"Reusing saved baseline ({saved_baseline.shape[0]} samples, "
            f"means = ({baseline_mean[0]:.2f}, {baseline_mean[1]:.2f}))."
        )
    else:
        baseline, baseline_mean = acquire_baseline(ser)
        exp_dict["baseline_pressures"] = baseline
        exp_dict["baseline_mean"] = baseline_mean
        with open(RESULTS_FILE, "wb") as f:
            pickle.dump(exp_dict, f)

    # ---- Robot + gripper ----
    robot = Robot(namespace="fr3")
    gripper = Gripper(namespace="fr3", gripper_config=gripper_cfg)

    robot.wait_until_ready()
    gripper.wait_until_ready(timeout=5.0)

    print("Switching to joint trajectory controller...")
    robot.controller_switcher_client.switch_controller("joint_trajectory_controller")

    gripper.open(speed=0.05)
    robot.move_to(pose=Pose(SAFE_POS, SAFE_ORI))

    # ---- Outer loop: in-program restart on failure ----
    while start_idx < num_points:
        last_angle = 0.0
        last_height = 0.0
        start_pose = coord_to_pose(last_angle, last_height)

        print("=================================")
        print("FR3 Pinch Main Experiment")
        print("=================================")
        print(
            f"Moving to start (angle: {np.degrees(last_angle):.2f}°, height: {last_height:.2f}m)"
        )
        try:
            robot.move_to(pose=start_pose)
            gripper.open(speed=0.05)
        except Exception as e:
            print(f"Failed to move to start position: {e}. Retrying in 2s...")
            time.sleep(2.0)
            continue

        last_joint_cfg = robot.q.copy()
        print("Robot initialization complete.")

        plan_queue = queue.Queue(maxsize=QUEUE_SIZE)
        abort_event = threading.Event()

        planner_thread = threading.Thread(
            target=planner_worker,
            args=(
                robot,
                angles[start_idx:],
                heights[start_idx:],
                last_angle,
                last_height,
                last_joint_cfg,
                plan_queue,
                abort_event,
            ),
            daemon=True,
        )
        planner_thread.start()

        all_done = True
        for i in range(start_idx, num_points):
            try:
                print(f"\nProbe {i + 1}/{num_points}")
                print(f"\tAngle: {np.degrees(angles[i]):.2f}°")
                print(f"\tHeight: {heights[i]:.3f} m")
                print(f"\tPinch Depth: {pinch_depths[i]:.3f} m")
                print(f"\tPinch Speed: {pinch_speeds[i]:.3f} m/s")

                item = plan_queue.get()
                if isinstance(item, Exception):
                    raise RuntimeError(f"Planner failed: {item}")
                joint_traj = item

                print("\tExecuting move...")
                robot.follow_joint_trajectory(joint_traj, blocking=True)

                print("\tPinching + recording...")
                ts, widths, pressures = execute_pinch(
                    gripper=gripper,
                    pinch_depth=float(pinch_depths[i]),
                    pinch_speed=float(pinch_speeds[i]),
                    ser=ser,
                )

                exp_dict["target_angles"].append(float(angles[i]))
                exp_dict["target_heights"].append(float(heights[i]))
                exp_dict["target_pinch_depths"].append(float(pinch_depths[i]))
                exp_dict["target_pinch_speeds"].append(float(pinch_speeds[i]))
                exp_dict["ts_gripper"].append(np.array(ts))
                exp_dict["gripper_widths"].append(np.array(widths))
                exp_dict["pressures"].append(pressures)

                with open(RESULTS_FILE, "wb") as f:
                    pickle.dump(exp_dict, f)

                generate_plot(
                    probe_idx=i + 1,
                    angle=float(angles[i]),
                    height=float(heights[i]),
                    pinch_depth=float(pinch_depths[i]),
                    pinch_speed=float(pinch_speeds[i]),
                    ts_gripper=ts,
                    gripper_widths=widths,
                    pressures=pressures,
                    baseline_mean=baseline_mean,
                )

                print(
                    f"\tSaved probe {i + 1}: "
                    f"{len(ts)} gripper samples, "
                    f"{pressures.shape[0] if pressures.size else 0} pressure samples, "
                    f"plot -> {PLOTS_DIR.name}/probe_{i + 1:03d}.png"
                )

                plan_queue.task_done()

            except Exception as e:
                print(f"\n[Error] Aborting at probe {i + 1}: {e}")
                all_done = False
                abort_event.set()

                while not plan_queue.empty():
                    try:
                        plan_queue.get_nowait()
                        plan_queue.task_done()
                    except queue.Empty:
                        break

                print("Moving back to safe position and restarting...")
                try:
                    robot.move_to(pose=Pose(SAFE_POS, SAFE_ORI))
                except Exception as move_err:
                    print(f"  (safe move failed: {move_err})")
                time.sleep(SETTLE_SEC)
                planner_thread.join(timeout=2.0)

                # Make sure the Teensy is not still in streaming mode.
                try:
                    ser.write(b"2")
                    time.sleep(0.2)
                    ser.reset_input_buffer()
                    ser.reset_output_buffer()
                except Exception:
                    pass

                start_idx = len(exp_dict["target_angles"])
                break

        if all_done:
            print("\nAll probing points completed successfully!")
            abort_event.set()
            planner_thread.join(timeout=1.0)
            break

    print("Returning to safe pos...")
    try:
        robot.move_to(pose=Pose(SAFE_POS, SAFE_ORI))
    except Exception as e:
        print(f"  (safe move failed: {e})")
    time.sleep(SETTLE_SEC)

    print("Shutting down...")
    ser.close()
    robot.shutdown()


if __name__ == "__main__":
    main()
