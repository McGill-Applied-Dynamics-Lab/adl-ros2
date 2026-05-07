import time
import threading
import numpy as np
import serial
import pickle
import matplotlib

matplotlib.use("Agg")  # Use non-interactive backend for headless saving
import matplotlib.pyplot as plt
from pathlib import Path

from scipy.spatial.transform import Rotation as R
from arm_client.robot import Pose, Robot
from arm_client.planning.waypoints import generate_linear_waypoints

# Configuration
SETTLE_SEC = 1.00
START_POSITION = np.array([0.493, 0.0455, 0.406])
START_ORIENTATION = R.from_euler("xyz", [-180, 0, 0], degrees=True)

# Teensy Configuration
SERIAL_PORT = "/dev/ttyACM0"  # <--- UPDATE THIS TO YOUR TEENSY PORT
BAUD_RATE = 3000000

# Baseline pressure acquisition (run once at startup)
BASELINE_DURATION_SEC = 3.0

# Teensy sample rate (Hz) — fallback only. The firmware estimates its own rate
# in setup() and appends it (plus a per-chunk rate + dt_since_last_us) to every
# chunk packet, so the host normally just reads the cadence off the wire and
# reconstructs per-sample timestamps. This constant is used only when the
# firmware never reports one (e.g. very old firmware) so timestamps still
# get produced.
TEENSY_SAMPLE_RATE_HZ_FALLBACK = 10000.0

# Display-only smoothing for the pressure plot. Saved data in the pickle is
# always raw; this only smooths what gets drawn. At ~10 kHz, 51 samples ≈ 5 ms.
PLOT_SMOOTH_WINDOW = 51

# File paths
PROJECT_ROOT = Path(__file__).resolve().parent
PARAMETERS_FILE = PROJECT_ROOT / "results" / "grids" / "rotation_params.pkl"
RESULTS_FILE = PROJECT_ROOT / "results" / "twist_100.pkl"
PLOTS_DIR = PROJECT_ROOT / "results" / "plots"


def read_exact_bytes(ser, num_bytes):
    """Helper to ensure we get every single byte requested."""
    data = bytearray()
    while len(data) < num_bytes:
        chunk = ser.read(num_bytes - len(data))
        if chunk:
            data.extend(chunk)
    return bytes(data)


def collect_teensy_data_streaming(ser, max_duration=10.0):
    """
    Collect Teensy pressure sensor data via streaming chunks.

    Reads chunks as they arrive from the Teensy, avoiding USB-CDC buffer
    overflow. The main thread should call ser.write(b"2") to end collection.

    Each 'C' chunk carries (see teensy_tx.ino for the wire format):
        - chunk_size                                          (uint32)
        - chunk_size uint16 samples for channel 1
        - chunk_size uint16 samples for channel 2
        - chunk_rate_hz: intra-chunk sample rate              (float32)
        - dt_since_last_us: first-sample-of-this-chunk minus
          first-sample-of-prev-chunk (or stream-start for k=0) (uint32)

    Per-sample timestamps are reconstructed from these as:
        t_chunk_start[k] = t_chunk_start[k-1] + dt_since_last_us[k] / 1e6
        t_sample[k][j]   = t_chunk_start[k] + j / chunk_rate_hz[k]
    Then anchored so the first sample's timestamp is 0.

    Returns:
        (data, timestamps):
            data (np.ndarray, shape (N, 2)): pressure readings.
            timestamps (np.ndarray, shape (N,)): seconds, anchored at 0.
    """
    all_samples_1 = []
    all_samples_2 = []
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
                    chunk_size = int.from_bytes(
                        read_exact_bytes(ser, 4), byteorder="little"
                    )
                    chunk_data_1 = read_exact_bytes(ser, chunk_size * 2)
                    chunk_data_2 = read_exact_bytes(ser, chunk_size * 2)
                    rate_bytes = read_exact_bytes(ser, 4)
                    dt_us = int.from_bytes(
                        read_exact_bytes(ser, 4), byteorder="little"
                    )

                    samples_1 = np.frombuffer(chunk_data_1, dtype=np.uint16)
                    samples_2 = np.frombuffer(chunk_data_2, dtype=np.uint16)
                    chunk_rate_hz = float(
                        np.frombuffer(rate_bytes, dtype=np.float32)[0]
                    )

                    cum_chunk_start_s += dt_us * 1e-6

                    if chunk_rate_hz > 0:
                        last_known_rate = chunk_rate_hz
                    effective_rate = (
                        last_known_rate
                        if last_known_rate is not None
                        else TEENSY_SAMPLE_RATE_HZ_FALLBACK
                    )
                    period = 1.0 / effective_rate
                    chunk_ts = cum_chunk_start_s + np.arange(chunk_size) * period

                    all_samples_1.extend(samples_1)
                    all_samples_2.extend(samples_2)
                    all_timestamps.append(chunk_ts)

                elif marker == b"E":
                    total_count = int.from_bytes(
                        read_exact_bytes(ser, 4), byteorder="little"
                    )
                    print(f"  ✓ Received {total_count} samples from Teensy")
                    break
            else:
                time.sleep(0.001)

        if len(all_samples_1) > 0:
            data = np.column_stack([all_samples_1, all_samples_2])
            timestamps = np.concatenate(all_timestamps)
            timestamps = timestamps - timestamps[0]  # anchor first sample at t=0
            return data, timestamps
        else:
            return np.array([]), np.array([])

    except Exception as e:
        print(f"Error in streaming data collection: {e}")
        return np.array([]), np.array([])


def acquire_baseline(ser, duration_sec=BASELINE_DURATION_SEC):
    """
    Stream pressure data for `duration_sec` with no robot motion, then
    return the raw samples and the per-sensor mean for baseline subtraction.
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
            holder.append((np.array([]), np.array([])))

    th = threading.Thread(target=worker)
    th.start()

    time.sleep(0.1 + duration_sec)
    ser.write(b"2")
    time.sleep(0.1)
    th.join(timeout=10.0)

    # Discard timestamps; baseline only needs the per-sensor mean.
    baseline = holder[0][0] if holder else np.array([])
    if baseline.size == 0:
        print("  Warning: no baseline data received; using zero baseline.")
        return baseline, np.zeros(2, dtype=np.float64)

    baseline_mean = baseline.astype(np.float64).mean(axis=0)
    print(
        f"  Baseline collected: {baseline.shape[0]} samples, "
        f"means = ({baseline_mean[0]:.2f}, {baseline_mean[1]:.2f})"
    )
    return baseline, baseline_mean


def generate_plot(
    run_idx, angle, speed, ts_fwd, angles_fwd, ts_rev, angles_rev, press_full,
    pressure_timestamps,
    baseline_mean=np.zeros(2, dtype=np.float64),
):
    """Generates and saves a dual-axis plot of (baseline-subtracted) pressures and robot angle."""
    fig, ax1 = plt.subplots(figsize=(10, 6))

    color1 = "tab:blue"
    color2 = "tab:orange"
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Pressure ADC - baseline (12-bit counts)", color="k")
    if len(press_full) > 0:
        t_teensy = pressure_timestamps
        p1 = press_full[:, 0].astype(np.float64) - baseline_mean[0]
        p2 = press_full[:, 1].astype(np.float64) - baseline_mean[1]

        # Display-only smoothing: simple moving average. Saved data stays raw.
        if PLOT_SMOOTH_WINDOW > 1 and p1.size >= PLOT_SMOOTH_WINDOW:
            kernel = np.ones(PLOT_SMOOTH_WINDOW, dtype=np.float64) / PLOT_SMOOTH_WINDOW
            p1 = np.convolve(p1, kernel, mode="same")
            p2 = np.convolve(p2, kernel, mode="same")

        ax1.plot(t_teensy, p1, color=color1, alpha=0.7, label="Sensor 1 (A7, baseline-sub)")
        ax1.plot(t_teensy, p2, color=color2, alpha=0.7, label="Sensor 2 (A5, baseline-sub)")
    ax1.tick_params(axis="y", labelcolor="k")
    ax1.grid(True, linestyle="--", alpha=0.5)

    ax2 = ax1.twinx()
    color3 = "tab:red"
    ax2.set_ylabel("Joint 7 Angle (Degrees)", color=color3)

    angles_fwd_deg = np.degrees(angles_fwd)
    angles_rev_deg = np.degrees(angles_rev)

    t_robot_fwd = np.array(ts_fwd)
    t_start_rev = t_robot_fwd[-1] if len(t_robot_fwd) > 0 else 0
    t_robot_rev = np.array(ts_rev) + t_start_rev

    ax2.plot(
        t_robot_fwd, angles_fwd_deg, color=color3, linewidth=2.5, label="Angle (Fwd)"
    )
    ax2.plot(
        t_robot_rev,
        angles_rev_deg,
        color=color3,
        linewidth=2.5,
        linestyle="--",
        label="Angle (Rev)",
    )
    ax2.tick_params(axis="y", labelcolor=color3)

    lines_1, labels_1 = ax1.get_legend_handles_labels()
    lines_2, labels_2 = ax2.get_legend_handles_labels()
    ax1.legend(lines_1 + lines_2, labels_1 + labels_2, loc="upper left")

    angle_deg = np.degrees(angle)
    plt.title(
        f"Run {run_idx}: Target Angle = {angle_deg:.1f}°, Speed Param = {speed:.3f}"
    )
    plt.tight_layout()

    filepath = PLOTS_DIR / f"run_{run_idx:03d}.png"
    plt.savefig(filepath, dpi=150)
    plt.close(fig)


def execute_wrist_rotation_pair(
    robot: Robot, target_angle_rad: float, speed_val: float, ser: serial.Serial
):
    # ================= 1. TRAJECTORY PLANNING =================
    # Start pose
    start_pose = robot.end_effector_pose.copy()

    rot_vec = np.array([0, 0, target_angle_rad])
    rotation = R.from_rotvec(rot_vec)

    waypoints_forward = generate_linear_waypoints(
        start_position=start_pose.position,
        start_orientation=start_pose.orientation,
        end_position=start_pose.position,
        end_orientation=rotation * start_pose.orientation,
        num_waypoints=10,
    )

    end_forward_pose = Pose(
        position=waypoints_forward[-1].position,
        orientation=waypoints_forward[-1].orientation,
    )

    waypoints_reverse = generate_linear_waypoints(
        start_position=start_pose.position,
        start_orientation=end_forward_pose.orientation,
        end_position=start_pose.position,
        end_orientation=start_pose.orientation,
        num_waypoints=10,
    )

    traj1, traj2 = robot.plan_joint_trajectory_sequence(
        [waypoints_forward, waypoints_reverse],
        [np.abs(target_angle_rad / speed_val), np.abs(target_angle_rad / speed_val)],
    )

    # ================= 2. START DATA COLLECTION =================
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    # Robot state collection
    ts_all = []
    angles_all = []
    positions_all = []
    orientations_all = []
    forces_all = []
    torques_all = []

    # Thread-safe lists for Teensy data
    pressure_list = []

    # Bound the streaming window by the worst-case cycle time so slow rotations
    # don't get clipped: forward + reverse trajectory durations + 5 s margin
    # for action handshake / final flush.
    per_traj_duration_s = abs(target_angle_rad) / max(speed_val, 1e-6)
    teensy_max_duration_s = 2.0 * per_traj_duration_s + 5.0

    def collect_teensy_in_thread():
        """Thread target: collect Teensy data during execution."""
        try:
            pressure_list.append(
                collect_teensy_data_streaming(ser, max_duration=teensy_max_duration_s)
            )
        except Exception as e:
            print(f"  Error in Teensy collection thread: {e}")
            pressure_list.append((np.array([]), np.array([])))

    # Start Teensy collection thread
    teensy_thread = threading.Thread(target=collect_teensy_in_thread)
    teensy_thread.start()

    # ================= 3. EXECUTE IN BACKGROUND =================
    exec_thread = threading.Thread(
        target=robot.execute_sequence,
        args=([traj1, traj2],),
        kwargs={
            "visualize_before_execution": False,
            "settle_time_between_trajectories": 0.0,
        },
    )

    t0 = time.perf_counter()
    exec_thread.start()

    # Collect robot state while executing
    while exec_thread.is_alive():
        try:
            pose = robot.end_effector_pose
            wrench = robot.end_effector_wrench

            ts_all.append(time.perf_counter() - t0)
            angles_all.append(robot.q[6])
            positions_all.append(pose.position.copy())
            orientations_all.append(pose.orientation.as_quat())
            forces_all.append(wrench["force"].copy())
            torques_all.append(wrench["torque"].copy())
        except Exception as e:
            pass

        time.sleep(0.01)

    # ================= 4. STOP RECORDING & WAIT FOR TEENSY =================
    ser.write(b"2")
    time.sleep(0.1)

    # Wait for Teensy collection to complete
    teensy_thread.join(timeout=teensy_max_duration_s)
    if pressure_list:
        pressures_full_cycle, pressure_timestamps_full = pressure_list[0]
    else:
        pressures_full_cycle, pressure_timestamps_full = np.array([]), np.array([])

    time.sleep(SETTLE_SEC)

    # ================= 5. SPLIT TELEMETRY AT PEAK ANGLE =================
    if len(angles_all) > 0:
        peak_idx = np.argmax(np.abs(np.array(angles_all) - angles_all[0]))

        # Split Forward
        ts_fwd = ts_all[: peak_idx + 1]
        angles_fwd = angles_all[: peak_idx + 1]
        positions_fwd = positions_all[: peak_idx + 1]
        orientations_fwd = orientations_all[: peak_idx + 1]
        forces_fwd = forces_all[: peak_idx + 1]
        torques_fwd = torques_all[: peak_idx + 1]

        # Split Reverse
        ts_rev_raw = ts_all[peak_idx + 1 :]
        ts_rev = [t - ts_rev_raw[0] for t in ts_rev_raw] if len(ts_rev_raw) > 0 else []
        angles_rev = angles_all[peak_idx + 1 :]
        positions_rev = positions_all[peak_idx + 1 :]
        orientations_rev = orientations_all[peak_idx + 1 :]
        forces_rev = forces_all[peak_idx + 1 :]
        torques_rev = torques_all[peak_idx + 1 :]
    else:
        ts_fwd, angles_fwd, positions_fwd, orientations_fwd, forces_fwd, torques_fwd = (
            [],
            [],
            [],
            [],
            [],
            [],
        )
        ts_rev, angles_rev, positions_rev, orientations_rev, forces_rev, torques_rev = (
            [],
            [],
            [],
            [],
            [],
            [],
        )

    return (
        ts_fwd,
        angles_fwd,
        positions_fwd,
        orientations_fwd,
        forces_fwd,
        torques_fwd,
        ts_rev,
        angles_rev,
        positions_rev,
        orientations_rev,
        forces_rev,
        torques_rev,
        pressures_full_cycle,
        pressure_timestamps_full,
    )


def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2)
        time.sleep(2)

        # --- NEW: SYNC & FLUSH TEENSY ---
        # If the Teensy was stuck waiting for a stop signal from a previously crashed run,
        # this dummy '2' breaks it out of the loop and clears the pipes.
        ser.write(b"2")
        time.sleep(0.5)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        # --------------------------------

        print(f"Connected to Teensy on {SERIAL_PORT}")
    except Exception as e:
        print(f"Failed to connect to Teensy: {e}")
        return

    robot = Robot(namespace="fr3")
    robot.wait_until_ready(timeout=2.0)
    robot.controller_switcher_client.switch_controller("joint_trajectory_controller")

    # --- NEW: CARTESIAN INITIALIZATION ---
    print("Initializing arm to safe Cartesian starting pose...")
    start_pose = robot.end_effector_pose.copy()
    downward_orientation = R.from_euler("xyz", [-180, 0, 0], degrees=True)
    start_pose.orientation = downward_orientation

    # Move via IK to ensure perfectly clean rotation vectors
    robot.move_to(pose=start_pose, time_to_move=3.0)
    time.sleep(SETTLE_SEC)
    # -------------------------------------

    if not PARAMETERS_FILE.exists():
        print(f"ERROR: Parameters file not found")
        robot.shutdown()
        return

    with open(PARAMETERS_FILE, "rb") as f:
        params = pickle.load(f)
        planned_angles = params["angles"]
        planned_speeds = params["speeds"]
        num_points = params["num_points"]

    RESULTS_FILE.parent.mkdir(parents=True, exist_ok=True)
    PLOTS_DIR.mkdir(parents=True, exist_ok=True)

    start_idx = 0
    if RESULTS_FILE.exists():
        with open(RESULTS_FILE, "rb") as f:
            exp_dict = pickle.load(f)
            start_idx = len(exp_dict["target_angles"])
    else:
        exp_dict = {
            "ts_forward": [],
            "joint7_angles_forward": [],
            "positions_forward": [],
            "orientations_forward": [],
            "forces_forward": [],
            "torques_forward": [],
            "ts_reverse": [],
            "joint7_angles_reverse": [],
            "positions_reverse": [],
            "orientations_reverse": [],
            "forces_reverse": [],
            "torques_reverse": [],
            "pressures_full_cycle": [],
            "pressure_timestamps_full_cycle": [],
            "target_angles": [],
            "target_speeds": [],
            "baseline_pressures": np.array([]),
            "baseline_mean": np.zeros(2, dtype=np.float64),
        }

    if start_idx >= num_points:
        print(f"All {num_points} rotation pairs have already been collected.")
        robot.shutdown()
        return

    # Acquire baseline pressure once at startup (or reuse saved baseline on resume).
    saved_baseline = exp_dict.get("baseline_pressures", np.array([]))
    if isinstance(saved_baseline, np.ndarray) and saved_baseline.size > 0:
        baseline_mean = np.asarray(
            exp_dict.get(
                "baseline_mean", saved_baseline.astype(np.float64).mean(axis=0)
            )
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

    for i in range(start_idx, num_points):
        angle = planned_angles[i]
        speed = planned_speeds[i]

        angle_deg = np.degrees(angle)

        print("\n" + "=" * 60)
        print(
            f"RUN {i + 1} / {num_points}  |  TARGET ANGLE: {angle_deg:.2f}°  |  SPEED PARAM: {speed:.3f}"
        )
        print("=" * 60)

        try:
            robot.move_to(
                pose=Pose(position=START_POSITION, orientation=START_ORIENTATION),
                time_to_move=1.0,
            )
            time.sleep(SETTLE_SEC)

            (
                ts_fwd,
                angles_fwd,
                pos_fwd,
                ori_fwd,
                frc_fwd,
                trq_fwd,
                ts_rev,
                angles_rev,
                pos_rev,
                ori_rev,
                frc_rev,
                trq_rev,
                press_full,
                press_ts_full,
            ) = execute_wrist_rotation_pair(
                robot=robot, target_angle_rad=angle, speed_val=speed, ser=ser
            )

            exp_dict["ts_forward"].append(ts_fwd)
            exp_dict["joint7_angles_forward"].append(angles_fwd)
            exp_dict["positions_forward"].append(pos_fwd)
            exp_dict["orientations_forward"].append(ori_fwd)
            exp_dict["forces_forward"].append(frc_fwd)
            exp_dict["torques_forward"].append(trq_fwd)

            exp_dict["ts_reverse"].append(ts_rev)
            exp_dict["joint7_angles_reverse"].append(angles_rev)
            exp_dict["positions_reverse"].append(pos_rev)
            exp_dict["orientations_reverse"].append(ori_rev)
            exp_dict["forces_reverse"].append(frc_rev)
            exp_dict["torques_reverse"].append(trq_rev)

            exp_dict["pressures_full_cycle"].append(press_full)
            exp_dict.setdefault("pressure_timestamps_full_cycle", []).append(press_ts_full)
            exp_dict["target_angles"].append(angle)
            exp_dict["target_speeds"].append(speed)

            with open(RESULTS_FILE, "wb") as f:
                pickle.dump(exp_dict, f)

            generate_plot(
                i + 1, angle, speed, ts_fwd, angles_fwd, ts_rev, angles_rev, press_full,
                pressure_timestamps=press_ts_full,
                baseline_mean=baseline_mean,
            )
            print(f"\t-> Saved data and plot for Run {i + 1} successfully.")

        except Exception as e:
            print(f"\tERROR: {e}")
            continue

    ser.close()
    robot.shutdown()
    print("\nSequence Complete.")


if __name__ == "__main__":
    main()
