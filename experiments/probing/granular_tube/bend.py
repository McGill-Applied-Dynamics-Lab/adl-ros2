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
from arm_client.planning.waypoints import generate_spherical_waypoints

# Configuration
SETTLE_SEC = 1.00
START_POSITION = np.array([0.493, 0.0455, 0.406])
START_ORIENTATION = R.from_euler("xyz", [-180, 0, 0], degrees=True)

# Bending Settings
RADIUS = 0.225 - 0.05  # Radius of spherical arc (meters)
NUM_WAYPOINTS = 30  # Waypoints per trajectory segment

# Teensy Configuration
SERIAL_PORT = "/dev/ttyACM0"  # <--- UPDATE THIS TO YOUR TEENSY PORT
BAUD_RATE = 3000000

# File paths
PROJECT_ROOT = Path(__file__).resolve().parent
PARAMETERS_FILE = PROJECT_ROOT / "results" / "grids" / "bend_params.pkl"
RESULTS_FILE = PROJECT_ROOT / "results" / "bend_100.pkl"
PLOTS_DIR = PROJECT_ROOT / "results" / "plots" / "bend"


def read_exact_bytes(ser, num_bytes):
    """Helper to ensure we get every single byte requested."""
    data = bytearray()
    while len(data) < num_bytes:
        chunk = ser.read(num_bytes - len(data))
        if chunk:
            data.extend(chunk)
    return bytes(data)


def fetch_teensy_dump(ser):
    """Fetches high-speed raw binary data from the Teensy."""
    while True:
        if ser.in_waiting > 0:
            char = ser.read(1)
            if char == b"S":
                break

    count_bytes = read_exact_bytes(ser, 4)
    num_samples = int.from_bytes(count_bytes, byteorder="little")

    bytes_per_buffer = num_samples * 2

    buf1_raw = read_exact_bytes(ser, bytes_per_buffer)
    buf2_raw = read_exact_bytes(ser, bytes_per_buffer)

    buf1 = np.frombuffer(buf1_raw, dtype=np.uint16)
    buf2 = np.frombuffer(buf2_raw, dtype=np.uint16)

    ser.readline()

    return np.column_stack((buf1, buf2))


def collect_teensy_data_streaming(ser, max_duration=10.0):
    """
    Collect Teensy pressure sensor data via streaming chunks.

    Reads chunks of data as they arrive from the Teensy, avoiding buffer overflow.
    Call ser.write(b"2") to end collection.

    Args:
        ser: Serial connection to Teensy
        max_duration: Maximum collection time (seconds)

    Returns:
        numpy array of shape (N, 2) with pressure readings
    """
    all_samples_1 = []
    all_samples_2 = []
    start_time = time.perf_counter()

    try:
        # Start streaming
        ser.write(b"1")
        time.sleep(0.1)

        while time.perf_counter() - start_time < max_duration:
            if ser.in_waiting > 0:
                marker = ser.read(1)

                if marker == b"B":  # Streaming mode start marker
                    continue

                elif marker == b"C":  # Chunk marker
                    # Read chunk size (4 bytes)
                    chunk_size_bytes = read_exact_bytes(ser, 4)
                    chunk_size = int.from_bytes(chunk_size_bytes, byteorder="little")

                    # Read chunk data for both sensors
                    chunk_data_1 = read_exact_bytes(
                        ser, chunk_size * 2
                    )  # 2 bytes per sample
                    chunk_data_2 = read_exact_bytes(ser, chunk_size * 2)

                    # Convert to uint16 arrays
                    samples_1 = np.frombuffer(chunk_data_1, dtype=np.uint16)
                    samples_2 = np.frombuffer(chunk_data_2, dtype=np.uint16)

                    all_samples_1.extend(samples_1)
                    all_samples_2.extend(samples_2)

                elif marker == b"E":  # End marker
                    # Read total count
                    total_bytes = read_exact_bytes(ser, 4)
                    total_count = int.from_bytes(total_bytes, byteorder="little")
                    print(f"  ✓ Received {total_count} samples from Teensy")
                    break
            else:
                time.sleep(0.001)

        # Combine into 2D array
        if len(all_samples_1) > 0:
            pressures = np.column_stack([all_samples_1, all_samples_2])
            return pressures
        else:
            return np.array([])

    except Exception as e:
        print(f"Error in streaming data collection: {e}")
        return np.array([])


def generate_plot(
    run_idx,
    phi_deg,
    theta_deg,
    angular_speed,
    ts_fwd,
    positions_fwd,
    ts_rev,
    positions_rev,
    press_full,
):
    """Generates and saves a dual-axis plot of pressures and end-effector displacement."""
    fig, ax1 = plt.subplots(figsize=(10, 6))

    t_teensy = np.arange(len(press_full)) / 10000.0

    color1 = "tab:blue"
    color2 = "tab:orange"
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Pressure Sensor ADC (12-bit)", color="k")
    ax1.plot(t_teensy, press_full[:, 0], color=color1, alpha=0.7, label="Sensor 1 (A7)")
    ax1.plot(t_teensy, press_full[:, 1], color=color2, alpha=0.7, label="Sensor 2 (A5)")
    ax1.tick_params(axis="y", labelcolor="k")
    ax1.grid(True, linestyle="--", alpha=0.5)

    ax2 = ax1.twinx()
    color3 = "tab:red"
    ax2.set_ylabel("End-Effector Displacement (m)", color=color3)

    positions_fwd_arr = (
        np.array(positions_fwd)
        if len(positions_fwd) > 0
        else np.array([]).reshape(0, 3)
    )
    positions_rev_arr = (
        np.array(positions_rev)
        if len(positions_rev) > 0
        else np.array([]).reshape(0, 3)
    )

    if len(positions_fwd_arr) > 0:
        origin = positions_fwd_arr[0]
        disp_fwd = np.linalg.norm(positions_fwd_arr - origin, axis=1)
        disp_rev = (
            np.linalg.norm(positions_rev_arr - origin, axis=1)
            if len(positions_rev_arr) > 0
            else np.array([])
        )
    else:
        disp_fwd = np.array([])
        disp_rev = np.array([])

    t_robot_fwd = np.array(ts_fwd)
    t_start_rev = t_robot_fwd[-1] if len(t_robot_fwd) > 0 else 0
    t_robot_rev = np.array(ts_rev) + t_start_rev

    ax2.plot(
        t_robot_fwd, disp_fwd, color=color3, linewidth=2.5, label="Displacement (Fwd)"
    )
    ax2.plot(
        t_robot_rev,
        disp_rev,
        color=color3,
        linewidth=2.5,
        linestyle="--",
        label="Displacement (Rev)",
    )
    ax2.tick_params(axis="y", labelcolor=color3)

    lines_1, labels_1 = ax1.get_legend_handles_labels()
    lines_2, labels_2 = ax2.get_legend_handles_labels()
    ax1.legend(lines_1 + lines_2, labels_1 + labels_2, loc="upper left")

    plt.title(
        f"Run {run_idx}: φ={phi_deg:.1f}°, θ={theta_deg:.1f}°, Speed={angular_speed:.2f}°/s"
    )
    plt.tight_layout()

    filepath = PLOTS_DIR / f"run_{run_idx:03d}.png"
    plt.savefig(filepath, dpi=150)
    plt.close(fig)


def execute_bending_pair(
    robot: Robot,
    phi_deg: float,
    theta_deg: float,
    angular_speed_deg_per_sec: float,
    ser: serial.Serial,
):
    # ================= 1. TRAJECTORY PLANNING =================
    # Start pose
    start_pose = robot.end_effector_pose.copy()

    waypoints_forward = generate_spherical_waypoints(
        start_position=start_pose.position,
        start_orientation=start_pose.orientation,
        radius=RADIUS,
        theta_deg=theta_deg,
        phi_deg=phi_deg,
        num_waypoints=NUM_WAYPOINTS,
    )

    end_forward_pose = Pose(
        position=waypoints_forward[-1].position,
        orientation=waypoints_forward[-1].orientation,
    )

    waypoints_reverse = generate_spherical_waypoints(
        start_position=end_forward_pose.position,
        start_orientation=end_forward_pose.orientation,
        radius=RADIUS,
        theta_deg=-theta_deg,
        phi_deg=phi_deg,
        num_waypoints=NUM_WAYPOINTS,
    )

    execution_time = np.abs(theta_deg / angular_speed_deg_per_sec)

    traj1, traj2 = robot.plan_joint_trajectory_sequence(
        [waypoints_forward, waypoints_reverse],
        [execution_time, execution_time],
    )

    # ================= 2. START DATA COLLECTION =================
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    # Robot state collection
    ts_all = []
    positions_all = []
    orientations_all = []
    forces_all = []
    torques_all = []

    # Thread-safe lists for Teensy data
    pressure_list = []

    def collect_teensy_in_thread():
        """Thread target: collect Teensy data during execution."""
        try:
            pressure = collect_teensy_data_streaming(
                ser, max_duration=execution_time * 2.5 + 2.0
            )
            pressure_list.append(pressure)
        except Exception as e:
            print(f"  Error in Teensy collection thread: {e}")
            pressure_list.append(np.array([]))

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
    teensy_thread.join(timeout=15.0)
    pressures_full_cycle = pressure_list[0] if len(pressure_list) > 0 else np.array([])

    time.sleep(SETTLE_SEC)

    # ================= 5. SPLIT TELEMETRY AT PEAK DISPLACEMENT =================
    if len(positions_all) > 0:
        positions_arr = np.array(positions_all)
        displacements = np.linalg.norm(positions_arr - positions_arr[0], axis=1)
        peak_idx = int(np.argmax(displacements))

        # Split Forward
        ts_fwd = ts_all[: peak_idx + 1]
        positions_fwd = positions_all[: peak_idx + 1]
        orientations_fwd = orientations_all[: peak_idx + 1]
        forces_fwd = forces_all[: peak_idx + 1]
        torques_fwd = torques_all[: peak_idx + 1]

        # Split Reverse
        ts_rev_raw = ts_all[peak_idx + 1 :]
        ts_rev = [t - ts_rev_raw[0] for t in ts_rev_raw] if len(ts_rev_raw) > 0 else []
        positions_rev = positions_all[peak_idx + 1 :]
        orientations_rev = orientations_all[peak_idx + 1 :]
        forces_rev = forces_all[peak_idx + 1 :]
        torques_rev = torques_all[peak_idx + 1 :]
    else:
        ts_fwd, positions_fwd, orientations_fwd, forces_fwd, torques_fwd = (
            [],
            [],
            [],
            [],
            [],
        )
        ts_rev, positions_rev, orientations_rev, forces_rev, torques_rev = (
            [],
            [],
            [],
            [],
            [],
        )

    return (
        ts_fwd,
        positions_fwd,
        orientations_fwd,
        forces_fwd,
        torques_fwd,
        ts_rev,
        positions_rev,
        orientations_rev,
        forces_rev,
        torques_rev,
        pressures_full_cycle,
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

    # Move via IK to ensure perfectly clean starting pose
    robot.move_to(pose=start_pose, time_to_move=3.0)
    time.sleep(SETTLE_SEC)
    # -------------------------------------

    if not PARAMETERS_FILE.exists():
        print(f"ERROR: Parameters file not found")
        robot.shutdown()
        return

    with open(PARAMETERS_FILE, "rb") as f:
        params = pickle.load(f)
        planned_phi = params["phi"]
        planned_theta = params["theta"]
        planned_speeds = params["angular_speed"]
        num_points = params["num_points"]

    RESULTS_FILE.parent.mkdir(parents=True, exist_ok=True)
    PLOTS_DIR.mkdir(parents=True, exist_ok=True)

    start_idx = 0
    if RESULTS_FILE.exists():
        with open(RESULTS_FILE, "rb") as f:
            exp_dict = pickle.load(f)
            start_idx = len(exp_dict["target_phi"])
    else:
        exp_dict = {
            "ts_forward": [],
            "positions_forward": [],
            "orientations_forward": [],
            "forces_forward": [],
            "torques_forward": [],
            "ts_reverse": [],
            "positions_reverse": [],
            "orientations_reverse": [],
            "forces_reverse": [],
            "torques_reverse": [],
            "pressures_full_cycle": [],
            "target_phi": [],
            "target_theta": [],
            "target_speeds": [],
        }

    if start_idx >= num_points:
        print(f"All {num_points} bending pairs have already been collected.")
        robot.shutdown()
        return

    for i in range(start_idx, num_points):
        phi = planned_phi[i]
        theta = planned_theta[i]
        angular_speed = planned_speeds[i]

        print("\n" + "=" * 60)
        print(
            f"RUN {i + 1} / {num_points}  |  φ: {phi:.2f}°  |  θ: {theta:.2f}°  |  SPEED: {angular_speed:.2f}°/s"
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
                pos_fwd,
                ori_fwd,
                frc_fwd,
                trq_fwd,
                ts_rev,
                pos_rev,
                ori_rev,
                frc_rev,
                trq_rev,
                press_full,
            ) = execute_bending_pair(
                robot=robot,
                phi_deg=phi,
                theta_deg=theta,
                angular_speed_deg_per_sec=angular_speed,
                ser=ser,
            )

            exp_dict["ts_forward"].append(ts_fwd)
            exp_dict["positions_forward"].append(pos_fwd)
            exp_dict["orientations_forward"].append(ori_fwd)
            exp_dict["forces_forward"].append(frc_fwd)
            exp_dict["torques_forward"].append(trq_fwd)

            exp_dict["ts_reverse"].append(ts_rev)
            exp_dict["positions_reverse"].append(pos_rev)
            exp_dict["orientations_reverse"].append(ori_rev)
            exp_dict["forces_reverse"].append(frc_rev)
            exp_dict["torques_reverse"].append(trq_rev)

            exp_dict["pressures_full_cycle"].append(press_full)
            exp_dict["target_phi"].append(phi)
            exp_dict["target_theta"].append(theta)
            exp_dict["target_speeds"].append(angular_speed)

            with open(RESULTS_FILE, "wb") as f:
                pickle.dump(exp_dict, f)

            generate_plot(
                i + 1,
                phi,
                theta,
                angular_speed,
                ts_fwd,
                pos_fwd,
                ts_rev,
                pos_rev,
                press_full,
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
