import time
import threading
import numpy as np
import serial
import pickle
import matplotlib.pyplot as plt
from pathlib import Path

from scipy.spatial.transform import Rotation as R
from arm_client.robot import Pose, Robot
from arm_client.planning.waypoints import generate_linear_waypoints

# Configuration
SETTLE_SEC = 1.00
START_POSITION = np.array([0.45, -0.045, 0.40])
START_ORIENTATION = R.from_euler("xyz", [-180, 0, 0], degrees=True)

# Teensy Configuration
SERIAL_PORT = "/dev/ttyACM0"  # <--- UPDATE THIS TO YOUR TEENSY PORT
BAUD_RATE = 3000000

# File paths
PROJECT_ROOT = Path(__file__).resolve().parent
PARAMETERS_FILE = PROJECT_ROOT / "results" / "grids" / "rotation_params.pkl"
RESULTS_FILE = PROJECT_ROOT / "results" / "TEST.pkl"
PLOTS_DIR = PROJECT_ROOT / "results" / "plots"


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


def generate_plot(
    run_idx, angle, speed, ts_fwd, angles_fwd, ts_rev, angles_rev, press_full
):
    """Generates and saves a dual-axis plot of pressures and robot angle."""
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

    # ================= 2. START RECORDING =================
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    ser.write(b"1")

    # ================= 3. EXECUTE IN BACKGROUND =================
    ts_all = []
    angles_all = []
    positions_all = []
    orientations_all = []
    forces_all = []
    torques_all = []

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

    # ================= 4. STOP RECORDING & FETCH BINARY =================
    ser.write(b"2")

    print("\tFetching high-speed binary data...")
    pressures_full_cycle = fetch_teensy_dump(ser)

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
            "target_angles": [],
            "target_speeds": [],
        }

    if start_idx >= num_points:
        print(f"All {num_points} rotation pairs have already been collected.")
        robot.shutdown()
        return

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
            exp_dict["target_angles"].append(angle)
            exp_dict["target_speeds"].append(speed)

            with open(RESULTS_FILE, "wb") as f:
                pickle.dump(exp_dict, f)

            generate_plot(
                i + 1, angle, speed, ts_fwd, angles_fwd, ts_rev, angles_rev, press_full
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
