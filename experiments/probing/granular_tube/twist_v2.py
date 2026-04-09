import time
import numpy as np
import serial
from arm_client.robot import Robot
from pathlib import Path
import pickle
import matplotlib.pyplot as plt

# Configuration
SETTLE_SEC = 1.00
# DELAY_SEC removed for continuous sweep
TARGET_TOLERANCE_RAD = np.radians(1.0)  # Break loop when within 1 degree of target

# Teensy Configuration
SERIAL_PORT = "/dev/ttyACM0"  # <--- UPDATE THIS TO YOUR TEENSY PORT
BAUD_RATE = 3000000

# File paths
PROJECT_ROOT = Path(__file__).resolve().parent
PARAMETERS_FILE = PROJECT_ROOT / "results" / "grids" / "rotation_params.pkl"
RESULTS_FILE = PROJECT_ROOT / "results" / "TEST_V2.pkl"
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
    speed_deg = np.degrees(speed)
    plt.title(
        f"Run {run_idx}: Target Angle = {angle_deg:.1f}°, Speed = {speed_deg:.1f}°/s"
    )
    plt.tight_layout()

    filepath = PLOTS_DIR / f"run_{run_idx:03d}.png"
    plt.savefig(filepath, dpi=150)
    plt.close(fig)


def execute_wrist_rotation_pair(
    robot: Robot, target_angle_rad: float, speed_rad_s: float, ser: serial.Serial
):
    q_init = robot.q.copy()
    q_init[6] = np.radians(45.0)
    robot.joint_trajectory_controller_client.send_joint_config(
        joint_names=robot.config.joint_names,
        joint_config=q_init.tolist(),
        time_to_goal=2.0,
        blocking=True,
    )
    time.sleep(SETTLE_SEC)

    start_q = robot.q.copy()
    start_joint7 = start_q[6]

    speed_rad_s = abs(speed_rad_s)
    duration_per_direction = abs(target_angle_rad / speed_rad_s)

    # ================= START RECORDING =================
    ser.reset_input_buffer()
    ser.write(b"1")

    # ================= FORWARD ROTATION =================
    target_joint7_forward = start_joint7 + target_angle_rad
    q_target_forward = start_q.copy()
    q_target_forward[6] = target_joint7_forward

    ts_fwd = []
    angles_fwd = []
    torques_fwd = []

    t0_fwd = time.perf_counter()
    robot.joint_trajectory_controller_client.send_joint_config(
        joint_names=robot.config.joint_names,
        joint_config=q_target_forward.tolist(),
        time_to_goal=duration_per_direction,
        blocking=False,
    )

    while (time.perf_counter() - t0_fwd) < duration_per_direction:
        elapsed = time.perf_counter() - t0_fwd
        current_q = robot.q.copy()
        current_j7 = current_q[6]

        ts_fwd.append(elapsed)
        angles_fwd.append(current_j7)
        torques_fwd.append(robot.end_effector_wrench["torque"].copy())

        # SMART BREAK: If we reached the target early, stop waiting!
        if abs(current_j7 - target_joint7_forward) <= TARGET_TOLERANCE_RAD:
            break

        time.sleep(0.01)

    # ================= REVERSE ROTATION =================
    q_target_reverse = start_q.copy()
    q_target_reverse[6] = start_joint7

    ts_rev = []
    angles_rev = []
    torques_rev = []

    t0_rev = time.perf_counter()
    robot.joint_trajectory_controller_client.send_joint_config(
        joint_names=robot.config.joint_names,
        joint_config=q_target_reverse.tolist(),
        time_to_goal=duration_per_direction,
        blocking=False,
    )

    while (time.perf_counter() - t0_rev) < duration_per_direction:
        elapsed = time.perf_counter() - t0_rev
        current_q = robot.q.copy()
        current_j7 = current_q[6]

        ts_rev.append(elapsed)
        angles_rev.append(current_j7)
        torques_rev.append(robot.end_effector_wrench["torque"].copy())

        # SMART BREAK: Stop waiting when we return home
        if abs(current_j7 - start_joint7) <= TARGET_TOLERANCE_RAD:
            break

        time.sleep(0.01)

    # ================= STOP RECORDING & FETCH BINARY =================
    ser.write(b"2")

    print("\tFetching high-speed binary data...")
    pressures_full_cycle = fetch_teensy_dump(ser)

    time.sleep(SETTLE_SEC)

    return (
        ts_fwd,
        angles_fwd,
        torques_fwd,
        ts_rev,
        angles_rev,
        torques_rev,
        pressures_full_cycle,
    )


def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2)
        time.sleep(2)
        print(f"Connected to Teensy on {SERIAL_PORT}")
    except Exception as e:
        print(f"Failed to connect to Teensy: {e}")
        return

    robot = Robot(namespace="fr3")
    robot.wait_until_ready(timeout=2.0)
    robot.controller_switcher_client.switch_controller("joint_trajectory_controller")

    print("Initializing joint 7 to 45 degrees...")
    q_init = robot.q.copy()
    q_init[6] = np.radians(45.0)
    robot.joint_trajectory_controller_client.send_joint_config(
        joint_names=robot.config.joint_names,
        joint_config=q_init.tolist(),
        time_to_goal=3.0,
        blocking=True,
    )

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
            "torques_forward": [],
            "ts_reverse": [],
            "joint7_angles_reverse": [],
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
        speed_deg_s = np.degrees(speed)

        print("\n" + "=" * 60)
        print(
            f"RUN {i + 1} / {num_points}  |  TARGET ANGLE: {angle_deg:.2f}°  |  SPEED: {speed_deg_s:.2f}°/s"
        )
        print("=" * 60)

        try:
            (
                ts_fwd,
                angles_fwd,
                torques_fwd,
                ts_rev,
                angles_rev,
                torques_rev,
                press_full,
            ) = execute_wrist_rotation_pair(
                robot=robot, target_angle_rad=angle, speed_rad_s=speed, ser=ser
            )

            exp_dict["ts_forward"].append(ts_fwd)
            exp_dict["joint7_angles_forward"].append(angles_fwd)
            exp_dict["torques_forward"].append(torques_fwd)

            exp_dict["ts_reverse"].append(ts_rev)
            exp_dict["joint7_angles_reverse"].append(angles_rev)
            exp_dict["torques_reverse"].append(torques_rev)

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
