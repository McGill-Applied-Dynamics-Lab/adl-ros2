#!/usr/bin/env python3
"""
Robust bending experiment runner with data logging and Teensy sensor integration.

This script cycles through random bending parameters (phi, theta, speeds),
logs robot data and serial sensor data, and supports restart from the last
completed experiment after crashes.
"""

import time
import threading
import numpy as np
import serial
import pickle
import matplotlib.pyplot as plt
from pathlib import Path

from scipy.spatial.transform import Rotation as R
from arm_client.robot import Pose, Robot
from arm_client.planning.waypoints import generate_spherical_waypoints

# ============================================================================
# CONFIGURATION
# ============================================================================

# Robot Settings
SETTLE_SEC = 1.0
START_POSITION = np.array([0.45, -0.045, 0.40])
START_ORIENTATION = R.from_euler("xyz", [-180, 0, 0], degrees=True)

# Bending Settings
RADIUS = 0.20  # Radius of spherical arc (meters)
NUM_WAYPOINTS = 20  # Waypoints per trajectory segment

# Teensy Configuration
SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 3000000

# File paths
PROJECT_ROOT = Path(__file__).resolve().parent
PARAMETERS_FILE = PROJECT_ROOT / "results" / "grids" / "bend_params.pkl"
RESULTS_FILE = PROJECT_ROOT / "results" / "bend_experiments.pkl"
PLOTS_DIR = PROJECT_ROOT / "results" / "plots" / "bend"

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================


def calculate_execution_time(
    theta_deg: float, angular_speed_deg_per_sec: float
) -> float:
    """
    Calculate execution time for a trajectory given polar angle and angular speed.

    Args:
        theta_deg: Polar angle (degrees)
        angular_speed_deg_per_sec: Angular speed (degrees/second)

    Returns:
        Execution time in seconds
    """
    if angular_speed_deg_per_sec <= 0:
        raise ValueError("Angular speed must be positive")

    execution_time = theta_deg / angular_speed_deg_per_sec
    return execution_time


# ============================================================================
# SERIAL COMMUNICATION HELPERS
# ============================================================================


def read_exact_bytes(ser, num_bytes):
    """Helper to ensure we get every single byte requested."""
    data = bytearray()
    while len(data) < num_bytes:
        chunk = ser.read(num_bytes - len(data))
        if chunk:
            data.extend(chunk)
    return bytes(data)


def fetch_teensy_dump(ser, timeout_sec=10.0):
    """Fetches high-speed raw binary data from the Teensy with timeout."""
    start_time = time.perf_counter()

    # Wait for start marker 'S'
    while time.perf_counter() - start_time < timeout_sec:
        if ser.in_waiting > 0:
            char = ser.read(1)
            if char == b"S":
                break
    else:
        raise TimeoutError("Timeout waiting for Teensy start marker")

    # Read sample count
    count_bytes = read_exact_bytes(ser, 4)
    num_samples = int.from_bytes(count_bytes, byteorder="little")

    bytes_per_buffer = num_samples * 2

    # Read both pressure buffers
    buf1_raw = read_exact_bytes(ser, bytes_per_buffer)
    buf2_raw = read_exact_bytes(ser, bytes_per_buffer)

    buf1 = np.frombuffer(buf1_raw, dtype=np.uint16)
    buf2 = np.frombuffer(buf2_raw, dtype=np.uint16)

    # Read end marker
    ser.readline()

    return np.column_stack((buf1, buf2))


# ============================================================================
# DATA COLLECTION
# ============================================================================


def collect_trajectory_data(robot: Robot, duration: float, freq: float = 100.0) -> dict:
    """
    Collect end-effector pose and wrench data during trajectory execution.

    Args:
        robot: Robot object
        duration: Duration to collect data (s)
        freq: Collection frequency (Hz)

    Returns:
        Dictionary containing timestamps, poses, and wrenches
    """
    dt = 1.0 / freq
    start_time = time.perf_counter()
    data = {
        "timestamps": [],
        "ee_positions": [],
        "ee_orientations": [],
        "ee_forces": [],
        "ee_torques": [],
    }

    while time.perf_counter() - start_time < duration:
        try:
            pose = robot.end_effector_pose
            wrench = robot.end_effector_wrench

            t = time.perf_counter() - start_time
            data["timestamps"].append(t)
            data["ee_positions"].append(pose.position.copy())
            data["ee_orientations"].append(pose.orientation.as_quat())
            data["ee_forces"].append(wrench["force"].copy())
            data["ee_torques"].append(wrench["torque"].copy())
        except Exception as e:
            print(f"Warning: Error collecting robot data: {e}")

        time.sleep(dt)

    return data


def collect_teensy_data(ser: serial.Serial) -> np.ndarray:
    """
    Trigger and collect Teensy pressure sensor data.

    Args:
        ser: Serial connection to Teensy

    Returns:
        numpy array of pressure data
    """
    try:
        # Trigger sampling
        ser.write(b"1")
        time.sleep(0.1)

        # Dump data
        pressures = fetch_teensy_dump(ser, timeout_sec=5.0)
        return pressures
    except Exception as e:
        print(f"Error collecting Teensy data: {e}")
        return np.array([])


# ============================================================================
# TRAJECTORY EXECUTION
# ============================================================================


def execute_bending_pair(
    robot: Robot,
    phi_deg: float,
    theta_deg: float,
    angular_speed_deg_per_sec: float,
    ser: serial.Serial,
) -> tuple:
    """
    Execute a spherical bending arc trajectory pair (forward + reverse) with data collection.

    Args:
        robot: Robot client
        phi_deg: Azimuthal angle (degrees)
        theta_deg: Polar angle (degrees)
        angular_speed_deg_per_sec: Angular speed (degrees/second)
        ser: Serial connection to Teensy

    Returns:
        Tuple of (robot_data_forward, robot_data_backward, pressure_data)
    """
    # Calculate execution time from angular speed and theta
    execution_time = calculate_execution_time(theta_deg, angular_speed_deg_per_sec)

    print(
        f"\n  Planning forward arc (φ={phi_deg:.1f}°, θ={theta_deg:.1f}°, {angular_speed_deg_per_sec:.1f}°/s → {execution_time:.2f}s)..."
    )

    # Get current pose
    start_pose = robot.end_effector_pose.copy()

    # Generate forward arc waypoints
    forward_waypoints = generate_spherical_waypoints(
        start_position=start_pose.position,
        start_orientation=start_pose.orientation,
        radius=RADIUS,
        theta_deg=theta_deg,
        phi_deg=phi_deg,
        num_waypoints=NUM_WAYPOINTS,
    )

    end_pose_forward = Pose(
        position=forward_waypoints[-1].position,
        orientation=forward_waypoints[-1].orientation,
    )

    # Generate reverse arc waypoints
    print(f"  Planning reverse arc...")
    reverse_waypoints = generate_spherical_waypoints(
        start_position=end_pose_forward.position,
        start_orientation=end_pose_forward.orientation,
        radius=RADIUS,
        theta_deg=-theta_deg,  # Reverse
        phi_deg=phi_deg,
        num_waypoints=NUM_WAYPOINTS,
    )

    # Plan trajectories
    print(f"  Planning joint trajectories...")
    traj_forward, traj_reverse = robot.plan_joint_trajectory_sequence(
        [forward_waypoints, reverse_waypoints],
        [execution_time, execution_time],
    )

    # Prepare data collection threads
    data_list = []

    def collect_robot_data(label):
        """Thread target: collect robot data during execution."""
        data = collect_trajectory_data(
            robot,
            duration=execution_time * 2.2,  # Collect slightly longer than execution
            freq=100.0,
        )
        data_list.append((label, data))

    # Start data collection thread
    collection_thread = threading.Thread(target=collect_robot_data, args=("robot",))
    collection_thread.start()

    # Execute trajectories
    print(f"  Executing trajectories...")
    robot.execute_sequence(
        [traj_forward, traj_reverse],
        visualize_before_execution=False,
        settle_time_between_trajectories=SETTLE_SEC,
    )

    # Collect Teensy data
    print(f"  Collecting Teensy pressure data...")
    pressure_data = collect_teensy_data(ser)

    # Wait for robot data collection to complete
    collection_thread.join()

    # Extract robot data
    robot_data = data_list[0][1] if len(data_list) > 0 else {}

    return robot_data, pressure_data


# ============================================================================
# PLOTTING
# ============================================================================


def generate_plot(
    run_idx: int,
    phi_deg: float,
    theta_deg: float,
    speed: float,
    robot_data: dict,
    pressure_data: np.ndarray,
) -> None:
    """Generate and save a plot of bending experiment data."""
    if len(robot_data) == 0 or len(pressure_data) == 0:
        print(f"  Skipping plot (missing data)")
        return

    fig, ax1 = plt.subplots(figsize=(12, 6))

    # Plot pressure data
    t_teensy = np.arange(len(pressure_data)) / 10000.0  # Assumes 10 kHz sampling

    color1 = "tab:blue"
    color2 = "tab:orange"
    ax1.set_xlabel("Time (s)", fontsize=11)
    ax1.set_ylabel("Pressure Sensor ADC (12-bit)", color="k", fontsize=11)
    ax1.plot(
        t_teensy, pressure_data[:, 0], color=color1, alpha=0.7, label="Sensor 1 (A7)"
    )
    ax1.plot(
        t_teensy, pressure_data[:, 1], color=color2, alpha=0.7, label="Sensor 2 (A5)"
    )
    ax1.tick_params(axis="y", labelcolor="k")
    ax1.grid(True, linestyle="--", alpha=0.5)

    # Plot robot position
    ax2 = ax1.twinx()
    if len(robot_data["ee_positions"]) > 0:
        positions = np.array(robot_data["ee_positions"])
        t_robot = np.array(robot_data["timestamps"])

        color3 = "tab:red"
        ax2.set_ylabel("End-Effector Z Position (m)", color=color3, fontsize=11)
        ax2.plot(
            t_robot, positions[:, 2], color=color3, linewidth=2.5, label="Z Position"
        )
        ax2.tick_params(axis="y", labelcolor=color3)

    lines_1, labels_1 = ax1.get_legend_handles_labels()
    lines_2, labels_2 = ax2.get_legend_handles_labels()
    ax1.legend(lines_1 + lines_2, labels_1 + labels_2, loc="upper left", fontsize=9)

    plt.title(
        f"Run {run_idx}: φ={phi_deg:.1f}°, θ={theta_deg:.1f}°, Speed={speed:.2f}s",
        fontsize=12,
        fontweight="bold",
    )
    plt.tight_layout()

    filepath = PLOTS_DIR / f"run_{run_idx:03d}.png"
    plt.savefig(filepath, dpi=150)
    plt.close(fig)
    print(f"  Saved plot to {filepath.name}")


# ============================================================================
# MAIN EXECUTION
# ============================================================================


def main():
    """Main execution loop with crash recovery."""
    print("\n" + "=" * 70)
    print("BENDING EXPERIMENT RUNNER")
    print("=" * 70)

    # ========== TEENSY SETUP ==========
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2)
        time.sleep(2)

        # Sync Teensy: send dummy '2' to break out of any stuck state
        ser.write(b"2")
        time.sleep(0.5)
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        print(f"✓ Connected to Teensy on {SERIAL_PORT}")
    except Exception as e:
        print(f"✗ Failed to connect to Teensy: {e}")
        return

    # ========== ROBOT SETUP ==========
    try:
        robot = Robot(namespace="fr3")
        robot.wait_until_ready(timeout=5.0)
        robot.controller_switcher_client.switch_controller(
            "joint_trajectory_controller"
        )
        print(f"✓ Robot ready (joint_trajectory_controller)")
    except Exception as e:
        print(f"✗ Failed to initialize robot: {e}")
        ser.close()
        return

    # Move to safe starting pose
    try:
        print("Moving to starting position...")
        start_pose = robot.end_effector_pose.copy()
        start_pose.orientation = START_ORIENTATION
        robot.move_to(pose=start_pose, time_to_move=3.0)
        time.sleep(SETTLE_SEC)
        print("✓ Ready for experiments")
    except Exception as e:
        print(f"✗ Failed to reach starting position: {e}")
        robot.shutdown()
        ser.close()
        return

    # ========== LOAD PARAMETERS ==========
    if not PARAMETERS_FILE.exists():
        print(f"✗ Parameters file not found: {PARAMETERS_FILE}")
        print(f"  Run: python generate_bend_parameters.py")
        robot.shutdown()
        ser.close()
        return

    with open(PARAMETERS_FILE, "rb") as f:
        params = pickle.load(f)
        phi_values = params["phi"]
        theta_values = params["theta"]
        angular_speed_values = params["angular_speed"]
        num_points = params["num_points"]

    print(f"✓ Loaded parameters for {num_points} experiments")

    # ========== CRASH RECOVERY ==========
    RESULTS_FILE.parent.mkdir(parents=True, exist_ok=True)
    PLOTS_DIR.mkdir(parents=True, exist_ok=True)

    start_idx = 0
    if RESULTS_FILE.exists():
        with open(RESULTS_FILE, "rb") as f:
            exp_dict = pickle.load(f)
            start_idx = len(exp_dict["phi_executed"])
            print(
                f"✓ Resuming from Run {start_idx + 1} (found {start_idx} completed runs)"
            )
    else:
        exp_dict = {
            "phi_executed": [],
            "theta_executed": [],
            "speed_executed": [],
            "robot_data": [],
            "pressure_data": [],
        }

    if start_idx >= num_points:
        print(f"✓ All {num_points} experiments already completed!")
        robot.shutdown()
        ser.close()
        return

    # ========== MAIN LOOP ==========
    print("\n" + "=" * 70)
    print(f"STARTING EXPERIMENTS ({start_idx + 1}/{num_points})")
    print("=" * 70)

    for i in range(start_idx, num_points):
        phi = phi_values[i]
        theta = theta_values[i]
        angular_speed = angular_speed_values[i]
        execution_time = calculate_execution_time(theta, angular_speed)

        print(
            f"\n[RUN {i + 1}/{num_points}] φ={phi:.1f}°, θ={theta:.1f}°, Speed={angular_speed:.1f}°/s (exec_time={execution_time:.2f}s)"
        )
        print("-" * 70)

        try:
            # Return to starting position
            robot.move_to(
                pose=Pose(position=START_POSITION, orientation=START_ORIENTATION),
                time_to_move=2.0,
            )
            time.sleep(SETTLE_SEC)

            # Execute bending pair
            robot_data, pressure_data = execute_bending_pair(
                robot=robot,
                phi_deg=phi,
                theta_deg=theta,
                angular_speed_deg_per_sec=angular_speed,
                ser=ser,
            )

            # Store results
            exp_dict["phi_executed"].append(phi)
            exp_dict["theta_executed"].append(theta)
            exp_dict["speed_executed"].append(angular_speed)
            exp_dict["robot_data"].append(robot_data)
            exp_dict["pressure_data"].append(pressure_data)

            # Save to disk (for crash recovery)
            with open(RESULTS_FILE, "wb") as f:
                pickle.dump(exp_dict, f)

            # Generate plot
            generate_plot(
                run_idx=i + 1,
                phi_deg=phi,
                theta_deg=theta,
                speed=angular_speed,
                robot_data=robot_data,
                pressure_data=pressure_data,
            )

            print(f"✓ Run {i + 1} completed and saved")

        except Exception as e:
            print(f"✗ Run {i + 1} failed: {e}")
            import traceback

            traceback.print_exc()
            print(f"  Will resume from this run on next execution")
            continue

    # ========== CLEANUP ==========
    print("\n" + "=" * 70)
    print(f"✓ ALL {num_points} EXPERIMENTS COMPLETED")
    print("=" * 70)
    print(f"Results saved to: {RESULTS_FILE}")
    print(f"Plots saved to: {PLOTS_DIR}")

    ser.close()
    robot.shutdown()


if __name__ == "__main__":
    main()
