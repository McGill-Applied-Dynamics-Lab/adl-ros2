#!/usr/bin/env python3
"""Modernized bend test using trajectory planning and execution with data collection and visualization."""

import time
import numpy as np
from pathlib import Path
import pickle
import threading

import matplotlib

matplotlib.use("Agg")  # Use non-GUI backend to avoid Qt issues
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

from arm_client.robot import Robot, Pose
from arm_client.planning.waypoints import generate_spherical_waypoints
from arm_client import CONFIG_DIR

# --- Configuration ---
SETTLE_SEC = 0.01  # Wait time after moves (s)
NUM_WAYPOINTS = 100  # Number of waypoints for smooth trajectory
BASE_ORI = R.from_euler("xyz", [-180, 0, 0], degrees=True)  # Base downward orientation


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
            print(f"Warning: Error collecting data: {e}")

        time.sleep(dt)

    return data


def collect_data_threaded(robot: Robot, duration: float, freq: float = 100.0) -> dict:
    """
    Wrapper for data collection that can be run in a thread.
    """
    return collect_trajectory_data(robot, duration, freq)


def plot_results(data: dict, expected_waypoints: list = None):
    """
    Plot end-effector positions and torques from collected data.

    Args:
        data: Dictionary containing trajectory data (timestamps, positions, forces, torques)
        expected_waypoints: List of expected Pose objects for comparison
    """
    timestamps = np.array(data["timestamps"])
    ee_positions = np.array(data["ee_positions"])
    ee_forces = np.array(data["ee_forces"])
    ee_torques = np.array(data["ee_torques"])

    fig = plt.figure(figsize=(15, 12))

    # --- Plot 1: End-Effector Position (XYZ) ---
    ax1 = plt.subplot(3, 2, 1)
    ax1.plot(timestamps, ee_positions[:, 0], "r-", label="X (actual)", linewidth=2)
    ax1.plot(timestamps, ee_positions[:, 1], "g-", label="Y (actual)", linewidth=2)
    ax1.plot(timestamps, ee_positions[:, 2], "b-", label="Z (actual)", linewidth=2)

    if expected_waypoints is not None:
        expected_positions = np.array([p.position for p in expected_waypoints])
        ax1.scatter(
            np.linspace(timestamps[0], timestamps[-1], len(expected_positions)),
            expected_positions[:, 0],
            marker="o",
            s=30,
            c="r",
            alpha=0.5,
            label="X (expected)",
        )
        ax1.scatter(
            np.linspace(timestamps[0], timestamps[-1], len(expected_positions)),
            expected_positions[:, 1],
            marker="s",
            s=30,
            c="g",
            alpha=0.5,
            label="Y (expected)",
        )
        ax1.scatter(
            np.linspace(timestamps[0], timestamps[-1], len(expected_positions)),
            expected_positions[:, 2],
            marker="^",
            s=30,
            c="b",
            alpha=0.5,
            label="Z (expected)",
        )

    ax1.set_xlabel("Time (s)", fontsize=11)
    ax1.set_ylabel("Position (m)", fontsize=11)
    ax1.set_title(
        "End-Effector Position: Actual vs Expected", fontsize=12, fontweight="bold"
    )
    ax1.legend(fontsize=9)
    ax1.grid(True, alpha=0.3)

    # --- Plot 2: Position Error ---
    ax2 = plt.subplot(3, 2, 2)
    if expected_waypoints is not None and len(ee_positions) > 0:
        # Interpolate expected positions to match actual timestamps
        expected_positions = np.array([p.position for p in expected_waypoints])
        expected_times = np.linspace(
            timestamps[0], timestamps[-1], len(expected_positions)
        )

        # Interpolate each axis separately
        expected_x = np.interp(timestamps, expected_times, expected_positions[:, 0])
        expected_y = np.interp(timestamps, expected_times, expected_positions[:, 1])
        expected_z = np.interp(timestamps, expected_times, expected_positions[:, 2])
        expected_interp = np.column_stack([expected_x, expected_y, expected_z])

        position_error = ee_positions - expected_interp
        ax2.plot(timestamps, position_error[:, 0], "r-", label="X error", linewidth=2)
        ax2.plot(timestamps, position_error[:, 1], "g-", label="Y error", linewidth=2)
        ax2.plot(timestamps, position_error[:, 2], "b-", label="Z error", linewidth=2)
        ax2.set_title(
            "Position Error: Expected - Actual", fontsize=12, fontweight="bold"
        )
    else:
        ax2.text(0.5, 0.5, "No expected waypoints provided", ha="center", va="center")
        ax2.set_title("Position Error (N/A)", fontsize=12, fontweight="bold")

    ax2.set_xlabel("Time (s)", fontsize=11)
    ax2.set_ylabel("Error (m)", fontsize=11)
    ax2.legend(fontsize=9)
    ax2.grid(True, alpha=0.3)

    # --- Plot 3: Forces (XYZ) ---
    ax3 = plt.subplot(3, 2, 3)
    ax3.plot(timestamps, ee_forces[:, 0], "r-", label="Fx", linewidth=2)
    ax3.plot(timestamps, ee_forces[:, 1], "g-", label="Fy", linewidth=2)
    ax3.plot(timestamps, ee_forces[:, 2], "b-", label="Fz", linewidth=2)
    ax3.set_xlabel("Time (s)", fontsize=11)
    ax3.set_ylabel("Force (N)", fontsize=11)
    ax3.set_title("End-Effector Forces", fontsize=12, fontweight="bold")
    ax3.legend(fontsize=9)
    ax3.grid(True, alpha=0.3)

    # --- Plot 4: Torques (XYZ) ---
    ax4 = plt.subplot(3, 2, 4)
    ax4.plot(timestamps, ee_torques[:, 0], "r-", label="τx", linewidth=2)
    ax4.plot(timestamps, ee_torques[:, 1], "g-", label="τy", linewidth=2)
    ax4.plot(timestamps, ee_torques[:, 2], "b-", label="τz", linewidth=2)
    ax4.set_xlabel("Time (s)", fontsize=11)
    ax4.set_ylabel("Torque (N⋅m)", fontsize=11)
    ax4.set_title("End-Effector Torques", fontsize=12, fontweight="bold")
    ax4.legend(fontsize=9)
    ax4.grid(True, alpha=0.3)

    # --- Plot 5: Force Magnitude ---
    ax5 = plt.subplot(3, 2, 5)
    force_magnitude = np.linalg.norm(ee_forces, axis=1)
    ax5.plot(timestamps, force_magnitude, "purple", linewidth=2, label="|F|")
    ax5.fill_between(timestamps, 0, force_magnitude, alpha=0.3, color="purple")
    ax5.set_xlabel("Time (s)", fontsize=11)
    ax5.set_ylabel("Force Magnitude (N)", fontsize=11)
    ax5.set_title("Total End-Effector Force Magnitude", fontsize=12, fontweight="bold")
    ax5.grid(True, alpha=0.3)

    # --- Plot 6: Torque Magnitude ---
    ax6 = plt.subplot(3, 2, 6)
    torque_magnitude = np.linalg.norm(ee_torques, axis=1)
    ax6.plot(timestamps, torque_magnitude, "orange", linewidth=2, label="|τ|")
    ax6.fill_between(timestamps, 0, torque_magnitude, alpha=0.3, color="orange")
    ax6.set_xlabel("Time (s)", fontsize=11)
    ax6.set_ylabel("Torque Magnitude (N⋅m)", fontsize=11)
    ax6.set_title("Total End-Effector Torque Magnitude", fontsize=12, fontweight="bold")
    ax6.grid(True, alpha=0.3)

    plt.tight_layout()
    return fig


def main():
    """Main execution: Plan and execute spherical arc trajectory with data collection."""
    # Setup robot
    robot = Robot(namespace="fr3")
    robot.wait_until_ready()

    # Switch to joint trajectory controller
    print("Switching to joint_trajectory_controller...")
    robot.controller_switcher_client.switch_controller("joint_trajectory_controller")

    # --- Trajectory Parameters ---
    RADIUS = 0.22  # 15 cm radius
    POLAR_ANGLE_DEG = 45.0  # 45 degrees tilt from vertical
    AZIMUTHAL_ANGLE_DEG = -90.0  # 0 degrees (x-z plane)
    EXECUTION_TIME = 5.0  # Execute trajectory in 5 seconds

    print("-" * 60)
    print("BEND TEST V2 (Modernized with Trajectory Planning)")
    print(
        f"Radius: {RADIUS}m | Polar Angle: {POLAR_ANGLE_DEG}° | Duration: {EXECUTION_TIME}s"
    )
    print("-" * 60)

    try:
        # Get starting position
        start_pose = robot.end_effector_pose.copy()

        # Set orientation to point downward (end-effector down)
        downward_orientation = R.from_euler("xyz", [-180, 0, 0], degrees=True)
        start_pose.orientation = downward_orientation

        print(f"Moving to starting position with downward orientation...")
        robot.move_to(pose=start_pose, time_to_move=2.0)
        time.sleep(SETTLE_SEC)

        print(f"Starting position: {start_pose.position}")

        # Generate forward arc waypoints
        print(f"Generating {NUM_WAYPOINTS} waypoints for forward arc...")
        forward_waypoints = generate_spherical_waypoints(
            start_position=start_pose.position,
            start_orientation=start_pose.orientation,
            radius=RADIUS,
            theta_deg=POLAR_ANGLE_DEG,
            phi_deg=AZIMUTHAL_ANGLE_DEG,
            num_waypoints=NUM_WAYPOINTS,
        )

        # Generate backward arc waypoints
        print(f"Generating {NUM_WAYPOINTS} waypoints for backward arc...")
        backward_waypoints = generate_spherical_waypoints(
            start_position=forward_waypoints[-1].position,
            start_orientation=forward_waypoints[-1].orientation,
            radius=RADIUS,
            theta_deg=-POLAR_ANGLE_DEG,  # Reverse direction
            phi_deg=AZIMUTHAL_ANGLE_DEG,
            num_waypoints=NUM_WAYPOINTS,
        )

        # Plan trajectories
        print("Planning trajectories...")
        traj_forward, traj_backward = robot.plan_joint_trajectory_sequence(
            [forward_waypoints, backward_waypoints],
            [EXECUTION_TIME, EXECUTION_TIME],
        )

        # Prepare data collection
        print("Executing trajectories and collecting data...")

        # Start data collection in background threads
        data_list = []

        def collect_and_store(index):
            data = collect_trajectory_data(
                robot,
                duration=EXECUTION_TIME * 1.2,  # Collect slightly longer than execution
                freq=100.0,
            )
            data_list.append((index, data))

        # Create and start collection threads
        thread1 = threading.Thread(target=collect_and_store, args=(0,))
        thread1.start()

        # Execute trajectories
        robot.execute_sequence(
            [traj_forward, traj_backward],
            visualize_before_execution=True,
            settle_time_between_trajectories=SETTLE_SEC,
        )

        # Wait for data collection to finish
        thread1.join()

        # Extract collected data
        data_forward = data_list[0][1] if len(data_list) > 0 else {}
        data_backward = {}  # Collect separately if needed

        print("✓ Trajectories executed successfully!")

        # Save collected data
        results_dir = Path(__file__).parent / "results"
        results_dir.mkdir(parents=True, exist_ok=True)

        data_file = results_dir / "bend_test_v2_data.pkl"
        with open(data_file, "wb") as f:
            pickle.dump(
                {
                    "forward": data_forward,
                    "backward": data_backward,
                    "waypoints_forward": forward_waypoints,
                    "waypoints_backward": backward_waypoints,
                },
                f,
            )
        print(f"✓ Data saved to {data_file}")

        # Create and save plots
        print("Generating plots...")
        fig_forward = plot_results(data_forward, forward_waypoints)
        plot_file_forward = results_dir / "bend_test_v2_forward.png"
        fig_forward.savefig(plot_file_forward, dpi=150, bbox_inches="tight")
        print(f"✓ Forward trajectory plot saved to {plot_file_forward}")
        plt.close(fig_forward)

        fig_backward = plot_results(data_backward, backward_waypoints)
        plot_file_backward = results_dir / "bend_test_v2_backward.png"
        fig_backward.savefig(plot_file_backward, dpi=150, bbox_inches="tight")
        print(f"✓ Backward trajectory plot saved to {plot_file_backward}")
        plt.close(fig_backward)

        print("✓ All plots saved successfully!")
        # Don't call plt.show() since we're using non-GUI backend

    except KeyboardInterrupt:
        print("\n✗ Test interrupted by user.")
    except Exception as e:
        print(f"\n✗ An error occurred: {e}")
        import traceback

        traceback.print_exc()
    finally:
        # Ensure safe shutdown
        robot.shutdown()
        print("✓ Robot shutdown sequence complete.")


if __name__ == "__main__":
    main()
