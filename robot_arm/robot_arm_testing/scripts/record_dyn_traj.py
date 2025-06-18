#!/usr/bin/env python3
"""
Franka Research 3 Dynamic Trajectory Recording Script

This script records trajectory data from the robot for dynamics model validation by:
1. Executing a rich multi-sine trajectory to excite robot dynamics
2. Collecting high-frequency sensor data (positions, velocities, torques)
3. Saving the raw data for later analysis

Usage:
    python3 record_dyn_traj.py --trajectory traj1
    python3 record_dyn_traj.py --trajectory validation_30s
"""

import os
import sys
import time
import argparse
import json
from pathlib import Path
from typing import Dict
import numpy as np
from datetime import datetime
from tqdm import tqdm
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from threading import Thread

from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from controller_manager_msgs.srv import ListControllers
from franka_msgs.msg import FrankaRobotState

DEFAULT_CONFIG = {
    "traj1": {
        "duration": 15.0,
        "frequencies": [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "amplitudes": [0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "max_amplitude": 0.4,
        "dt": 0.01,
        "buffer_time": 2.0,
        "description": "Only joint 1",
    },
    "traj2": {
        "duration": 30.0,
        "frequencies": [0.1, 0.15, 0.08, 0.12, 0.18, 0.25, 0.3],
        "amplitudes": [0.3, 0.4, 0.3, 0.5, 0.3, 0.4, 0.3],
        "max_amplitude": 0.4,
        "dt": 0.01,
        "buffer_time": 2.0,
        "description": "Standard validation trajectory - 30 seconds",
    },
    "traj3": {
        "duration": 60.0,
        "frequencies": [0.05, 0.08, 0.04, 0.06, 0.09, 0.12, 0.15],
        "amplitudes": [0.4, 0.5, 0.4, 0.6, 0.4, 0.5, 0.4],
        "max_amplitude": 0.6,
        "dt": 0.01,
        "buffer_time": 3.0,
        "description": "Extended validation trajectory - 60 seconds with larger amplitudes",
    },
    "validation_30s": {
        "duration": 30.0,
        "frequencies": [0.1, 0.15, 0.08, 0.12, 0.18, 0.25, 0.3],
        "amplitudes": [0.3, 0.4, 0.3, 0.5, 0.3, 0.4, 0.3],
        "max_amplitude": 0.4,
        "dt": 0.01,
        "buffer_time": 2.0,
        "description": "Standard 30-second validation trajectory",
    },
    "slow_motion": {
        "duration": 45.0,
        "frequencies": [0.05, 0.07, 0.04, 0.06, 0.08, 0.10, 0.12],
        "amplitudes": [0.5, 0.6, 0.5, 0.7, 0.5, 0.6, 0.5],
        "max_amplitude": 0.7,
        "dt": 0.01,
        "buffer_time": 3.0,
        "description": "Slow motion trajectory with large amplitudes",
    },
    "high_freq": {
        "duration": 20.0,
        "frequencies": [0.2, 0.3, 0.15, 0.25, 0.35, 0.4, 0.45],
        "amplitudes": [0.2, 0.25, 0.2, 0.3, 0.2, 0.25, 0.2],
        "max_amplitude": 0.3,
        "dt": 0.01,
        "buffer_time": 2.0,
        "description": "High frequency trajectory with smaller amplitudes",
    },
}


class TrajectoryRecorderNode(Node):
    """ROS2 node for recording trajectory data from the robot."""

    def __init__(self, trajectory_config: Dict):
        super().__init__("trajectory_recorder_node")
        self.trajectory_config = trajectory_config

        self.home_position = [
            0.0,  # Joint 1
            -np.pi / 4,  # Joint 2
            0.0,  # Joint 3
            -3 * np.pi / 4,  # Joint 4
            0.0,  # Joint 5
            np.pi / 2,  # Joint 6
            np.pi / 4,  # Joint 7
        ]

        # Data storage
        self.recorded_data = {
            "timestamps": [],
            "joint_positions": [],
            "joint_velocities": [],
            "joint_torques": [],
            "external_torques": [],
        }

        # State tracking
        self.current_joint_state = None
        self.current_robot_state = None
        self.data_collection_active = False
        self.trajectory_start_time = None

        # Setup communication
        self._setup_communication()
        self._setup_services()

        self.get_logger().info("Trajectory recorder node initialized")

    def _setup_communication(self):
        """Setup publishers and subscribers."""
        # Check the interface is running
        time.sleep(1.0)  # Allow time for nodes to start
        nodes_alive = self.get_node_names()

        if "fr3_interface" not in nodes_alive:
            self.get_logger().error("'fr3_interface' node not found. Ensure the controller manager is running.")
            sys.exit(1)

        if "franka_robot_state_broadcaster" not in nodes_alive:
            self.get_logger().error(
                "'franka_robot_state_broadcaster' node not found. Ensure the robot state broadcaster is running."
            )
            sys.exit(1)

        # Subscribe to Franka robot state (comprehensive state information)
        self.robot_state_subscription = self.create_subscription(
            FrankaRobotState, "/franka_robot_state_broadcaster/robot_state", self._robot_state_callback, 10
        )

        # Subscribe to joint states for trajectory generation reference
        self.joint_state_subscription = self.create_subscription(
            JointState, "/joint_states", self._joint_state_callback, 10
        )

        # Publisher for trajectory commands
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10
        )

    def _setup_services(self):
        """Setup service clients."""
        self.list_controllers_client = self.create_client(ListControllers, "/controller_manager/list_controllers")

        # Wait for services
        self.get_logger().info("Waiting for controller manager service...")
        while not self.list_controllers_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Controller manager service not available, waiting...")

    def _robot_state_callback(self, msg: FrankaRobotState):
        """Callback to store comprehensive robot state data during collection."""
        self.current_robot_state = msg

        if self.data_collection_active:
            # Calculate time from start of trajectory
            current_ros_time = self.get_clock().now().nanoseconds / 1e9
            if self.trajectory_start_time is None:
                self.trajectory_start_time = current_ros_time
                trajectory_time = 0.0
            else:
                trajectory_time = current_ros_time - self.trajectory_start_time

            # Extract joint data from measured_joint_state
            measured_joint_state = msg.measured_joint_state
            desired_joint_state = msg.desired_joint_state
            tau_ext_filtered = msg.tau_ext_hat_filtered

            # Get FR3 joint data (7 joints)
            fr3_positions = list(measured_joint_state.position[:7])
            fr3_velocities = list(measured_joint_state.velocity[:7])
            fr3_torques = list(measured_joint_state.effort[:7])

            # Get desired joint data
            fr3_positions_desired = list(desired_joint_state.position[:7])
            fr3_velocities_desired = list(desired_joint_state.velocity[:7])
            fr3_torques_desired = list(desired_joint_state.effort[:7])

            # Get external torques
            fr3_tau_ext = list(tau_ext_filtered.effort[:7])

            # Get end-effector pose (measured)
            ee_pose = msg.o_t_ee.pose
            ee_position = [ee_pose.position.x, ee_pose.position.y, ee_pose.position.z]
            ee_orientation = [
                ee_pose.orientation.x,
                ee_pose.orientation.y,
                ee_pose.orientation.z,
                ee_pose.orientation.w,
            ]

            self.recorded_data["timestamps"].append(trajectory_time)
            self.recorded_data["joint_positions"].append(fr3_positions)
            self.recorded_data["joint_velocities"].append(fr3_velocities)
            self.recorded_data["joint_torques"].append(fr3_torques)

            self.recorded_data["joint_positions_desired"].append(fr3_positions_desired)
            self.recorded_data["joint_velocities_desired"].append(fr3_velocities_desired)
            self.recorded_data["joint_torques_desired"].append(fr3_torques_desired)
            self.recorded_data["tau_ext_filtered"].append(fr3_tau_ext)
            self.recorded_data["robot_mode"].append(msg.robot_mode)
            self.recorded_data["ee_position"].append(ee_position)
            self.recorded_data["ee_orientation"].append(ee_orientation)

    def _joint_state_callback(self, msg: JointState):
        """Callback for joint states (used for trajectory generation reference)."""
        self.current_joint_state = msg

    def _external_torque_callback(self, msg: WrenchStamped):
        """Callback for external torque data."""
        if self.data_collection_active:
            # Store external wrench data if needed
            pass

    def generate_trajectory(self) -> JointTrajectory:
        """Generate a rich multi-sine trajectory for exciting robot dynamics."""
        # Wait for current joint state
        while self.current_joint_state is None:
            time.sleep(0.1)
            self.get_logger().info("Waiting for joint states...")

        start_positions = self.home_position

        # Multi-sine trajectory parameters
        frequencies = self.trajectory_config["frequencies"]
        amplitudes = self.trajectory_config["amplitudes"]
        duration = self.trajectory_config["duration"]

        # Safety check: reduce amplitudes if too large
        max_amplitude = self.trajectory_config["max_amplitude"]
        amplitudes = [min(amp, max_amplitude) for amp in amplitudes]

        # Time parameters
        dt = self.trajectory_config["dt"]
        num_points = int(duration / dt)

        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ""
        msg.joint_names = [f"fr3_joint{i + 1}" for i in range(7)]

        self.get_logger().info(f"Generating trajectory with {num_points} points over {duration}s")

        # Start from home position
        point = JointTrajectoryPoint()
        point.positions = start_positions
        point.velocities = [0.0] * 7
        point.accelerations = [0.0] * 7
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 0
        msg.points.append(point)

        # Generate trajectory points
        start_wait_time = 1.0
        for i in range(num_points + 1):
            t = i * dt
            point = JointTrajectoryPoint()

            # Set timing
            point.time_from_start.sec = int(t) + int(start_wait_time)
            point.time_from_start.nanosec = int((t - int(t)) * 1e9)

            # Calculate multi-sine motion for each joint
            positions = []
            velocities = []
            accelerations = []

            for j in range(7):
                pos = start_positions[j] + amplitudes[j] * np.sin(2 * np.pi * frequencies[j] * t)
                vel = amplitudes[j] * 2 * np.pi * frequencies[j] * np.cos(2 * np.pi * frequencies[j] * t)
                acc = -amplitudes[j] * (2 * np.pi * frequencies[j]) ** 2 * np.sin(2 * np.pi * frequencies[j] * t)

                positions.append(pos)
                velocities.append(vel)
                accelerations.append(acc)

            point.positions = positions
            point.velocities = velocities
            point.accelerations = accelerations

            msg.points.append(point)

        return msg

    def record_trajectory(self) -> bool:
        """Execute trajectory and record data."""
        try:
            # Clear previous data - include new fields
            self.recorded_data = {
                "timestamps": [],
                "joint_positions": [],
                "joint_velocities": [],
                "joint_torques": [],
                "joint_positions_desired": [],
                "joint_velocities_desired": [],
                "joint_torques_desired": [],
                "tau_ext_filtered": [],
                "robot_mode": [],
                "ee_position": [],
                "ee_orientation": [],
            }

            # Reset trajectory start time
            self.trajectory_start_time = None

            # Check robot is in correct position
            current_position = self.current_joint_state.position[:7]
            if not np.allclose(current_position, self.home_position, atol=1e-1):
                self.get_logger().error("Robot is not in the home position.")
                return False

            # Generate trajectory
            trajectory = self.generate_trajectory()
            duration = self.trajectory_config["duration"]

            self.get_logger().info(f"Publishing trajectory with {len(trajectory.points)} points")

            # Start data collection
            self.data_collection_active = True

            # Publish trajectory
            self.trajectory_publisher.publish(trajectory)

            # Wait for trajectory execution
            execution_time = duration + self.trajectory_config["buffer_time"]
            self.get_logger().info(f"Recording data for {execution_time}s...")

            for i in tqdm(range(int(execution_time)), desc="Recording trajectory data", unit="seconds"):
                time.sleep(1.0)

            # Stop data collection
            self.data_collection_active = False

            final_data_points = len(self.recorded_data["timestamps"])
            self.get_logger().info(f"Data recording complete. Total points: {final_data_points}")

            return final_data_points > 100

        except Exception as e:
            self.get_logger().error(f"Failed to record trajectory: {e}")
            return False


def get_available_trajectories() -> Dict:
    """Get list of available trajectory configurations."""
    return DEFAULT_CONFIG


def load_trajectory_config(trajectory_name: str) -> Dict:
    """Load specific trajectory configuration."""
    available_trajectories = get_available_trajectories()

    if trajectory_name not in available_trajectories:
        print(f"ERROR: Trajectory '{trajectory_name}' not found.")
        print(f"Available trajectories: {list(available_trajectories.keys())}")
        sys.exit(1)

    return available_trajectories[trajectory_name]


def save_recorded_data(data: Dict, trajectory_name: str, trajectory_config: Dict):
    """Save recorded data to trajectory-specific directory."""
    print("Saving recorded trajectory data...")

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    # Create trajectory-specific output directory
    output_dir = Path(__file__).parent.parent / "results" / "trajectories" / trajectory_name
    output_dir.mkdir(parents=True, exist_ok=True)

    # Save raw data with all available fields
    data_fields = {
        "timestamps": data["timestamps"],
        "joint_positions": data["joint_positions"],
        "joint_velocities": data["joint_velocities"],
        "joint_torques": data["joint_torques"],
        "trajectory_config": trajectory_config,
        "trajectory_name": trajectory_name,
    }

    data_fields.update(
        {
            "joint_positions_desired": data["joint_positions_desired"],
            "joint_velocities_desired": data["joint_velocities_desired"],
            "joint_torques_desired": data["joint_torques_desired"],
            "tau_ext_filtered": data["tau_ext_filtered"],
            "robot_mode": data["robot_mode"],
            "ee_position": data["ee_position"],
            "ee_orientation": data["ee_orientation"],
        }
    )

    data_file = output_dir / f"{trajectory_name}.npz"
    np.savez_compressed(data_file, **data_fields)

    # Save metadata
    metadata_file = output_dir / f"metadata.json"
    metadata = {
        "timestamp": timestamp,
        "trajectory_name": trajectory_name,
        "trajectory_config": trajectory_config,
        "data_points": len(data["timestamps"]),
        "duration": data["timestamps"][-1] - data["timestamps"][0] if data["timestamps"] else 0,
        "data_file": str(data_file.name),
    }

    with open(metadata_file, "w") as f:
        json.dump(metadata, f, indent=2)

    # Generate trajectory plots
    plot_trajectory_data(data, trajectory_name, trajectory_config, output_dir)

    print(f"Data saved to: {data_file}")
    print(f"Metadata saved to: {metadata_file}")

    return data_file


def plot_trajectory_data(data: Dict, trajectory_name: str, trajectory_config: Dict, output_dir: Path):
    """Generate and save plots of the recorded trajectory data."""
    if len(data["timestamps"]) < 10:
        print("Insufficient data for plotting")
        return

    # Convert to numpy arrays for easier plotting
    timestamps = np.array(data["timestamps"])
    positions = np.array(data["joint_positions"])
    velocities = np.array(data["joint_velocities"])
    torques = np.array(data["joint_torques"])

    # Timestamps are already relative to trajectory start, no need to normalize
    # timestamps = timestamps - timestamps[0]  # Remove this line

    # Apply light filtering for cleaner plots
    sampling_freq = 1.0 / np.mean(np.diff(timestamps))
    cutoff_freq = min(20.0, sampling_freq / 4)  # Conservative cutoff
    nyquist_freq = sampling_freq / 2
    normalized_cutoff = cutoff_freq / nyquist_freq

    if normalized_cutoff < 1.0:
        b, a = butter(2, normalized_cutoff, "low")

        positions_filtered = np.zeros_like(positions)
        velocities_filtered = np.zeros_like(velocities)
        torques_filtered = np.zeros_like(torques)

        for i in range(7):
            positions_filtered[:, i] = filtfilt(b, a, positions[:, i])
            velocities_filtered[:, i] = filtfilt(b, a, velocities[:, i])
            torques_filtered[:, i] = filtfilt(b, a, torques[:, i])
    else:
        positions_filtered = positions
        velocities_filtered = velocities
        torques_filtered = torques

    # 1. Joint Positions Plot
    _plot_joint_positions(timestamps, positions_filtered, trajectory_name, trajectory_config, output_dir)

    # 2. Joint Velocities Plot
    _plot_joint_velocities(timestamps, velocities_filtered, trajectory_name, trajectory_config, output_dir)

    # 3. Joint Torques Plot
    _plot_joint_torques(timestamps, torques_filtered, trajectory_name, trajectory_config, output_dir)

    # 4. Combined Overview Plot
    _plot_trajectory_overview(
        timestamps,
        positions_filtered,
        velocities_filtered,
        torques_filtered,
        trajectory_name,
        trajectory_config,
        output_dir,
    )

    _plot_end_effector_position(
        timestamps, np.array(data["ee_position"]), trajectory_name, trajectory_config, output_dir
    )

    print(f"Trajectory plots saved to: {output_dir}")


def _plot_joint_positions(
    timestamps: np.ndarray, positions: np.ndarray, trajectory_name: str, trajectory_config: Dict, output_dir: Path
):
    """Plot joint positions over time."""
    fig, axes = plt.subplots(7, 1, figsize=(12, 14))
    fig.suptitle(f"Joint Positions - {trajectory_name}", fontsize=16)

    for i in range(7):
        ax = axes[i]
        ax.plot(timestamps, positions[:, i], "b-", linewidth=1.5, label=f"Joint {i + 1}")
        ax.set_ylabel(f"J{i + 1}\n(rad)", fontsize=10)
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)

        # Add amplitude info
        amplitude = trajectory_config.get("amplitudes", [0] * 7)[i]
        frequency = trajectory_config.get("frequencies", [0] * 7)[i]
        ax.text(
            0.02,
            0.98,
            f"Amp: {amplitude:.2f}\nFreq: {frequency:.2f} Hz",
            transform=ax.transAxes,
            verticalalignment="top",
            bbox=dict(boxstyle="round", facecolor="lightblue", alpha=0.7),
            fontsize=8,
        )

    axes[-1].set_xlabel("Time (s)", fontsize=12)
    plt.tight_layout()
    plt.savefig(output_dir / f"{trajectory_name}_positions.png", dpi=300, bbox_inches="tight")
    plt.close()


def _plot_joint_velocities(
    timestamps: np.ndarray, velocities: np.ndarray, trajectory_name: str, trajectory_config: Dict, output_dir: Path
):
    """Plot joint velocities over time."""
    fig, axes = plt.subplots(7, 1, figsize=(12, 14))
    fig.suptitle(f"Joint Velocities - {trajectory_name}", fontsize=16)

    for i in range(7):
        ax = axes[i]
        ax.plot(timestamps, velocities[:, i], "g-", linewidth=1.5, label=f"Joint {i + 1}")
        ax.set_ylabel(f"J{i + 1}\n(rad/s)", fontsize=10)
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)

        # Add velocity statistics
        max_vel = np.max(np.abs(velocities[:, i]))
        rms_vel = np.sqrt(np.mean(velocities[:, i] ** 2))
        ax.text(
            0.02,
            0.98,
            f"Max: {max_vel:.2f}\nRMS: {rms_vel:.2f}",
            transform=ax.transAxes,
            verticalalignment="top",
            bbox=dict(boxstyle="round", facecolor="lightgreen", alpha=0.7),
            fontsize=8,
        )

    axes[-1].set_xlabel("Time (s)", fontsize=12)
    plt.tight_layout()
    plt.savefig(output_dir / f"{trajectory_name}_velocities.png", dpi=300, bbox_inches="tight")
    plt.close()


def _plot_joint_torques(
    timestamps: np.ndarray, torques: np.ndarray, trajectory_name: str, trajectory_config: Dict, output_dir: Path
):
    """Plot joint torques over time."""
    fig, axes = plt.subplots(7, 1, figsize=(12, 14))
    fig.suptitle(f"Joint Torques - {trajectory_name}", fontsize=16)

    for i in range(7):
        ax = axes[i]
        ax.plot(timestamps, torques[:, i], "r-", linewidth=1.5, label=f"Joint {i + 1}")
        ax.set_ylabel(f"J{i + 1}\n(Nm)", fontsize=10)
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)

        # Add torque statistics
        max_torque = np.max(np.abs(torques[:, i]))
        rms_torque = np.sqrt(np.mean(torques[:, i] ** 2))
        ax.text(
            0.02,
            0.98,
            f"Max: {max_torque:.2f}\nRMS: {rms_torque:.2f}",
            transform=ax.transAxes,
            verticalalignment="top",
            bbox=dict(boxstyle="round", facecolor="lightcoral", alpha=0.7),
            fontsize=8,
        )

    axes[-1].set_xlabel("Time (s)", fontsize=12)
    plt.tight_layout()
    plt.savefig(output_dir / f"{trajectory_name}_torques.png", dpi=300, bbox_inches="tight")
    plt.close()


def _plot_trajectory_overview(
    timestamps: np.ndarray,
    positions: np.ndarray,
    velocities: np.ndarray,
    torques: np.ndarray,
    trajectory_name: str,
    trajectory_config: Dict,
    output_dir: Path,
):
    """Plot overview with all joint data in subplots."""
    fig = plt.figure(figsize=(18, 12))
    fig.suptitle(f"Trajectory Overview - {trajectory_name}", fontsize=16)

    # Create a 3x7 grid for positions, velocities, and torques
    gs = fig.add_gridspec(3, 7, hspace=0.3, wspace=0.3)

    # Plot positions (top row)
    for i in range(7):
        ax = fig.add_subplot(gs[0, i])
        ax.plot(timestamps, positions[:, i], "b-", linewidth=1)
        ax.set_title(f"J{i + 1} Position", fontsize=10)
        ax.set_ylabel("rad", fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.tick_params(labelsize=8)

    # Plot velocities (middle row)
    for i in range(7):
        ax = fig.add_subplot(gs[1, i])
        ax.plot(timestamps, velocities[:, i], "g-", linewidth=1)
        ax.set_title(f"J{i + 1} Velocity", fontsize=10)
        ax.set_ylabel("rad/s", fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.tick_params(labelsize=8)

    # Plot torques (bottom row)
    for i in range(7):
        ax = fig.add_subplot(gs[2, i])
        ax.plot(timestamps, torques[:, i], "r-", linewidth=1)
        ax.set_title(f"J{i + 1} Torque", fontsize=10)
        ax.set_ylabel("Nm", fontsize=8)
        ax.set_xlabel("Time (s)", fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.tick_params(labelsize=8)

    # Add trajectory info
    duration = trajectory_config.get("duration", 0)
    description = trajectory_config.get("description", "No description")
    fig.text(
        0.02,
        0.02,
        f"Duration: {duration}s | {description}",
        fontsize=10,
        bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.8),
    )

    plt.savefig(output_dir / f"{trajectory_name}_overview.png", dpi=300, bbox_inches="tight")
    plt.close()


def _plot_end_effector_position(
    timestamps: np.ndarray, ee_positions: np.ndarray, trajectory_name: str, trajectory_config: Dict, output_dir: Path
):
    """Plot end-effector position over time."""
    fig, axes = plt.subplots(4, 1, figsize=(12, 10))
    fig.suptitle(f"End-Effector Position - {trajectory_name}", fontsize=16)

    # Plot X, Y, Z positions
    coord_names = ["X", "Y", "Z"]
    colors = ["r", "g", "b"]

    for i in range(3):
        ax = axes[i]
        ax.plot(timestamps, ee_positions[:, i], color=colors[i], linewidth=1.5, label=f"{coord_names[i]} position")
        ax.set_ylabel(f"{coord_names[i]}\n(m)", fontsize=10)
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)

        # Add position statistics
        max_pos = np.max(ee_positions[:, i])
        min_pos = np.min(ee_positions[:, i])
        range_pos = max_pos - min_pos
        ax.text(
            0.02,
            0.98,
            f"Max: {max_pos:.4f}\nMin: {min_pos:.4f}\nRange: {range_pos:.4f}",
            transform=ax.transAxes,
            verticalalignment="top",
            bbox=dict(boxstyle="round", facecolor="lightcyan", alpha=0.7),
            fontsize=8,
        )

    # Plot 3D trajectory path
    ax_3d = axes[3]
    ax_3d.plot(ee_positions[:, 0], ee_positions[:, 1], "k-", linewidth=1, alpha=0.7)
    ax_3d.scatter(ee_positions[0, 0], ee_positions[0, 1], color="green", s=50, label="Start", zorder=5)
    ax_3d.scatter(ee_positions[-1, 0], ee_positions[-1, 1], color="red", s=50, label="End", zorder=5)
    ax_3d.set_xlabel("X (m)", fontsize=10)
    ax_3d.set_ylabel("Y (m)", fontsize=10)
    ax_3d.set_title("End-Effector Path (X-Y view)", fontsize=10)
    ax_3d.grid(True, alpha=0.3)
    ax_3d.legend(fontsize=8)
    ax_3d.axis("equal")

    axes[-1].set_xlabel("Time (s)", fontsize=12)
    plt.tight_layout()
    plt.savefig(output_dir / f"{trajectory_name}_ee_position.png", dpi=300, bbox_inches="tight")
    plt.close()


def main():
    parser = argparse.ArgumentParser(description="Record Franka Research 3 dynamic trajectory data")
    parser.add_argument(
        "--traj",
        type=str,
        help="Trajectory name to execute (e.g., traj1, validation_30s, slow_motion)",
    )
    parser.add_argument("--list-trajectories", action="store_true", help="List available trajectory configurations")

    args = parser.parse_args()

    # List available trajectories if requested
    if args.list_trajectories:
        available_trajectories = get_available_trajectories()
        print("Available trajectory configurations:")
        print("=" * 50)
        for name, config in available_trajectories.items():
            print(f"{name}:")
            print(f"  Duration: {config['duration']}s")
            print(f"  Description: {config.get('description', 'No description')}")
            print(f"  Max Amplitude: {config['max_amplitude']}")
            print()
        return 0

    if not args.traj:
        print("ERROR: No trajectory specified. Use --trajectory to select one.")
        return 1

    # Load trajectory configuration
    trajectory_config = load_trajectory_config(args.traj)

    print("Franka Research 3 Dynamic Trajectory Recorder")
    print("=" * 50)
    print(f"Trajectory: {args.traj}")
    print(f"Description: {trajectory_config.get('description', 'No description')}")
    print(f"Configuration: {json.dumps(trajectory_config, indent=2)}")

    # Initialize ROS2
    rclpy.init()

    try:
        # Create node and executor
        node = TrajectoryRecorderNode(trajectory_config)
        executor = SingleThreadedExecutor()
        executor.add_node(node)

        # Start executor in background thread
        spin_thread = Thread(target=executor.spin)
        spin_thread.daemon = True
        spin_thread.start()

        # Wait for system initialization
        print("Waiting for system initialization...")
        time.sleep(3.0)

        # Record trajectory
        success = node.record_trajectory()

        if not success:
            print("FATAL ERROR: Failed to record trajectory data")
            return 1

        # Save data
        data_file = save_recorded_data(node.recorded_data, args.traj, trajectory_config)

        print(f"\nRECORDING SUMMARY")
        print("=" * 30)
        print(f"Trajectory: {args.traj}")
        print(f"Success: {success}")
        print(f"Data points: {len(node.recorded_data['timestamps'])}")
        print(f"Data file: {data_file}")

        return 0

    finally:
        executor.shutdown()
        rclpy.shutdown()
        if "spin_thread" in locals() and spin_thread.is_alive():
            spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    try:
        sys.exit(main())

    except KeyboardInterrupt:
        print("\nRecording interrupted by user")
        sys.exit(0)

    except Exception as e:
        print(f"ERROR: {e}")
        sys.exit(1)
