# %%
import matplotlib.pyplot as plt
import numpy as np
import sys
import time
from datetime import datetime
from pathlib import Path
from dataclasses import dataclass, asdict
from typing import Dict, List, Any, Optional


# Check for required dependencies
def check_dependencies():
    """Check if all required dependencies are available."""
    missing_deps = []

    try:
        import pandas as pd
    except ImportError:
        missing_deps.append("pandas (install with: pip install pandas)")

    try:
        import yaml
    except ImportError:
        missing_deps.append("pyyaml (install with: pip install pyyaml)")

    # Test parquet support
    try:
        import pandas as pd

        df = pd.DataFrame({"test": [1]})
        df.to_parquet("/tmp/dep_test.parquet")
        import os

        os.remove("/tmp/dep_test.parquet")
    except Exception:
        missing_deps.append("pyarrow (install with: pip install pyarrow)")

    if missing_deps:
        print("❌ Missing required dependencies:")
        for dep in missing_deps:
            print(f"  - {dep}")
        print("\nPlease install the missing dependencies and try again.")
        sys.exit(1)
    else:
        print("✅ All dependencies available")


# Run dependency check
check_dependencies()

# Import after dependency check
import pandas as pd
import yaml

from arm_client.robot import Robot
from arm_client import CONFIG_DIR


@dataclass
class ExperimentData:
    """Data structure to store all experiment measurements."""

    timestamps: List[float]
    target_positions: List[np.ndarray]  # [x, y, z]
    actual_positions: List[np.ndarray]  # [x, y, z]
    target_orientations: List[np.ndarray]  # quaternions [x, y, z, w]
    actual_orientations: List[np.ndarray]  # quaternions [x, y, z, w]
    joint_positions: List[np.ndarray]  # 7 joint values
    joint_velocities: List[np.ndarray]  # 7 joint velocities
    joint_torques: List[np.ndarray]  # 7 joint torques

    def to_dataframe(self) -> pd.DataFrame:
        """Convert experiment data to pandas DataFrame for easy analysis."""
        data = {
            "timestamp": self.timestamps,
            "target_x": [pos[0] for pos in self.target_positions],
            "target_y": [pos[1] for pos in self.target_positions],
            "target_z": [pos[2] for pos in self.target_positions],
            "actual_x": [pos[0] for pos in self.actual_positions],
            "actual_y": [pos[1] for pos in self.actual_positions],
            "actual_z": [pos[2] for pos in self.actual_positions],
        }

        # Add joint positions
        for i in range(7):
            data[f"joint_pos_{i + 1}"] = [joints[i] for joints in self.joint_positions]
            data[f"joint_vel_{i + 1}"] = [joints[i] for joints in self.joint_velocities]
            data[f"joint_tau_{i + 1}"] = [joints[i] for joints in self.joint_torques]

        # Add quaternions
        for i, axis in enumerate(["qx", "qy", "qz", "qw"]):
            data[f"target_{axis}"] = [quat[i] for quat in self.target_orientations]
            data[f"actual_{axis}"] = [quat[i] for quat in self.actual_orientations]

        return pd.DataFrame(data)


@dataclass
class ExperimentMetadata:
    """Metadata about the experiment configuration and results."""

    experiment_name: str
    timestamp: str
    trajectory_type: str
    controller_name: str
    controller_parameters: Dict[str, Any]
    trajectory_parameters: Dict[str, Any]
    duration: float
    frequency: float
    total_points: int


class ExperimentManager:
    """Manages experiment execution, data collection, and result saving."""

    def __init__(self, experiment_name: Optional[str] = None):
        """Initialize experiment manager.

        Args:
            experiment_name: Name for the experiment. If None, auto-generated from timestamp.
        """
        if experiment_name is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            experiment_name = f"sin_traj_{timestamp}"

        self.experiment_name = experiment_name
        self.results_dir = Path("experiments/benchmarks/results") / experiment_name
        self.data = ExperimentData([], [], [], [], [], [], [], [])
        self.metadata = None

    def setup_experiment_directory(self):
        """Create directory structure for experiment results."""
        self.results_dir.mkdir(parents=True, exist_ok=True)
        print(f"Experiment directory created: {self.results_dir}")

    def collect_controller_metadata(self, robot: Robot, trajectory_params: Dict[str, Any]) -> ExperimentMetadata:
        """Collect experiment metadata including controller parameters."""
        try:
            # Get controller parameters
            ctrl_params = robot.haptic_controller_parameters_client.list_parameters()
            params_values = robot.haptic_controller_parameters_client.get_parameters(ctrl_params)
            controller_parameters = dict(zip(ctrl_params, params_values))
        except Exception as e:
            print(f"Warning: Could not retrieve controller parameters: {e}")
            controller_parameters = {}

        metadata = ExperimentMetadata(
            experiment_name=self.experiment_name,
            timestamp=datetime.now().isoformat(),
            trajectory_type="sinusoidal_z",
            controller_name="haptic_controller",
            controller_parameters=controller_parameters,
            trajectory_parameters=trajectory_params,
            duration=0.0,  # Will be updated after experiment
            frequency=0.0,  # Will be updated after experiment
            total_points=0,  # Will be updated after experiment
        )
        return metadata

    def collect_data_point(self, robot: Robot, target_pose, timestamp: float):
        """Collect a single data point during trajectory execution."""
        current_pose = robot.end_effector_pose

        # Positions
        self.data.target_positions.append(target_pose.position.copy())
        self.data.actual_positions.append(current_pose.position.copy())

        # Orientations (as quaternions)
        self.data.target_orientations.append(target_pose.orientation.as_quat().copy())
        self.data.actual_orientations.append(current_pose.orientation.as_quat().copy())

        # Joint states
        self.data.joint_positions.append(robot.q.copy())
        self.data.joint_velocities.append(robot.dq.copy())
        self.data.joint_torques.append(robot.tau.copy())

        # Timestamp
        self.data.timestamps.append(timestamp)

    def calculate_metrics(self) -> Dict[str, Any]:
        """Calculate tracking performance metrics."""
        if not self.data.timestamps:
            return {}

        # Convert to numpy arrays for easier computation
        target_pos = np.array(self.data.target_positions)
        actual_pos = np.array(self.data.actual_positions)
        timestamps = np.array(self.data.timestamps)

        # Position tracking errors
        position_errors = actual_pos - target_pos
        position_errors_3d = np.linalg.norm(position_errors, axis=1)

        # Timing analysis
        if len(timestamps) > 1:
            dt_actual = np.diff(timestamps)
            freq_actual = 1.0 / dt_actual
            mean_frequency = np.mean(freq_actual)
            freq_std = np.std(freq_actual)
        else:
            mean_frequency = 0.0
            freq_std = 0.0

        metrics = {
            "tracking_errors": {
                "mean_error_x_mm": float(np.mean(np.abs(position_errors[:, 0])) * 1000),
                "mean_error_y_mm": float(np.mean(np.abs(position_errors[:, 1])) * 1000),
                "mean_error_z_mm": float(np.mean(np.abs(position_errors[:, 2])) * 1000),
                "mean_error_3d_mm": float(np.mean(position_errors_3d) * 1000),
                "max_error_x_mm": float(np.max(np.abs(position_errors[:, 0])) * 1000),
                "max_error_y_mm": float(np.max(np.abs(position_errors[:, 1])) * 1000),
                "max_error_z_mm": float(np.max(np.abs(position_errors[:, 2])) * 1000),
                "max_error_3d_mm": float(np.max(position_errors_3d) * 1000),
                "rms_error_x_mm": float(np.sqrt(np.mean(position_errors[:, 0] ** 2)) * 1000),
                "rms_error_y_mm": float(np.sqrt(np.mean(position_errors[:, 1] ** 2)) * 1000),
                "rms_error_z_mm": float(np.sqrt(np.mean(position_errors[:, 2] ** 2)) * 1000),
                "rms_error_3d_mm": float(np.sqrt(np.mean(position_errors_3d**2)) * 1000),
            },
            "execution_performance": {
                "mean_frequency_hz": float(mean_frequency),
                "frequency_std_hz": float(freq_std),
                "total_duration_s": float(timestamps[-1] - timestamps[0]) if len(timestamps) > 1 else 0.0,
                "total_points": len(timestamps),
                "frequency_consistency_percent": float((1 - freq_std / mean_frequency) * 100)
                if mean_frequency > 0
                else 0.0,
            },
        }

        return metrics

    def create_plots(self):
        """Create all required plots for the experiment."""
        if not self.data.timestamps:
            print("No data to plot")
            return

        timestamps = np.array(self.data.timestamps)
        target_pos = np.array(self.data.target_positions)
        actual_pos = np.array(self.data.actual_positions)
        joint_pos = np.array(self.data.joint_positions)
        joint_vel = np.array(self.data.joint_velocities)
        joint_tau = np.array(self.data.joint_torques)

        # Plot 1: Position tracking (4x1)
        self._plot_position_tracking(timestamps, target_pos, actual_pos)

        # Plot 2: Joint torques (7x1)
        self._plot_joint_data(timestamps, joint_tau, "Joint Torques", "Torque (Nm)", "joint_torques.png")

        # Plot 3: Joint positions (7x1)
        self._plot_joint_data(timestamps, joint_pos, "Joint Positions", "Position (rad)", "joint_positions.png")

        # Plot 4: Joint velocities (7x1)
        self._plot_joint_data(timestamps, joint_vel, "Joint Velocities", "Velocity (rad/s)", "joint_velocities.png")

    def _plot_position_tracking(self, timestamps, target_pos, actual_pos):
        """Create 4x1 position tracking plots."""
        fig, axes = plt.subplots(4, 1, figsize=(12, 16))

        position_errors = actual_pos - target_pos
        axes_labels = ["X", "Y", "Z"]

        # Position tracking for each axis
        for i in range(3):
            axes[i].plot(timestamps, target_pos[:, i], "b-", label=f"Target {axes_labels[i]}", linewidth=2)
            axes[i].plot(timestamps, actual_pos[:, i], "r--", label=f"Actual {axes_labels[i]}", linewidth=1)
            axes[i].set_xlabel("Time (s)")
            axes[i].set_ylabel(f"{axes_labels[i]} Position (m)")
            axes[i].set_title(f"{axes_labels[i]} Position Tracking")
            axes[i].legend()
            axes[i].grid(True)

        # Combined tracking errors
        axes[3].plot(timestamps, position_errors[:, 0] * 1000, "r-", label="X Error", linewidth=1)
        axes[3].plot(timestamps, position_errors[:, 1] * 1000, "g-", label="Y Error", linewidth=1)
        axes[3].plot(timestamps, position_errors[:, 2] * 1000, "b-", label="Z Error", linewidth=1)
        axes[3].set_xlabel("Time (s)")
        axes[3].set_ylabel("Tracking Error (mm)")
        axes[3].set_title("Position Tracking Errors")
        axes[3].legend()
        axes[3].grid(True)

        plt.tight_layout()
        plt.savefig(self.results_dir / "position_tracking.png", dpi=150, bbox_inches="tight")
        plt.close()

    def _plot_joint_data(self, timestamps, joint_data, title, ylabel, filename):
        """Create 7x1 joint data plots."""
        fig, axes = plt.subplots(7, 1, figsize=(12, 20))

        for i in range(7):
            axes[i].plot(timestamps, joint_data[:, i], "b-", linewidth=1)
            axes[i].set_xlabel("Time (s)")
            axes[i].set_ylabel(f"Joint {i + 1} {ylabel}")
            axes[i].set_title(f"Joint {i + 1} {title}")
            axes[i].grid(True)

        plt.tight_layout()
        plt.savefig(self.results_dir / filename, dpi=150, bbox_inches="tight")
        plt.close()

    def save_results(self):
        """Save all experiment data and results."""
        if not self.data.timestamps:
            print("No data to save")
            return

        # Calculate metrics
        metrics = self.calculate_metrics()

        # Update metadata with final values
        if self.metadata:
            self.metadata.duration = metrics["execution_performance"]["total_duration_s"]
            self.metadata.frequency = metrics["execution_performance"]["mean_frequency_hz"]
            self.metadata.total_points = metrics["execution_performance"]["total_points"]

        # Save experiment data as parquet
        df = self.data.to_dataframe()
        df.to_parquet(self.results_dir / "experiment_data.parquet")

        # Save results.yaml
        results = {"metadata": asdict(self.metadata) if self.metadata else {}, "metrics": metrics}

        with open(self.results_dir / "results.yaml", "w") as f:
            yaml.safe_dump(results, f, default_flow_style=False, indent=2)

        # Create plots
        self.create_plots()

        print(f"\nExperiment results saved to: {self.results_dir}")
        print(f"Data points collected: {len(self.data.timestamps)}")
        print(f"Mean tracking error (3D): {metrics['tracking_errors']['mean_error_3d_mm']:.2f} mm")
        print(f"Max tracking error (3D): {metrics['tracking_errors']['max_error_3d_mm']:.2f} mm")
        print(f"Mean execution frequency: {metrics['execution_performance']['mean_frequency_hz']:.2f} Hz")


# Initialize experiment manager
experiment = ExperimentManager()
experiment.setup_experiment_directory()

robot = Robot(namespace="fr3")
robot.wait_until_ready()

# %%
print(robot.end_effector_pose)
print(robot.q)

# %%
# Parameters for the trajectory
start_position = np.array([0.4, 0.0, 0.4])
traj_freq = 50.0
sin_freq_x = 0.2  # rot / s
amplitude = 0.1  # [m]
max_time = 5

# Store trajectory parameters for metadata
trajectory_params = {
    "start_position": start_position.tolist(),
    "trajectory_frequency_hz": traj_freq,
    "sinusoidal_frequency_hz": sin_freq_x,
    "amplitude_m": amplitude,
    "duration_s": max_time,
    "trajectory_type": "sinusoidal_z_axis",
}

# %%
robot.controller_switcher_client.switch_controller("osc_pd_controller")
robot.haptic_controller_parameters_client.load_param_config(
    file_path=CONFIG_DIR / "controllers" / "osc_pd" / "default.yaml"
)

# Collect experiment metadata
metadata = experiment.collect_controller_metadata(robot, trajectory_params)
experiment.metadata = metadata

print(f"Starting experiment: {experiment.experiment_name}")
print(f"Controller: {metadata.controller_name}")
print(f"Trajectory: {metadata.trajectory_type}")

# %%
# The move_to function will publish a pose to /target_pose while interpolation linearly
print("Moving to start position...")
robot.move_to(position=start_position, speed=0.15)

# %%
# Enhanced trajectory execution with comprehensive data collection
print("Starting trajectory execution...")
time.sleep(1.0)  # Wait a moment to ensure everything is settled

target_pose = robot.end_effector_pose.copy()
rate = robot.node.create_rate(traj_freq)

omega = 2.0 * np.pi * sin_freq_x

# Timing variables
start_time = time.perf_counter()
dt = 1.0 / traj_freq
count = 0
t = 0.0
slow_loops_count = 0

while t < max_time:
    loop_start_time = time.perf_counter()

    # Use deterministic time based on count, not wall clock time
    t = loop_start_time - start_time

    # Generate target trajectory
    x = start_position[0]
    y = start_position[1]
    z = start_position[2] + amplitude * np.sin(omega * t)
    target_pose.position = np.array([x, y, z])

    # Send target to robot
    robot.set_target(pose=target_pose)

    # Collect comprehensive data point
    experiment.collect_data_point(robot, target_pose, t)

    # Print progress every 0.5 seconds
    if count % int(traj_freq / 2) == 0:
        elapsed_time = time.perf_counter() - start_time
        expected_time = count * dt
        print(f"Time: {elapsed_time:.2f}s, Points: {count}, Target Z: {z:.3f}m")

    count += 1

    # Calculate precise sleep time to maintain exact frequency
    loop_execution_time = time.perf_counter() - loop_start_time
    sleep_time = max(0, dt - loop_execution_time)

    # Performance monitoring
    if loop_execution_time > dt:
        slow_loops_count += 1
        if slow_loops_count <= 5:
            print(f"⚠️  Slow loop {slow_loops_count}: {loop_execution_time * 1000:.2f}ms vs target {dt * 1000:.2f}ms")

    if sleep_time > 0:
        time.sleep(sleep_time)

stop_time = time.perf_counter()
print(f"\nTrajectory execution completed!")
print(f"Total time: {stop_time - start_time:.4f} seconds")
print(f"Data points collected: {len(experiment.data.timestamps)}")

# %%
# Save all experiment data and generate results
print("\nSaving experiment results...")
experiment.save_results()

print("\nExperiment completed successfully!")
print(f"Results saved to: {experiment.results_dir}")

# Display quick summary
if experiment.data.timestamps:
    metrics = experiment.calculate_metrics()
    print(f"\n=== Quick Summary ===")
    print(f"Mean 3D tracking error: {metrics['tracking_errors']['mean_error_3d_mm']:.2f} mm")
    print(f"Max 3D tracking error: {metrics['tracking_errors']['max_error_3d_mm']:.2f} mm")
    print(f"RMS 3D tracking error: {metrics['tracking_errors']['rms_error_3d_mm']:.2f} mm")
    print(f"Mean execution frequency: {metrics['execution_performance']['mean_frequency_hz']:.2f} Hz")
    print(f"Frequency consistency: {metrics['execution_performance']['frequency_consistency_percent']:.1f}%")

robot.shutdown()
