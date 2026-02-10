#!/usr/bin/env python3
"""
Franka Research 3 Dynamics Model Validation Script

This script validates a Pinocchio dynamics model against real robot data by:
1. Executing a rich trajectory to excite robot dynamics
2. Collecting high-frequency sensor data (positions, velocities, torques)
3. Processing and filtering the data
4. Comparing Pinocchio RNEA predictions with measured torques
5. Generating comprehensive validation reports and plots

Usage:
    python3 validate_dynamics_model.py --duration 15 --save-data /tmp/validation_data.npz
"""

import os
import sys
import time
import argparse
import json
from pathlib import Path
from typing import Dict, List, Optional, Tuple
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from threading import Thread

from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from controller_manager_msgs.srv import ListControllers
from ament_index_python.packages import get_package_share_directory

# Pinocchio for model validation
try:
    import pinocchio as pin

    PINOCCHIO_AVAILABLE = True
except ImportError:
    PINOCCHIO_AVAILABLE = False
    print("ERROR: Pinocchio not available. Please install pinocchio.")
    sys.exit(1)


class ModelValidationNode(Node):
    """ROS2 node for collecting validation data from the robot."""

    def __init__(self, config: Dict):
        super().__init__("model_validation_node")
        self.config = config

        # Data storage
        self.validation_data = {
            "timestamps": [],
            "joint_positions": [],
            "joint_velocities": [],
            "joint_torques": [],
            "external_torques": [],
        }

        # State tracking
        self.current_joint_state = None
        self.data_collection_active = False
        self.pinocchio_model = None
        self.pinocchio_data = None

        # Setup communication
        self._setup_communication()
        self._setup_services()

        self.get_logger().info("Model validation node initialized")

    def _setup_communication(self):
        """Setup publishers and subscribers."""
        # Subscribe to joint states
        self.joint_state_subscription = self.create_subscription(
            JointState, "/joint_states", self._joint_state_callback, 10
        )

        # Subscribe to external torques (if available)
        self.external_torque_subscription = self.create_subscription(
            WrenchStamped, "/franka_state_controller/F_ext", self._external_torque_callback, 10
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

    def _joint_state_callback(self, msg: JointState):
        """Callback to store joint state data during collection."""
        self.current_joint_state = msg

        if self.data_collection_active:
            current_time = self.get_clock().now().nanoseconds / 1e9

            # Get FR3 joint data (first 7 joints)
            fr3_positions = list(msg.position[:7])
            fr3_velocities = list(msg.velocity[:7]) if msg.velocity else [0.0] * 7
            fr3_torques = list(msg.effort[:7]) if msg.effort else [0.0] * 7

            self.validation_data["timestamps"].append(current_time)
            self.validation_data["joint_positions"].append(fr3_positions)
            self.validation_data["joint_velocities"].append(fr3_velocities)
            self.validation_data["joint_torques"].append(fr3_torques)

    def _external_torque_callback(self, msg: WrenchStamped):
        """Callback for external torque data."""
        if self.data_collection_active:
            # Store external wrench data if needed
            pass

    def load_pinocchio_model(self) -> bool:
        """Load the Pinocchio model for the FR3."""
        try:
            # Try to find the URDF file
            urdf_path = None
            possible_paths = [
                "/home/csirois/workspaces/franka_ros2_ws/src/adg_ros2/robot_arm/robot_arm_description/urdf/fr3.urdf",
                os.path.join(get_package_share_directory("robot_arm_description"), "urdf", "fr3.urdf"),
            ]

            for path in possible_paths:
                if os.path.exists(path):
                    urdf_path = path
                    break

            if urdf_path is None:
                self.get_logger().error("Could not find FR3 URDF file")
                return False

            # Load model
            self.pinocchio_model = pin.buildModelFromUrdf(urdf_path)
            self.pinocchio_data = self.pinocchio_model.createData()

            self.get_logger().info(f"Loaded Pinocchio model from: {urdf_path}")
            self.get_logger().info(f"Model has {self.pinocchio_model.nq} DOF")
            return True

        except Exception as e:
            self.get_logger().error(f"Failed to load Pinocchio model: {e}")
            return False

    def generate_validation_trajectory(self) -> JointTrajectory:
        """Generate a rich multi-sine trajectory for exciting robot dynamics."""
        # Wait for current joint state
        while self.current_joint_state is None:
            time.sleep(0.1)
            self.get_logger().info("Waiting for joint states...")

        start_positions = list(self.current_joint_state.position[:7])

        # Multi-sine trajectory parameters (designed to excite different frequency modes)
        frequencies = self.config["trajectory"]["frequencies"]
        amplitudes = self.config["trajectory"]["amplitudes"]
        duration = self.config["trajectory"]["duration"]

        # Safety check: reduce amplitudes if too large
        max_amplitude = self.config["trajectory"]["max_amplitude"]
        amplitudes = [min(amp, max_amplitude) for amp in amplitudes]

        # Time parameters
        dt = self.config["trajectory"]["dt"]
        num_points = int(duration / dt)

        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ""
        msg.joint_names = [f"fr3_joint{i + 1}" for i in range(7)]

        self.get_logger().info(f"Generating trajectory with {num_points} points over {duration}s")

        # Generate trajectory points
        for i in range(num_points + 1):
            t = i * dt
            point = JointTrajectoryPoint()

            # Set timing
            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int((t - int(t)) * 1e9)

            # Calculate multi-sine motion for each joint
            positions = []
            velocities = []
            accelerations = []

            for j in range(7):
                # Multi-sine with different frequencies per joint
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

    def execute_validation_trajectory(self) -> bool:
        """Execute validation trajectory and collect data."""
        try:
            # Clear previous data
            self.validation_data = {
                "timestamps": [],
                "joint_positions": [],
                "joint_velocities": [],
                "joint_torques": [],
                "external_torques": [],
            }

            # Generate trajectory
            trajectory = self.generate_validation_trajectory()
            duration = self.config["trajectory"]["duration"]

            self.get_logger().info(f"Publishing trajectory with {len(trajectory.points)} points")

            # Start data collection
            self.data_collection_active = True

            # Publish trajectory
            self.trajectory_publisher.publish(trajectory)

            # Wait for trajectory execution
            execution_time = duration + self.config["trajectory"]["buffer_time"]
            self.get_logger().info(f"Collecting data for {execution_time}s...")

            # Show progress
            for i in range(int(execution_time)):
                time.sleep(1.0)
                data_points = len(self.validation_data["timestamps"])
                self.get_logger().info(f"Progress: {i + 1}/{int(execution_time)}s, collected {data_points} data points")

            # Stop data collection
            self.data_collection_active = False

            final_data_points = len(self.validation_data["timestamps"])
            self.get_logger().info(f"Data collection complete. Total points: {final_data_points}")

            return final_data_points > 100  # Minimum data requirement

        except Exception as e:
            self.get_logger().error(f"Failed to execute validation trajectory: {e}")
            return False


class ModelValidator:
    """Main class for processing data and validating the dynamics model."""

    def __init__(self, config: Dict):
        self.config = config
        self.validation_data = None
        self.processed_data = None
        self.pinocchio_model = None
        self.pinocchio_data = None

    def load_pinocchio_model(self) -> bool:
        """Load Pinocchio model for validation."""
        try:
            urdf_path = None
            # possible_paths = [
            #     "/home/csirois/workspaces/franka_ros2_ws/src/adg_ros2/robot_arm/robot_arm_description/urdf/fr3.urdf",
            #     os.path.join(get_package_share_directory("robot_arm_description"), "urdf", "fr3.urdf"),
            # ]

            urdf_filename = "fr3_franka_hand.urdf"
            pkg_share = get_package_share_directory("franka_rim")
            urdf_path = os.path.join(pkg_share, "models", urdf_filename)

            # for path in possible_paths:
            #     if os.path.exists(path):
            #         urdf_path = path
            #         break

            if urdf_path is None or not os.path.exists(urdf_path):
                print("ERROR: Could not find FR3 URDF file")
                return False

            self.pinocchio_model, collision_model, visual_model = pin.buildModelsFromUrdf(urdf_path)
            self.pinocchio_data = self.pinocchio_model.createData()

            print(f"Loaded Pinocchio model from: {urdf_path}")
            print(f"Model has {self.pinocchio_model.nq} DOF")
            return True

        except Exception as e:
            print(f"ERROR: Failed to load Pinocchio model: {e}")
            return False

    def load_data(self, data_file: str) -> bool:
        """Load validation data from file."""
        try:
            if data_file.endswith(".npz"):
                data = np.load(data_file)
                self.validation_data = {
                    "timestamps": data["timestamps"],
                    "joint_positions": data["joint_positions"],
                    "joint_velocities": data["joint_velocities"],
                    "joint_torques": data["joint_torques"],
                }
            else:
                print(f"ERROR: Unsupported data file format: {data_file}")
                return False

            print(f"Loaded validation data from: {data_file}")
            print(f"Data points: {len(self.validation_data['timestamps'])}")
            return True

        except Exception as e:
            print(f"ERROR: Failed to load data: {e}")
            return False

    def set_data(self, validation_data: Dict):
        """Set validation data directly."""
        self.validation_data = validation_data

    def filter_and_process_data(self) -> bool:
        """Filter sensor data and compute accelerations."""
        try:
            if len(self.validation_data["timestamps"]) < 100:
                print("ERROR: Insufficient data for processing")
                return False

            print("Processing and filtering data...")

            # Convert to numpy arrays
            timestamps = np.array(self.validation_data["timestamps"])
            positions = np.array(self.validation_data["joint_positions"])
            velocities = np.array(self.validation_data["joint_velocities"])
            torques = np.array(self.validation_data["joint_torques"])

            # Design low-pass filter
            sampling_freq = self.config["processing"]["sampling_freq"]
            cutoff_freq = self.config["processing"]["cutoff_freq"]
            nyquist_freq = sampling_freq / 2
            normalized_cutoff = cutoff_freq / nyquist_freq
            b, a = butter(2, normalized_cutoff, "low")

            # Filter positions and velocities
            positions_filtered = np.zeros_like(positions)
            velocities_filtered = np.zeros_like(velocities)

            for i in range(7):  # For each joint
                positions_filtered[:, i] = filtfilt(b, a, positions[:, i])
                velocities_filtered[:, i] = filtfilt(b, a, velocities[:, i])

            # Compute accelerations by differentiating filtered velocities
            dt = np.diff(timestamps)
            dt_mean = np.mean(dt)

            accelerations = np.gradient(velocities_filtered, dt_mean, axis=0)

            # Apply another filter to accelerations to reduce noise
            accelerations_filtered = np.zeros_like(accelerations)
            for i in range(7):
                accelerations_filtered[:, i] = filtfilt(b, a, accelerations[:, i])

            self.processed_data = {
                "timestamps": timestamps,
                "positions": positions_filtered,
                "velocities": velocities_filtered,
                "accelerations": accelerations_filtered,
                "torques_measured": torques,
                "dt_mean": dt_mean,
            }

            print(f"Data processing complete. Effective sampling rate: {1 / dt_mean:.1f} Hz")
            return True

        except Exception as e:
            print(f"ERROR: Data processing failed: {e}")
            return False

    def validate_dynamics_model(self) -> Dict:
        """Validate the Pinocchio model against processed data."""
        try:
            print("Validating dynamics model...")

            # Compute predicted torques using Pinocchio RNEA
            torques_predicted = []

            num_points = len(self.processed_data["timestamps"])
            print_interval = max(1, num_points // 20)  # Print progress every 5%

            for i in range(num_points):
                if i % print_interval == 0:
                    progress = (i / num_points) * 100
                    print(f"Computing RNEA: {progress:.1f}% complete")

                q = self.processed_data["positions"][i]
                v = self.processed_data["velocities"][i]
                a = self.processed_data["accelerations"][i]

                # Compute torques using inverse dynamics (RNEA)
                tau_pred = pin.rnea(self.pinocchio_model, self.pinocchio_data, q, v, a)
                torques_predicted.append(tau_pred.copy())

            torques_predicted = np.array(torques_predicted)
            torques_measured = self.processed_data["torques_measured"]

            # Compute validation metrics
            metrics = self._compute_validation_metrics(torques_predicted, torques_measured)

            # Determine validation success based on criteria
            validation_criteria = self.config["validation"]["criteria"]
            validation_passed = self._check_validation_criteria(metrics, validation_criteria)

            results = {
                "valid": validation_passed,
                "metrics": metrics,
                "torques_predicted": torques_predicted,
                "torques_measured": torques_measured,
                "timestamps": self.processed_data["timestamps"],
            }

            print("Dynamics model validation complete!")
            return results

        except Exception as e:
            print(f"ERROR: Validation failed: {e}")
            return {"valid": False, "error": str(e)}

    def _compute_validation_metrics(self, torques_pred: np.ndarray, torques_meas: np.ndarray) -> Dict:
        """Compute comprehensive validation metrics."""
        metrics = {"rmse": [], "mae": [], "correlation": [], "r_squared": [], "max_error": [], "std_error": []}

        for i in range(7):  # For each joint
            pred = torques_pred[:, i]
            meas = torques_meas[:, i]
            error = pred - meas

            # RMSE
            rmse = np.sqrt(np.mean(error**2))
            metrics["rmse"].append(rmse)

            # MAE
            mae = np.mean(np.abs(error))
            metrics["mae"].append(mae)

            # Correlation
            corr = np.corrcoef(pred, meas)[0, 1]
            metrics["correlation"].append(corr)

            # R-squared
            ss_res = np.sum(error**2)
            ss_tot = np.sum((meas - np.mean(meas)) ** 2)
            r2 = 1 - (ss_res / ss_tot) if ss_tot != 0 else 0
            metrics["r_squared"].append(r2)

            # Max error
            max_err = np.max(np.abs(error))
            metrics["max_error"].append(max_err)

            # Standard deviation of error
            std_err = np.std(error)
            metrics["std_error"].append(std_err)

        return metrics

    def _check_validation_criteria(self, metrics: Dict, criteria: Dict) -> bool:
        """Check if validation metrics meet the specified criteria."""
        for i in range(7):
            joint_name = f"joint_{i + 1}"

            # Check RMSE
            if metrics["rmse"][i] > criteria["max_rmse"]:
                print(f"VALIDATION FAILED: {joint_name} RMSE {metrics['rmse'][i]:.3f} > {criteria['max_rmse']}")
                return False

            # Check correlation
            if metrics["correlation"][i] < criteria["min_correlation"]:
                print(
                    f"VALIDATION FAILED: {joint_name} correlation {metrics['correlation'][i]:.3f} < {criteria['min_correlation']}"
                )
                return False

        return True

    def generate_validation_report(self, results: Dict, output_dir: str):
        """Generate comprehensive validation report with plots and metrics."""
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        # Generate plots
        self._plot_torque_comparison(results, output_path / f"torque_comparison_{timestamp}.png")
        self._plot_error_analysis(results, output_path / f"error_analysis_{timestamp}.png")
        self._plot_validation_metrics(results, output_path / f"validation_metrics_{timestamp}.png")

        # Generate text report
        self._generate_text_report(results, output_path / f"validation_report_{timestamp}.txt")

        # Save data
        self._save_validation_data(results, output_path / f"validation_data_{timestamp}.npz")

        print(f"Validation report generated in: {output_path}")

    def _plot_torque_comparison(self, results: Dict, output_file: Path):
        """Plot predicted vs measured torques for all joints."""
        fig, axes = plt.subplots(7, 1, figsize=(14, 16))
        fig.suptitle("Dynamics Model Validation: Predicted vs Measured Torques", fontsize=16)

        timestamps = results["timestamps"]
        torques_pred = results["torques_predicted"]
        torques_meas = results["torques_measured"]

        for i in range(7):
            ax = axes[i]

            ax.plot(timestamps, torques_meas[:, i], "b-", label="Measured", linewidth=1, alpha=0.8)
            ax.plot(timestamps, torques_pred[:, i], "r--", label="Predicted", linewidth=1)

            ax.set_ylabel(f"Joint {i + 1}\nTorque (Nm)", fontsize=10)
            ax.grid(True, alpha=0.3)
            ax.legend(fontsize=8)

            # Add metrics
            rmse = results["metrics"]["rmse"][i]
            corr = results["metrics"]["correlation"][i]
            ax.text(
                0.02,
                0.98,
                f"RMSE: {rmse:.3f} Nm\nCorr: {corr:.3f}",
                transform=ax.transAxes,
                verticalalignment="top",
                bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.7),
                fontsize=8,
            )

        axes[-1].set_xlabel("Time (s)", fontsize=12)
        plt.tight_layout()
        plt.savefig(output_file, dpi=300, bbox_inches="tight")
        plt.close()

    def _plot_error_analysis(self, results: Dict, output_file: Path):
        """Plot error analysis including error histograms and time series."""
        fig = plt.figure(figsize=(16, 12))
        gs = fig.add_gridspec(4, 4, hspace=0.3, wspace=0.3)

        torques_pred = results["torques_predicted"]
        torques_meas = results["torques_measured"]
        timestamps = results["timestamps"]

        for i in range(7):
            error = torques_pred[:, i] - torques_meas[:, i]

            # Error time series (top row)
            if i < 4:
                ax_time = fig.add_subplot(gs[0, i])
                ax_time.plot(timestamps, error, "g-", linewidth=0.8)
                ax_time.set_title(f"Joint {i + 1} Error vs Time", fontsize=10)
                ax_time.set_ylabel("Error (Nm)", fontsize=9)
                ax_time.grid(True, alpha=0.3)

            # Error histograms (bottom rows)
            row = 1 + i // 4
            col = i % 4
            ax_hist = fig.add_subplot(gs[row, col])
            ax_hist.hist(error, bins=50, alpha=0.7, color="skyblue", edgecolor="black", linewidth=0.5)
            ax_hist.set_title(f"Joint {i + 1} Error Distribution", fontsize=10)
            ax_hist.set_xlabel("Error (Nm)", fontsize=9)
            ax_hist.set_ylabel("Frequency", fontsize=9)
            ax_hist.grid(True, alpha=0.3)

            # Add statistics
            mean_err = np.mean(error)
            std_err = np.std(error)
            ax_hist.axvline(mean_err, color="red", linestyle="--", linewidth=2, label=f"Mean: {mean_err:.3f}")
            ax_hist.legend(fontsize=8)

        plt.suptitle("Error Analysis", fontsize=16)
        plt.savefig(output_file, dpi=300, bbox_inches="tight")
        plt.close()

    def _plot_validation_metrics(self, results: Dict, output_file: Path):
        """Plot validation metrics summary."""
        metrics = results["metrics"]
        joint_names = [f"J{i + 1}" for i in range(7)]

        fig, axes = plt.subplots(2, 3, figsize=(15, 10))
        axes = axes.flatten()

        # RMSE
        axes[0].bar(joint_names, metrics["rmse"], color="skyblue", edgecolor="black")
        axes[0].set_title("Root Mean Square Error (RMSE)")
        axes[0].set_ylabel("RMSE (Nm)")
        axes[0].grid(True, alpha=0.3)

        # MAE
        axes[1].bar(joint_names, metrics["mae"], color="lightgreen", edgecolor="black")
        axes[1].set_title("Mean Absolute Error (MAE)")
        axes[1].set_ylabel("MAE (Nm)")
        axes[1].grid(True, alpha=0.3)

        # Correlation
        axes[2].bar(joint_names, metrics["correlation"], color="orange", edgecolor="black")
        axes[2].set_title("Correlation Coefficient")
        axes[2].set_ylabel("Correlation")
        axes[2].set_ylim([0, 1])
        axes[2].grid(True, alpha=0.3)

        # R-squared
        axes[3].bar(joint_names, metrics["r_squared"], color="pink", edgecolor="black")
        axes[3].set_title("R-squared")
        axes[3].set_ylabel("R²")
        axes[3].grid(True, alpha=0.3)

        # Max Error
        axes[4].bar(joint_names, metrics["max_error"], color="red", alpha=0.7, edgecolor="black")
        axes[4].set_title("Maximum Absolute Error")
        axes[4].set_ylabel("Max Error (Nm)")
        axes[4].grid(True, alpha=0.3)

        # Standard Deviation of Error
        axes[5].bar(joint_names, metrics["std_error"], color="purple", alpha=0.7, edgecolor="black")
        axes[5].set_title("Error Standard Deviation")
        axes[5].set_ylabel("Std Error (Nm)")
        axes[5].grid(True, alpha=0.3)

        plt.suptitle("Validation Metrics Summary", fontsize=16)
        plt.tight_layout()
        plt.savefig(output_file, dpi=300, bbox_inches="tight")
        plt.close()

    def _generate_text_report(self, results: Dict, output_file: Path):
        """Generate detailed text report."""
        with open(output_file, "w") as f:
            f.write("FRANKA RESEARCH 3 DYNAMICS MODEL VALIDATION REPORT\n")
            f.write("=" * 60 + "\n\n")

            f.write(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"Configuration: {json.dumps(self.config, indent=2)}\n\n")

            f.write("VALIDATION RESULT\n")
            f.write("-" * 20 + "\n")
            f.write(f"Overall Result: {'PASSED' if results['valid'] else 'FAILED'}\n\n")

            f.write("DETAILED METRICS\n")
            f.write("-" * 20 + "\n")
            metrics = results["metrics"]

            for i in range(7):
                f.write(f"Joint {i + 1}:\n")
                f.write(f"  RMSE:        {metrics['rmse'][i]:.4f} Nm\n")
                f.write(f"  MAE:         {metrics['mae'][i]:.4f} Nm\n")
                f.write(f"  Correlation: {metrics['correlation'][i]:.4f}\n")
                f.write(f"  R-squared:   {metrics['r_squared'][i]:.4f}\n")
                f.write(f"  Max Error:   {metrics['max_error'][i]:.4f} Nm\n")
                f.write(f"  Std Error:   {metrics['std_error'][i]:.4f} Nm\n\n")

            f.write("SUMMARY STATISTICS\n")
            f.write("-" * 20 + "\n")
            f.write(f"Average RMSE:        {np.mean(metrics['rmse']):.4f} ± {np.std(metrics['rmse']):.4f} Nm\n")
            f.write(
                f"Average Correlation: {np.mean(metrics['correlation']):.4f} ± {np.std(metrics['correlation']):.4f}\n"
            )
            f.write(f"Average R-squared:   {np.mean(metrics['r_squared']):.4f} ± {np.std(metrics['r_squared']):.4f}\n")

    def _save_validation_data(self, results: Dict, output_file: Path):
        """Save all validation data for later analysis."""
        np.savez_compressed(
            output_file,
            timestamps=results["timestamps"],
            torques_predicted=results["torques_predicted"],
            torques_measured=results["torques_measured"],
            positions=self.processed_data["positions"],
            velocities=self.processed_data["velocities"],
            accelerations=self.processed_data["accelerations"],
            metrics=results["metrics"],
            config=self.config,
        )


def load_config(config_file: Optional[str] = None) -> Dict:
    """Load configuration from file or return default config."""
    default_config = {
        "trajectory": {
            "duration": 15.0,
            "frequencies": [0.1, 0.15, 0.08, 0.12, 0.18, 0.25, 0.3],
            "amplitudes": [0.3, 0.4, 0.3, 0.5, 0.3, 0.4, 0.3],
            "max_amplitude": 0.4,
            "dt": 0.01,
            "buffer_time": 2.0,
        },
        "processing": {"sampling_freq": 1000.0, "cutoff_freq": 50.0},
        "validation": {"criteria": {"max_rmse": 5.0, "min_correlation": 0.7}},
    }

    if config_file and os.path.exists(config_file):
        with open(config_file, "r") as f:
            user_config = json.load(f)

        # Merge with default config
        def merge_dict(default, user):
            result = default.copy()
            for key, value in user.items():
                if key in result and isinstance(result[key], dict) and isinstance(value, dict):
                    result[key] = merge_dict(result[key], value)
                else:
                    result[key] = value
            return result

        return merge_dict(default_config, user_config)

    return default_config


def main():
    parser = argparse.ArgumentParser(description="Validate Franka Research 3 dynamics model")
    parser.add_argument("--duration", type=float, default=15.0, help="Trajectory duration (seconds)")
    parser.add_argument("--config", type=str, help="Configuration file path")
    parser.add_argument(
        "--output-dir", type=str, default="/tmp/validation_results", help="Output directory for results"
    )
    parser.add_argument("--save-data", type=str, help="Save raw data to file")
    parser.add_argument("--load-data", type=str, help="Load data from file (skip collection)")
    parser.add_argument("--no-plots", action="store_true", help="Skip generating plots")

    args = parser.parse_args()

    # Load configuration
    config = load_config(args.config)
    if args.duration != 15.0:  # Override duration if specified
        config["trajectory"]["duration"] = args.duration

    print("Franka Research 3 Dynamics Model Validation")
    print("=" * 50)
    print(f"Configuration: {json.dumps(config, indent=2)}")

    # Initialize validator
    validator = ModelValidator(config)

    if not validator.load_pinocchio_model():
        print("FATAL ERROR: Could not load Pinocchio model")
        return 1

    # Load or collect data
    if args.load_data:
        print(f"Loading data from: {args.load_data}")
        if not validator.load_data(args.load_data):
            return 1
    else:
        print("Collecting data from robot...")

        # Initialize ROS2
        rclpy.init()

        try:
            # Create node and executor
            node = ModelValidationNode(config)
            executor = SingleThreadedExecutor()
            executor.add_node(node)

            # Start executor in background thread
            spin_thread = Thread(target=executor.spin)
            spin_thread.daemon = True
            spin_thread.start()

            # Load Pinocchio model in node
            if not node.load_pinocchio_model():
                print("FATAL ERROR: Node could not load Pinocchio model")
                return 1

            # Wait for system initialization
            print("Waiting for system initialization...")
            time.sleep(3.0)

            # Execute validation trajectory
            if not node.execute_validation_trajectory():
                print("FATAL ERROR: Failed to collect validation data")
                return 1

            # Transfer data to validator
            validator.set_data(node.validation_data)

        finally:
            executor.shutdown()
            rclpy.shutdown()
            if spin_thread.is_alive():
                spin_thread.join(timeout=1.0)

    # Save raw data if requested
    if args.save_data:
        print(f"Saving raw data to: {args.save_data}")
        os.makedirs(os.path.dirname(args.save_data), exist_ok=True)
        np.savez_compressed(
            args.save_data,
            timestamps=validator.validation_data["timestamps"],
            joint_positions=validator.validation_data["joint_positions"],
            joint_velocities=validator.validation_data["joint_velocities"],
            joint_torques=validator.validation_data["joint_torques"],
        )

    # Process data
    if not validator.filter_and_process_data():
        print("FATAL ERROR: Failed to process data")
        return 1

    # Validate model
    results = validator.validate_dynamics_model()
    if "error" in results:
        print(f"FATAL ERROR: {results['error']}")
        return 1

    # Generate report
    if not args.no_plots:
        print(f"Generating validation report in: {args.output_dir}")
        validator.generate_validation_report(results, args.output_dir)

    # Print summary
    print("\nVALIDATION SUMMARY")
    print("=" * 30)
    print(f"Result: {'PASSED' if results['valid'] else 'FAILED'}")

    metrics = results["metrics"]
    print(f"Average RMSE: {np.mean(metrics['rmse']):.3f} ± {np.std(metrics['rmse']):.3f} Nm")
    print(f"Average Correlation: {np.mean(metrics['correlation']):.3f} ± {np.std(metrics['correlation']):.3f}")

    for i in range(7):
        status = (
            "✓"
            if (
                metrics["rmse"][i] < config["validation"]["criteria"]["max_rmse"]
                and metrics["correlation"][i] > config["validation"]["criteria"]["min_correlation"]
            )
            else "✗"
        )
        print(f"Joint {i + 1}: RMSE={metrics['rmse'][i]:.3f}Nm, Corr={metrics['correlation'][i]:.3f} {status}")

    return 0 if results["valid"] else 1


if __name__ == "__main__":
    sys.exit(main())
