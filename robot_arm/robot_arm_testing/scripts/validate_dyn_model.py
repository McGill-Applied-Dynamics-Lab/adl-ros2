#!/usr/bin/env python3
"""
Franka Research 3 Dynamics Model Validation Script

This script validates a Pinocchio dynamics model against recorded robot data by:
1. Loading recorded trajectory data
2. Processing and filtering the data
3. Comparing Pinocchio RNEA predictions with measured torques
4. Generating comprehensive validation reports and plots

Usage:
    python3 validate_dyn_model.py --traj traj1
    python3 validate_dyn_model.py --traj validation_30s
"""

import os
import sys
import argparse
import json
from pathlib import Path
from typing import Dict, List
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt
from datetime import datetime
from tqdm import tqdm

from ament_index_python.packages import get_package_share_directory
from franka_msgs.msg import FrankaRobotState

# Pinocchio for model validation
try:
    import pinocchio as pin

    PINOCCHIO_AVAILABLE = True
except ImportError:
    PINOCCHIO_AVAILABLE = False
    print("ERROR: Pinocchio not available. Please install pinocchio.")
    sys.exit(1)

DEFAULT_PROCESSING_CONFIG = {
    "processing": {"sampling_freq": 1000.0, "cutoff_freq": 50.0},
    "validation": {"criteria": {"max_rmse": 5.0, "min_correlation": 0.7}},
}


class DynamicsModelValidator:
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
            urdf_filename = "fr3_franka_hand.urdf"
            pkg_share = get_package_share_directory("franka_rim")
            urdf_path = os.path.join(pkg_share, "models", urdf_filename)

            if not os.path.exists(urdf_path):
                print("ERROR: Could not find FR3 URDF file")
                return False

            model, _, _ = pin.buildModelsFromUrdf(urdf_path)

            # Lock Fingers
            jointsToLock = ["fr3_finger_joint1", "fr3_finger_joint2"]
            joints2lock_IDs = []
            for jn in jointsToLock:
                if model.existJointName(jn):
                    joints2lock_IDs.append(model.getJointId(jn))
                else:
                    print("Warning: joint " + str(jn) + " does not belong to the model!")

            initial_joint_config = np.array(
                [
                    0,
                    -np.pi / 4,
                    0,
                    -3 * np.pi / 4,
                    0,
                    np.pi / 2,
                    np.pi / 4,
                    0,
                    0,
                ]
            )

            model_reduced = pin.buildReducedModel(model, joints2lock_IDs, initial_joint_config)

            self.pinocchio_model = model_reduced
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
                data = np.load(data_file, allow_pickle=True)
                self.validation_data = {
                    "timestamps": data["timestamps"],
                    "joint_positions": data["joint_positions"],
                    "joint_velocities": data["joint_velocities"],
                    "joint_torques": data["joint_torques"],
                }

                # Load additional fields if available (from new FrankaRobotState format)
                if "joint_positions_desired" in data:
                    self.validation_data.update(
                        {
                            "joint_positions_desired": data["joint_positions_desired"],
                            "joint_velocities_desired": data["joint_velocities_desired"],
                            "joint_torques_desired": data["joint_torques_desired"],
                            "tau_ext_filtered": data["tau_ext_filtered"],
                            "robot_mode": data["robot_mode"],
                        }
                    )
                    print("Loaded additional FrankaRobotState data fields")

                # Load end-effector data if available
                if "ee_position" in data:
                    self.validation_data.update(
                        {
                            "ee_position": data["ee_position"],
                            "ee_orientation": data["ee_orientation"],
                        }
                    )
                    print("Loaded end-effector pose data")

                # Load config if available
                if "trajectory_config" in data:
                    trajectory_config = data["trajectory_config"].item()
                    print(f"Loaded trajectory config: {json.dumps(trajectory_config, indent=2)}")

            else:
                print(f"ERROR: Unsupported data file format: {data_file}")
                return False

            print(f"Loaded validation data from: {data_file}")
            print(f"Data points: {len(self.validation_data['timestamps'])}")

            # Print available data fields
            available_fields = list(self.validation_data.keys())
            print(f"Available data fields: {available_fields}")

            return True

        except Exception as e:
            print(f"ERROR: Failed to load data: {e}")
            return False

    def load_data_from_trajectory(self, trajectory_name: str) -> bool:
        """Load validation data from trajectory name."""
        try:
            # Construct path to trajectory data
            trajectories_dir = Path(__file__).parent.parent / "results" / "trajectories" / trajectory_name
            data_file = trajectories_dir / f"{trajectory_name}.npz"

            if not data_file.exists():
                print(f"ERROR: Data file not found: {data_file}")
                print(f"Make sure you have recorded trajectory '{trajectory_name}' first.")
                return False

            return self.load_data(str(data_file))

        except Exception as e:
            print(f"ERROR: Failed to load trajectory data: {e}")
            return False

    def filter_and_process_data(self) -> bool:
        """Filter sensor data and compute accelerations."""
        try:
            if len(self.validation_data["timestamps"]) < 100:
                print("ERROR: Insufficient data for processing")
                return False

            print("\n--- Processing and filtering data ---")

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

            for i in tqdm(range(7), desc="Filtering raw data", unit="joint"):
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
            print("\n--- Validating dynamics model ---")

            # Compute predicted torques using Pinocchio RNEA
            torques_predicted = []
            num_points = len(self.processed_data["timestamps"])

            for i in tqdm(range(num_points), desc="Computing RNEA", unit="points"):
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

            # Add forward kinematics validation if end-effector data is available
            fk_results = self._validate_forward_kinematics()
            results.update(fk_results)

            print("Dynamics model validation complete!")
            return results

        except Exception as e:
            print(f"ERROR: Validation failed: {e}")
            return {"valid": False, "error": str(e)}

    def _validate_forward_kinematics(self) -> Dict:
        """Validate forward kinematics by comparing measured vs computed end-effector poses."""
        print("Validating forward kinematics...")

        ee_positions_measured = np.array(self.validation_data["ee_position"])
        ee_positions_computed = []

        num_points = len(self.processed_data["positions"])

        # Get end-effector frame ID
        ee_frame_name = "fr3_hand_tcp"  # or "panda_hand" depending on your URDF
        if not self.pinocchio_model.existFrame(ee_frame_name):
            # Try alternative frame names
            possible_frames = ["panda_hand", "fr3_link8", "panda_link8"]
            ee_frame_name = None
            for frame_name in possible_frames:
                if self.pinocchio_model.existFrame(frame_name):
                    ee_frame_name = frame_name
                    break

            if ee_frame_name is None:
                print("WARNING: Could not find end-effector frame in model")
                return {"fk_valid": False, "fk_metrics": {}}

        ee_frame_id = self.pinocchio_model.getFrameId(ee_frame_name)

        # Compute forward kinematics for each joint configuration
        for i in tqdm(range(num_points), desc="Computing FK", unit="points"):
            q = self.processed_data["positions"][i]

            # Update model with current joint positions
            pin.framesForwardKinematics(self.pinocchio_model, self.pinocchio_data, q)

            # Get end-effector transform
            ee_transform = self.pinocchio_data.oMf[ee_frame_id]
            ee_position_computed = ee_transform.translation

            ee_positions_computed.append(ee_position_computed.copy())

        ee_positions_computed = np.array(ee_positions_computed)

        # Compute FK validation metrics
        fk_metrics = self._compute_fk_validation_metrics(ee_positions_measured, ee_positions_computed)

        # Check if FK validation passes
        fk_max_error = np.max(fk_metrics["position_errors"])
        fk_valid = fk_max_error < 0.01  # 1cm threshold

        return {
            "fk_valid": fk_valid,
            "fk_metrics": fk_metrics,
            "ee_positions_computed": ee_positions_computed,
            "ee_positions_measured": ee_positions_measured,
        }

    def _compute_fk_validation_metrics(self, ee_measured: np.ndarray, ee_computed: np.ndarray) -> Dict:
        """Compute forward kinematics validation metrics."""
        # Position errors (L2 norm for each time step)
        position_errors = np.linalg.norm(ee_measured - ee_computed, axis=1)

        # Component-wise errors
        x_errors = ee_measured[:, 0] - ee_computed[:, 0]
        y_errors = ee_measured[:, 1] - ee_computed[:, 1]
        z_errors = ee_measured[:, 2] - ee_computed[:, 2]

        metrics = {
            "position_errors": position_errors,
            "mean_position_error": np.mean(position_errors),
            "max_position_error": np.max(position_errors),
            "std_position_error": np.std(position_errors),
            "rmse_position": np.sqrt(np.mean(position_errors**2)),
            "x_errors": {"mean": np.mean(x_errors), "std": np.std(x_errors), "max": np.max(np.abs(x_errors))},
            "y_errors": {"mean": np.mean(y_errors), "std": np.std(y_errors), "max": np.max(np.abs(y_errors))},
            "z_errors": {"mean": np.mean(z_errors), "std": np.std(z_errors), "max": np.max(np.abs(z_errors))},
        }

        return metrics

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

            # Correlation (handle NaN case)
            try:
                corr_matrix = np.corrcoef(pred, meas)
                if corr_matrix.shape == (2, 2):
                    corr = corr_matrix[0, 1]
                else:
                    corr = 0.0  # Fallback for constant signals

                # Handle NaN correlation (e.g., when one signal is constant)
                if np.isnan(corr):
                    corr = 0.0

            except Exception:
                corr = 0.0

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
        validation_passed = True

        for i in range(7):
            joint_name = f"joint_{i + 1}"

            # Check RMSE (ensure it's a scalar)
            rmse_val = float(metrics["rmse"][i])
            max_rmse_val = float(criteria["max_rmse"])

            if rmse_val > max_rmse_val:
                print(f"VALIDATION FAILED: {joint_name} RMSE {rmse_val:.3f} > {max_rmse_val}")
                validation_passed = False

            # Check correlation (ensure it's a scalar and handle NaN)
            corr_val = float(metrics["correlation"][i])
            min_corr_val = float(criteria["min_correlation"])

            # Skip correlation check if we have NaN or very low correlation is expected
            if not np.isnan(corr_val) and corr_val < min_corr_val:
                print(f"VALIDATION FAILED: {joint_name} correlation {corr_val:.3f} < {min_corr_val}")
                validation_passed = False
            elif np.isnan(corr_val):
                print(f"WARNING: {joint_name} correlation is NaN (possibly constant signal)")

        return validation_passed

    def generate_validation_report(self, results: Dict, output_dir: str):
        """Generate comprehensive validation report with plots and metrics."""
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        # Generate plots
        self._plot_torque_comparison(results, output_path / f"torque_comparison_{timestamp}.png")
        self._plot_error_analysis(results, output_path / f"error_analysis_{timestamp}.png")
        self._plot_validation_metrics(results, output_path / f"validation_metrics_{timestamp}.png")

        # Generate additional plots if we have desired values
        self._plot_desired_vs_measured(results, output_path / f"desired_vs_measured_{timestamp}.png")

        # Generate end-effector plots if available
        self._plot_end_effector_trajectory(results, output_path / f"ee_trajectory_{timestamp}.png")

        # Generate forward kinematics validation plots if available
        if "fk_valid" in results and "ee_positions_computed" in results:
            self._plot_fk_validation(results, output_path / f"fk_validation_{timestamp}.png")

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
            std_err = np.std(error)  # noqa
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
            f.write(f"Dynamics Model: {'PASSED' if results['valid'] else 'FAILED'}\n")

            # Add FK validation results if available
            if "fk_valid" in results:
                f.write(f"Forward Kinematics: {'PASSED' if results['fk_valid'] else 'FAILED'}\n")
            f.write("\n")

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
            f.write(
                f"Average R-squared:   {np.mean(metrics['r_squared']):.4f} ± {np.std(metrics['r_squared']):.4f}\n\n"
            )

            # Add FK validation statistics if available
            if "fk_metrics" in results and results["fk_metrics"]:
                f.write("FORWARD KINEMATICS VALIDATION\n")
                f.write("-" * 30 + "\n")
                fk_metrics = results["fk_metrics"]
                f.write(f"Mean Position Error:   {fk_metrics['mean_position_error'] * 1000:.2f} mm\n")
                f.write(f"Max Position Error:    {fk_metrics['max_position_error'] * 1000:.2f} mm\n")
                f.write(f"Std Position Error:    {fk_metrics['std_position_error'] * 1000:.2f} mm\n")
                f.write(f"RMSE Position:         {fk_metrics['rmse_position'] * 1000:.2f} mm\n\n")

                f.write("Component-wise Errors:\n")
                for coord, errors in [
                    ("X", fk_metrics["x_errors"]),
                    ("Y", fk_metrics["y_errors"]),
                    ("Z", fk_metrics["z_errors"]),
                ]:
                    f.write(
                        f"  {coord}: Mean={errors['mean'] * 1000:.2f}mm, Std={errors['std'] * 1000:.2f}mm, Max={errors['max'] * 1000:.2f}mm\n"
                    )

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

    def _plot_desired_vs_measured(self, results: Dict, output_file: Path):
        """Plot desired vs measured torques if available."""
        if "joint_torques_desired" not in self.validation_data:
            return

        fig, axes = plt.subplots(7, 1, figsize=(14, 16))
        fig.suptitle("Desired vs Measured vs Predicted Torques", fontsize=16)

        timestamps = results["timestamps"]
        torques_pred = results["torques_predicted"]
        torques_meas = results["torques_measured"]
        torques_desired = np.array(self.validation_data["joint_torques_desired"])

        for i in range(7):
            ax = axes[i]

            ax.plot(timestamps, torques_desired[:, i], "g-", label="Desired", linewidth=1, alpha=0.8)
            ax.plot(timestamps, torques_meas[:, i], "b-", label="Measured", linewidth=1, alpha=0.8)
            ax.plot(timestamps, torques_pred[:, i], "r--", label="Predicted", linewidth=1)

            ax.set_ylabel(f"Joint {i + 1}\nTorque (Nm)", fontsize=10)
            ax.grid(True, alpha=0.3)
            ax.legend(fontsize=8)

            # Add tracking error metrics
            tracking_error = torques_meas[:, i] - torques_desired[:, i]
            tracking_rmse = np.sqrt(np.mean(tracking_error**2))
            prediction_rmse = results["metrics"]["rmse"][i]

            ax.text(
                0.02,
                0.98,
                f"Track RMSE: {tracking_rmse:.3f} Nm\nPred RMSE: {prediction_rmse:.3f} Nm",
                transform=ax.transAxes,
                verticalalignment="top",
                bbox=dict(boxstyle="round", facecolor="lightgreen", alpha=0.7),
                fontsize=8,
            )

        axes[-1].set_xlabel("Time (s)", fontsize=12)
        plt.tight_layout()
        plt.savefig(output_file, dpi=300, bbox_inches="tight")
        plt.close()

    def _plot_end_effector_trajectory(self, results: Dict, output_file: Path):
        """Plot end-effector trajectory if available."""
        if "ee_position" not in self.validation_data:
            return

        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        fig.suptitle("End-Effector Trajectory Analysis", fontsize=16)

        timestamps = results["timestamps"]
        ee_positions = np.array(self.validation_data["ee_position"])

        # Plot X, Y, Z positions over time
        coord_names = ["X", "Y", "Z"]
        colors = ["r", "g", "b"]

        ax = axes[0, 0]
        for i in range(3):
            ax.plot(timestamps, ee_positions[:, i], color=colors[i], linewidth=1.5, label=f"{coord_names[i]} position")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Position (m)")
        ax.set_title("End-Effector Position vs Time")
        ax.grid(True, alpha=0.3)
        ax.legend()

        # Add position range statistics
        pos_ranges = [np.max(ee_positions[:, i]) - np.min(ee_positions[:, i]) for i in range(3)]
        ax.text(
            0.02,
            0.02,
            f"Ranges: X={pos_ranges[0]:.4f}m, Y={pos_ranges[1]:.4f}m, Z={pos_ranges[2]:.4f}m",
            transform=ax.transAxes,
            bbox=dict(boxstyle="round", facecolor="lightcyan", alpha=0.7),
            fontsize=8,
        )

        # Plot X-Y trajectory
        ax = axes[0, 1]
        ax.plot(ee_positions[:, 0], ee_positions[:, 1], "k-", linewidth=1, alpha=0.7)
        ax.scatter(ee_positions[0, 0], ee_positions[0, 1], color="green", s=50, label="Start", zorder=5)
        ax.scatter(ee_positions[-1, 0], ee_positions[-1, 1], color="red", s=50, label="End", zorder=5)
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_title("End-Effector Path (X-Y view)")
        ax.grid(True, alpha=0.3)
        ax.legend()
        ax.axis("equal")

        # Plot X-Z trajectory
        ax = axes[1, 0]
        ax.plot(ee_positions[:, 0], ee_positions[:, 2], "k-", linewidth=1, alpha=0.7)
        ax.scatter(ee_positions[0, 0], ee_positions[0, 2], color="green", s=50, label="Start", zorder=5)
        ax.scatter(ee_positions[-1, 0], ee_positions[-1, 2], color="red", s=50, label="End", zorder=5)
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Z (m)")
        ax.set_title("End-Effector Path (X-Z view)")
        ax.grid(True, alpha=0.3)
        ax.legend()
        ax.axis("equal")

        # Plot Y-Z trajectory
        ax = axes[1, 1]
        ax.plot(ee_positions[:, 1], ee_positions[:, 2], "k-", linewidth=1, alpha=0.7)
        ax.scatter(ee_positions[0, 1], ee_positions[0, 2], color="green", s=50, label="Start", zorder=5)
        ax.scatter(ee_positions[-1, 1], ee_positions[-1, 2], color="red", s=50, label="End", zorder=5)
        ax.set_xlabel("Y (m)")
        ax.set_ylabel("Z (m)")
        ax.set_title("End-Effector Path (Y-Z view)")
        ax.grid(True, alpha=0.3)
        ax.legend()
        ax.axis("equal")

        # Add trajectory statistics
        total_distance = np.sum(np.sqrt(np.sum(np.diff(ee_positions, axis=0) ** 2, axis=1)))
        workspace_volume = np.prod(pos_ranges)

        fig.text(
            0.02,
            0.02,
            f"Total path length: {total_distance:.4f}m | Workspace volume: {workspace_volume:.6f}m³",
            fontsize=10,
            bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.8),
        )

        plt.tight_layout()
        plt.savefig(output_file, dpi=300, bbox_inches="tight")
        plt.close()

    def _plot_fk_validation(self, results: Dict, output_file: Path):
        """Plot forward kinematics validation results."""
        if "ee_positions_computed" not in results or "ee_positions_measured" not in results:
            return

        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        fig.suptitle("Forward Kinematics Validation", fontsize=16)

        timestamps = results["timestamps"]
        ee_measured = results["ee_positions_measured"]
        ee_computed = results["ee_positions_computed"]
        fk_metrics = results["fk_metrics"]

        # Plot measured vs computed positions over time
        coord_names = ["X", "Y", "Z"]
        colors = ["r", "g", "b"]

        ax = axes[0, 0]
        for i in range(3):
            ax.plot(
                timestamps,
                ee_measured[:, i],
                color=colors[i],
                linewidth=1.5,
                label=f"{coord_names[i]} Measured",
                alpha=0.8,
            )
            ax.plot(
                timestamps,
                ee_computed[:, i],
                color=colors[i],
                linewidth=1,
                linestyle="--",
                label=f"{coord_names[i]} Computed",
            )
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Position (m)")
        ax.set_title("End-Effector Position: Measured vs Computed")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)

        # Plot position errors over time
        ax = axes[0, 1]
        position_errors = fk_metrics["position_errors"]
        ax.plot(timestamps, position_errors * 1000, "k-", linewidth=1)  # Convert to mm
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Position Error (mm)")
        ax.set_title("Forward Kinematics Position Error")
        ax.grid(True, alpha=0.3)

        # Add error statistics
        mean_error = fk_metrics["mean_position_error"] * 1000
        max_error = fk_metrics["max_position_error"] * 1000
        ax.text(
            0.02,
            0.98,
            f"Mean: {mean_error:.2f}mm\nMax: {max_error:.2f}mm",
            transform=ax.transAxes,
            verticalalignment="top",
            bbox=dict(boxstyle="round", facecolor="lightcyan", alpha=0.7),
            fontsize=8,
        )

        # Plot 3D comparison (X-Y view)
        ax = axes[1, 0]
        ax.plot(ee_measured[:, 0], ee_measured[:, 1], "b-", linewidth=1, alpha=0.7, label="Measured")
        ax.plot(ee_computed[:, 0], ee_computed[:, 1], "r--", linewidth=1, alpha=0.7, label="Computed")
        ax.scatter(ee_measured[0, 0], ee_measured[0, 1], color="green", s=50, label="Start", zorder=5)
        ax.scatter(ee_measured[-1, 0], ee_measured[-1, 1], color="red", s=50, label="End", zorder=5)
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_title("FK Validation (X-Y view)")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)
        ax.axis("equal")

        # Plot error histogram
        ax = axes[1, 1]
        ax.hist(position_errors * 1000, bins=30, alpha=0.7, color="skyblue", edgecolor="black")
        ax.set_xlabel("Position Error (mm)")
        ax.set_ylabel("Frequency")
        ax.set_title("Position Error Distribution")
        ax.grid(True, alpha=0.3)

        # Add vertical line for mean error
        ax.axvline(mean_error, color="red", linestyle="--", linewidth=2, label=f"Mean: {mean_error:.2f}mm")
        ax.legend(fontsize=8)

        plt.tight_layout()
        plt.savefig(output_file, dpi=300, bbox_inches="tight")
        plt.close()


def get_available_trajectories() -> List[str]:
    """Get list of available recorded trajectories."""
    trajectories_dir = Path(__file__).parent.parent / "results" / "trajectories"

    if not trajectories_dir.exists():
        return []

    available_trajs = []
    for traj_dir in trajectories_dir.iterdir():
        if traj_dir.is_dir():
            # Check if the trajectory data file exists
            data_file = traj_dir / f"{traj_dir.name}.npz"
            if data_file.exists():
                available_trajs.append(traj_dir.name)

    return sorted(available_trajs)


def load_config(config_file: str = None) -> Dict:
    """Load configuration from file or return default config."""
    if config_file and os.path.exists(config_file):
        with open(config_file, "r") as f:
            user_config = json.load(f)

        # Merge with default
        config = DEFAULT_PROCESSING_CONFIG.copy()
        config.update(user_config)
        return config

    return DEFAULT_PROCESSING_CONFIG


def main():
    parser = argparse.ArgumentParser(description="Validate Franka Research 3 dynamics model from recorded trajectory")
    parser.add_argument("--traj", type=str, help="Trajectory name to validate (e.g., traj1, validation_30s)")
    parser.add_argument("--list-trajectories", action="store_true", help="List available recorded trajectories")
    parser.add_argument("--config", type=str, help="Configuration file path")

    args = parser.parse_args()

    # List available trajectories if requested
    if args.list_trajectories:
        available_trajectories = get_available_trajectories()
        if not available_trajectories:
            print("No recorded trajectories found.")
            print("Record some trajectories first using record_dyn_traj.py")
        else:
            print("Available recorded trajectories:")
            print("=" * 40)
            for traj_name in available_trajectories:
                # Try to load metadata for additional info
                metadata_file = Path(__file__).parent.parent / "results" / "trajectories" / traj_name / "metadata.json"
                if metadata_file.exists():
                    try:
                        with open(metadata_file, "r") as f:
                            metadata = json.load(f)
                        duration = metadata.get("trajectory_config", {}).get("duration", "Unknown")
                        description = metadata.get("trajectory_config", {}).get("description", "No description")
                        data_points = metadata.get("data_points", "Unknown")
                        print(f"{traj_name}:")
                        print(f"  Duration: {duration}s")
                        print(f"  Description: {description}")
                        print(f"  Data points: {data_points}")
                        print()

                    except Exception as e:
                        print(f"{traj_name}: (metadata unavailable) - {e}")

                else:
                    print(f"{traj_name}: (no metadata)")
        return 0

    if not args.traj:
        args.traj = "traj1"

    if not args.traj:
        print("ERROR: No trajectory specified. Use --traj to select one.")
        print("Use --list-trajectories to see available trajectories.")
        return 1

    # Verify trajectory exists
    available_trajectories = get_available_trajectories()
    if args.traj not in available_trajectories:
        print(f"ERROR: Trajectory '{args.traj}' not found.")
        if available_trajectories:
            print(f"Available trajectories: {available_trajectories}")
        else:
            print("No recorded trajectories found. Record some trajectories first using record_dyn_traj.py")
        return 1

    # Setup output directory
    validation_dir_name = "validation_results"
    output_dir = (
        Path(__file__).parent.parent
        / "results"
        / validation_dir_name
        / f"{args.traj}_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    )

    # Load configuration
    config = load_config(args.config)

    print("Franka Research 3 Dynamics Model Validation")
    print("=" * 50)
    print(f"Trajectory: {args.traj}")
    print(f"Configuration: {json.dumps(config, indent=2)}")

    # Initialize validator
    validator = DynamicsModelValidator(config)

    if not validator.load_pinocchio_model():
        print("FATAL ERROR: Could not load Pinocchio model")
        return 1

    # Load trajectory data
    if not validator.load_data_from_trajectory(args.traj):
        print("FATAL ERROR: Could not load trajectory data")
        return 1

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
    print("\n--- Reports ---")
    print(f"Generating validation reports")
    validator.generate_validation_report(results, output_dir)

    # Print summary
    print("\nVALIDATION SUMMARY")
    print("=" * 30)
    print(f"Trajectory: {args.traj}")
    print(f"Dynamics Model: {'PASSED' if results['valid'] else 'FAILED'}")

    # Add FK validation summary
    if "fk_valid" in results:
        print(f"Forward Kinematics: {'PASSED' if results['fk_valid'] else 'FAILED'}")
        if "fk_metrics" in results and results["fk_metrics"]:
            fk_metrics = results["fk_metrics"]
            print(f"FK Mean Error: {fk_metrics['mean_position_error'] * 1000:.2f}mm")
            print(f"FK Max Error: {fk_metrics['max_position_error'] * 1000:.2f}mm")

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
    try:
        main()

    except KeyboardInterrupt:
        print("\nValidation interrupted by user.")

    # except Exception as e:
    #     print(f"FATAL ERROR: {e}")
    #     sys.exit(1)
