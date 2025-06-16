import os
import sys
import time
import pytest
from threading import Thread

import launch
import launch_pytest
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import JointState
from controller_manager_msgs.srv import ListControllers

# Pinocchio for model validation
try:
    import pinocchio as pin

    PINOCCHIO_AVAILABLE = True
except ImportError:
    PINOCCHIO_AVAILABLE = False

CTRL_NAME = "joint_trajectory_controller"


@launch_pytest.fixture
def launch_description():
    """Sets up the launch description for the test environment."""
    fr3_launch_file = PathJoinSubstitution([FindPackageShare("robot_arm_bringup"), "launch", "fr3.launch.py"])

    robot_ip = os.environ.get("FR3_IP")
    if not robot_ip:
        raise RuntimeError("Environment variable FR3_IP must be set to the robot's IP address.")

    fr3_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([fr3_launch_file]),
        launch_arguments={
            "robot_ip": robot_ip,
            "arm_id": "fr3",
            "load_gripper": "false",
            "hw_type": "real",
            "use_rviz": "false",
        }.items(),
    )

    return launch.LaunchDescription([fr3_launch, launch_pytest.actions.ReadyToTest()])


@pytest.fixture(scope="module")
def rclpy_init_shutdown():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.mark.launch(fixture=launch_description)
def test_system_ready_for_validation():
    """Test that the system is ready for model validation (basic connectivity test)."""
    rclpy.init()
    node = BasicTestNode("validation_readiness_test")
    executor = SingleThreadedExecutor()
    spin_thread = None

    try:
        executor.add_node(node)
        spin_thread = Thread(target=executor.spin)
        spin_thread.daemon = True
        spin_thread.start()

        # Wait for system initialization
        time.sleep(3.0)

        # Basic checks for validation readiness
        assert node.is_controller_active(CTRL_NAME), f"Controller '{CTRL_NAME}' not active"
        assert node.has_joint_states(), "Joint states not available"

        if PINOCCHIO_AVAILABLE:
            assert node.can_load_urdf(), "URDF model not loadable"

        node.get_logger().info("System is ready for model validation!")

    finally:
        executor.shutdown()
        if spin_thread and spin_thread.is_alive():
            spin_thread.join(timeout=1.0)
        rclpy.shutdown()
        if node:
            node.destroy_node()


@pytest.mark.skipif(not PINOCCHIO_AVAILABLE, reason="Pinocchio not available")
@pytest.mark.launch(fixture=launch_description)
def test_pinocchio_model_loads():
    """Test that the Pinocchio model can be loaded successfully."""
    rclpy.init()
    node = BasicTestNode("pinocchio_test")
    executor = SingleThreadedExecutor()
    spin_thread = None

    try:
        executor.add_node(node)
        spin_thread = Thread(target=executor.spin)
        spin_thread.daemon = True
        spin_thread.start()

        time.sleep(2.0)

        # Test Pinocchio model loading
        model, data = node.load_pinocchio_model()
        assert model is not None, "Failed to load Pinocchio model"
        assert data is not None, "Failed to create Pinocchio data"
        assert model.nq == 7, f"Expected 7 DOF, got {model.nq}"

        node.get_logger().info("Pinocchio model loaded successfully!")

    finally:
        executor.shutdown()
        if spin_thread and spin_thread.is_alive():
            spin_thread.join(timeout=1.0)
        rclpy.shutdown()
        if node:
            node.destroy_node()


class BasicTestNode(Node):
    """Lightweight test node for basic validation checks."""

    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.current_joint_state = None

        # Setup basic communication
        self.joint_state_subscription = self.create_subscription(
            JointState, "/joint_states", self._joint_state_callback, 10
        )

        self.list_controllers_client = self.create_client(ListControllers, "/controller_manager/list_controllers")

        # Wait for services
        while not self.list_controllers_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for controller manager service...")

    def _joint_state_callback(self, msg: JointState):
        """Store current joint state."""
        self.current_joint_state = msg

    def is_controller_active(self, controller_name: str) -> bool:
        """Check if controller is active."""
        request = ListControllers.Request()
        future = self.list_controllers_client.call_async(request)

        start_time = time.time()
        while not future.done() and (time.time() - start_time) < 5.0:
            time.sleep(0.01)

        if future.done():
            try:
                result = future.result()
                for controller in result.controller:
                    if controller.name == controller_name:
                        return controller.state == "active"
            except Exception as e:
                self.get_logger().error(f"Error checking controller state: {e}")

        return False

    def has_joint_states(self) -> bool:
        """Check if joint states are being received."""
        start_time = time.time()
        while self.current_joint_state is None and (time.time() - start_time) < 5.0:
            time.sleep(0.1)

        return self.current_joint_state is not None

    def can_load_urdf(self) -> bool:
        """Check if URDF can be found."""
        from ament_index_python.packages import get_package_share_directory

        possible_paths = [
            "/home/csirois/workspaces/franka_ros2_ws/src/adg_ros2/robot_arm/robot_arm_description/urdf/fr3.urdf",
            os.path.join(get_package_share_directory("robot_arm_description"), "urdf", "fr3.urdf"),
        ]

        for path in possible_paths:
            if os.path.exists(path):
                return True

        return False

    def load_pinocchio_model(self):
        """Load Pinocchio model for testing."""
        if not PINOCCHIO_AVAILABLE:
            return None, None

        try:
            from ament_index_python.packages import get_package_share_directory

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
                return None, None

            model = pin.buildModelFromUrdf(urdf_path)
            data = model.createData()

            return model, data

        except Exception as e:
            self.get_logger().error(f"Failed to load Pinocchio model: {e}")
            return None, None
            self.validation_data["joint_velocities"].append(fr3_velocities)
            self.validation_data["joint_torques"].append(fr3_torques)

    def _external_torque_callback(self, msg: WrenchStamped):
        """Callback for external torque data (if available)."""
        if self.data_collection_active:
            # Store external torque data if needed for validation
            pass

    def is_controller_active(self, controller_name: str) -> bool:
        """Check if controller is active."""
        request = ListControllers.Request()
        future = self.list_controllers_client.call_async(request)

        start_time = time.time()
        while not future.done() and (time.time() - start_time) < 5.0:
            time.sleep(0.01)

        if future.done():
            try:
                result = future.result()
                for controller in result.controller:
                    if controller.name == controller_name:
                        return controller.state == "active"
            except Exception as e:
                self.get_logger().error(f"Error checking controller state: {e}")

        return False

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

            self.get_logger().info(f"Loaded Pinocchio model with {self.pinocchio_model.nq} DOF")
            return True

        except Exception as e:
            self.get_logger().error(f"Failed to load Pinocchio model: {e}")
            return False

    def generate_validation_trajectory(self, duration: float = 15.0) -> JointTrajectory:
        """Generate a smooth multi-sine trajectory for exciting robot dynamics."""
        # Wait for current joint state
        while self.current_joint_state is None:
            time.sleep(0.1)
            self.get_logger().info("Waiting for joint states...")

        # Get current position as starting point
        start_positions = list(self.current_joint_state.position[:7])

        # Multi-sine trajectory parameters for each joint
        frequencies = [0.1, 0.15, 0.08, 0.12, 0.18, 0.25, 0.3]  # Hz
        amplitudes = [0.3, 0.4, 0.3, 0.5, 0.3, 0.4, 0.3]  # radians

        # Safety: reduce amplitudes to stay within joint limits
        amplitudes = [amp * 0.5 for amp in amplitudes]

        # Time parameters
        dt = 0.01  # 100 Hz trajectory
        num_points = int(duration / dt)

        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ""
        msg.joint_names = [f"fr3_joint{i + 1}" for i in range(7)]

        # Generate trajectory points
        for i in range(num_points + 1):
            t = i * dt
            point = JointTrajectoryPoint()

            # Set timing
            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int((t - int(t)) * 1e9)

            # Calculate positions, velocities, and accelerations
            positions = []
            velocities = []
            accelerations = []

            for j in range(7):
                # Multi-sine motion
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
            trajectory = self.generate_validation_trajectory(duration=15.0)

            self.get_logger().info(f"Generated trajectory with {len(trajectory.points)} points")

            # Start data collection
            self.data_collection_active = True

            # Publish trajectory
            self.trajectory_publisher.publish(trajectory)

            # Wait for trajectory execution + buffer time
            execution_time = 17.0  # 15s trajectory + 2s buffer
            self.get_logger().info(f"Collecting data for {execution_time}s...")
            time.sleep(execution_time)

            # Stop data collection
            self.data_collection_active = False

            data_points = len(self.validation_data["timestamps"])
            self.get_logger().info(f"Collected {data_points} data points")

            return data_points > 1000  # Ensure we have sufficient data

        except Exception as e:
            self.get_logger().error(f"Failed to execute validation trajectory: {e}")
            return False

    def filter_and_process_data(self) -> Dict:
        """Filter sensor data and compute accelerations."""
        if len(self.validation_data["timestamps"]) < 100:
            raise ValueError("Insufficient data for processing")

        # Convert to numpy arrays
        timestamps = np.array(self.validation_data["timestamps"])
        positions = np.array(self.validation_data["joint_positions"])
        velocities = np.array(self.validation_data["joint_velocities"])
        torques = np.array(self.validation_data["joint_torques"])

        # Design low-pass filter
        nyquist_freq = SAMPLING_FREQ / 2
        normalized_cutoff = CUTOFF_FREQ / nyquist_freq
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

        return {
            "timestamps": timestamps,
            "positions": positions_filtered,
            "velocities": velocities_filtered,
            "accelerations": accelerations_filtered,
            "torques_measured": torques,
            "dt_mean": dt_mean,
        }

    def validate_dynamics_model(self) -> Dict:
        """Validate the Pinocchio model against collected data."""
        try:
            # Process the collected data
            processed_data = self.filter_and_process_data()

            # Compute predicted torques using Pinocchio RNEA
            torques_predicted = []

            for i in range(len(processed_data["timestamps"])):
                q = processed_data["positions"][i]
                v = processed_data["velocities"][i]
                a = processed_data["accelerations"][i]

                # Compute torques using inverse dynamics (RNEA)
                tau_pred = pin.rnea(self.pinocchio_model, self.pinocchio_data, q, v, a)
                torques_predicted.append(tau_pred.copy())

            torques_predicted = np.array(torques_predicted)
            torques_measured = processed_data["torques_measured"]

            # Compute validation metrics
            metrics = self._compute_validation_metrics(torques_predicted, torques_measured)

            # Plot comparison
            self._plot_torque_comparison(processed_data["timestamps"], torques_predicted, torques_measured)

            # Determine if validation passed
            # Criteria: RMSE < 5 Nm for each joint, correlation > 0.7
            validation_passed = all(metrics["rmse"][i] < 5.0 and metrics["correlation"][i] > 0.7 for i in range(7))

            return {
                "valid": validation_passed,
                "metrics": metrics,
                "errors": [] if validation_passed else ["RMSE or correlation criteria not met"],
            }

        except Exception as e:
            self.get_logger().error(f"Validation failed: {e}")
            return {"valid": False, "metrics": {}, "errors": [str(e)]}

    def _compute_validation_metrics(self, torques_pred: np.ndarray, torques_meas: np.ndarray) -> Dict:
        """Compute validation metrics comparing predicted vs measured torques."""
        metrics = {"rmse": [], "mae": [], "correlation": [], "r_squared": []}

        for i in range(7):  # For each joint
            pred = torques_pred[:, i]
            meas = torques_meas[:, i]

            # RMSE
            rmse = np.sqrt(np.mean((pred - meas) ** 2))
            metrics["rmse"].append(rmse)

            # MAE
            mae = np.mean(np.abs(pred - meas))
            metrics["mae"].append(mae)

            # Correlation
            corr = np.corrcoef(pred, meas)[0, 1]
            metrics["correlation"].append(corr)

            # R-squared
            ss_res = np.sum((meas - pred) ** 2)
            ss_tot = np.sum((meas - np.mean(meas)) ** 2)
            r2 = 1 - (ss_res / ss_tot) if ss_tot != 0 else 0
            metrics["r_squared"].append(r2)

        return metrics

    def _plot_torque_comparison(self, timestamps: np.ndarray, torques_pred: np.ndarray, torques_meas: np.ndarray):
        """Plot comparison between predicted and measured torques."""
        fig, axes = plt.subplots(7, 1, figsize=(12, 14))
        fig.suptitle("Dynamics Model Validation: Predicted vs Measured Torques")

        for i in range(7):
            ax = axes[i]

            ax.plot(timestamps, torques_meas[:, i], "b-", label="Measured", linewidth=1)
            ax.plot(timestamps, torques_pred[:, i], "r--", label="Predicted", linewidth=1)

            ax.set_ylabel(f"Joint {i + 1}\nTorque (Nm)")
            ax.grid(True, alpha=0.3)
            ax.legend()

            # Compute and display RMSE
            rmse = np.sqrt(np.mean((torques_pred[:, i] - torques_meas[:, i]) ** 2))
            ax.text(
                0.02,
                0.98,
                f"RMSE: {rmse:.3f} Nm",
                transform=ax.transAxes,
                verticalalignment="top",
                bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5),
            )

        axes[-1].set_xlabel("Time (s)")
        plt.tight_layout()

        # Save plot
        plot_path = "/tmp/dynamics_validation.png"
        plt.savefig(plot_path, dpi=300, bbox_inches="tight")
        self.get_logger().info(f"Validation plot saved to: {plot_path}")

        plt.show(block=False)
        plt.pause(3.0)
        plt.close()


@pytest.mark.skipif(not PINOCCHIO_AVAILABLE, reason="Pinocchio not available")
@pytest.mark.launch(fixture=launch_description)
def test_kinematics_validation():
    """Test kinematics validation by comparing forward kinematics with known poses."""
    rclpy.init()
    node = ModelValidationNode("kinematics_validation_node")
    executor = rclpy.executors.SingleThreadedExecutor()
    spin_thread = None

    try:
        executor.add_node(node)
        spin_thread = Thread(target=executor.spin)
        spin_thread.daemon = True
        spin_thread.start()

        node.get_logger().info("Kinematics validation test started")

        # Wait for initialization
        time.sleep(2.0)

        # Load Pinocchio model
        assert node.load_pinocchio_model(), "Failed to load Pinocchio model"

        # Test forward kinematics at current position
        while node.current_joint_state is None:
            time.sleep(0.1)

        joint_positions = list(node.current_joint_state.position[:7])

        # Compute forward kinematics
        pin.forwardKinematics(node.pinocchio_model, node.pinocchio_data, np.array(joint_positions))
        pin.updateFramePlacements(node.pinocchio_model, node.pinocchio_data)

        # Get end-effector pose
        end_effector_frame_id = node.pinocchio_model.getFrameId("fr3_hand")
        ee_pose = node.pinocchio_data.oMf[end_effector_frame_id]

        node.get_logger().info(f"End-effector position: {ee_pose.translation}")
        node.get_logger().info(f"End-effector orientation: {ee_pose.rotation}")

        # Basic sanity checks
        assert not np.any(np.isnan(ee_pose.translation)), "NaN in end-effector position"
        assert not np.any(np.isnan(ee_pose.rotation)), "NaN in end-effector rotation"

        # Check if position is reasonable (within robot workspace)
        pos_magnitude = np.linalg.norm(ee_pose.translation)
        assert 0.3 < pos_magnitude < 1.2, f"End-effector position seems unreasonable: {pos_magnitude}m"

        node.get_logger().info("Kinematics validation passed!")

    finally:
        executor.shutdown()
        if spin_thread and spin_thread.is_alive():
            spin_thread.join(timeout=1.0)
        rclpy.shutdown()
        if node:
            node.destroy_node()
