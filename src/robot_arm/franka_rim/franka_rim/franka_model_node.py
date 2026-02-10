import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from franka_msgs.msg import FrankaRobotState
from arm_interfaces.msg import FrankaModel
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
import pinocchio as pin
import numpy as np
from ament_index_python.packages import get_package_share_directory
import os
import threading
import time
import statistics
from collections import deque

from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterType

from adg_ros2_utils.debug_utils import wait_for_debugger

NODE_NAME = "franka_model_node"

PIN_FRAME = pin.WORLD  # pin.WORLD or pin.LOCAL_WORLD_ALIGNED


class LowPassFilter:
    """Simple first-order low-pass filter: y[k] = alpha * x[k] + (1 - alpha) * y[k-1]"""

    def __init__(self, alpha: float, initial_value=None):
        """
        Args:
            alpha: Filter coefficient (0 < alpha <= 1). Higher = less filtering.
                   alpha = 1 means no filtering.
            initial_value: Initial filtered value (can be scalar or array)
        """
        self.alpha = np.clip(alpha, 0.0, 1.0)
        self.value = initial_value
        self.initialized = initial_value is not None

    def update(self, measurement):
        """Update filter with new measurement."""
        if not self.initialized:
            self.value = np.array(measurement)
            self.initialized = True
        else:
            self.value = self.alpha * np.array(measurement) + (1 - self.alpha) * self.value
        return self.value

    def reset(self, value=None):
        """Reset filter state."""
        self.value = value
        self.initialized = value is not None

    def get(self):
        """Get current filtered value."""
        return self.value if self.initialized else None


class FrankaModelNode(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.get_logger().info("Initializing FrankaModelNode")

        # Declare model update frequency as a ROS 2 parameter
        self.declare_parameter("update_freq", 100.0)  # Hz

        # Declare topic parameters
        self.declare_parameter("input_topic", "/fr3/franka_robot_state_broadcaster/robot_state")
        self.declare_parameter("output_topic", "/fr3_model")
        self.declare_parameter("vel_thres", 0.005)  # Velocity threshold for applied forces
        self.declare_parameter("external_force_estimation_mode", "measured")  # measured, estimated

        self.declare_parameter("log.enabled", False)
        self.declare_parameter("log.timing", False)
        self.declare_parameter("log.period", 1.0)

        self.declare_parameter("rim_axis", "x")  # Axis for RIM ('x', 'y', 'z')

        # Filter parameters (alpha = dt / (dt + tau_filter))
        # For 1000Hz (dt=0.001s), alpha=0.1 gives tau ≈ 9ms
        # alpha=1.0 means no filtering
        self.declare_parameter("filter.enabled", True)
        self.declare_parameter("filter.alpha_q", 0.3)  # Joint positions
        self.declare_parameter("filter.alpha_q_dot", 0.2)  # Joint velocities (more filtering)
        self.declare_parameter("filter.alpha_q_ddot", 0.1)  # Joint accelerations (most filtering)
        self.declare_parameter("filter.alpha_tau", 0.2)  # Joint torques
        self.declare_parameter("filter.alpha_f_ext", 0.15)  # External forces
        self.declare_parameter("filter.alpha_x_ee", 0.3)  # End-effector position
        self.declare_parameter("filter.alpha_v_ee", 0.2)  # End-effector velocity

        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value

        # Loop monitoring
        self._update_freq = self.get_parameter("update_freq").get_parameter_value().double_value
        self._update_period = 1.0 / self._update_freq

        self._rim_axis_str = self.get_parameter("rim_axis").get_parameter_value().string_value
        self._external_force_estimation_mode = (
            self.get_parameter("external_force_estimation_mode").get_parameter_value().string_value.lower()
        )

        self._loop_count: int = 0
        self._last_loop_time = self.get_clock().now().nanoseconds / 1e9

        self._log_enabled = self.get_parameter("log.enabled").get_parameter_value().bool_value
        self._log_timing = self.get_parameter("log.timing").get_parameter_value().bool_value
        self._log_period = self.get_parameter("log.period").get_parameter_value().double_value

        # Filter parameters
        self._filter_enabled = self.get_parameter("filter.enabled").get_parameter_value().bool_value
        self._alpha_q = self.get_parameter("filter.alpha_q").get_parameter_value().double_value
        self._alpha_q_dot = self.get_parameter("filter.alpha_q_dot").get_parameter_value().double_value
        self._alpha_q_ddot = self.get_parameter("filter.alpha_q_ddot").get_parameter_value().double_value
        self._alpha_tau = self.get_parameter("filter.alpha_tau").get_parameter_value().double_value
        self._alpha_f_ext = self.get_parameter("filter.alpha_f_ext").get_parameter_value().double_value
        self._alpha_x_ee = self.get_parameter("filter.alpha_x_ee").get_parameter_value().double_value
        self._alpha_v_ee = self.get_parameter("filter.alpha_v_ee").get_parameter_value().double_value

        # Add detailed timing diagnostics
        self._timer_create_time = time.time()
        self._actual_periods = deque(maxlen=100)

        self._is_initialized = False
        self._model_initialized = False
        self._robot_state_received = False
        self._ext_force_received = False

        # Subscribe to FrankaRobotState
        self._robot_state_sub = self.create_subscription(
            FrankaRobotState,
            input_topic,
            self._robot_state_callback,
            10,
        )

        # Subscribe to cartesian force from OSC PD controller
        self._cartesian_force_sub = self.create_subscription(
            WrenchStamped,
            "/fr3/interface_force",
            self._cartesian_force_callback,
            10,
        )

        # Publisher for model matrices
        self._model_pub = self.create_publisher(FrankaModel, output_topic, 10)

        # Publishers for external force estimates
        self._f_ext_est_pub = self.create_publisher(WrenchStamped, "/f_ext_est", 10)
        self._f_ext_robot_pub = self.create_publisher(WrenchStamped, "/f_ext_robot", 10)

        self._tau_grav_pub = self.create_publisher(JointState, "/tau_grav_est", 10)

        self._robot_model = None  # Pinocchio model
        self._collision_model = None  # Pinocchio collision model
        self._visual_model = None  # Pinocchio visual model
        self._robot_data = None  # Pinocchio data

        self.q = None
        self.q_dot = None
        self.q_ddot = None

        # Last values from robot state
        self._last_q = None
        self._last_q_dot = None
        self._last_q_ddot = None  # Last joint accelerations

        self.v_ee = None  # End-effector velocity (3D vector)

        # Model matrices/attributes
        self.M = None  # Mass matrix (n x n)
        self.c = None  # Coriolis and nonlinear terms (n,)

        # tau = tau_d + tau_grav + tau_f + tau_ext
        self.tau = None  # Measured joint torques, includes gravity and friction (n,)
        self.tau_d = None  # Driving joint torques. What is sent to the control box (n,)
        self.tau_ext = None  # External joint torques from robot, filtered
        self.tau_grav = None  # Gravity torques (n,)
        self.tau_f = None  # Friction torques (n,)

        self.fa = None  # Applied forces to general coordinates (n,)
        self.Ai = None  # Interaction Jacobian (1 x n)
        self.Ai_dot = None  # Derivative of interaction Jacobian (1 x n)
        self.Ai_dot_q_dot = None  # Ai_dot @ q_dot (1,)

        self.J_ee = None  # End-effector Jacobian in base frame (6 x n)
        self.J_dot_ee = None  # Derivative of end-effector Jacobian in base frame (6 x n)

        self._vel_thres = self.get_parameter("vel_thres").get_parameter_value().double_value

        # Contact force estimation
        self.f_ext = None  # External force wrench (6D)
        self._f_ext_estimated = None  # Estimated external forces (6D wrench)
        self._f_ext_robot = None  # Robot's own force estimates (6D wrench)

        self.f_d = None  # Driving Cartesian force from OSC PD controller (6D wrench)

        # Interaction surface normal in base frame
        if self._rim_axis_str == "x":
            self.p_i = np.array([1, 0, 0])
        elif self._rim_axis_str == "y":
            self.p_i = np.array([0, 1, 0])
        elif self._rim_axis_str == "z":
            self.p_i = np.array([0, 0, 1])
        else:
            raise ValueError(f"Invalid rim_axis: {self._rim_axis_str}, must be 'x', 'y', or 'z'")

        self.Di = np.hstack([self.p_i, np.zeros(3)])

        # External force estimation mode
        if self._external_force_estimation_mode not in ["measured", "estimated"]:
            raise ValueError(
                f"Invalid external_force_estimation_mode: {self._external_force_estimation_mode}, must be 'measured' or 'estimated'"
            )

        # Initialize filters (will be properly initialized on first measurement)
        self._filter_q = LowPassFilter(self._alpha_q)
        self._filter_q_dot = LowPassFilter(self._alpha_q_dot)
        self._filter_q_ddot = LowPassFilter(self._alpha_q_ddot)
        self._filter_tau = LowPassFilter(self._alpha_tau)
        self._filter_f_ext_robot = LowPassFilter(self._alpha_f_ext)
        self._filter_x_ee = LowPassFilter(self._alpha_x_ee)
        self._filter_v_ee = LowPassFilter(self._alpha_v_ee)

        # Add parameter callback for runtime updates
        self.add_on_set_parameters_callback(self._parameters_callback)

        # Timer for computing and publishing model matrices
        # Automatically use wall timer when sim_time is enabled to bypass /clock quantization
        use_sim_time = self.get_parameter("use_sim_time").get_parameter_value().bool_value

        if use_sim_time:
            self.get_logger().info("Simulation time detected - using wall timer to bypass /clock quantization (~25ms)")
            self._model_update_timer = self.create_timer(
                self._update_period,
                self._update_model,
                clock=rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.STEADY_TIME),
            )
        else:
            self.get_logger().info("Using standard ROS timer for real-time operation")
            self._model_update_timer = self.create_timer(self._update_period, self._update_model)

        # Log timing and threading information
        filter_status = "ENABLED" if self._filter_enabled else "DISABLED"
        filter_info = ""
        if self._filter_enabled:
            filter_info = (
                f"  - alpha_q: {self._alpha_q}\n"
                f"  - alpha_q_dot: {self._alpha_q_dot}\n"
                f"  - alpha_q_ddot: {self._alpha_q_ddot}\n"
                f"  - alpha_tau: {self._alpha_tau}\n"
                f"  - alpha_f_ext: {self._alpha_f_ext}\n"
                f"  - alpha_x_ee: {self._alpha_x_ee}\n"
                f"  - alpha_v_ee: {self._alpha_v_ee}\n"
            )

        self.get_logger().info(
            f"Parameters\n"
            f"- rim_axis: {self._rim_axis_str}, \n"
            f"- external force estimation mode: {self._external_force_estimation_mode}, \n"
            f"- use sim time: {use_sim_time}, \n"
            f"- frequency: {self._update_freq} Hz ({self._update_period * 1000} ms), \n"
            f"- input_topic: {input_topic}, \n"
            f"- output_topic: {output_topic}, \n"
            f"- vel_thres: {self._vel_thres}, \n"
            f"- filter: {filter_status}\n"
            f"{filter_info}"
            f"- log.enabled: {self._log_enabled}, \n"
            f"- log.period: {self._log_period}, \n"
            f"- log.timing: {self._log_timing}, \n"
        )

        self.get_logger().info(f"Timer created at: {self._timer_create_time}")
        self.get_logger().info(f"Current thread: {threading.current_thread().name}")
        self.get_logger().info(f"Thread ID: {threading.get_ident()}")

        # Add sim time diagnostic
        if use_sim_time:
            self.get_logger().info("Wall timer enabled automatically to prevent timer quantization to /clock rate")

        # Declare URDF filename as a parameter
        self.declare_parameter("robot_urdf_filename", "fr3_franka_hand.urdf")
        # self._load_pinocchio_model_from_file()
        self._load_pinocchio_model_from_param()

        self.get_logger().info(f"Model update frequency: {self._update_freq} Hz ({self._update_period * 1000} ms)")
        self.get_logger().info(
            f"FrankaModelNode initialized successfully - subscribing to {input_topic}, publishing to {output_topic}"
        )

    def _load_pinocchio_model_from_file(self):
        # Get URDF path from package share and parameter
        urdf_filename = self.get_parameter("robot_urdf_filename").get_parameter_value().string_value
        try:
            pkg_share = get_package_share_directory("franka_rim")
            urdf_path = os.path.join(pkg_share, "models", urdf_filename)
            model, collision_model, visual_model = pin.buildModelsFromUrdf(urdf_path)

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

            self._robot_model = model_reduced
            self._collision_model = collision_model
            self._visual_model = visual_model
            self._robot_data = self._robot_model.createData()
            self._model_initialized = True
            self._n = self._robot_model.nv  # Number of degrees of freedom

            self.get_logger().info(f"Loaded Pinocchio model, collision model, and visual model from {urdf_path}")

        except Exception as e:
            self.get_logger().error(f"Failed to load Pinocchio models: {e}")
            self._model_initialized = False

    def _load_pinocchio_model_from_param(self):
        # Load URDF from robot_state_publisher node parameter
        try:
            # Create a service client to get parameters from the robot_state_publisher node
            # Create service client
            get_params_client = self.create_client(GetParameters, "/fr3/robot_state_publisher/get_parameters")

            # Wait for the service to be available
            if not get_params_client.wait_for_service(timeout_sec=10.0):
                raise RuntimeError("GetParameters service for /fr3/robot_state_publisher not available")

            # Create request
            request = GetParameters.Request()
            request.names = ["robot_description"]

            # Call the service
            future = get_params_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

            if not future.done():
                raise RuntimeError("Failed to get robot_description parameter - service call timeout")

            response = future.result()
            if len(response.values) == 0:
                raise ValueError("robot_description parameter not found")

            robot_description_param = response.values[0]

            if robot_description_param.type != ParameterType.PARAMETER_STRING:
                raise ValueError("robot_description parameter is not a string")

            urdf_string = robot_description_param.string_value

            if not urdf_string:
                raise ValueError("robot_description parameter is empty")

            # Build models from URDF string
            # Create temporary file and use ROS package path substitution
            import tempfile

            # Create a temporary URDF file
            with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf", delete=False) as temp_file:
                temp_file.write(urdf_string)
                temp_urdf_path = temp_file.name

            try:
                # Use xacro to process package:// paths if needed, otherwise use direct file loading
                # First try with pinocchio directly
                model, collision_model, visual_model = pin.buildModelsFromUrdf(temp_urdf_path)
            except Exception as e:
                self.get_logger().warn(f"Failed to load URDF with meshes: {e}")
                # Fallback: try building model without visual/collision data
                try:
                    # Create a simplified URDF without mesh references for dynamics only
                    import xml.etree.ElementTree as ET

                    root = ET.fromstring(urdf_string)

                    # Remove visual and collision elements that reference meshes
                    for elem in root.iter():
                        if elem.tag in ["visual", "collision"]:
                            mesh_elem = elem.find(".//mesh")
                            if mesh_elem is not None:
                                elem.clear()  # Remove the visual/collision element

                    # Write simplified URDF
                    simplified_urdf = ET.tostring(root, encoding="unicode")
                    with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf", delete=False) as simple_file:
                        simple_file.write(simplified_urdf)
                        simple_urdf_path = simple_file.name

                    try:
                        model = pin.buildModelFromUrdf(simple_urdf_path)
                        collision_model = None
                        visual_model = None
                        self.get_logger().info("Loaded simplified URDF model without meshes for dynamics calculations")
                    finally:
                        import os

                        os.unlink(simple_urdf_path)

                except Exception as fallback_e:
                    raise RuntimeError(f"Failed to load URDF even without meshes: {fallback_e}")
            finally:
                # Clean up temp file
                import os

                os.unlink(temp_urdf_path)

            # Lock Fingers (same as file-based method)
            jointsToLock = ["fr3_finger_joint1", "fr3_finger_joint2"]
            joints2lock_IDs = []
            for jn in jointsToLock:
                if model.existJointName(jn):
                    joints2lock_IDs.append(model.getJointId(jn))
                else:
                    self.get_logger().warn(f"Joint {jn} does not belong to the model!")

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

            self._robot_model = model_reduced
            self._collision_model = collision_model
            self._visual_model = visual_model
            self._robot_data = self._robot_model.createData()
            self._model_initialized = True
            self._n = self._robot_model.nv  # Number of degrees of freedom

            self.get_logger().info(
                f"Loaded Pinocchio model from /fr3/robot_state_publisher/robot_description parameter with {self._n} DOF"
            )

        except Exception as e:
            self.get_logger().error(f"Failed to load Pinocchio models from /fr3/robot_state_publisher parameter: {e}")
            self.get_logger().info("Falling back to file-based model loading...")
            self._load_pinocchio_model_from_file()
        finally:
            # Clean up the service client
            if "get_params_client" in locals():
                self.destroy_client(get_params_client)

    def _parameters_callback(self, params):
        """Callback for runtime parameter changes."""
        from rclpy.parameter import Parameter

        successful = True
        for param in params:
            param_name = param.name

            # Handle filter enable/disable
            if param_name == "filter.enabled":
                self._filter_enabled = param.value
                self.get_logger().info(f"Filter {'enabled' if param.value else 'disabled'}")

            # Handle alpha parameters - update both the stored value and the filter object
            elif param_name == "filter.alpha_q":
                self._alpha_q = param.value
                self._filter_q.alpha = np.clip(param.value, 0.0, 1.0)
                self.get_logger().info(f"Updated alpha_q to {param.value}")

            elif param_name == "filter.alpha_q_dot":
                self._alpha_q_dot = param.value
                self._filter_q_dot.alpha = np.clip(param.value, 0.0, 1.0)
                self.get_logger().info(f"Updated alpha_q_dot to {param.value}")

            elif param_name == "filter.alpha_q_ddot":
                self._alpha_q_ddot = param.value
                self._filter_q_ddot.alpha = np.clip(param.value, 0.0, 1.0)
                self.get_logger().info(f"Updated alpha_q_ddot to {param.value}")

            elif param_name == "filter.alpha_tau":
                self._alpha_tau = param.value
                self._filter_tau.alpha = np.clip(param.value, 0.0, 1.0)
                self.get_logger().info(f"Updated alpha_tau to {param.value}")

            elif param_name == "filter.alpha_f_ext":
                self._alpha_f_ext = param.value
                self._filter_f_ext_robot.alpha = np.clip(param.value, 0.0, 1.0)
                self.get_logger().info(f"Updated alpha_f_ext to {param.value}")

            elif param_name == "filter.alpha_x_ee":
                self._alpha_x_ee = param.value
                self._filter_x_ee.alpha = np.clip(param.value, 0.0, 1.0)
                self.get_logger().info(f"Updated alpha_x_ee to {param.value}")

            elif param_name == "filter.alpha_v_ee":
                self._alpha_v_ee = param.value
                self._filter_v_ee.alpha = np.clip(param.value, 0.0, 1.0)
                self.get_logger().info(f"Updated alpha_v_ee to {param.value}")

        from rcl_interfaces.msg import SetParametersResult

        return SetParametersResult(successful=successful)

    def _robot_state_callback(self, msg: FrankaRobotState):
        self.get_logger().debug("Received FrankaRobotState message")
        # Only update state, do not compute matrices here
        if not self._model_initialized:
            return

        if not self._robot_state_received:
            self._robot_state_received = True

        if self._last_q is None:
            self.get_logger().info("Robot state received")

        # Extract raw measurements
        raw_q = np.array(msg.measured_joint_state.position)[: self._n]
        raw_q_dot = np.array(msg.measured_joint_state.velocity)[: self._n]
        raw_q_ddot = np.array(msg.ddq_d)[: self._n]
        raw_tau = np.array(msg.measured_joint_state.effort)

        # Read robot's force estimates (base frame)
        wrench = msg.o_f_ext_hat_k.wrench
        raw_f_ext_robot = np.array(
            [wrench.force.x, wrench.force.y, wrench.force.z, wrench.torque.x, wrench.torque.y, wrench.torque.z]
        )

        # End-effector position and velocity
        raw_x_ee = np.array([msg.o_t_ee.pose.position.x, msg.o_t_ee.pose.position.y, msg.o_t_ee.pose.position.z])
        raw_v_ee = np.array([msg.o_dp_ee_d.twist.linear.x, msg.o_dp_ee_d.twist.linear.y, msg.o_dp_ee_d.twist.linear.z])

        # Apply filtering if enabled
        if self._filter_enabled:
            self._last_q = self._filter_q.update(raw_q)
            self._last_q_dot = self._filter_q_dot.update(raw_q_dot)
            self._last_q_ddot = self._filter_q_ddot.update(raw_q_ddot)
            self.tau = self._filter_tau.update(raw_tau)
            self._f_ext_robot = self._filter_f_ext_robot.update(raw_f_ext_robot)
            self.x_ee = self._filter_x_ee.update(raw_x_ee)
            self.v_ee = self._filter_v_ee.update(raw_v_ee)
        else:
            self._last_q = raw_q
            self._last_q_dot = raw_q_dot
            self._last_q_ddot = raw_q_ddot
            self.tau = raw_tau
            self._f_ext_robot = raw_f_ext_robot
            self.x_ee = raw_x_ee
            self.v_ee = raw_v_ee

        # These don't need filtering (command signals)
        self.tau_d = np.array(msg.desired_joint_state.effort)
        self.tau_ext = np.array(msg.tau_ext_hat_filtered.effort)

        # print(f"Vee: {self.v_ee[0]:>10.3f} | {self.v_ee[1]:>10.3f} | {self.v_ee[2]:>10.3f}")
        # print(f"Xee: {self.x_ee[0]:>10.3f} | {self.x_ee[1]:>10.3f} | {self.x_ee[2]:>10.3f}")

    def _cartesian_force_callback(self, msg: WrenchStamped):
        """Callback for cartesian force from OSC PD controller."""
        if not self._ext_force_received:
            self.get_logger().info("Received cartesian force from OSC PD controller")
            self._ext_force_received = True

        # Extract 6D wrench from message
        self.f_d = np.array(
            [
                msg.wrench.force.x,
                msg.wrench.force.y,
                msg.wrench.force.z,
                msg.wrench.torque.x,
                msg.wrench.torque.y,
                msg.wrench.torque.z,
            ]
        )

    # --- Model computation
    def _update_model(self):
        """
        Main function. Runs at the frequency specified by `model_update_freq`.

        """
        if self._model_initialized and self._robot_state_received and self._ext_force_received:
            self._is_initialized = True

        if not self._is_initialized:
            self.get_logger().warn("Cannot update model: missing robot state.", throttle_duration_sec=2)
            return

        current_time = time.time()
        loop_time = self.get_clock().now().nanoseconds / 1e9

        # Track timing
        if self._loop_count > 0:
            actual_period = current_time - self._last_timer_time
            self._actual_periods.append(actual_period)

        self._last_timer_time = current_time

        self.q = self._last_q
        self.q_dot = self._last_q_dot
        self.q_ddot = self._last_q_ddot

        self._compute_matrices()

        self._compute_external_forces()

        self._compute_applied_forces()

        msg = self._build_model_message()
        self._model_pub.publish(msg)

        # Enhanced loop monitoring
        self._log_debug_info()

        self._loop_count += 1
        self._last_loop_time = loop_time

    def _compute_matrices(self) -> None:
        """Compute the model matrices.

        Sets the following class attributes
        -- Kinematics
            J_ee: End-effector Jacobian (6 x n)
            J_dot_ee: Time derivative of end-effector Jacobian (6 x n)
            Ai: Interaction Jacobian (1 x n)
            Ai_dot: Derivative of interaction Jacobian (1 x n)
            Ai_dot_q_dot: Ai_dot @ q_dot (1,)
            v_ee: End-effector velocity (3D vector)  # TODO: From jacobian or robot_state?

        -- Dynamics
            M: Mass matrix (n x n)
            c: Coriolis and nonlinear terms (n,)
        """
        q = self.q
        q_dot = self.q_dot
        q_ddot = self.q_ddot

        # Update Pinocchio data with current joint state
        pin.computeAllTerms(self._robot_model, self._robot_data, q, q_dot)
        pin.updateFramePlacements(self._robot_model, self._robot_data)

        # --- Jacobians ---
        ee_frame = self._robot_model.getFrameId("fr3_hand_tcp")
        self.J_ee = pin.computeFrameJacobian(self._robot_model, self._robot_data, q, ee_frame, PIN_FRAME)
        self.J_dot_ee = pin.frameJacobianTimeVariation(
            self._robot_model, self._robot_data, q, q_dot, ee_frame, PIN_FRAME
        )

        # Interaction Jacobian
        self.Ai = self.Di @ self.J_ee  # Interaction Jacobian (1 x n)

        # Derivative of interaction Jacobian
        self.Ai_dot = self.Di @ self.J_dot_ee

        # Return computed matrices
        self.Ai_dot_q_dot = self.Ai_dot @ q_dot

        v_ee_jaco = self.J_ee @ q_dot  # End-effector velocity from Jacobian

        self.v_ee = v_ee_jaco[:3]  # End-effector velocity (3D vector)
        # print(f"Vz: {self.v_ee[2]:>10.3f} | {v_ee_jaco[2]:>10.3f} | {self.Ai @ q_dot:>10.3f}")

        # --- Dynamics ---
        # Mass matrix
        self.M = self._robot_data.M.copy()

        # Coriolis and nonlinear terms
        q_dot = np.zeros(self._n)
        self.c = self._robot_data.nle - self._robot_data.g  # Remove gravity
        # self.g = self._robot_data.g
        ...

        # # Joint torques
        # if self.tau is None:
        #     self.tau = np.zeros_like(q)
        #     self.get_logger().warn("Joint torques (tau) not set, using zeros")

    def _compute_external_forces(self):
        """Compute estimated external forces at end-effector.

        Returns:
            f_ext_estimated: Estimated external wrench in base frame (6,)

        """
        f_ext = None

        q = self.q
        q_dot = self.q_dot
        q_ddot = self.q_ddot

        if self._external_force_estimation_mode == "estimated":
            #! Estimated
            if self.tau is None:
                self.get_logger().warn("External torque not available for force estimation")
                return np.zeros(6), np.zeros(6)

            # Get end-effector Jacobian in base frame
            ee_frame = self._robot_model.getFrameId("fr3_hand_tcp")
            J_ee = pin.computeFrameJacobian(self._robot_model, self._robot_data, q, ee_frame, pin.LOCAL_WORLD_ALIGNED)

            # Estimate external wrench using: f_ext = pinv(J^T) * tau_ext
            # This assumes that external forces act primarily at the end-effector
            try:
                tau_grav = pin.computeGeneralizedGravity(self._robot_model, self._robot_data, q)
                J_ee_T_pinv = np.linalg.pinv(J_ee.T)

                # q_dot = np.array([1,  0.5,  2,  0.001, -0.   , -0.001,  0.002])
                # c = pin.nle(self._robot_model, self._robot_data, q, q_dot)
                # print(c)
                if q_ddot is None:
                    self.get_logger().error("Joint accelerations (q_ddot) not provided for force estimation")

                tau_inertial = self.M @ q_ddot
                tau_nle = self.c - tau_grav

                tau_ext = self.tau - tau_grav - tau_inertial - tau_nle

                f_ext_estimated = J_ee_T_pinv @ tau_ext

                # OR
                # f_ext_estimated = J_ee_T_pinv @ self.tau_ext

            except np.linalg.LinAlgError:
                self.get_logger().warn("Singular Jacobian in force estimation")
                f_ext_estimated = np.zeros(6)

            f_ext = f_ext_estimated

        if self._external_force_estimation_mode == "measured":
            #! Measured
            f_ext_measured = self._f_ext_robot if self._f_ext_robot is not None else np.zeros(6)

            f_ext = f_ext_measured

        self.f_ext = f_ext

    def _compute_applied_forces(self):
        """Compute applied forces at end-effector.

        TODO: Difference between this and external forces? Coming from friction? Contact?

        The applied force is computed as all the forces, other than the control force, acting on the end-effector.

        $$
        f_a = J (\tau_{meas} - \tau_{des})
        $$

        Args:
            q: Joint positions (n,)
            q_dot: Joint velocities (n,)
            q_ddot: Joint accelerations (n,)
        """
        q = self.q
        q_dot = self.q_dot
        q_ddot = self.q_ddot

        # #! Block pushing
        # if np.abs(self.v_ee[0]) > self._vel_thres:
        #     # Only friction in x for now
        #     f_friction = np.array([self.f_ext_robot[0], 0, 0])  # Friction force in x direction

        #     # friction_val = 10 * np.sign(self.v_ee[0])  # Friction proportional to velocity
        #     # f_friction = np.array([friction_val, 0, 0])  # Friction force in x direction

        #     # print("Slip")
        #     fa_slip = self.J_ee.T[:, :3] @ f_friction

        #     self.fa = fa_slip

        # else:
        #     # print("Stick")
        #     fa_stick = np.zeros_like(q)  # Placeholder for stick forces
        #     self.fa = fa_stick

        # #! Free movement
        # ee_frame = self._robot_model.getFrameId("fr3_hand_tcp")
        # J_ee = pin.computeFrameJacobian(self._robot_model, self._robot_data, q, ee_frame, pin.LOCAL_WORLD_ALIGNED)

        # tau_grav = pin.computeGeneralizedGravity(self._robot_model, self._robot_data, q)
        # J_ee_T_pinv = np.linalg.pinv(J_ee.T)

        # # q_dot = np.array([1,  0.5,  2,  0.001, -0.   , -0.001,  0.002])
        # # c = pin.nle(self._robot_model, self._robot_data, q, q_dot)
        # # print(c)
        # if q_ddot is None:
        #     self.get_logger().error("Joint accelerations (q_ddot) not provided for force estimation")

        # tau_inertial = self.M @ q_ddot
        # tau_nle = self.c - tau_grav

        # tau_applied = self.tau - tau_grav

        # self.fa = J_ee_T_pinv @ tau_applied
        # self.fa = self.fa[0].reshape(
        #     1,
        # )

        #! Applied forces
        # self.fa = self.tau_meas - self.tau_des
        # self.fa = self.fa.reshape(
        #     self._n,
        # )

        # Zero, for debugging
        self.fa = np.zeros(self._n).reshape(
            self._n,
        )

    def _build_model_message(self):
        """Build a FrankaModel message from the computed/measured kinematics and dynamics.

        Returns:
            FrankaModel: The constructed message with all matrix data
        """
        # Prepare FrankaModel message
        msg = FrankaModel()

        msg.header.stamp = self.get_clock().now().to_msg()

        # --- Kinematics
        n = self._robot_model.nv
        msg.n = n
        msg.q = self.q.tolist()
        msg.q_dot = self.q_dot.tolist()

        msg.x_ee = self.x_ee.tolist()  # End-effector position
        msg.v_ee = self.v_ee.tolist()  # End-effector velocity

        # --- Dynamics
        # Matrices
        msg.mass_matrix = self.M.flatten().tolist()
        msg.coriolis = self.c.tolist()
        msg.ai = self.Ai.flatten().tolist()
        msg.ai_dot_q_dot = self.Ai_dot_q_dot.flatten().tolist()
        msg.jacobian = self.J_ee.flatten().tolist()

        msg.tau = self.tau.tolist()
        msg.tau_d = self.tau_d.tolist()
        msg.tau_ext = self.tau_ext.tolist()

        # Forces
        msg.f_ext_ee = self.f_ext.tolist()
        msg.f_d_ee = self.f_d.tolist()
        msg.f_a = self.fa.tolist()

        return msg

    # def _publish_external_forces(self, f_ext_estimated):
    #     """Publish external force estimates to separate topics for visualization."""
    #     current_time = self.get_clock().now()

    #     # Publish estimated external forces
    #     est_msg = WrenchStamped()
    #     est_msg.header.stamp = current_time.to_msg()
    #     est_msg.header.frame_id = "fr3_link0"  # Base frame
    #     est_msg.wrench.force.x = float(f_ext_estimated[0])
    #     est_msg.wrench.force.y = float(f_ext_estimated[1])
    #     est_msg.wrench.force.z = float(f_ext_estimated[2])
    #     est_msg.wrench.torque.x = float(f_ext_estimated[3])
    #     est_msg.wrench.torque.y = float(f_ext_estimated[4])
    #     est_msg.wrench.torque.z = float(f_ext_estimated[5])
    #     self._f_ext_est_pub.publish(est_msg)

    # --- Utilities
    def _log_debug_info(self):
        """
        To log debug information about the model update loop.
        Logs every `log_period` seconds.
        """

        if not self._log_enabled:
            return

        if self._loop_count % (self._log_period * self._update_freq) == 0 and self._loop_count > 0:
            avg_period = statistics.mean(self._actual_periods) * 1000  # Convert to ms
            # min_period = min(self._actual_periods) * 1000
            # max_period = max(self._actual_periods) * 1000
            # std_period = statistics.stdev(self._actual_periods) * 1000 if len(self._actual_periods) > 1 else 0

            if avg_period > self._update_period * 1.5 * 1000:
                self.get_logger().warn(
                    f"High update period detected: {avg_period:.2f}ms (target: {self._update_period * 1000:.2f}ms)"
                )

            if self._log_timing:
                self.get_logger().info(
                    f"Timer diagnostics - Avg: {avg_period:.2f}ms | Target: {self._update_period * 1000:.2f}ms | "
                    # f"Min: {min_period:.2f}ms | Max: {max_period:.2f}ms | "
                    # f"Std: {std_period:.2f}ms | Thread: {threading.current_thread().name}"
                )


def main(args=None):
    wait_for_debugger(NODE_NAME)  # Wait for debugger if env variables is set

    rclpy.init(args=args)

    # Force single-threaded executor with custom configuration
    node = FrankaModelNode()

    # Check if we're running from launch file by looking for specific environment variables
    import os

    launched_from_file = (
        os.environ.get("_LAUNCH_SHELL", None) is not None or os.environ.get("ROS_LAUNCH_LOG_DIR", None) is not None
    )

    if launched_from_file:
        node.get_logger().info("Detected launch file execution - using optimized executor configuration")

        # Use single-threaded executor with custom settings for better timer resolution
        executor = SingleThreadedExecutor()
        executor.add_node(node)

        try:
            # Spin with shorter timeout for better timer precision
            while rclpy.ok():
                executor.spin_once(timeout_sec=0.001)  # 1ms timeout for better precision
        except KeyboardInterrupt:
            pass
        finally:
            executor.remove_node(node)
            node.destroy_node()
            rclpy.shutdown()
    else:
        # Standard spin for ros2 run
        node.get_logger().info("Using standard spin execution")
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
