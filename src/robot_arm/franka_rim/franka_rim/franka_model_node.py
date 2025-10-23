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

from adg_ros2_utils.debug_utils import wait_for_debugger

NODE_NAME = "franka_model_node"

PIN_FRAME = pin.LOCAL_WORLD_ALIGNED  # pin.WORLD or pin.LOCAL_WORLD_ALIGNED


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

        self.declare_parameter("log.enabled", False)
        self.declare_parameter("log.timing", False)
        self.declare_parameter("log.period", 1.0)

        self.declare_parameter("rim_axis", "x")  # Axis for RIM ('x', 'y', 'z')

        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value

        # Loop monitoring
        self._update_freq = self.get_parameter("update_freq").get_parameter_value().double_value
        self._update_period = 1.0 / self._update_freq

        self._rim_axis_str = self.get_parameter("rim_axis").get_parameter_value().string_value

        self._loop_count: int = 0
        self._last_loop_time = self.get_clock().now().nanoseconds / 1e9

        self._log_enabled = self.get_parameter("log.enabled").get_parameter_value().bool_value
        self._log_timing = self.get_parameter("log.timing").get_parameter_value().bool_value
        self._log_period = self.get_parameter("log.period").get_parameter_value().double_value

        # Add detailed timing diagnostics
        self._timer_create_time = time.time()
        self._actual_periods = deque(maxlen=100)

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
            "/fr3/osc_pd_controller/cartesian_force",
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

        self._last_q = None
        self._last_dq = None
        self._last_ddq = None  # Last joint accelerations

        self.v_ee = None  # End-effector velocity (3D vector)

        self._model_loaded = False

        # Model matrices/attributes
        self.M = None  # Mass matrix (n x n)
        self.c = None  # Coriolis and nonlinear terms (n,)

        self.tau_meas = None  # Measured joint torques (n,) (include gravity and friction)
        self.tau_des = None  # Desired joint torques (n,) (sent to the control box)
        self.tau_ext = None  # External joint torques from robot, filtered

        self.fa = None  # Applied forces to general coordinates (n,)
        self.Ai = None  # Interaction Jacobian (1 x n)
        self.Ai_dot = None  # Derivative of interaction Jacobian (1 x n)
        self.Ai_dot_q_dot = None  # Ai_dot @ q_dot (1,)

        self.J_ee = None  # End-effector Jacobian in base frame (6 x n)
        self.J_dot_ee = None  # Derivative of end-effector Jacobian in base frame (6 x n)

        self._vel_thres = self.get_parameter("vel_thres").get_parameter_value().double_value

        # Contact force estimation
        self.f_ext_estimated = None  # Estimated external forces (6D wrench)
        self.f_ext_robot = None  # Robot's own force estimates (6D wrench)
        self.cartesian_force = None  # Cartesian force from OSC PD controller (6D wrench)

        # Interaction surface normal in base frame
        if self._rim_axis_str == "x":
            self.p_i = np.array([-1, 0, 0])
        elif self._rim_axis_str == "y":
            self.p_i = np.array([0, -1, 0])
        elif self._rim_axis_str == "z":
            self.p_i = np.array([0, 0, -1])
        else:
            raise ValueError(f"Invalid rim_axis: {self._rim_axis_str}, must be 'x', 'y', or 'z'")

        self.Di = np.hstack([self.p_i, np.zeros(3)])

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
        self.get_logger().info(
            f"Parameters\n"
            f"- rim_axis: {self._rim_axis_str}, \n"
            f"- use sim time: {use_sim_time}, \n"
            f"- frequency: {self._update_freq} Hz ({self._update_period * 1000} ms), \n"
            f"- input_topic: {input_topic}, \n"
            f"- output_topic: {output_topic}, \n"
            f"- vel_thres: {self._vel_thres}, \n"
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
        self._load_pinocchio_model()

        self.get_logger().info(f"Model update frequency: {self._update_freq} Hz ({self._update_period * 1000} ms)")
        self.get_logger().info(
            f"FrankaModelNode initialized successfully - subscribing to {input_topic}, publishing to {output_topic}"
        )

    def _load_pinocchio_model(self):
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
            self._model_loaded = True
            self._n = self._robot_model.nv  # Number of degrees of freedom

            self.get_logger().info(f"Loaded Pinocchio model, collision model, and visual model from {urdf_path}")

        except Exception as e:
            self.get_logger().error(f"Failed to load Pinocchio models: {e}")
            self._model_loaded = False

    def _robot_state_callback(self, msg: FrankaRobotState):
        self.get_logger().debug("Received FrankaRobotState message")
        # Only update state, do not compute matrices here
        if not self._model_loaded:
            return

        if self._last_q is None:
            self.get_logger().info("Robot state received")

        self._last_q = np.array(msg.measured_joint_state.position)[: self._n]
        self._last_dq = np.array(msg.measured_joint_state.velocity)[: self._n]
        self._last_ddq = np.array(msg.ddq_d)[: self._n]

        # Set tau from measured_joint_state.effort
        self.tau_meas = np.array(msg.measured_joint_state.effort)
        self.tau_des = np.array(msg.desired_joint_state.effort)
        self.tau_ext = np.array(msg.tau_ext_hat_filtered.effort)

        # Read robot's force estimates (base frame)
        wrench = msg.o_f_ext_hat_k.wrench
        self.f_ext_robot = np.array(
            [wrench.force.x, wrench.force.y, wrench.force.z, wrench.torque.x, wrench.torque.y, wrench.torque.z]
        )

        # End-effector position and velocity
        self.x_ee = np.array([msg.o_t_ee.pose.position.x, msg.o_t_ee.pose.position.y, msg.o_t_ee.pose.position.z])
        self.v_ee = np.array([msg.o_dp_ee_d.twist.linear.x, msg.o_dp_ee_d.twist.linear.y, msg.o_dp_ee_d.twist.linear.z])
        # print(f"Vee: {self.v_ee[0]:>10.3f} | {self.v_ee[1]:>10.3f} | {self.v_ee[2]:>10.3f}")
        # print(f"Xee: {self.x_ee[0]:>10.3f} | {self.x_ee[1]:>10.3f} | {self.x_ee[2]:>10.3f}")

    def _cartesian_force_callback(self, msg: WrenchStamped):
        """Callback for cartesian force from OSC PD controller."""
        self.get_logger().debug("Received cartesian force from OSC PD controller")

        # Extract 6D wrench from message
        self.cartesian_force = np.array(
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
        current_time = time.time()
        loop_time = self.get_clock().now().nanoseconds / 1e9

        # Track timing
        if self._loop_count > 0:
            actual_period = current_time - self._last_timer_time
            self._actual_periods.append(actual_period)

        self._last_timer_time = current_time

        if not self._model_loaded or self._last_q is None or self._last_dq is None:
            self.get_logger().warn("Cannot update model: missing robot state.", throttle_duration_sec=2)
            return

        q = self._last_q
        q_dot = self._last_dq
        q_ddot = self._last_ddq

        self._compute_matrices(q, q_dot, q_ddot)

        # self._compute_external_forces(q, q_dot, q_ddot)

        self._compute_applied_forces(q, q_dot, q_ddot)

        msg = self._build_model_message(
            q, q_dot, self.x_ee, self.v_ee, self.M, self.c, self.tau_meas, self.Ai, self.Ai_dot_q_dot, self.fa
        )
        self._model_pub.publish(msg)
        # self.get_logger().info("Published FrankaModel message to fr3_model topic")

        # Enhanced loop monitoring
        self._log_debug_info()

        self._loop_count += 1
        self._last_loop_time = loop_time

    def _compute_matrices(self, q, q_dot, q_ddot=None) -> None:
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

        Args:
            q: Joint positions (n,)
            q_dot: Joint velocities (n,)
            q_ddot: Joint accelerations (n,)
        """
        # Update Pinocchio data with current joint state
        pin.forwardKinematics(self._robot_model, self._robot_data, q, q_dot)
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

        v_ee_jaco = self.J_ee @ q_dot  # Interaction velocity in x direction
        v_ee_x_int = self.Ai @ q_dot  # Interaction velocity in x direction

        # print(f"Vx: {self.v_ee[0]:>10.3f} | {v_ee_jaco[0]:>10.3f} | {v_ee_x_int:>10.3f}")
        self.v_ee = v_ee_jaco[:3]  # End-effector velocity (3D vector)

        # --- Dynamics ---
        # Mass matrix
        self.M = pin.crba(self._robot_model, self._robot_data, q)
        # Coriolis and nonlinear terms
        self.c = pin.nle(self._robot_model, self._robot_data, q, q_dot)

        # # Joint torques
        # if self.tau is None:
        #     self.tau = np.zeros_like(q)
        #     self.get_logger().warn("Joint torques (tau) not set, using zeros")

    def _compute_external_forces(self, q, q_dot, q_ddot=None):
        """Compute estimated external forces at end-effector.

        Args:
            q: Joint positions (n,)
            q_dot: Joint velocities (n,)
            q_ddot: Joint accelerations (n,)

        Returns:
            f_ext_estimated: Estimated external wrench in base frame (6,)

        """
        mode = "MEASURED"  # MEASURED, ESTIMATED
        f_ext = None

        if mode == "ESTIMATED":
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

        if mode == "MEASURED":
            #! Measured
            f_ext_measured = self.f_ext_robot if self.f_ext_robot is not None else np.zeros(6)

            f_ext = f_ext_measured

        #! Return
        self._publish_external_forces(f_ext)

        return f_ext

    def _compute_applied_forces(self, q, q_dot, q_ddot=None):
        """Compute applied forces at end-effector.

        The applied force is computed as all the forces, other than the control force, acting on the end-effector.

        $$
        f_a = J (\tau_{meas} - \tau_{des})
        $$

        Args:
            q: Joint positions (n,)
            q_dot: Joint velocities (n,)
            q_ddot: Joint accelerations (n,)
        """
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

    def _build_model_message(self, q, q_dot, x_ee, v_ee, M, c, tau, Ai, Ai_dot_q_dot, fa):
        """Build a FrankaModel message from the computed model matrices.

        Args:
            M: Mass matrix (n x n)
            c: Coriolis vector (n,)
            tau: Torque vector (n,)
            Ai: Analytical Jacobian matrix (1 x n)
            Ai_dot_q_dot: Ai_dot @ q_dot (1,)
            fa: Applied forces (n,)

        Returns:
            FrankaModel: The constructed message with all matrix data
        """
        # Prepare FrankaModel message
        msg = FrankaModel()

        msg.header.stamp = self.get_clock().now().to_msg()

        n = self._robot_model.nv
        msg.n = n
        msg.q = q.tolist()
        msg.q_dot = q_dot.tolist()

        msg.x_ee = x_ee.tolist()  # End-effector position
        msg.v_ee = v_ee.tolist()  # End-effector velocity

        msg.mass_matrix = M.flatten().tolist()
        msg.coriolis = c.tolist()
        msg.tau = tau.tolist()
        msg.ai = Ai.flatten().tolist()
        msg.ai_dot_q_dot = Ai_dot_q_dot.flatten().tolist()

        msg.fa = fa.tolist()

        # # Add force estimation data
        # msg.f_ext_estimated = f_ext_estimated.tolist()
        # if self.f_ext_robot is not None:
        #     msg.f_ext_robot = self.f_ext_robot.tolist()
        # else:
        #     msg.f_ext_robot = [0.0] * 6  # Default to zeros if not available

        return msg

    def _publish_external_forces(self, f_ext_estimated):
        """Publish external force estimates to separate topics for visualization."""
        current_time = self.get_clock().now()

        # Publish estimated external forces
        est_msg = WrenchStamped()
        est_msg.header.stamp = current_time.to_msg()
        est_msg.header.frame_id = "fr3_link0"  # Base frame
        est_msg.wrench.force.x = float(f_ext_estimated[0])
        est_msg.wrench.force.y = float(f_ext_estimated[1])
        est_msg.wrench.force.z = float(f_ext_estimated[2])
        est_msg.wrench.torque.x = float(f_ext_estimated[3])
        est_msg.wrench.torque.y = float(f_ext_estimated[4])
        est_msg.wrench.torque.z = float(f_ext_estimated[5])
        self._f_ext_est_pub.publish(est_msg)

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
