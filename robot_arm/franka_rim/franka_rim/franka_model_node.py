import rclpy
from rclpy.node import Node
from franka_msgs.msg import FrankaRobotState
from arm_interfaces.msg import FrankaModel
from geometry_msgs.msg import WrenchStamped
import pinocchio as pin
import numpy as np
from ament_index_python.packages import get_package_share_directory
import os


class FrankaModelNode(Node):
    def __init__(self):
        super().__init__("franka_model_node")
        self.get_logger().info("Initializing FrankaModelNode")

        # Declare model update frequency as a ROS 2 parameter
        self.declare_parameter("model_update_freq", 100.0)  # Hz
        update_freq = self.get_parameter("model_update_freq").get_parameter_value().double_value
        model_update_timer_period = 1.0 / update_freq
        self.get_logger().info(f"Model update frequency: {update_freq} Hz")

        # Declare topic parameters
        self.declare_parameter("input_topic", "/franka_robot_state_broadcaster/robot_state")
        self.declare_parameter("output_topic", "/fr3_model")
        self.declare_parameter("vel_thres", 0.005)  # Velocity threshold for applied forces

        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value

        # Subscribe to FrankaRobotState
        self._robot_state_sub = self.create_subscription(
            FrankaRobotState,
            input_topic,
            self._robot_state_callback,
            10,
        )

        # Publisher for model matrices
        self._model_pub = self.create_publisher(FrankaModel, output_topic, 10)

        # Publishers for external force estimates
        self._f_ext_est_pub = self.create_publisher(WrenchStamped, "/f_ext_est", 10)
        self._f_ext_robot_pub = self.create_publisher(WrenchStamped, "/f_ext_robot", 10)

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
        self.tau = None  # Joint torques (n,)
        self.fa = None  # Applied forces (n,)
        self.Ai = None  # Interaction Jacobian (1 x n)
        self.Ai_dot = None  # Derivative of interaction Jacobian (1 x n)
        self.Ai_dot_q_dot = None  # Ai_dot @ q_dot (1,)

        self.J_ee = None  # End-effector Jacobian in base frame (6 x n)
        self.J_dot_ee = None  # Derivative of end-effector Jacobian in base frame (6 x n)

        self._vel_thres = self.get_parameter("vel_thres").get_parameter_value().double_value

        # Contact force estimation
        self.f_ext_estimated = None  # Estimated external forces (6D wrench)
        self.f_ext_robot = None  # Robot's own force estimates (6D wrench)
        self.tau_ext = None  # External joint torques from robot

        self.p_i = np.array([-1, 0, 0])  # Interaction surface normal in base frame
        self.Di = np.hstack([self.p_i, np.zeros(3)])

        # Timer for computing and publishing model matrices
        self._model_timer = self.create_timer(model_update_timer_period, self._compute_and_publish_model)

        # Declare URDF filename as a parameter
        self.declare_parameter("robot_urdf_filename", "fr3_franka_hand.urdf")
        self._load_pinocchio_model()

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

        self._last_q = np.array(msg.measured_joint_state.position)[: self._n]
        self._last_dq = np.array(msg.measured_joint_state.velocity)[: self._n]
        self._last_ddq = np.array(msg.ddq_d)[: self._n]

        # Set tau from measured_joint_state.effort
        self.tau = np.array(msg.measured_joint_state.effort)

        # Read external torque estimates from robot
        if len(msg.tau_ext_hat_filtered.effort) > 0:
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
            J_ee = pin.computeFrameJacobian(self._robot_model, self._robot_data, q, ee_frame, pin.WORLD)

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

    def _compute_applied_forces(self, q, q_dot):
        """Compute applied forces at end-effector.

        Args:
            q: Joint positions (n,)
            q_dot: Joint velocities (n,)
        """
        if np.abs(self.v_ee[0]) > self._vel_thres:
            # Only friction in x for now
            f_friction = np.array([self.f_ext_robot[0], 0, 0])  # Friction force in x direction

            # friction_val = 10 * np.sign(self.v_ee[0])  # Friction proportional to velocity
            # f_friction = np.array([friction_val, 0, 0])  # Friction force in x direction

            # print("Slip")
            fa_slip = self.J_ee.T[:, :3] @ f_friction

            self.fa = fa_slip

        else:
            # print("Stick")
            fa_stick = np.zeros_like(q)  # Placeholder for stick forces
            self.fa = fa_stick

        self.fa = np.zeros_like(q)

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

    def _update_model(self, q, q_dot, q_ddot=None):
        """Compute the model matrices M, c, tau, Ai, Ai_dot, and Ai_dot_q_dot.
        Args:
            q: Joint positions (n,)
            q_dot: Joint velocities (n,)
            q_ddot: Joint accelerations (n,)
        Returns:
            M: Mass matrix (n x n)
            c: Coriolis and nonlinear terms (n,)
            tau: Joint torques (n,)
            Ai: Interaction Jacobian (1 x n)
            Ai_dot: Derivative of interaction Jacobian (1 x n)
            Ai_dot_q_dot: Ai_dot @ q_dot (1,)
            fa: Applied forces (n,)
        """
        # Update Pinocchio data with current joint state
        pin.forwardKinematics(self._robot_model, self._robot_data, q, q_dot)
        pin.updateFramePlacements(self._robot_model, self._robot_data)

        # --- Jacobians ---
        ee_frame = self._robot_model.getFrameId("fr3_hand_tcp")
        self.J_ee = pin.computeFrameJacobian(self._robot_model, self._robot_data, q, ee_frame, pin.WORLD)
        self.J_dot_ee = pin.frameJacobianTimeVariation(
            self._robot_model, self._robot_data, q, q_dot, ee_frame, pin.WORLD
        )

        # Interaction Jacobian (placeholder, to be computed for a specific frame or task)
        self.Ai = self.Di @ self.J_ee  # Interaction Jacobian (1 x n)

        # Derivative of interaction Jacobian
        self.Ai_dot = self.Di @ self.J_dot_ee

        # Return computed matrices
        self.Ai_dot_q_dot = self.Ai_dot @ q_dot

        v_ee_jaco = self.J_ee @ q_dot  # Interaction velocity in x direction
        v_ee_x_int = self.Ai @ q_dot  # Interaction velocity in x direction

        # print(f"Vx: {self.v_ee[0]:>10.3f} | {v_ee_jaco[0]:>10.3f} | {v_ee_x_int:>10.3f}")

        # --- Dynamics ---
        # Mass matrix
        self.M = pin.crba(self._robot_model, self._robot_data, q)
        # Coriolis and nonlinear terms
        self.c = pin.nle(self._robot_model, self._robot_data, q, q_dot)

        # Joint torques
        if self.tau is None:
            self.tau = np.zeros_like(q)
            self.get_logger().warn("Joint torques (tau) not set, using zeros")

        # Forces
        self._compute_external_forces(q, q_dot, q_ddot)
        self._compute_applied_forces(q, q_dot)
        # self.f_ext_estimated = self._compute_contact_forces(q, q_dot)
        self.fa = np.zeros_like(q)

    def _compute_and_publish_model(self):
        if not self._model_loaded or self._last_q is None or self._last_dq is None:
            self.get_logger().warn("Cannot update model: missing robot state.", throttle_duration_sec=2)

            return

        q = self._last_q
        q_dot = self._last_dq
        q_ddot = self._last_ddq

        self._update_model(q, q_dot, q_ddot)

        msg = self._build_model_message(
            q, q_dot, self.x_ee, self.v_ee, self.M, self.c, self.tau, self.Ai, self.Ai_dot_q_dot, self.fa
        )
        self._model_pub.publish(msg)
        # self.get_logger().info("Published FrankaModel message to fr3_model topic")

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


def main(args=None):
    rclpy.init(args=args)
    node = FrankaModelNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
