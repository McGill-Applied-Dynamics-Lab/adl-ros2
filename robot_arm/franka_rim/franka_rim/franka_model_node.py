import rclpy
from rclpy.node import Node
from franka_msgs.msg import FrankaRobotState
from arm_interfaces.msg import FrankaModel
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

        self._robot_model = None  # Pinocchio model
        self._collision_model = None  # Pinocchio collision model
        self._visual_model = None  # Pinocchio visual model
        self._robot_data = None  # Pinocchio data

        self._last_q = None
        self._last_dq = None

        self._model_loaded = False

        # Model matrices/attributes
        self.M = None  # Mass matrix (n x n)
        self.c = None  # Coriolis and nonlinear terms (n,)
        self.tau = None  # Joint torques (n,)
        self.fa = None  # Applied forces (n,)
        self.Ai = None  # Interaction Jacobian (1 x n)
        self.Ai_dot = None  # Derivative of interaction Jacobian (1 x n)
        self.Ai_dot_q_dot = None  # Ai_dot @ dq (1,)

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

            self.get_logger().info(f"Loaded Pinocchio model, collision model, and visual model from {urdf_path}")

        except Exception as e:
            self.get_logger().error(f"Failed to load Pinocchio models: {e}")
            self._model_loaded = False

    def _robot_state_callback(self, msg: FrankaRobotState):
        self.get_logger().debug("Received FrankaRobotState message")
        # Only update state, do not compute matrices here
        if not self._model_loaded:
            return

        self._last_q = np.array(msg.measured_joint_state.position)
        self._last_dq = np.array(msg.measured_joint_state.velocity)

        # Set tau from measured_joint_state.effort
        self.tau = np.array(msg.measured_joint_state.effort)

        # Read external torque estimates from robot
        if len(msg.tau_ext_hat_filtered.effort) > 0:
            self.tau_ext = np.array(msg.tau_ext_hat_filtered.effort)

        # Read robot's force estimates (base frame)
        if hasattr(msg.o_f_ext_hat_k, "wrench"):
            wrench = msg.o_f_ext_hat_k.wrench
            self.f_ext_robot = np.array(
                [wrench.force.x, wrench.force.y, wrench.force.z, wrench.torque.x, wrench.torque.y, wrench.torque.z]
            )

    def _compute_contact_forces(self, q, dq):
        """Compute estimated contact forces using momentum observer.

        Args:
            q: Joint positions (n,)
            dq: Joint velocities (n,)

        Returns:
            f_ext_estimated: Estimated external wrench in base frame (6,)
        """
        if self.tau is None:
            self.get_logger().warn("External torque not available for force estimation")
            return np.zeros(6)

        # Get end-effector Jacobian in base frame
        ee_frame = self._robot_model.getFrameId("fr3_hand_tcp")
        J_ee = pin.computeFrameJacobian(self._robot_model, self._robot_data, q, ee_frame, pin.WORLD)

        # Estimate external wrench using: f_ext = pinv(J^T) * tau_ext
        # This assumes that external forces act primarily at the end-effector
        try:
            tau_grav = pin.computeGeneralizedGravity(self._robot_model, self._robot_data, q)
            J_ee_T_pinv = np.linalg.pinv(J_ee.T)

            f_ext_estimated = J_ee_T_pinv @ (self.tau - tau_grav)

            # OR
            # f_ext_estimated = J_ee_T_pinv @ self.tau_ext

        except np.linalg.LinAlgError:
            self.get_logger().warn("Singular Jacobian in force estimation")
            f_ext_estimated = np.zeros(6)

        return f_ext_estimated

    def _update_model(self, q, dq):
        """Compute the model matrices M, c, tau, Ai, Ai_dot, and Ai_dot_q_dot.
        Args:
            q: Joint positions (n,)
            dq: Joint velocities (n,)
        Returns:
            M: Mass matrix (n x n)
            c: Coriolis and nonlinear terms (n,)
            tau: Joint torques (n,)
            Ai: Interaction Jacobian (1 x n)
            Ai_dot: Derivative of interaction Jacobian (1 x n)
            Ai_dot_q_dot: Ai_dot @ dq (1,)
            fa: Applied forces (n,)
        """
        # Update Pinocchio data with current joint state
        pin.forwardKinematics(self._robot_model, self._robot_data, q, dq)
        pin.updateFramePlacements(self._robot_model, self._robot_data)

        # Jacobian
        ee_frame = self._robot_model.getFrameId("fr3_hand_tcp")
        J_ee = pin.computeFrameJacobian(self._robot_model, self._robot_data, q, ee_frame, pin.WORLD)
        J_dot_ee = pin.frameJacobianTimeVariation(self._robot_model, self._robot_data, q, dq, ee_frame, pin.WORLD)

        Ai = self.Di @ J_ee  # Interaction Jacobian (1 x n)
        Ai_dot = self.Di @ J_dot_ee

        # Mass matrix
        self.M = pin.crba(self._robot_model, self._robot_data, q)
        # Coriolis and nonlinear terms
        self.c = pin.rnea(self._robot_model, self._robot_data, q, dq, np.zeros_like(q))

        # Joint torques (placeholder, to be computed from control law or input)
        if self.tau is None:
            self.tau = np.zeros_like(q)
            self.get_logger().warn("Joint torques (tau) not set, using zeros")

        # Interaction Jacobian (placeholder, to be computed for a specific frame or task)
        self.Ai = Ai

        # Derivative of interaction Jacobian (placeholder)
        self.Ai_dot = Ai_dot

        # Return computed matrices
        self.Ai_dot_q_dot = self.Ai_dot @ dq

        # Applied forces
        # TODO: Compute applied forces
        # self.f_ext_estimated = self._compute_contact_forces(q, dq)
        self.fa = np.zeros_like(q)

    def _compute_and_publish_model(self):
        if not self._model_loaded or self._last_q is None or self._last_dq is None:
            return

        q = self._last_q
        dq = self._last_dq

        self._update_model(q, dq)

        msg = self._build_model_message(self.M, self.c, self.tau, self.Ai, self.Ai_dot_q_dot, self.fa)
        self._model_pub.publish(msg)
        self.get_logger().info("Published FrankaModel message to fr3_model topic")

    def _build_model_message(self, M, c, tau, Ai, Ai_dot_q_dot, fa):
        """Build a FrankaModel message from the computed model matrices.

        Args:
            M: Mass matrix (n x n)
            c: Coriolis vector (n,)
            tau: Torque vector (n,)
            Ai: Analytical Jacobian matrix (1 x n)
            Ai_dot_q_dot: Ai_dot @ dq (1,)
            fa: Applied forces (n,)

        Returns:
            FrankaModel: The constructed message with all matrix data
        """
        # Prepare FrankaModel message
        msg = FrankaModel()

        msg.header.stamp = self.get_clock().now().to_msg()

        n = self._robot_model.nv
        msg.n = n
        msg.mass_matrix = M.flatten().tolist()
        msg.coriolis = c.tolist()
        msg.tau = tau.tolist()
        msg.ai = Ai.flatten().tolist()
        msg.ai_dot_q_dot = Ai_dot_q_dot.flatten().tolist()

        msg.fa = self.fa.tolist()

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
