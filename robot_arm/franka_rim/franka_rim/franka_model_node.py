import rclpy
from rclpy.node import Node
from franka_msgs.msg import FrankaRobotState
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

        # Subscribe to FrankaRobotState
        self._robot_state_sub = self.create_subscription(
            FrankaRobotState,
            "/franka_robot_state_broadcaster/robot_state",
            self._robot_state_callback,
            10,
        )

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
        self.Ai = None  # Interaction Jacobian (1 x n)
        self.Ai_dot = None  # Derivative of interaction Jacobian (1 x n)

        # Timer for computing and publishing model matrices
        self._model_timer = self.create_timer(model_update_timer_period, self._compute_and_publish_model)

        # TODO: Publisher for model matrices (to be defined)
        # self._model_pub = self.create_publisher(...)
        # Load URDF and build Pinocchio model

        # Declare URDF filename as a parameter
        self.declare_parameter("robot_urdf_filename", "fr3_franka_hand.urdf")
        self._load_pinocchio_model()

        self.get_logger().info("FrankaModelNode initialized successfully")

    def _load_pinocchio_model(self):
        # Get URDF path from package share and parameter
        urdf_filename = self.get_parameter("robot_urdf_filename").get_parameter_value().string_value
        try:
            pkg_share = get_package_share_directory("franka_rim")
            urdf_path = os.path.join(pkg_share, "models", urdf_filename)
            model, collision_model, visual_model = pin.buildModelsFromUrdf(urdf_path)
            self._robot_model = model
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

    def _compute_model_matrices(self, q, dq):
        # Mass matrix
        self.M = pin.crba(self._robot_model, self._robot_data, q)
        # Coriolis and nonlinear terms
        self.c = pin.rnea(self._robot_model, self._robot_data, q, dq, np.zeros_like(q))

        # Joint torques (placeholder, to be computed from control law or input)
        if self.tau is None:
            self.tau = np.zeros_like(q)  # TODO: Replace with actual computation

        # Interaction Jacobian (placeholder, to be computed for a specific frame or task)
        self.Ai = np.zeros((1, self._robot_model.nv))  # TODO: Replace with actual computation

        # Derivative of interaction Jacobian (placeholder)
        self.Ai_dot = np.zeros((1, self._robot_model.nv))  # TODO: Replace with actual computation

        return self.M, self.c, self.tau, self.Ai, self.Ai_dot

    def _compute_and_publish_model(self):
        if not self._model_loaded or self._last_q is None or self._last_dq is None:
            return
        q = self._last_q
        dq = self._last_dq
        M, c, tau, Ai, Ai_dot = self._compute_model_matrices(q, dq)

        # TODO: Publish M, c, tau, Ai, Ai_dot
        self.get_logger().info("Computed model matrices")
        # self.get_logger().info(f"Mass matrix:\n{M}")
        # self.get_logger().info(f"Coriolis: {c}")
        # self.get_logger().info(f"Tau: {tau}")
        # self.get_logger().info(f"Ai: {Ai}")
        # self.get_logger().info(f"Ai_dot: {Ai_dot}")


def main(args=None):
    rclpy.init(args=args)
    node = FrankaModelNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
