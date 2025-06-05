import rclpy
from rclpy.node import Node
from arm_interfaces.msg import FrankaModel
import numpy as np


class FrankaRIMNode(Node):
    def __init__(self):
        super().__init__("franka_rim_node")
        self.get_logger().info("Initializing FrankaRIMNode")

        self.declare_parameter("rim_period", 0.1)  # Default: 0.1s (10 Hz)
        rim_period = self.get_parameter("rim_period").get_parameter_value().double_value
        self._last_model_msg = None

        # Model matrices
        self._M = None
        self._c = None
        self._tau = None
        self._Ai = None
        self._Ai_dot_q_dot = None
        self._sub = self.create_subscription(FrankaModel, "/fr3_model", self._franka_model_callback, 10)

        # RIM Timer
        self._rim_timer = self.create_timer(rim_period, self._rim_timer_callback)
        self.get_logger().info(f"FrankaRIMNode started, subscribing to /fr3_model, rim_period={rim_period}s")

    def _franka_model_callback(self, msg: FrankaModel):
        """
        Callback for receiving FrankaModel messages. Extracts model matrices and updates internal state.
        """
        self._last_model_msg = msg

        n = msg.n if hasattr(msg, "n") and msg.n > 0 else int(np.sqrt(len(msg.mass_matrix)))
        self._M = np.array(msg.mass_matrix).reshape((n, n))
        self._c = np.array(msg.coriolis)
        self._tau = np.array(msg.tau)
        self._Ai = np.array(msg.ai).reshape((1, n))
        self._Ai_dot_q_dot = np.array(msg.ai_dot_q_dot).reshape((1, n))
        self.get_logger().info(f"Received FrankaModel: M.shape={self._M.shape}, c.shape={self._c.shape}")

    def _compute_rim(self):
        """
        Compute the RIM based on the last received FrankaModel message. Called from `_rim_timer_callback` at a fixed rate of rim_period
        """
        if self._last_model_msg is None:
            self.get_logger().warn("Cannot compute RIM: missing M or Ai.")
            return None

        try:
            rim = None
            # TODO: @Chantal Compute RIM here

            return rim

        except Exception as e:
            self.get_logger().error(f"RIM computation failed: {e}")
            return None

    def _rim_timer_callback(self):
        # 1. Compute RIM
        self._compute_rim()

        # 2. Compute forces, ...

        # TODO: Publish outputs


def main(args=None):
    rclpy.init(args=args)
    node = FrankaRIMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
