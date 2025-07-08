import rclpy
from rclpy.node import Node
from arm_interfaces.msg import FrankaModel, FrankaRIM
import numpy as np


class FrankaRIMNode(Node):
    def __init__(self):
        super().__init__("franka_rim_node")
        self.get_logger().info("Initializing FrankaRIMNode")

        self.declare_parameter("rim_period", 0.1)  # Default: 0.1s (10 Hz)
        self.declare_parameter("interface_stiffness", 3000.0)  # Default stiffness
        self.declare_parameter("interface_damping", 2.0)  # Default damping

        rim_period = self.get_parameter("rim_period").get_parameter_value().double_value
        self._interface_stiffness = self.get_parameter("interface_stiffness").get_parameter_value().double_value
        self._interface_damping = self.get_parameter("interface_damping").get_parameter_value().double_value

        self._last_model_msg = None

        # Model matrices
        self.q = None
        self.q_dot = None
        self.x_ee = None
        self.v_ee = None
        self.M = None
        self.c = None
        self.tau = None
        self.Ai = None
        self.Ai_dot_q_dot = None
        self.fa = None

        self._model_sub = self.create_subscription(FrankaModel, "/fr3_model", self._franka_model_callback, 10)

        # RIM Parameters
        self.m = 1  # Interface dimension
        self._rim_msg = None
        self._rim_pub = self.create_publisher(FrankaRIM, "/fr3_rim", 10)

        # RIM Timer
        self._rim_timer = self.create_timer(rim_period, self._rim_timer_callback)
        self.get_logger().info(
            f"FrankaRIMNode started, subscribing to /fr3_model, publishing to /fr3_rim, rim_period={rim_period}s"
        )

    def _franka_model_callback(self, msg: FrankaModel):
        """
        Callback for receiving FrankaModel messages. Extracts model matrices and updates internal state.
        """
        self._last_model_msg = msg

        n = msg.n if hasattr(msg, "n") and msg.n > 0 else int(np.sqrt(len(msg.mass_matrix)))

        self.q = np.array(msg.q)
        self.q_dot = np.array(msg.q_dot)
        self.M = np.array(msg.mass_matrix).reshape((n, n))
        self.c = np.array(msg.coriolis)
        self.tau = np.array(msg.tau)
        self.Ai = np.array(msg.ai).reshape((1, n))
        self.Ai_dot_q_dot = np.array(msg.ai_dot_q_dot)
        self.fa = np.array(msg.fa)

        self.x_ee = np.array(msg.x_ee)
        self.v_ee = np.array(msg.v_ee)

        if self.Ai.shape[0] != self.m:
            self.get_logger().error(f"Received Ai with shape {self.Ai.shape}, expected (1, {self.m}).")

        # self.get_logger().info(f"Received FrankaModel: M.shape={self.M.shape}, c.shape={self.c.shape}")

        # print(f"Xee: {self.x_ee[0]:>10.3f} | {self.x_ee[1]:>10.3f} | {self.x_ee[2]:>10.3f}")

    def _compute_rim(self):
        """
        Compute the RIM based on the last received FrankaModel message. Called from `_rim_timer_callback` at a fixed rate of rim_period
        """
        if self._last_model_msg is None:
            self.get_logger().warn("Cannot compute RIM: missing model.", throttle_duration_sec=2)
            return None

        try:
            self._rim_msg = FrankaRIM()  # Create RIM message

            # Create RIM message
            self._rim_msg.header.stamp = self.get_clock().now().to_msg()
            # self._rim_msg.header.frame_id = "fr3"
            self._rim_msg.m = self.m  # Interface dimension

            # Set interface parameters
            self._rim_msg.interface_stiffness = self._interface_stiffness
            self._rim_msg.interface_damping = self._interface_damping

            # Compute effective mass: M_eff = (Ai * M^-1 * Ai^T)^{-1}
            # Where Ai is the interaction Jacobian (1 x n)
            M_inv = np.linalg.inv(self.M)
            M_eff = np.linalg.inv(self.Ai @ M_inv @ self.Ai.T)

            # Compute effective force: f_eff = M_eff * (Ai * inv(M) * (tau - c) + Ai_dot * q_dot)
            effective_force = M_eff @ [self.Ai @ M_inv @ (self.fa - self.c) + self.Ai_dot_q_dot]

            # RIM state
            rim_position = self.x_ee
            rim_velocity = self.v_ee

            # Fill message fields
            self._rim_msg.effective_mass = M_eff.flatten().tolist()
            self._rim_msg.effective_force = effective_force.flatten().tolist()
            self._rim_msg.rim_position = rim_position.flatten().tolist()
            self._rim_msg.rim_velocity = rim_velocity.flatten().tolist()

        except Exception as e:
            self.get_logger().error(f"RIM computation failed: {e}")
            self._rim_msg = None

    def _rim_timer_callback(self):
        # Compute RIM
        self._compute_rim()

        # Publish RIM message
        if self._rim_msg is not None:
            self._rim_pub.publish(self._rim_msg)
            self.get_logger().debug("Published RIM message")


def main(args=None):
    rclpy.init(args=args)
    node = FrankaRIMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
