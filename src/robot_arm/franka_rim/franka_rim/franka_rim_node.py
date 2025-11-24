import rclpy
from rclpy.node import Node
from arm_interfaces.msg import FrankaModel, FrankaRIM
from geometry_msgs.msg import WrenchStamped
import numpy as np

import time
import statistics
from collections import deque


from adg_ros2_utils.debug_utils import wait_for_debugger

NODE_NAME = "franka_rim_node"


class FrankaRIMNode(Node):
    def __init__(self):
        super().__init__("franka_rim_node")
        self.get_logger().info("Initializing FrankaRIMNode")

        self.declare_parameter("update_freq", 10.0)  # Default: 0.1s (10 Hz)

        self.declare_parameter("interface_stiffness", 3000.0)  # Default stiffness
        self.declare_parameter("interface_damping", 2.0)  # Default damping

        self.declare_parameter("rim_axis", "x")  # Axis for RIM ('x', 'y', 'z')

        self.declare_parameter("log.enabled", False)
        self.declare_parameter("log.timing", False)
        self.declare_parameter("log.period", 1.0)

        self._rim_axis_str = self.get_parameter("rim_axis").get_parameter_value().string_value

        self._interface_stiffness = self.get_parameter("interface_stiffness").get_parameter_value().double_value
        self._interface_damping = self.get_parameter("interface_damping").get_parameter_value().double_value

        self._update_freq = self.get_parameter("update_freq").get_parameter_value().double_value
        self._update_period = 1 / self._update_freq
        self._loop_count = 0
        self._last_loop_time = time.time()
        self._timer_create_time = time.time()
        self._actual_periods = deque(maxlen=100)

        self._log_enabled = self.get_parameter("log.enabled").get_parameter_value().bool_value
        self._log_timing = self.get_parameter("log.timing").get_parameter_value().bool_value
        self._log_period = self.get_parameter("log.period").get_parameter_value().double_value

        self._last_model_msg = None

        # Model matrices
        self.q = None
        self.q_dot = None
        self.x_ee = None
        self.v_ee = None
        self.J_ee = None  # End-effector Jacobian
        self.Ai = None
        self.Ai_dot_q_dot = None

        self.M = None
        self.c = None

        self.tau = None  # Total joint torques
        self.tau_d = None  # Driving joint torques
        self.tau_ext = None  # External joint torques

        self.f_ext_ee = None  # External force at end-effector
        self.f_d_ee = None  # Desired force at end-effector
        self.f_a = None  # Applied forces

        self._rim_axis_idx = None  # Index of the RIM axis in the task space
        if self._rim_axis_str == "x":
            self._rim_axis_idx = 0
        elif self._rim_axis_str == "y":
            self._rim_axis_idx = 1
        elif self._rim_axis_str == "z":
            self._rim_axis_idx = 2

        self._model_sub = self.create_subscription(FrankaModel, "/fr3_model", self._franka_model_callback, 10)

        # Subscribe to cartesian force from OSC PD controller
        self._cartesian_force_sub = self.create_subscription(
            WrenchStamped,
            "/fr3/interface_force",
            self._cartesian_force_callback,
            10,
        )

        # RIM Parameters
        self.m = 1  # Interface dimension
        self._rim_msg = None
        self._rim_pub = self.create_publisher(FrankaRIM, "/fr3_rim", 10)

        # State variables
        self.cartesian_force = None  # Cartesian force from OSC PD controller (6D wrench)

        # RIM Timer - automatically use wall timer when sim_time is enabled to bypass /clock quantization
        use_sim_time = self.get_parameter("use_sim_time").get_parameter_value().bool_value

        if use_sim_time:
            self.get_logger().info("Simulation time detected - using wall timer to bypass /clock quantization (~25ms)")
            self._rim_timer = self.create_timer(
                self._update_period,
                self._rim_timer_callback,
                clock=rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.STEADY_TIME),
            )
        else:
            self.get_logger().info("Using standard ROS timer for real-time operation")
            self._rim_timer = self.create_timer(self._update_period, self._rim_timer_callback)

        self.get_logger().info(f"FrankaRIMNode initialized.")
        self.get_logger().info(
            f"Parameters\n"
            f"- use sim time: {use_sim_time}, \n"
            f"- frequency: {self._update_freq} Hz ({self._update_period * 1000} ms), \n"
            f"- interface_stiffness: {self._interface_stiffness}, \n"
            f"- interface_damping: {self._interface_damping}, \n"
            f"- rim_axis: {self._rim_axis_str} (idx: {self._rim_axis_idx}), \n"
            f"- interface dimension: {self.m}, \n"
            f"- [Topic] Model input: '/fr3_model', \n"
            f"- [Topic] Ctrl force topic: '/fr3/interface_force', \n"
            f"- [Topic] RIM state: '/fr3_rim', \n"
            f"- log.enabled: {self._log_enabled}, \n"
            f"- log.period: {self._log_period}, \n"
            f"- log.timing: {self._log_timing}, \n"
        )

    def _franka_model_callback(self, msg: FrankaModel):
        """
        Callback for receiving FrankaModel messages. Extracts model matrices and updates internal state.
        """
        if self._last_model_msg is None:
            self.get_logger().info("Franka model initialized")

        self._last_model_msg = msg

        n = msg.n if hasattr(msg, "n") and msg.n > 0 else int(np.sqrt(len(msg.mass_matrix)))

        # --- Kinematics
        self.q = np.array(msg.q)
        self.q_dot = np.array(msg.q_dot)

        self.x_ee = np.array(msg.x_ee)
        self.v_ee = np.array(msg.v_ee)

        # --- Dynamics
        self.M = np.array(msg.mass_matrix).reshape((n, n))
        self.c = np.array(msg.coriolis)
        self.J_ee = np.array(msg.jacobian).reshape((6, n))
        self.Ai = np.array(msg.ai).reshape((1, n))
        self.Ai_dot_q_dot = np.array(msg.ai_dot_q_dot)

        # Forces
        self.tau = np.array(msg.tau)
        self.tau_d = np.array(msg.tau_d)
        self.tau_ext = np.array(msg.tau_ext)

        self.f_ext_ee = np.array(msg.f_ext_ee)
        self.f_d_ee = np.array(msg.f_d_ee)
        self.f_a = np.array(msg.f_a)

        if self.Ai.shape[0] != self.m:
            self.get_logger().error(f"Received Ai with shape {self.Ai.shape}, expected (1, {self.m}).")

        # self.get_logger().info(f"Received FrankaModel: M.shape={self.M.shape}, c.shape={self.c.shape}")

        # print(f"Xee: {self.x_ee[0]:>10.3f} | {self.x_ee[1]:>10.3f} | {self.x_ee[2]:>10.3f}")

    def _cartesian_force_callback(self, msg: WrenchStamped):
        """Callback for cartesian force from OSC PD controller.

        TODO: Remove, all info needed should be in the Model message.
        """
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

    def _compute_rim(self):
        """
        Compute the RIM based on the last received FrankaModel message. Called from `_rim_timer_callback` at a fixed rate of rim_period
        """
        if self._last_model_msg is None:
            self.get_logger().warn("Cannot compute RIM: missing model.", throttle_duration_sec=2)
            return None

        try:
            # --- Create RIM message
            self._rim_msg = FrankaRIM()  # Create RIM message

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
            # M_eff = np.array([[10.0]])

            # --- zi
            z_i = M_eff @ [self.Ai @ M_inv @ (self.c) - self.Ai_dot_q_dot]

            # --- effective force
            # Compute effective force: f_eff = M_eff * (Ai * inv(M) * (tau - c) + Ai_dot * q_dot)
            # effective_force = M_eff @ [self.Ai @ M_inv @ (self.f_a - self.c) + self.Ai_dot_q_dot]
            # effective_force = np.array([[0.0]])

            # ext_force = -self.f_ext_ee[self._rim_axis_idx].reshape((1, 1))
            # driving_force = self.f_d_ee[self._rim_axis_idx].reshape((1, 1))
            # effective_force = M_eff @ [self.Ai @ M_inv] @ self.J_ee.T @ self.f_ext_ee  #
            # effective_force = -self.f_ext_ee[self._rim_axis_idx].reshape((1, 1))
            effective_force = np.array([[0.0]]).reshape((1, 1))

            # if self.cartesian_force is not None:
            #     effective_force = -self.cartesian_force[0].reshape((1, 1))
            # else:
            #     effective_force = np.array([[0.0]])

            # RIM state
            rim_position = self.x_ee[self._rim_axis_idx]
            rim_velocity = self.v_ee[self._rim_axis_idx]

            # --- Contacts
            if rim_position <= 0.05:
                M_eff += 1_000_000

            # M_eff += 100

            # Fill message fields
            self._rim_msg.effective_mass = M_eff.flatten().tolist()
            self._rim_msg.effective_force = effective_force.flatten().tolist()
            self._rim_msg.z_i = z_i.flatten().tolist()
            self._rim_msg.rim_position = rim_position.flatten().tolist()
            self._rim_msg.rim_velocity = rim_velocity.flatten().tolist()

            # Monitoring
            self._loop_count += 1

        except Exception as e:
            self.get_logger().error(f"RIM computation failed: {e}")
            self._rim_msg = None

    def _rim_timer_callback(self):
        # -- Track timing
        loop_time = time.time()
        if self._loop_count > 0:
            actual_period = loop_time - self._last_timer_time
            self._actual_periods.append(actual_period)

        self._last_timer_time = loop_time

        # --- Compute RIM
        self._compute_rim()

        # --- Publish RIM message
        if self._rim_msg is not None:
            self._rim_pub.publish(self._rim_msg)
            self.get_logger().debug("Published RIM message")

        self._log_debug_info()

    def _log_debug_info(self, log_period=1.0):
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

            if avg_period > self._update_period * 1.2 * 1000:
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
    node = FrankaRIMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
