import rclpy
from rclpy.node import Node
from arm_interfaces.msg import FrankaRIM, Teleop
from teleop_interfaces.msg import Inverse3State
from geometry_msgs.msg import WrenchStamped, Twist, Point, Quaternion
import numpy as np
from enum import Enum


class DelayCompensationMethod(Enum):
    ZOH = "ZOH"  # Zero-Order Hold
    ZOH_PHI = "ZOHPhi"  # Zero-Order Hold with Phi
    DELAY_RIM = "DelayRIM"  # Delay-Resistant Impedance Model


class ReducedModel:
    def __init__(self):
        self.stiffness = 3000.0
        self.damping = 2.0
        self.hl = 0.001  # Control period in seconds (1kHz)

        self.rim_position = np.zeros((3, 1))
        self.rim_velocity = np.zeros((3, 1))
        self.phi_position = np.zeros((3, 1))
        self.phi_velocity = np.zeros((3, 1))
        self.interface_force = np.zeros((3, 1))


class DelayRIMNode(Node):
    def __init__(self):
        super().__init__("delay_rim_node")
        self.get_logger().info("Initializing DelayRIMNode")

        # Parameters
        self.declare_parameter("rim_topic", "/fr3_rim")
        self.declare_parameter("cmd_topic", "/teleop/ee_des")

        self.declare_parameter("control_period", 0.01)  # 1kHz control rate
        self.declare_parameter("delay_compensation_method", "DelayRIM")
        self.declare_parameter("interface_stiffness", 3000.0)
        self.declare_parameter("interface_damping", 2.0)
        self.declare_parameter("force_scaling", 0.02)  # Force scaling factor

        # Get parameters
        self.control_period = self.get_parameter("control_period").get_parameter_value().double_value  # hl
        method_str = self.get_parameter("delay_compensation_method").get_parameter_value().string_value
        self._interface_stiffness = self.get_parameter("interface_stiffness").get_parameter_value().double_value
        self._interface_damping = self.get_parameter("interface_damping").get_parameter_value().double_value
        self._force_scaling = self.get_parameter("force_scaling").get_parameter_value().double_value

        rim_topic = self.get_parameter("rim_topic").get_parameter_value().string_value
        cmd_topic = self.get_parameter("cmd_topic").get_parameter_value().string_value

        # Validate and set delay compensation method
        try:
            self._delay_method = DelayCompensationMethod(method_str)
        except ValueError:
            self.get_logger().warn(f"Invalid delay compensation method: {method_str}, using DelayRIM")
            self._delay_method = DelayCompensationMethod.DELAY_RIM

        # State variables
        self._last_rim_msg = None
        self._last_inverse3_msg = None
        self._haptic_position = np.zeros((3, 1))
        self._haptic_velocity = np.zeros((3, 1))
        self._reduced_model = ReducedModel()
        self._reduced_model.stiffness = self._interface_stiffness
        self._reduced_model.damping = self._interface_damping
        self._reduced_model.hl = self.control_period  # hl is the control period

        # Subscribers
        self._rim_sub = self.create_subscription(FrankaRIM, rim_topic, self._rim_callback, 10)
        self._inverse3_sub = self.create_subscription(Inverse3State, "/inverse3/state", self._inverse3_callback, 10)

        # Publishers
        self._force_pub = self.create_publisher(WrenchStamped, "/inverse3/wrench_des", 10)
        self._teleop_pub = self.create_publisher(Teleop, cmd_topic, 10)

        # Control timer
        self._control_timer = self.create_timer(self.control_period, self._control_timer_callback)

        self.get_logger().info(
            f"DelayRIMNode started with method={self._delay_method.value}, "
            f"control_period={self.control_period}s, K={self._interface_stiffness}, "
            f"D={self._interface_damping}, "
            f"RIM topic: {rim_topic} ,"
            f"Command topic: {cmd_topic}"
        )

    def _rim_callback(self, msg: FrankaRIM):
        """
        Callback for receiving FrankaRIM messages.,
        Updates the reduced model parameters based on the received RIM data.
        """
        self._last_rim_msg = msg

        # Update RIM position and velocity from msg
        self._reduced_model.rim_position = np.array(msg.rim_position[:3]).reshape((3, 1))
        self._reduced_model.rim_velocity = np.array(msg.rim_velocity[:3]).reshape((3, 1))

        # # Log reception (throttled)
        # self.get_logger().debug(f"Received RIM: M_eff={msg.effective_mass[:3]}, f_eff={msg.effective_force[:3]}")

    def _inverse3_callback(self, msg: Inverse3State):
        """
        Callback for receiving Inverse3State messages.
        Updates the haptic device position and velocity.
        """
        self._last_inverse3_msg = msg

        # Extract I3 position
        # Axis: Robot ->  I3: xyz -> y, -x, z
        self._haptic_position = np.array([msg.pose.position.y, -msg.pose.position.x, msg.pose.position.z]).reshape(
            (3, 1)
        )
        self._haptic_position[0] += 0.4253

        # Extract linear velocity from twist
        self._haptic_velocity = np.array([msg.twist.linear.y, -msg.twist.linear.x, msg.twist.linear.z]).reshape((3, 1))

        # Log reception (throttled)
        self.get_logger().debug(
            f"Received Inverse3: pos=[{self._haptic_position[0, 0]:.3f}, "
            f"{self._haptic_position[1, 0]:.3f}, {self._haptic_position[2, 0]:.3f}], "
            f"vel=[{self._haptic_velocity[0, 0]:.3f}, {self._haptic_velocity[1, 0]:.3f}, "
            f"{self._haptic_velocity[2, 0]:.3f}]"
        )

    def _compute_rendered_force(self):
        """
        Compute the rendered force based on the current delay compensation method.
        This is a simplified version for Part 1 (no delays).
        """
        if self._last_rim_msg is None:
            self.get_logger().debug("No RIM data available for force computation")
            return np.zeros((3, 1))

        # Update phi (constraint deviation)
        self._reduced_model.phi_position = self._reduced_model.rim_position[0] - self._haptic_position[0]
        self._reduced_model.phi_velocity = self._reduced_model.rim_velocity[0] - self._haptic_velocity[0]

        # Compute interface force based on method
        if self._delay_method == DelayCompensationMethod.DELAY_RIM:
            # DelayRIM force computation (simplified for now)
            self._reduced_model.interface_force = -(
                self._reduced_model.stiffness * self._reduced_model.phi_position
            ) - (self._reduced_model.damping * self._reduced_model.phi_velocity)

            # (
            #     self.control_period * self._reduced_model.stiffness + self._reduced_model.damping
            # ) * self._reduced_model.phi_velocity

        else:
            # ZOH and ZOHPhi use same basic spring-damper for now
            self._reduced_model.interface_force = (
                -self._reduced_model.stiffness * self._reduced_model.phi_position
                - self._reduced_model.damping * self._reduced_model.phi_velocity
            )

        # ? Print debug information
        # self.get_logger().info(
        #     f"Xee: {self._reduced_model.rim_position[0, 0]:>10.3f} | Xi3: {self._haptic_position[0, 0]:>10.3f} | phi: {self._reduced_model.phi_position[0]:>10.3f} | F: {self._reduced_model.interface_force[0]:>10.3f}|"
        # )

        # Apply force scaling (negative for haptic feedback direction)
        rendered_force = -self._force_scaling * self._reduced_model.interface_force

        return rendered_force

    def _publish_force(self, force):
        """
        Publish the computed force to the haptic device.
        """
        force_msg = WrenchStamped()
        force_msg.header.stamp = self.get_clock().now().to_msg()
        force_msg.header.frame_id = "haptic_device"

        # force_msg.wrench.force.x = float(force[0, 0])
        force_msg.wrench.force.y = float(force[0])
        # force_msg.wrench.force.z = float(force[2, 0])
        force_msg.wrench.force.x = float(0.0)
        force_msg.wrench.force.z = float(0.0)

        self._force_pub.publish(force_msg)

    def _publish_teleop_command(self):
        """
        Publish teleoperation command to the robot based on haptic input.
        For now, this is a placeholder.
        """
        teleop_msg = Teleop()
        teleop_msg.header.stamp = self.get_clock().now().to_msg()
        teleop_msg.control_mode = Teleop.CONTROL_MODE_POSITION

        ee_pos_des_msg = Point()
        ee_pos_des_msg.x = self._haptic_position[0, 0]
        ee_pos_des_msg.y = 0.0
        ee_pos_des_msg.z = 0.025

        ee_quat_des = Quaternion()
        ee_quat_des.x = 1.0
        ee_quat_des.y = 0.0
        ee_quat_des.z = 0.0
        ee_quat_des.w = 0.0

        teleop_msg.ee_des.position = ee_pos_des_msg
        teleop_msg.ee_des.orientation = ee_quat_des

        teleop_msg.ee_vel_des = Twist()

        self._teleop_pub.publish(teleop_msg)

    def _control_timer_callback(self):
        """
        Main control loop executed at high frequency (1kHz).
        """
        # Check if we have Inverse3 data
        if self._last_inverse3_msg is None:
            self.get_logger().debug("No Inverse3 state available", throttle_duration_sec=2.0)
            # Publish zero force if no haptic data
            self._publish_force(np.zeros((3, 1)))
            return

        # Compute rendered force using real haptic device data
        rendered_force = self._compute_rendered_force()

        # Publish force to haptic device
        self._publish_force(rendered_force)

        # Publish teleop command to robot
        self._publish_teleop_command()


def main(args=None):
    rclpy.init(args=args)
    node = DelayRIMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
