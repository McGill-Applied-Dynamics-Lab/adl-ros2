import rclpy
from rclpy.node import Node
from arm_interfaces.msg import FrankaRIM
from teleop_interfaces.msg import Inverse3State
from geometry_msgs.msg import WrenchStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import numpy as np
import time


class SimpleMassSystemNode(Node):
    """
    Simple 1D mass-spring-damper system for DelayRIM testing.

    Linear system dynamics (x-direction only):
    m * x_ddot = F_haptic + F_spring
    where:
    - F_haptic = K * (x_haptic - x_mass) + D * (v_haptic - v_mass)
    - F_spring = - spring * (x_mass - x_haptic) - damping * v_mass
    """

    def __init__(self):
        super().__init__("simple_mass_system_node")
        self.get_logger().info("Initializing SimpleMassSystemNode")

        # Parameters
        self.declare_parameter("mass", 1.0)  # kg
        self.declare_parameter("spring_constant", 500.0)  # N/m
        self.declare_parameter("damping_coefficient", 10.0)  # N*s/m
        self.declare_parameter("simulation_frequency", 1000.0)  # Hz
        self.declare_parameter("rim_publish_frequency", 100.0)  # Hz
        self.declare_parameter("interface_stiffness", 3000.0)  # N/m
        self.declare_parameter("interface_damping", 2.0)  # N*s/m

        # Get parameters
        self.mass = self.get_parameter("mass").get_parameter_value().double_value
        self.spring_k = self.get_parameter("spring_constant").get_parameter_value().double_value
        self.damping_c = self.get_parameter("damping_coefficient").get_parameter_value().double_value
        sim_freq = self.get_parameter("simulation_frequency").get_parameter_value().double_value
        rim_freq = self.get_parameter("rim_publish_frequency").get_parameter_value().double_value
        self.interface_stiffness = self.get_parameter("interface_stiffness").get_parameter_value().double_value
        self.interface_damping = self.get_parameter("interface_damping").get_parameter_value().double_value

        # Simulation timestep
        self.dt = 1.0 / sim_freq
        self._haptic_pos_scale = 5

        # System state (1D linear system in x-direction only)
        self.mass_position_x = 0.0  # Start at origin
        self.mass_velocity_x = 0.0

        self._spring_force = 0.0
        self._haptic_force = 0.0

        # Haptic state (only x-direction matters)
        self.haptic_position_x = 0.0
        self.haptic_velocity_x = 0.0
        self.last_haptic_time = None

        # Subscribers
        self._inverse3_sub = self.create_subscription(Inverse3State, "/inverse3/state", self._inverse3_callback, 10)

        # Publishers
        self._rim_pub = self.create_publisher(FrankaRIM, "/rim_msg", 10)
        self._marker_pub = self.create_publisher(MarkerArray, "/simple_system/visualization", 10)

        # Timers
        self._sim_timer = self.create_timer(self.dt, self._simulation_step)
        self._rim_timer = self.create_timer(1.0 / rim_freq, self._publish_rim)
        self._vis_timer = self.create_timer(0.1, self._publish_visualization)  # 10Hz visualization

        self.get_logger().info(
            f"SimpleMassSystem initialized: mass={self.mass}kg, "
            f"k={self.spring_k}N/m, c={self.damping_c}N*s/m, "
            f"sim_freq={sim_freq}Hz, rim_freq={rim_freq}Hz"
        )

    def _inverse3_callback(self, msg: Inverse3State):
        """Update haptic device state"""
        current_time = time.time()

        # Extract haptic position (same coordinate transform as DelayRIM)
        # Only care about x-direction
        self.haptic_position_x = msg.pose.position.y * self._haptic_pos_scale

        # Extract haptic velocity (only x-direction)
        self.haptic_velocity_x = msg.twist.linear.y

        self.last_haptic_time = current_time

    def _simulation_step(self):
        """Perform one simulation timestep using Euler integration"""
        if self.last_haptic_time is None:
            return  # No haptic data yet

        # Check for stale haptic data
        if time.time() - self.last_haptic_time > 0.1:  # 100ms timeout
            self.get_logger().debug("Stale haptic data, using last known state", throttle_duration_sec=2.0)

        # Compute forces acting on the mass (x-direction only)
        # 1. Spring force from haptic device
        position_error = self.haptic_position_x - self.mass_position_x
        velocity_error = self.haptic_velocity_x - self.mass_velocity_x
        self._haptic_spring_force = self.interface_stiffness * position_error + self.interface_damping * velocity_error

        # 2. Internal damping
        self._spring_force = -self.spring_k * self.mass_position_x - self.damping_c * self.mass_velocity_x

        # Total force (scalar, x-direction only)
        total_force = self._haptic_spring_force + self._spring_force

        # Compute acceleration (F = ma)
        acceleration = total_force / self.mass

        # Euler integration
        self.mass_velocity_x += acceleration * self.dt
        self.mass_position_x += self.mass_velocity_x * self.dt

        # Log system state periodically
        # info_str = (
        #     f"x={self.mass_position_x:>4.3f} | "
        #     f"dx={self.mass_velocity_x:>4.3f} | "
        #     f"x_error={position_error:>4.3f} | "
        #     f"Fs={self._spring_force:>4.3f}N | "
        #     f"Fh={self._haptic_spring_force:>4.3f}N | "
        #     f"F_total={total_force:>4.3f}N"
        # )
        # self.get_logger().info(
        #     info_str,
        #     throttle_duration_sec=0.5,
        # )

    def _compute_rim(self):
        """
        Compute the Reduced Impedance Model for the linear mass-spring-damper system.
        For this simple 1D system:
        - Effective mass = system mass (scalar)
        - Effective force = internal damping + external forces (no gravity)
        """
        # For a simple 1D mass system, effective mass is just the mass
        # Return as 3D for compatibility but only x-direction has meaningful dynamics
        effective_mass = np.array([self.mass])

        # Effective force
        effective_force = np.array([self._spring_force])

        # Current mass state (RIM position/velocity)
        # Publish 3D state but only x-direction is meaningful
        rim_position = np.array([self.mass_position_x])
        rim_velocity = np.array([self.mass_velocity_x])

        return effective_mass, effective_force, rim_position, rim_velocity

    def _publish_rim(self):
        """Publish the RIM message"""
        if self.last_haptic_time is None:
            return

        try:
            effective_mass, effective_force, rim_position, rim_velocity = self._compute_rim()

            rim_msg = FrankaRIM()
            rim_msg.header.stamp = self.get_clock().now().to_msg()
            rim_msg.header.frame_id = "world"

            # Interface parameters
            rim_msg.interface_stiffness = self.interface_stiffness
            rim_msg.interface_damping = self.interface_damping
            rim_msg.m = 1

            # RIM state
            rim_msg.effective_mass = effective_mass.tolist()
            rim_msg.effective_force = effective_force.tolist()
            rim_msg.rim_position = rim_position.tolist()
            rim_msg.rim_velocity = rim_velocity.tolist()

            self._rim_pub.publish(rim_msg)

        except Exception as e:
            self.get_logger().error(f"Failed to publish RIM: {e}")

    def _publish_visualization(self):
        """Publish visualization markers for RViz"""
        marker_array = MarkerArray()

        # Mass marker (red sphere)
        mass_marker = Marker()
        mass_marker.header.frame_id = "world"
        mass_marker.header.stamp = self.get_clock().now().to_msg()
        mass_marker.ns = "simple_system"
        mass_marker.id = 0
        mass_marker.type = Marker.SPHERE
        mass_marker.action = Marker.ADD

        mass_marker.pose.position.x = self.mass_position_x
        mass_marker.pose.position.y = 0.0  # Fixed at y=0
        mass_marker.pose.position.z = 0.0  # Fixed at z=0
        mass_marker.pose.orientation.w = 1.0

        mass_marker.scale.x = 0.1
        mass_marker.scale.y = 0.1
        mass_marker.scale.z = 0.1

        mass_marker.color.r = 1.0
        mass_marker.color.g = 0.0
        mass_marker.color.b = 0.0
        mass_marker.color.a = 0.8

        marker_array.markers.append(mass_marker)

        # Haptic device marker (blue sphere)
        if self.last_haptic_time is not None:
            haptic_marker = Marker()
            haptic_marker.header.frame_id = "world"
            haptic_marker.header.stamp = self.get_clock().now().to_msg()
            haptic_marker.ns = "simple_system"
            haptic_marker.id = 1
            haptic_marker.type = Marker.SPHERE
            haptic_marker.action = Marker.ADD

            haptic_marker.pose.position.x = self.haptic_position_x
            haptic_marker.pose.position.y = 0.0  # Fixed at y=0
            haptic_marker.pose.position.z = 0.0  # Fixed at z=0
            haptic_marker.pose.orientation.w = 1.0

            haptic_marker.scale.x = 0.08
            haptic_marker.scale.y = 0.08
            haptic_marker.scale.z = 0.08

            haptic_marker.color.r = 0.0
            haptic_marker.color.g = 0.0
            haptic_marker.color.b = 1.0
            haptic_marker.color.a = 0.8

            marker_array.markers.append(haptic_marker)

            # Connection line between mass and haptic device
            line_marker = Marker()
            line_marker.header.frame_id = "world"
            line_marker.header.stamp = self.get_clock().now().to_msg()
            line_marker.ns = "simple_system"
            line_marker.id = 2
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD

            # Add points for the line
            from geometry_msgs.msg import Point

            p1 = Point()
            p1.x = self.mass_position_x
            p1.y = 0.0
            p1.z = 0.0

            p2 = Point()
            p2.x = self.haptic_position_x
            p2.y = 0.0
            p2.z = 0.0

            line_marker.points = [p1, p2]

            line_marker.scale.x = 0.01  # Line width

            line_marker.color.r = 0.0
            line_marker.color.g = 1.0
            line_marker.color.b = 0.0
            line_marker.color.a = 0.6

            marker_array.markers.append(line_marker)

        self._marker_pub.publish(marker_array)
        self._marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleMassSystemNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
