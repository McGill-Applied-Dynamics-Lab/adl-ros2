import rclpy
from rclpy.node import Node
from teleop_interfaces.msg import Inverse3State
from geometry_msgs.msg import Point, Vector3, Pose, Twist
import numpy as np
import math
import time


class HapticSimulatorNode(Node):
    """Simulates haptic device input by publishing sine waves to /inverse3/state"""

    def __init__(self):
        super().__init__("haptic_simulator_node")
        self.get_logger().info("Initializing HapticSimulatorNode")

        # Parameters
        self.declare_parameter("publish_frequency", 1000.0)  # Hz
        self.declare_parameter("sine_frequency", 0.2)  # Hz (2 cycles)
        self.declare_parameter("amplitude", 0.2)  # meters
        self.declare_parameter("n_cycles", 2)  # cycles to complete
        self.declare_parameter("center_x", 0.0)  # center position 0.30676
        self.declare_parameter("center_y", 0.0)
        self.declare_parameter("center_z", 0.0)  # 0.4844

        # Get parameters
        self.publish_freq = self.get_parameter("publish_frequency").get_parameter_value().double_value
        self.sine_freq = self.get_parameter("sine_frequency").get_parameter_value().double_value
        self.amplitude = self.get_parameter("amplitude").get_parameter_value().double_value
        self.n_cycles = self.get_parameter("n_cycles").get_parameter_value().integer_value
        self.center_x = self.get_parameter("center_x").get_parameter_value().double_value
        self.center_y = self.get_parameter("center_y").get_parameter_value().double_value
        self.center_z = self.get_parameter("center_z").get_parameter_value().double_value

        # State variables
        self.duration = self.n_cycles / self.sine_freq  # Total duration for n cycles

        self.init_time = time.perf_counter()
        self.start_time = self.init_time + 1  # Start after 1 second to allow system to stabilize
        self.simulation_active = True

        # Publisher
        self._inverse3_pub = self.create_publisher(Inverse3State, "/inverse3/state", 10)

        # Timer
        publish_period = 1.0 / self.publish_freq
        self._timer = self.create_timer(publish_period, self._i3_traj_callback)

        self.get_logger().info(
            f"HapticSimulator started: freq={self.publish_freq}Hz, "
            f"sine_freq={self.sine_freq}Hz, amplitude={self.amplitude}m, "
            f"duration={self.duration}s, center=({self.center_x}, {self.center_y}, {self.center_z})"
        )

    def _i3_traj_callback(self):
        """Publish simulated haptic state with sine wave motion"""
        current_time = time.perf_counter()

        position = np.zeros(3)
        velocity = np.zeros(3)

        if current_time < self.start_time:
            # Wait until start time
            self._publish_i3_state(position, velocity)
            return

        elapsed_time = current_time - self.start_time

        # Check if simulation duration has elapsed
        if elapsed_time > self.duration:
            if self.simulation_active:
                self.get_logger().info(f"Simulation completed after {elapsed_time:.3f}s")
                self.simulation_active = False

            # Continue publishing at final position
            elapsed_time = self.duration

        # Generate sine wave motion in x-direction
        omega = 2.0 * math.pi * self.sine_freq

        # Position: sine wave
        position[0] = self.center_x + self.amplitude * math.sin(omega * elapsed_time)
        position[1] = self.center_y  # Fixed
        position[2] = self.center_z  # Fixed

        # Velocity: derivative of sine wave
        velocity[0] = self.amplitude * omega * math.cos(omega * elapsed_time)
        velocity[1] = 0.0  # Fixed
        velocity[2] = 0.0  # Fixed

        # If simulation is complete, set velocity to zero
        if not self.simulation_active:
            velocity = np.zeros(3)

        self._publish_i3_state(position, velocity)

    def _publish_i3_state(self, position: np.ndarray, velocity: np.ndarray):
        # Create Inverse3State message
        msg = Inverse3State()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "haptic_device"

        # Set pose
        msg.pose = Pose()
        msg.pose.position = Point()
        msg.pose.position.x = position[1]  # To map correctly to robot, inverse x and y
        msg.pose.position.y = position[0]
        msg.pose.position.z = position[2]

        # Set orientation (identity quaternion)
        msg.pose.orientation.w = 1.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0

        # Set twist
        msg.twist = Twist()
        msg.twist.linear = Vector3()
        msg.twist.linear.x = velocity[1]  # Inverse x and y
        msg.twist.linear.y = velocity[0]  # Inverse x and y
        msg.twist.linear.z = velocity[2]  # Inverse x and y

        # Angular velocity (zero)
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0

        # Publish message
        self._inverse3_pub.publish(msg)

        # # Log progress periodically
        # if int(elapsed_time * 10) % 10 == 0:  # Every 0.1 seconds
        #     cycles_completed = elapsed_time * self.sine_freq
        #     self.get_logger().info(
        #         f"t={elapsed_time:.2f}s, cycles={cycles_completed:.2f}, x={x_position:.4f}m, v_x={x_velocity:.4f}m/s",
        #         throttle_duration_sec=0.2,
        #     )


def main(args=None):
    rclpy.init(args=args)
    node = HapticSimulatorNode()

    try:
        node.get_logger().info("HapticSimulatorNode started, generating 2 sine cycles")
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt occurred, shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
