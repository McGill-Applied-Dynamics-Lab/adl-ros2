import rclpy
from rclpy.node import Node
from arm_interfaces.msg import FrankaRIM
from teleop_interfaces.msg import Inverse3State
from geometry_msgs.msg import WrenchStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import numpy as np
import time


class TestNode(Node):
    def __init__(self):
        super().__init__("simple_mass_system_node")
        self.get_logger().info("Initializing SimpleMassSystemNode")

        # Parameters
        freq = 1000  # Hz
        self.dt = 1.0 / freq

        self.count = 0
        self.last_time = self.get_clock().now().nanoseconds / 1e9

        # Timers
        self._sim_timer = self.create_timer(self.dt, self._cb)

    def _cb(self):
        loop_time = self.get_clock().now().nanoseconds / 1e9

        if self.count % 500 == 0:
            dt = (loop_time - self.last_time) * 1000
            self.get_logger().info(f"{dt:.6f}ms, count: {self.count}")

        self.count += 1
        self.last_time = loop_time


def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
