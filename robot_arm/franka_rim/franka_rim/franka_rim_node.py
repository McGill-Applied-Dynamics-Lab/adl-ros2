import rclpy
from rclpy.node import Node
# import numpy as np

class FrankaRIMNode(Node):
    def __init__(self):
        super().__init__('franka_rim_node')
        # TODO: Subscribe to robot model message
        # TODO: Compute RIM
        # TODO: Publish RIM message


def main(args=None):
    rclpy.init(args=args)
    node = FrankaRIMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
