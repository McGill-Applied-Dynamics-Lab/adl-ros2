import rclpy
from rclpy.node import Node  # node class makes a ROS2 Node


class I3Teleop(Node):
    def __init__(self):
        super().__init__("i3_teleop")


def main(args=None):
    rclpy.init(args=args)
    node = I3Teleop()

    try:
        node.get_logger().info("i3_teleop launched, end with CTRL-C")
        node.spin()

    except KeyboardInterrupt:
        node.stop_robot()
        node.get_logger().info("KeyboardInterrupt, shutting down.\n")

    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
