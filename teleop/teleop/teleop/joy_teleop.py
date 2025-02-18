import rclpy
from rclpy.node import Node  # node class makes a ROS2 Node

from sensor_msgs.msg import Joy


class JoyTeleop(Node):
    def __init__(self):
        super().__init__("joy_teleop")
        self.get_logger().info("Initializing JoyTeleop...")

        self._init_subscribers()

    def _init_subscribers(self):
        self.get_logger().info("Initializing subscribers...")

        # --- joystick ---
        joy_topic = "/joy"
        self._joy_sub = self.create_subscription(Joy, joy_topic, self._joy_topic_callback, 10)

        # --- robot ---
        ...

    #! Callbacks
    def _joy_topic_callback(self, msg: Joy):
        self.get_logger().info(f"Joy msg: {msg}")


def main(args=None):
    rclpy.init(args=args)
    node = JoyTeleop()

    try:
        node.get_logger().info("joy_teleop launched, end with CTRL-C")
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt, shutting down.\n")

    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
