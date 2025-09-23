import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from rclpy.callback_groups import ReentrantCallbackGroup
import time

from teleop.Gamepad import Gamepad, XBOX_GAMEPAD_MAP
from robot_arm_client.FrankaArm import FrankaArm


class TeleopNode(Node):
    def __init__(self, franka_arm: FrankaArm):
        super().__init__("teleop_node")
        self.get_logger().info("Starting teleop node")

        # Initialize gamepad
        self._gamepad = Gamepad()
        self._button_pressed = {}
        self._button_last_press_time = {}
        self._button_debounce_time = 1.0  # seconds to wait before allowing another press

        # Initialize robot arm
        self._robot_arm = franka_arm

        # Subscribe to joystick
        self._sub_joy = self.create_subscription(
            Joy, "/joy", self._sub_joy_callback, 10, callback_group=ReentrantCallbackGroup()
        )

    def _sub_joy_callback(self, joy_msg: Joy):
        self._gamepad.set_state(joy_msg.axes, joy_msg.buttons)

        # Get current time
        current_time = self.get_clock().now().nanoseconds / 1e9

        # Handle button presses
        if self._gamepad.is_button_pressed(XBOX_GAMEPAD_MAP.buttons.A):
            self.get_logger().info("Button A pressed!")
            # Add any A button functionality here

        elif self._gamepad.is_button_pressed(XBOX_GAMEPAD_MAP.buttons.B):
            self.get_logger().info("Button B pressed!")

            cmd = "gripper_toggle"
            last_press = self._button_last_press_time.get(cmd, 0)

            # Check debounce
            if (current_time - last_press) > self._button_debounce_time:
                self.get_logger().info("Toggling gripper from B button press")
                self._button_last_press_time[cmd] = current_time
                self._robot_arm.gripper_toggle()

        elif self._gamepad.is_button_pressed(XBOX_GAMEPAD_MAP.buttons.Y):
            self.get_logger().info("Button Y pressed!")

            cmd = "gripper_homing"
            last_press = self._button_last_press_time.get(cmd, 0)

            # Check debounce
            if (current_time - last_press) > self._button_debounce_time:
                self.get_logger().info("Homing gripper from Y button press")
                self._button_last_press_time[cmd] = current_time
                self._robot_arm.gripper_homing()

        elif self._gamepad.is_button_pressed(XBOX_GAMEPAD_MAP.buttons.X):
            self.get_logger().info("Button X pressed!")
            # Add any X button functionality here


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt, shutting down.\n")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
