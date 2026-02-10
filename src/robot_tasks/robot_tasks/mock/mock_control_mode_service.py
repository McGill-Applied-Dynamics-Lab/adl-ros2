import rclpy
from rclpy.node import Node
from arm_interfaces.srv import SetControlMode  # Use actual import


class MockControlModeService(Node):
    def __init__(self):
        super().__init__("mock_control_mode_service")
        self.srv = self.create_service(
            SetControlMode,
            "/fr3_interface/set_control_mode",  # Match service name used by RLAgent
            self.handle_request,
        )
        self.get_logger().info("Mock SetControlMode service ready.")

    def handle_request(self, request, response):
        self.get_logger().info(f"Mock SetControlMode received request: mode={request.control_mode}")
        response.success = True  # Always succeed in mock
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MockControlModeService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
