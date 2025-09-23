import rclpy
from rclpy.node import Node
from arm_interfaces.srv import SetGoalSource  # Use actual import


class MockGoalSourceService(Node):
    def __init__(self):
        super().__init__("mock_goal_source_service")
        self.srv = self.create_service(
            SetGoalSource,
            "/fr3_interface/set_goal_source",  # Match service name used by RLAgent
            self.handle_request,
        )
        self.get_logger().info("Mock SetGoalSource service ready.")

    def handle_request(self, request, response):
        self.get_logger().info(f"Mock SetGoalSource received request: source={request.goal_source}")
        response.success = True  # Always succeed in mock
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MockGoalSourceService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
