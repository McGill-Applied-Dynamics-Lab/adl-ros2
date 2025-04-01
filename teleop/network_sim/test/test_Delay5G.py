import launch
import launch_ros.actions

import launch_pytest
from launch_pytest.tools import process as process_tools

import pytest
import rclpy
from rclpy.node import Node

from threading import Event
from threading import Thread

from std_msgs.msg import String


@pytest.fixture
def network_sim_node():
    # Launch a process to test
    node = launch_ros.actions.Node(
        package="network_sim",
        executable="network_sim_5g",
        output="both",
        parameters=[{"delay": 0.2}],
    )

    return node


# This function specifies the processes to be run for our test.
@launch_pytest.fixture
def launch_description(network_sim_node):
    """Launch a simple process to print 'hello_world'."""
    return launch.LaunchDescription(
        [
            network_sim_node,
            # Tell launch when to start the test
            # If no ReadyToTest action is added, one will be appended automatically.
            launch_pytest.actions.ReadyToTest(),
        ]
    )


@pytest.mark.launch(fixture=launch_description, shutdown=True)
def test_check_if_msgs_published():
    rclpy.init()
    node = MakeTestNode("test_node")
    try:
        node.start_subscriber()
        msgs_received_flag = node.msg_event_object.wait(timeout=2.0)
        print("ALLO")
        assert msgs_received_flag, "Did not receive msgs !"
        print("Test completed successfully!")
    finally:
        # Call shutdown first to signal all ROS operations to terminate
        print("SHUTTING DOWN")
        rclpy.shutdown()
        node.destroy_node()  # Now destroy the node


class MakeTestNode(Node):
    def __init__(self, name="test_node"):
        super().__init__(name)
        self.msg_event_object = Event()
        self.ros_spin_thread = None

    def destroy_node(self):
        if hasattr(self, "ros_spin_thread") and self.ros_spin_thread is not None:
            # Add timeout to second join to prevent indefinite hanging
            self.ros_spin_thread.join(timeout=1.0)
            if self.ros_spin_thread.is_alive():
                self.get_logger().warn("Spin thread did not terminate, forcefully shutting down")
                # Add timeout to the second join attempt as well
                self.ros_spin_thread.join(timeout=1.0)
                if self.ros_spin_thread.is_alive():
                    self.get_logger().error("Could not terminate spin thread - forcing test to continue")
        return super().destroy_node()

    def start_subscriber(self):
        # Create a subscriber
        self.get_logger().info("Starting subscriber")
        self.subscription = self.create_subscription(String, "chatter", self.subscriber_callback, 10)

        # Add a spin thread that's a daemon (will exit when main thread exits)
        self.ros_spin_thread = Thread(
            target=lambda node: rclpy.spin(node),
            args=(self,),
            daemon=True,  # Make thread a daemon so it won't prevent program exit
        )
        self.ros_spin_thread.start()

    def subscriber_callback(self, data):
        self.msg_event_object.set()
