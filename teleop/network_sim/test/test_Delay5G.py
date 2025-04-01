import time
import launch
import launch_ros.actions
import launch_pytest
from launch_pytest.tools import process as process_tools

import pytest
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from threading import Event
from threading import Thread

import rclpy.time
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

import numpy as np


@pytest.fixture
def network_sim_node():
    # Launch a process to test
    node = launch_ros.actions.Node(
        package="network_sim",
        executable="network_sim_5g",
        output="both",
        parameters=[{"input_topic": "input", "output_topic": "output", "delay": 0.2}],
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


@pytest.mark.launch(fixture=launch_description)
def test_delay():
    rclpy.init()
    node = SubPubNode("sub_pub_node")
    executor = rclpy.executors.SingleThreadedExecutor()
    spin_thread = None

    try:
        executor.add_node(node)

        # Start spinning in a separate thread
        spin_thread = Thread(target=executor.spin)
        spin_thread.daemon = True
        spin_thread.start()

        run_time = 5  # seconds
        time.sleep(run_time)

        mean_delay, std_delay = node.analyse_delay()

        assert mean_delay > 0.0, "Mean delay is not greater than 0"
        assert std_delay > 0.0, "Standard deviation of delay is not greater than 0"

        exp_mean = 200
        epsilon = 10
        assert (mean_delay < exp_mean + epsilon) and (mean_delay > exp_mean - epsilon), (
            f"Mean delay {mean_delay} is not within {epsilon} ms of expected {exp_mean} ms"
        )
        assert True, "Did not receive msgs !"

    finally:
        # executor.shutdown()

        # # Wait for the spin thread to complete with a timeout
        # if spin_thread and spin_thread.is_alive():
        #     spin_thread.join(timeout=1.0)

        # # Now shutdown rclpy
        # node.destroy_node()

        # rclpy.shutdown()

        if node:
            node.destroy_node()


class SubPubNode(Node):
    def __init__(self, name="test_node"):
        super().__init__(name)
        self.get_logger().info("Test node started")

        self.msg_event_object = Event()

        self.publisher = self.create_publisher(PoseStamped, "input", 10)
        self.pub_timer = self.create_timer(0.01, self.publisher_callback)

        self.subscription = self.create_subscription(PoseStamped, "output", self.subscriber_callback, 10)

        self.delays = np.array([], dtype=np.float64)

    def subscriber_callback(self, msg: PoseStamped):
        sent_time = msg.header.stamp
        sent_time = rclpy.time.Time.from_msg(sent_time)

        current_time = self.get_clock().now()
        # self.get_logger().info(f"Received msg at {current_time} with sent time {sent_time}")

        delay = (current_time.nanoseconds - sent_time.nanoseconds) / 1e6  # convert to milliseconds
        self.get_logger().info(f"Delay: {delay} ms")

        self.delays = np.append(self.delays, delay)

    def publisher_callback(self):
        msg = PoseStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)

    def analyse_delay(self):
        if len(self.delays) == 0:
            return 0.0, 0.0

        mean_delay = np.mean(self.delays)
        std_delay = np.std(self.delays)

        self.get_logger().info(f"Mean delay: {mean_delay} ms")
        self.get_logger().info(f"Standard deviation of delay: {std_delay} ms")

        return mean_delay, std_delay
