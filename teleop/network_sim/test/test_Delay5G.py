import time
import launch
import launch_ros.actions
import launch_pytest
from launch_pytest.tools import process as process_tools
from ament_index_python.packages import get_package_share_directory

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
import pandas as pd
from pathlib import Path
import pickle


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


def bootstrap_data(df: pd.DataFrame) -> float:
    """
    Bootstrap the data to get a mean and standard deviation of the delay.
    """
    num_bootstrap_samples = 1000
    bootstrap_means = []

    for _ in range(num_bootstrap_samples):
        sample = df["latency"].sample(n=len(df), replace=True)
        bootstrap_means.append(sample.mean())

    # # Plotting the histogram
    # plt.hist(bootstrap_means, bins=30, edgecolor="k")
    # plt.xlabel("Bootstrapped Mean Latency")
    # plt.ylabel("Frequency")
    # plt.title("Bootstrap Distribution of the Mean")
    # plt.show()

    # # Calculating mean and standard deviation
    # mean_delay = np.mean(bootstrap_means)
    # std_delay = 0.0

    return bootstrap_means


@pytest.mark.launch(fixture=launch_description)
def test_delay():
    rclpy.init()
    node = SubPubNode("sub_pub_node")
    executor = rclpy.executors.SingleThreadedExecutor()
    spin_thread = None

    # Load data
    package_share_path = Path(get_package_share_directory("network_sim"))
    csv_file_path = package_share_path / "data/5G-data.csv"
    data_5g = pd.read_csv(csv_file_path)
    node.get_logger().info("5G dataset loaded")

    # Bootstrap data
    bootstrap_means = bootstrap_data(data_5g)

    try:
        executor.add_node(node)

        # Start spinning in a separate thread
        spin_thread = Thread(target=executor.spin)
        spin_thread.daemon = True
        spin_thread.start()

        run_time = 5  # seconds
        time.sleep(run_time)

        mean_delay, std_delay = node.analyse_delay()

        node.get_logger().info(f"Mean delay: {mean_delay} ms")
        node.get_logger().info(f"Standard deviation of delay: {std_delay} ms")
        node.get_logger().info(f"Bootstrap means: {np.mean(bootstrap_means)}")

        assert mean_delay > 0.0, "Mean delay is not greater than 0"
        assert std_delay > 0.0, "Standard deviation of delay is not greater than 0"

        exp_mean = np.mean(bootstrap_means)
        epsilon = 10
        assert (mean_delay < exp_mean + epsilon) and (mean_delay > exp_mean - epsilon), (
            f"Mean delay {mean_delay} is not within {epsilon} ms of expected {exp_mean} ms"
        )

    finally:
        # First properly shutdown the executor
        executor.shutdown()

        # Wait for the spin thread to complete with a timeout
        if spin_thread and spin_thread.is_alive():
            spin_thread.join(timeout=1.0)

        rclpy.shutdown()

        if node:
            node.destroy_node()

        # Instead of plotting directly, save the bootstrap means to a file
        try:
            test_data = {
                "bootstrap_means": bootstrap_means,
                "node_mean_delay": mean_delay,
                "node_std_delay": std_delay,
            }
            save_file = Path(__file__).parent / "bootstrap_test_data"

            with open(save_file.with_suffix(".pkl"), "wb") as f:
                pickle.dump(test_data, f)

            print(f"Test data saved to {save_file}")
            print(f"To generate plot, run: python3 plot_bootstrap.py {save_file}")

        except Exception as e:
            print(f"Error saving bootstrap data: {e}")


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

        return mean_delay, std_delay
