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
from arm_interfaces.msg import Delayed

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


@pytest.mark.launch(fixture=launch_description)
def test_delay():
    rclpy.init()
    node = SubPubNode("sub_pub_node")
    executor = rclpy.executors.SingleThreadedExecutor()
    spin_thread = None

    # Load data
    node.get_logger().info("5G dataset loaded")

    # Bootstrap data
    try:
        executor.add_node(node)

        # Start spinning in a separate thread
        spin_thread = Thread(target=executor.spin)
        spin_thread.daemon = True
        spin_thread.start()

        run_time = 10  # seconds
        time.sleep(run_time)

        latency_analysis = node.analyse_delay()

        time.sleep(2)  # Wait for the last message to be processed
        # Log the mean and std for all keys in a table format
        node.get_logger().info("Latency Analysis:")
        node.get_logger().info(f"{'Latency Type':<20}{'Mean (ms)':<15}{'Std (ms)':<15}")
        node.get_logger().info("-" * 50)
        for key, (mean, std) in latency_analysis.items():
            node.get_logger().info(f"{key:<20}{mean:<15.4f}{std:<15.4f}")

        # assert mean_delay > 0.0, "Mean delay is not greater than 0"
        # assert std_delay > 0.0, "Standard deviation of delay is not greater than 0"

        # exp_mean = np.mean(bootstrap_means)
        # epsilon = 10
        # assert (mean_delay < exp_mean + epsilon) and (mean_delay > exp_mean - epsilon), (
        #     f"Mean delay {mean_delay} is not within {epsilon} ms of expected {exp_mean} ms"
        # )

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
            save_file = Path(__file__).parent / "latency_analysis"

            with open(save_file.with_suffix(".pkl"), "wb") as f:
                pickle.dump(latency_analysis, f)

            print(f"Test data saved to {save_file}")
            print("To generate plot, run: python3 src/adg_ros2/teleop/network_sim/test/plot_bootstrap.py")

        except Exception as e:
            print(f"Error saving bootstrap data: {e}")


class SubPubNode(Node):
    def __init__(self, name="test_node"):
        super().__init__(name)
        self.get_logger().info("Test node started")

        self.msg_event_object = Event()

        self.publisher = self.create_publisher(Delayed, "input", 10)
        self.pub_timer = self.create_timer(0.001, self.publisher_callback)

        self.subscription = self.create_subscription(Delayed, "output", self.subscriber_callback, 10)

        self.latencies = {
            "end-to-end": np.array([], dtype=np.float64),  # End-to-end latency
            "netsim": np.array([], dtype=np.float64),  # Netsim latency
            "desired": np.array([], dtype=np.float64),  # Desired netsim latency
            "source-netsim": np.array([], dtype=np.float64),  # Source to netsim latency
            "netsim-dest": np.array([], dtype=np.float64),  # Netsim to destination latency
        }

    def subscriber_callback(self, msg: Delayed):
        current_time = self.get_clock().now()
        # self.get_logger().info(f"Received msg at {current_time} with sent time {sent_time}")

        msg.time_received_dest = current_time.to_msg()

        # Supposed latency of the message
        des_latency_ns = rclpy.time.Duration.from_msg(msg.latency).nanoseconds
        des_latency_ms = des_latency_ns / 1e6

        # Time stamps
        time_sent_source = rclpy.time.Time.from_msg(msg.header.stamp)
        time_received_netsim = rclpy.time.Time.from_msg(msg.time_received_netsim)
        time_sent_netsim = rclpy.time.Time.from_msg(msg.time_sent_netsim)
        time_received_dest = rclpy.time.Time.from_msg(msg.time_received_dest)

        # -- Latencies
        # End-to-end latency
        lat_end_to_end = (time_received_dest - time_sent_source).nanoseconds / 1e6
        lat_source_netsim = (time_received_netsim - time_sent_source).nanoseconds / 1e6
        lat_netsim_dest = (time_received_dest - time_sent_netsim).nanoseconds / 1e6
        lat_netsim = (time_sent_netsim - time_received_netsim).nanoseconds / 1e6

        self.get_logger().info(f"Lat: {des_latency_ms:.4f} ms \t EE Lat: {lat_end_to_end:.4f} ms")

        self.latencies["end-to-end"] = np.append(self.latencies["end-to-end"], lat_end_to_end)
        self.latencies["source-netsim"] = np.append(self.latencies["source-netsim"], lat_source_netsim)
        self.latencies["netsim-dest"] = np.append(self.latencies["netsim-dest"], lat_netsim_dest)
        self.latencies["netsim"] = np.append(self.latencies["netsim"], lat_netsim)
        self.latencies["desired"] = np.append(self.latencies["desired"], des_latency_ms)

    def publisher_callback(self):
        msg = Delayed()

        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)

    def analyse_delay(self):
        if not self.latencies["end-to-end"].size:
            return {key: (0.0, 0.0) for key in self.latencies}

        # Compute mean and std for each latency type
        lat_analysis = {key: (np.mean(values), np.std(values)) for key, values in self.latencies.items()}

        return lat_analysis
