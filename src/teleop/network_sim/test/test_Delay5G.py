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


@pytest.fixture
def network_sim_node_fixed():
    # Launch a process to test fixed delay
    node = launch_ros.actions.Node(
        package="network_sim",
        executable="network_sim_5g",
        output="both",
        parameters=[{"input_topic": "input", "output_topic": "output", "delay": 0.1, "delay_type": "fixed"}],
    )

    return node


@pytest.fixture
def network_sim_node_string():
    # Test with String message type
    node = launch_ros.actions.Node(
        package="network_sim",
        executable="network_sim_5g",
        output="both",
        parameters=[
            {
                "input_topic": "string_input",
                "output_topic": "string_output",
                "delay": 0.05,
                "delay_type": "fixed",
                "message_type": "std_msgs/msg/String",
            }
        ],
    )
    return node


@pytest.fixture
def network_sim_node_pose():
    # Test with PoseStamped message type
    node = launch_ros.actions.Node(
        package="network_sim",
        executable="network_sim_5g",
        output="both",
        parameters=[
            {
                "input_topic": "pose_in",
                "output_topic": "pose_out",
                "delay": 0.08,
                "delay_type": "fixed",
                "message_type": "geometry_msgs/msg/PoseStamped",
            }
        ],
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


@launch_pytest.fixture
def launch_description_fixed(network_sim_node_fixed):
    """Launch network sim with fixed delay."""
    return launch.LaunchDescription(
        [
            network_sim_node_fixed,
            launch_pytest.actions.ReadyToTest(),
        ]
    )


@launch_pytest.fixture
def launch_description_string(network_sim_node_string):
    """Launch network sim with String message type."""
    return launch.LaunchDescription(
        [
            network_sim_node_string,
            launch_pytest.actions.ReadyToTest(),
        ]
    )


@launch_pytest.fixture
def launch_description_pose(network_sim_node_pose):
    """Launch network sim with PoseStamped message type."""
    return launch.LaunchDescription(
        [
            network_sim_node_pose,
            launch_pytest.actions.ReadyToTest(),
        ]
    )


@pytest.mark.launch(fixture=launch_description)
def test_5g_delay():
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

        time.sleep(1)  # Wait for the last message to be processed
        # Log the mean and std for all keys in a table format
        node.get_logger().info("Latency Analysis:")
        node.get_logger().info(f"{'Latency Type':<20}{'Mean (ms)':<15}{'Std (ms)':<15}")
        node.get_logger().info("-" * 50)
        for key, (mean, std) in latency_analysis.items():
            node.get_logger().info(f"{key:<20}{mean:<15.4f}{std:<15.4f}")

        node.get_logger().info(
            f"End-to-end Error: {latency_analysis['end-to-end'][0] - latency_analysis['desired'][0]} ms"
        )

        node.get_logger().info(f"Netsim Error: {latency_analysis['netsim'][0] - latency_analysis['desired'][0]} ms")

        # Test assertions for fixed delay
        tolerance = 10.0  # 10ms tolerance

        ee_delay_mean, ee_delay_std = latency_analysis["end-to-end"]

        assert ee_delay_mean > 0.0, "End-to-end delay mean should be greater than 0ms"

        desired_mean, desired_std = latency_analysis["desired"]
        assert abs(desired_mean - ee_delay_mean) < tolerance, (
            f"Fixed delay mean {desired_mean:.2f}ms not within {tolerance}ms of expected {ee_delay_mean}ms"
        )

        # # Fixed delay should have very low standard deviation
        # assert desired_std < 5.0, f"Fixed delay std {desired_std:.2f}ms too high for fixed delay"

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


@pytest.mark.launch(fixture=launch_description_fixed)
def test_fixed_delay():
    rclpy.init()
    node = SubPubNode("sub_pub_node_fixed")
    executor = rclpy.executors.SingleThreadedExecutor()
    spin_thread = None

    try:
        executor.add_node(node)

        # Start spinning in a separate thread
        spin_thread = Thread(target=executor.spin)
        spin_thread.daemon = True
        spin_thread.start()

        node.get_logger().info("Fixed delay test started... ")

        run_time = 5  # seconds
        time.sleep(run_time)

        latency_analysis = node.analyse_delay()

        time.sleep(1)  # Wait for the last message to be processed

        node.get_logger().info("Fixed Delay Latency Analysis:")
        node.get_logger().info(f"{'Latency Type':<20}{'Mean (ms)':<15}{'Std (ms)':<15}")
        node.get_logger().info("-" * 50)
        for key, (mean, std) in latency_analysis.items():
            node.get_logger().info(f"{key:<20}{mean:<15.4f}{std:<15.4f}")

        node.get_logger().info(
            f"End-to-end Error: {latency_analysis['end-to-end'][0] - latency_analysis['desired'][0]} ms"
        )

        node.get_logger().info(f"Netsim Error: {latency_analysis['netsim'][0] - latency_analysis['desired'][0]} ms")

        # Test assertions for fixed delay
        expected_delay_ms = 100.0  # 0.1 seconds = 100ms
        tolerance = 10.0  # 10ms tolerance

        if latency_analysis["desired"]:
            desired_mean, desired_std = latency_analysis["desired"]
            assert abs(desired_mean - expected_delay_ms) < tolerance, (
                f"Fixed delay mean {desired_mean:.2f}ms not within {tolerance}ms of expected {expected_delay_ms}ms"
            )

            # Fixed delay should have very low standard deviation
            assert desired_std < 5.0, f"Fixed delay std {desired_std:.2f}ms too high for fixed delay"

    finally:
        executor.shutdown()
        if spin_thread and spin_thread.is_alive():
            spin_thread.join(timeout=1.0)
        rclpy.shutdown()
        if node:
            node.destroy_node()

        # Save fixed delay test data
        try:
            save_file = Path(__file__).parent / "fixed_delay_analysis"
            with open(save_file.with_suffix(".pkl"), "wb") as f:
                pickle.dump(latency_analysis, f)
            print(f"Fixed delay test data saved to {save_file}")
        except Exception as e:
            print(f"Error saving fixed delay data: {e}")


@pytest.mark.launch(fixture=launch_description_string)
def test_string_message_type():
    rclpy.init()
    node = GenericTestNode("string_test_node", String, "string_input", "string_output")
    executor = rclpy.executors.SingleThreadedExecutor()
    spin_thread = None

    try:
        executor.add_node(node)
        spin_thread = Thread(target=executor.spin)
        spin_thread.daemon = True
        spin_thread.start()

        node.get_logger().info("String message type test started...")

        run_time = 3  # seconds
        time.sleep(run_time)

        # Verify messages were received
        assert node.messages_received > 0, "No String messages received through delay node"
        assert node.messages_sent > 0, "No String messages sent"

        # # Calculate approximate delay
        # if node.receive_times and node.send_times:
        #     avg_delay = np.mean([r - s for r, s in zip(node.receive_times, node.send_times)]) / 1e9
        #     expected_delay = 0.05  # 50ms
        #     tolerance = 0.02  # 20ms tolerance

        #     node.get_logger().info(f"Average delay: {avg_delay * 1000:.2f}ms (expected: {expected_delay * 1000}ms)")
        #     assert abs(avg_delay - expected_delay) < tolerance, f"Delay {avg_delay * 1000:.2f}ms not within tolerance"

    finally:
        executor.shutdown()
        if spin_thread and spin_thread.is_alive():
            spin_thread.join(timeout=1.0)
        rclpy.shutdown()
        if node:
            node.destroy_node()


@pytest.mark.launch(fixture=launch_description_pose)
def test_pose_message_type():
    rclpy.init()
    node = GenericTestNode("pose_test_node", PoseStamped, "pose_in", "pose_out")
    executor = rclpy.executors.SingleThreadedExecutor()
    spin_thread = None

    try:
        executor.add_node(node)
        spin_thread = Thread(target=executor.spin)
        spin_thread.daemon = True
        spin_thread.start()

        node.get_logger().info("PoseStamped message type test started...")

        run_time = 3  # seconds
        time.sleep(run_time)

        # Verify messages were received
        assert node.messages_received > 0, "No PoseStamped messages received through delay node"
        assert node.messages_sent > 0, "No PoseStamped messages sent"

        # # Calculate approximate delay
        # if node.receive_times and node.send_times:
        #     avg_delay = np.mean([r - s for r, s in zip(node.receive_times, node.send_times)]) / 1e9
        #     expected_delay = 0.08  # 80ms
        #     tolerance = 0.02  # 20ms tolerance

        #     node.get_logger().info(f"Average delay: {avg_delay * 1000:.2f}ms (expected: {expected_delay * 1000}ms)")
        #     assert abs(avg_delay - expected_delay) < tolerance, f"Delay {avg_delay * 1000:.2f}ms not within tolerance"

    finally:
        executor.shutdown()
        if spin_thread and spin_thread.is_alive():
            spin_thread.join(timeout=1.0)
        rclpy.shutdown()
        if node:
            node.destroy_node()


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

        # self.get_logger().info(f"Lat: {des_latency_ms:.4f} ms \t EE Lat: {lat_end_to_end:.4f} ms")

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


class GenericTestNode(Node):
    def __init__(self, name: str, message_type, input_topic: str, output_topic: str):
        super().__init__(name)
        self.message_type = message_type
        self.messages_sent = 0
        self.messages_received = 0
        self.send_times = []
        self.receive_times = []

        # Create publisher and subscriber
        self.publisher = self.create_publisher(message_type, input_topic, 10)
        self.subscription = self.create_subscription(message_type, output_topic, self._message_callback, 10)

        # Timer to publish messages
        self.pub_timer = self.create_timer(0.01, self._publish_callback)  # 100Hz

        self.get_logger().info(f"Generic test node started for {message_type.__name__}")

    def _publish_callback(self):
        msg = self.message_type()

        # Set message content based on type
        if isinstance(msg, String):
            msg.data = f"test_message_{self.messages_sent}"
        elif isinstance(msg, PoseStamped):
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "test_frame"
            msg.pose.position.x = float(self.messages_sent % 10)
            msg.pose.position.y = float(self.messages_sent % 5)
            msg.pose.position.z = 1.0
            msg.pose.orientation.w = 1.0
        elif isinstance(msg, Delayed):
            msg.header.stamp = self.get_clock().now().to_msg()

        self.publisher.publish(msg)
        self.send_times.append(self.get_clock().now().nanoseconds)
        self.messages_sent += 1

    def _message_callback(self, msg):
        self.receive_times.append(self.get_clock().now().nanoseconds)
        self.messages_received += 1

        if self.messages_received % 50 == 0:  # Log every 50 messages
            self.get_logger().info(f"Received {self.messages_received} messages")
