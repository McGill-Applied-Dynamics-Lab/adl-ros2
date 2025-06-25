import os
import sys
import time
import uuid
import asyncio  # Import asyncio for async functions if needed
from typing import Generator, Tuple, List
import numpy as np
import pytest
import matplotlib.pyplot as plt
import copy

import launch
from launch_testing.actions import ReadyToTest
from launch_testing.util import KeepAliveProc
import launch_pytest

from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    # OpaqueFunction,
    ExecuteProcess,
    Shutdown,
    TimerAction,  # noqa: F401
)

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    Command,
    FindExecutable,
)

from launch_ros.substitutions import FindPackageShare
from sensor_msgs.msg import JointState

# import threading
from threading import Event
from threading import Thread

import rclpy
import rclpy.action
import rclpy.executors
from rclpy.node import Node
import rclpy.publisher
from ament_index_python.packages import get_package_share_directory

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from controller_manager_msgs.srv import ListControllers
from controller_manager_msgs.msg import ControllerState


# Package name where RLAgent node and dummy files are located
PACKAGE_NAME = "fr3_controllers"
# ACTION_SERVER_NAME = "/lift_action"  # Match parameter default or override in launch
# ACTION_TYPE = RlAgent
# RL_AGENT_NODE_NAME = "lift_agent_node"  # Match node name in RLAgent class
CTRL_NAME = "joint_trajectory_controller"  # Name of the controller to test


# --- Test Fixture for Launch Description ---
@launch_pytest.fixture
def launch_description():
    """Sets up the launch description for the test environment."""
    # pkg_share = get_package_share_directory(PACKAGE_NAME)

    fr3_launch_file = PathJoinSubstitution([FindPackageShare("robot_arm_bringup"), "launch", "fr3.launch.py"])

    robot_ip = os.environ.get("FR3_IP")
    if not robot_ip:
        raise RuntimeError("Environment variable FR3_IP must be set to the robot's IP address.")

    fr3_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([fr3_launch_file]),
        launch_arguments={
            "robot_ip": robot_ip,
            "arm_id": "fr3",
            "load_gripper": "false",
            "hw_type": "real",
            "use_rviz": "false",
        }.items(),
    )

    launch_description = [
        fr3_launch,
        launch_pytest.actions.ReadyToTest(),  # Signal readiness
    ]

    return launch.LaunchDescription(launch_description)


@pytest.fixture(scope="module")
def rclpy_init_shutdown():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.mark.launch(fixture=launch_description)
def test_controller_init():
    rclpy.init()
    node = TestNode("test_node")
    executor: rclpy.executors.SingleThreadedExecutor = rclpy.executors.SingleThreadedExecutor()
    spin_thread = None

    try:
        executor.add_node(node)
        spin_thread = Thread(target=executor.spin)
        spin_thread.daemon = True
        spin_thread.start()

        node.get_logger().info("Controller test started")

        node.get_logger().info("Waiting for fr3_launch to be over...")
        run_time = 5  # seconds
        time.sleep(run_time)

        # Verify the controller is still active
        assert node.is_controller_active(CTRL_NAME), f"Controller '{CTRL_NAME}' is not active"

        # Verify the controller topic exists
        assert node.topic_exists(f"/{CTRL_NAME}/joint_trajectory"), (
            f"Topic '/{CTRL_NAME}/joint_tddrajectory' does not exist"
        )

    finally:
        executor.shutdown()
        if spin_thread and spin_thread.is_alive():
            spin_thread.join(timeout=1.0)
        rclpy.shutdown()
        if node:
            node.destroy_node()


@pytest.mark.launch(fixture=launch_description)
def test_j2_traj():
    rclpy.init()
    node = TestNode("test_node")
    executor: rclpy.executors.SingleThreadedExecutor = rclpy.executors.SingleThreadedExecutor()
    spin_thread = None

    try:
        executor.add_node(node)
        spin_thread = Thread(target=executor.spin)
        spin_thread.daemon = True
        spin_thread.start()

        node.get_logger().info("Joint 2 trajectory test started")

        node.get_logger().info("Waiting for fr3_launch to be over...")
        spin_up_time = 1  # seconds
        time.sleep(spin_up_time)

        # Verify the controller is active
        assert node.is_controller_active(CTRL_NAME), f"Controller '{CTRL_NAME}' is not active"

        # Verify the controller topic exists
        controller_topic = f"/{CTRL_NAME}/joint_trajectory"
        assert node.topic_exists(controller_topic), f"Topic '{controller_topic}' does not exist"

        # Send sinusoidal trajectory for joint 2
        node.get_logger().info("Sending sinusoidal trajectory to joint 2...")
        duration, expected_final_positions = node.send_joint_sin_trajectory(controller_topic, joint_idx=1)

        # Wait for trajectory executions
        node.get_logger().info("Waiting for trajectory execution to complete...")
        time.sleep(duration + 2.0)  # Extra time for safety

        # Verify final position matches trajectory endpoint
        node.get_logger().info("Trajectory execution complete, verifying final position...")
        position_match = node.verify_final_position(expected_final_positions, tolerance=0.05)
        assert position_match, "Robot final position does not match trajectory endpoint within tolerance"

        node.get_logger().info("Joint 2 trajectory test completed successfully")

    finally:
        executor.shutdown()
        if spin_thread and spin_thread.is_alive():
            spin_thread.join(timeout=1.0)
        rclpy.shutdown()
        if node:
            node.destroy_node()


class TestNode(Node):
    def __init__(
        self,
        node_name: str,
        message_type=JointTrajectory,
        input_topic: str = "/input_topic",
        output_topic: str = "/output_topic",
        pub_rate: float = 0.01,  # 100 Hz
    ):
        super().__init__(node_name)
        self.message_type = JointTrajectory
        self.messages_sent = 0
        self.messages_received = 0

        # Joint state storage
        self.current_joint_positions = None
        self.joint_names_order = None

        # Create publisher and subscriber
        self.publisher = self.create_publisher(self.message_type, input_topic, 10)
        self.subscription = self.create_subscription(self.message_type, output_topic, self._message_callback, 10)

        # Timer to publish messages
        pub_period = 1.0 / pub_rate  # Convert Hz to seconds
        self.pub_timer = self.create_timer(pub_period, self._publish_callback)

        # Service client for controller manager
        self.list_controllers_client = self.create_client(ListControllers, "/controller_manager/list_controllers")

        # Wait for controller manager service
        self.get_logger().info("Waiting for controller manager service...")
        while not self.list_controllers_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Controller manager service not available, waiting again...")

        # Publisher for controller trajectory
        self.trajectory_publisher = None

        # Subscribe to joint states to get current robot position
        self.joint_state_subscription = self.create_subscription(
            JointState, "/joint_states", self._joint_state_callback, 10
        )

        self.get_logger().info(f"Generic test node started for {self.message_type.__name__}")

    def _joint_state_callback(self, msg: JointState):
        """Callback to store current joint positions."""
        # print("JOINT MSG")
        self.current_joint_positions = list(msg.position)
        self.joint_names_order = list(msg.name)

    def get_current_joint_positions(self, timeout: float = 5.0) -> List[float]:
        """Wait for and return current joint positions."""
        start_time = time.time()

        while self.current_joint_positions is None and (time.time() - start_time) < timeout:
            self.get_logger().info("Waiting for joint state message...")
            time.sleep(0.1)

        if self.current_joint_positions is None:
            self.get_logger().error(f"Failed to get joint positions within {timeout}s")
            # Return default positions as fallback
            return None

        # Reorder positions to match fr3_joint1-7 order
        expected_joint_names = [
            "fr3_joint1",
            "fr3_joint2",
            "fr3_joint3",
            "fr3_joint4",
            "fr3_joint5",
            "fr3_joint6",
            "fr3_joint7",
        ]

        ordered_positions = []
        for joint_name in expected_joint_names:
            if joint_name in self.joint_names_order:
                idx = self.joint_names_order.index(joint_name)
                ordered_positions.append(self.current_joint_positions[idx])
            else:
                # self.get_logger().warn(f"Joint {joint_name} not found in joint states")
                # ordered_positions.append(0.0)  # Default value
                raise ValueError(f"Joint {joint_name} not found in joint states")

        self.get_logger().info(f"Current joint positions: {ordered_positions}")
        return ordered_positions

    def get_controller_state(self, controller_name: str) -> str:
        """Get the current state of a controller."""
        request = ListControllers.Request()
        future = self.list_controllers_client.call_async(request)

        # Don't use spin_until_future_complete when already in a spinning executor
        # Instead, wait for the future with a timeout
        start_time = time.time()
        timeout = 5.0

        while not future.done() and (time.time() - start_time) < timeout:
            time.sleep(0.01)  # Small sleep to avoid busy waiting

        if not future.done():
            self.get_logger().error("Controller state request timed out")
            return "error"

        try:
            result = future.result()
            if result is not None:
                for controller in result.controller:
                    if controller.name == controller_name:
                        return controller.state
                self.get_logger().warn(f"Controller '{controller_name}' not found")
                return "not_found"
            else:
                self.get_logger().error("Failed to get controller list")
                return "error"
        except Exception as e:
            self.get_logger().error(f"Exception getting controller state: {e}")
            return "error"

    def is_controller_active(self, controller_name: str) -> bool:
        """Check if a controller is in active state."""
        state = self.get_controller_state(controller_name)
        return state == "active"

    def wait_for_controller_state(self, controller_name: str, desired_state: str, timeout: float = 10.0) -> bool:
        """Wait for a controller to reach a desired state within timeout."""
        start_time = time.time()

        while time.time() - start_time < timeout:
            current_state = self.get_controller_state(controller_name)
            if current_state == desired_state:
                self.get_logger().info(f"Controller '{controller_name}' reached state '{desired_state}'")
                return True
            elif current_state in ["not_found", "error"]:
                self.get_logger().error(f"Error checking controller '{controller_name}' state: {current_state}")
                return False

            time.sleep(0.1)  # Check every 100ms

        self.get_logger().error(
            f"Controller '{controller_name}' did not reach state '{desired_state}' within {timeout}s"
        )
        return False

    def topic_exists(self, topic_name: str) -> bool:
        """Check if a topic exists by looking at available topics."""
        topic_names_and_types = self.get_topic_names_and_types()

        for name, _ in topic_names_and_types:
            if name == topic_name:
                self.get_logger().info(f"Topic '{topic_name}' found")
                return True

        self.get_logger().warn(f"Topic '{topic_name}' not found")
        available_topics = [name for name, _ in topic_names_and_types]
        self.get_logger().info(f"Available topics: {available_topics}")
        return False

    def wait_for_topic(self, topic_name: str, timeout: float = 10.0) -> bool:
        """Wait for a topic to become available within timeout."""
        start_time = time.time()

        while time.time() - start_time < timeout:
            if self.topic_exists(topic_name):
                self.get_logger().info(f"Topic '{topic_name}' is now available")
                return True

            time.sleep(0.1)  # Check every 100ms

        self.get_logger().error(f"Topic '{topic_name}' did not become available within {timeout}s")
        return False

    def _publish_callback(self):
        msg = self.message_type()

        # # Set message content based on type
        # if isinstance(msg, String):
        #     msg.data = f"test_message_{self.messages_sent}"
        # elif isinstance(msg, PoseStamped):
        #     msg.header.stamp = self.get_clock().now().to_msg()
        #     msg.header.frame_id = "test_frame"
        #     msg.pose.position.x = float(self.messages_sent % 10)
        #     msg.pose.position.y = float(self.messages_sent % 5)
        #     msg.pose.position.z = 1.0
        #     msg.pose.orientation.w = 1.0
        # elif isinstance(msg, Delayed):
        #     msg.header.stamp = self.get_clock().now().to_msg()

        self.publisher.publish(msg)
        self.messages_sent += 1

    def _message_callback(self, msg):
        self.messages_received += 1

        if self.messages_received % 50 == 0:  # Log every 50 messages
            self.get_logger().info(f"Received {self.messages_received} messages")

    def send_joint_sin_trajectory(self, topic_name: str, joint_idx: int = 1):
        """Send a sinusoidal trajectory to joint 2 over the specified duration."""
        if self.trajectory_publisher is None:
            self.trajectory_publisher = self.create_publisher(JointTrajectory, topic_name, 10)
            time.sleep(0.5)  # Allow publisher to establish connection

        # Joint names for Franka FR3
        joint_names = [
            "fr3_joint1",
            "fr3_joint2",
            "fr3_joint3",
            "fr3_joint4",
            "fr3_joint5",
            "fr3_joint6",
            "fr3_joint7",
        ]

        # Trajectory parameters - much smoother
        frequency = 0.05  # Hz for sinusoidal motion (very slow!)
        amplitude = np.pi / 6  # radians (smaller amplitude)

        duration = (1 / frequency) / 2  # Duration for one full cycle

        # Wait 2 seconds for joint positions to stabilize
        self.get_logger().info("Waiting for 2 seconds before starting trajectory...")
        while self.current_joint_positions[1] == 0.0:  # Wait until joint positions are initialized
            self.get_logger().info("Waiting for joint2 position to be non-zero", throttle_duration_sec=1.0)
            time.sleep(0.1)

        # Get current robot position instead of hardcoded values
        start_positions = self.current_joint_positions[:7].copy()  # Get current positions of all joints

        center_position = start_positions[joint_idx]  # center position for joint 2

        # Much finer time step for smoother trajectory
        dt = 0.01  # 10ms between points (100Hz)
        num_points = int(duration / dt)

        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ""
        msg.joint_names = joint_names

        # Arrays for plotting
        times = []
        positions = []
        velocities = []
        accelerations = []

        # Add initial point (current position with zero velocities)
        initial_point = JointTrajectoryPoint()
        initial_point.time_from_start.sec = 0
        initial_point.time_from_start.nanosec = 0
        initial_point.positions = start_positions.copy()
        initial_point.velocities = [0.0] * len(joint_names)
        initial_point.accelerations = [0.0] * len(joint_names)
        msg.points.append(initial_point)

        times.append(0.0)
        positions.append(start_positions[joint_idx])
        velocities.append(0.0)
        accelerations.append(0.0)

        wait_time = 2.0

        for i in range(1, num_points + 1):
            t = i * dt  # Start after wait_time seconds
            point = JointTrajectoryPoint()

            # Set time from start
            traj_time = wait_time + t  # Start after wait_time seconds
            point.time_from_start.sec = int(traj_time)
            point.time_from_start.nanosec = int((traj_time - int(traj_time)) * 1e9)

            # Initialize all joint positions to start positions
            point.positions = start_positions.copy()
            point.velocities = [0.0] * len(joint_names)
            point.accelerations = [0.0] * len(joint_names)

            # Set sinusoidal motion for joint 2 (index 1)
            joint_position = center_position + amplitude * np.sin(2 * np.pi * frequency * t)
            # joint_velocity = amplitude * 2 * np.pi * frequency * np.cos(2 * np.pi * frequency * t)
            # joint_acceleration = -amplitude * (2 * np.pi * frequency) ** 2 * np.sin(2 * np.pi * frequency * t)

            # joint2_position = center_position + i * 0.0000001

            point.positions[joint_idx] = joint_position
            # point.velocities[1] = joint2_velocity
            # point.accelerations[1] = joint2_acceleration

            msg.points.append(point)

            # Store for plotting
            times.append(traj_time)
            positions.append(joint_position)
            # velocities.append(joint2_velocity)
            # accelerations.append(joint2_acceleration)

        final_point: JointTrajectoryPoint = copy.deepcopy(msg.points[-1])  # Copy the last point to ensure smooth stop
        final_t = final_point.time_from_start.sec + 1.0  # Extra second to come to rest

        final_point.time_from_start.sec = int(final_t)
        final_point.time_from_start.nanosec = int((final_t - int(final_t)) * 1e9)
        # final_point.positions = start_positions.copy()
        # final_point.velocities = [0.0] * len(joint_names)
        # final_point.accelerations = [0.0] * len(joint_names)
        msg.points.append(final_point)

        expected_final_positions = final_point.positions

        times.append(final_t)
        positions.append(expected_final_positions[joint_idx])
        # velocities.append(0.0)
        # accelerations.append(0.0)

        # Plot the trajectory
        # self._plot_trajectory(times, positions, frequency, amplitude)

        # Publish the trajectory
        self.get_logger().info(
            f"Publishing smooth trajectory with {len(msg.points)} points over {duration}s for {joint_names[joint_idx]}"
        )
        self.get_logger().info(f"Frequency: {frequency}Hz, Amplitude: {amplitude}rad, dt: {dt}s")
        self.trajectory_publisher.publish(msg)

        # # Wait for trajectory executions
        # self.get_logger().info("Waiting for trajectory execution to complete...")
        # time.sleep(duration + 2.0)  # Extra time for safety

        # Return expected final positions for verification
        return duration, expected_final_positions

    def verify_final_position(self, expected_positions: List[float], tolerance: float = 0.01) -> bool:
        """Verify that the robot's current position matches the expected final position."""
        # Reset joint positions to get fresh reading
        self.current_joint_positions = None

        # Wait a bit for new joint state message
        time.sleep(0.1)

        # Get current position
        actual_positions = self.get_current_joint_positions(timeout=2.0)

        if actual_positions is None:
            self.get_logger().error("Failed to get current joint positions for verification")
            return False

        if len(actual_positions) != len(expected_positions):
            self.get_logger().error(
                f"Position array length mismatch: {len(actual_positions)} vs {len(expected_positions)}"
            )
            return False

        # Check each joint
        all_within_tolerance = True
        for i, (actual, expected) in enumerate(zip(actual_positions, expected_positions)):
            error = abs(actual - expected)
            joint_name = f"fr3_joint{i + 1}"

            if error > tolerance:
                self.get_logger().error(f"{joint_name}: error {error:.6f} > tolerance {tolerance}")
                all_within_tolerance = False
            else:
                self.get_logger().info(f"{joint_name}: error {error:.6f} (OK)")

        if all_within_tolerance:
            self.get_logger().info("All joints within tolerance - trajectory execution successful!")
        else:
            self.get_logger().error("Some joints outside tolerance - trajectory execution may have failed")

        return all_within_tolerance

    def _publish_callback(self):
        msg = self.message_type()

        # # Set message content based on type
        # if isinstance(msg, String):
        #     msg.data = f"test_message_{self.messages_sent}"
        # elif isinstance(msg, PoseStamped):
        #     msg.header.stamp = self.get_clock().now().to_msg()
        #     msg.header.frame_id = "test_frame"
        #     msg.pose.position.x = float(self.messages_sent % 10)
        #     msg.pose.position.y = float(self.messages_sent % 5)
        #     msg.pose.position.z = 1.0
        #     msg.pose.orientation.w = 1.0
        # elif isinstance(msg, Delayed):
        #     msg.header.stamp = self.get_clock().now().to_msg()

        self.publisher.publish(msg)
        self.messages_sent += 1

    def _message_callback(self, msg):
        self.messages_received += 1

        if self.messages_received % 50 == 0:  # Log every 50 messages
            self.get_logger().info(f"Received {self.messages_received} messages")
        plt.show(block=False)

    def _plot_trajectory(self, times, positions, frequency, amplitude):
        """Plot the joint trajectory positions over time."""
        fig, ax = plt.subplots(1, 1, figsize=(12, 6))

        # Position plot
        ax.plot(
            times,
            positions,
            "b-",
            linewidth=2,
            label=f"Joint Position (f={frequency}Hz, A={amplitude:.3f}rad)",
        )
        ax.set_ylabel("Position (rad)")
        ax.set_xlabel("Time (s)")
        ax.set_title("Joint Trajectory - Position vs Time")
        ax.grid(True, alpha=0.3)
        ax.legend()

        # Add some trajectory info as text
        min_pos = min(positions)
        max_pos = max(positions)
        range_pos = max_pos - min_pos

        info_text = f"Min: {min_pos:.3f} rad\nMax: {max_pos:.3f} rad\nRange: {range_pos:.3f} rad"
        ax.text(
            0.02,
            0.98,
            info_text,
            transform=ax.transAxes,
            verticalalignment="top",
            bbox=dict(boxstyle="round", facecolor="lightblue", alpha=0.7),
        )

        plt.tight_layout()

        # Save plot to file
        plot_path = "/tmp/joint_trajectory.png"
        plt.savefig(plot_path, dpi=300, bbox_inches="tight")
        self.get_logger().info(f"Trajectory plot saved to: {plot_path}")

        # Show plot (non-blocking)
        plt.show(block=True)

        # Keep plot open for a short time then close
        plt.pause(5.0)
        plt.close()
