"""Franka gripper client based on ROS2 actions."""

import threading
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import rclpy
import yaml
from franka_msgs.action import Grasp, Homing, Move
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from sensor_msgs.msg import JointState

from arm_client.utils.callback_monitor import CallbackMonitor


@dataclass
class GripperConfig:
    """Franka gripper configuration.

    Configuration for the Franka gripper client that uses ROS2 actions.
    """

    max_width: float = 0.08  # Maximum gripper width in meters
    min_width: float = 0.0  # Minimum gripper width in meters
    default_speed: float = 0.1  # Default movement speed in m/s
    default_force: float = 50.0  # Default grasping force in N
    grasp_epsilon_inner: float = 0.005  # Inner epsilon for grasp action
    grasp_epsilon_outer: float = 0.005  # Outer epsilon for grasp action
    joint_state_topic: str = "franka_gripper/joint_states"
    grasp_action: str = "franka_gripper/grasp"
    move_action: str = "franka_gripper/move"
    homing_action: str = "franka_gripper/homing"
    gripper_command_action: str = "franka_gripper/gripper_action"
    max_joint_delay: float = 1.0
    action_timeout: float = 10.0  # Timeout for action calls in seconds

    @classmethod
    def from_yaml(cls, path: str | Path) -> "GripperConfig":
        """Create a GripperConfig from a YAML configuration file.

        Args:
            path (str | Path): Path to the YAML configuration file from the project root or directly full path from the filesystem.
        """
        if isinstance(path, str):
            project_root_path = Path(__file__).parent.parent.parent
            full_path = project_root_path / path
        elif isinstance(path, Path):
            full_path = path
        else:
            raise TypeError("Path must be a string or a Path object.")

        with open(full_path, "r") as file:
            config = yaml.safe_load(file)
            config = {
                "max_width": config.get("max_width", 0.08),
                "min_width": config.get("min_width", 0.0),
                "default_speed": config.get("default_speed", 0.1),
                "default_force": config.get("default_force", 50.0),
                "grasp_epsilon_inner": config.get("grasp_epsilon_inner", 0.005),
                "grasp_epsilon_outer": config.get("grasp_epsilon_outer", 0.005),
                "joint_state_topic": config.get("joint_state_topic", "franka_gripper/joint_states"),
                "grasp_action": config.get("grasp_action", "franka_gripper/grasp"),
                "move_action": config.get("move_action", "franka_gripper/move"),
                "homing_action": config.get("homing_action", "franka_gripper/homing"),
                "gripper_command_action": config.get("gripper_command_action", "franka_gripper/gripper_action"),
                "max_joint_delay": config.get("max_joint_delay", 1.0),
                "action_timeout": config.get("action_timeout", 10.0),
            }
        return cls(**config)


class Gripper:
    """Franka gripper client using ROS2 actions."""

    THREADS_REQUIRED = 2

    def __init__(
        self,
        node: Node | None = None,
        namespace: str = "",
        gripper_config: GripperConfig | None = None,
        spin_node: bool = True,
    ):
        """Initialize the Franka gripper client.

        Args:
            node (Node, optional): ROS2 node to use. If None, creates a new node.
            namespace (str, optional): ROS2 namespace for the gripper.
            gripper_config (GripperConfig, optional): configuration for the gripper class.
            spin_node (bool, optional): Whether to spin the node in a separate thread.
        """
        if not rclpy.ok() and node is None:
            rclpy.init()

        self.node = (
            rclpy.create_node(node_name="franka_gripper_client", namespace=namespace, parameter_overrides=[])
            if not node
            else node
        )
        self.config = gripper_config if gripper_config else GripperConfig()

        self._prefix = f"{namespace}_" if namespace else ""
        self._joint_positions = None
        self._current_width = None
        self._callback_monitor = CallbackMonitor(self.node, stale_threshold=self.config.max_joint_delay)

        # Subscribe to joint states
        self.node.create_subscription(
            JointState,
            self.config.joint_state_topic,
            self._callback_monitor.monitor(f"{namespace.capitalize()} Gripper Joint State", self._callback_joint_state),
            qos_profile_system_default,
            callback_group=ReentrantCallbackGroup(),
        )

        # Create action clients
        self._grasp_client = ActionClient(self.node, Grasp, self.config.grasp_action)
        self._move_client = ActionClient(self.node, Move, self.config.move_action)
        self._homing_client = ActionClient(self.node, Homing, self.config.homing_action)

        if spin_node:
            threading.Thread(target=self._spin_node, daemon=True).start()

    def _spin_node(self):
        """Spin the node in a separate thread."""
        if not rclpy.ok():
            rclpy.init()
        executor = MultiThreadedExecutor(num_threads=self.THREADS_REQUIRED)
        executor.add_node(self.node)
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)

    @property
    def max_width(self) -> float:
        """Returns the maximum width of the gripper."""
        return self.config.max_width

    @property
    def min_width(self) -> float:
        """Returns the minimum width of the gripper."""
        return self.config.min_width

    @property
    def joint_states(self) -> np.ndarray | None:
        """Returns the current joint positions of both gripper fingers or None if not initialized."""
        if self._joint_positions is None:
            raise RuntimeError(f"{self._prefix}Gripper is not initialized. Call wait_until_ready() first.")
        return self._joint_positions.copy()

    @property
    def value(self) -> float | None:
        """Returns the current width of the gripper or None if not initialized."""
        if self._current_width is None:
            raise RuntimeError(f"{self._prefix}Gripper is not initialized. Call wait_until_ready() first.")

        # Check if joint state callback is stale
        namespace_part = self._prefix.rstrip("_").capitalize() if self._prefix else ""
        callback_name = f"{namespace_part} Gripper Joint State".strip()
        try:
            joint_callback_data = self._callback_monitor.get_callback_data(callback_name)
            if joint_callback_data and joint_callback_data.is_stale:
                self.node.get_logger().warn(f"{self._prefix}Gripper joint state is stale")
        except ValueError:
            # Callback not found, which is expected if no data has been received yet
            pass

        return self._current_width

    def is_ready(self) -> bool:
        """Returns True if the gripper is fully ready to operate."""
        return self._joint_positions is not None and self._current_width is not None

    def wait_until_ready(self, timeout: float = 10.0, check_frequency: float = 10.0):
        """Wait until the gripper is available."""
        rate = self.node.create_rate(check_frequency)
        elapsed = 0.0
        while not self.is_ready():
            rate.sleep()
            elapsed += 1.0 / check_frequency
            if elapsed >= timeout:
                raise TimeoutError("Timeout waiting for gripper to be ready.")

    def is_open(self, open_threshold: float = 0.01) -> bool:
        """Returns True if the gripper is open."""
        if self.value is None:
            raise RuntimeError("Gripper value is not initialized. Call wait_until_ready() first.")
        return self.value > open_threshold

    def close(self, force: float | None = None, speed: float | None = None) -> bool:
        """Close the gripper using grasp action.

        Args:
            force (float, optional): Grasping force in N. Uses default_force if None.
            speed (float, optional): Movement speed in m/s. Uses default_speed if None.

        Returns:
            bool: True if the action succeeded, False otherwise.
        """
        force = force if force is not None else self.config.default_force
        speed = speed if speed is not None else self.config.default_speed

        goal = Grasp.Goal()
        goal.width = self.config.min_width
        goal.speed = speed
        goal.force = force
        goal.epsilon.inner = self.config.grasp_epsilon_inner
        goal.epsilon.outer = self.config.grasp_epsilon_outer

        return self._send_goal(self._grasp_client, goal)

    def open(self, speed: float | None = None) -> bool:
        """Open the gripper using move action.

        Args:
            speed (float, optional): Movement speed in m/s. Uses default_speed if None.

        Returns:
            bool: True if the action succeeded, False otherwise.
        """
        speed = speed if speed is not None else self.config.default_speed

        goal = Move.Goal()
        goal.width = self.config.max_width
        goal.speed = speed

        return self._send_goal(self._move_client, goal)

    def reset(self) -> bool:
        """Reset/home the gripper using homing action.

        Returns:
            bool: True if the action succeeded, False otherwise.
        """
        goal = Homing.Goal()
        return self._send_goal(self._homing_client, goal)

    def set_target(self, target_width: float, speed: float | None = None) -> bool:
        """Set target width for the gripper using move action.

        Args:
            target_width (float): Target width in meters.
            speed (float, optional): Movement speed in m/s. Uses default_speed if None.

        Returns:
            bool: True if the action succeeded, False otherwise.
        """
        if not (self.config.min_width <= target_width <= self.config.max_width):
            raise ValueError(
                f"Target width {target_width} is out of bounds [{self.config.min_width}, {self.config.max_width}]"
            )

        speed = speed if speed is not None else self.config.default_speed

        goal = Move.Goal()
        goal.width = target_width
        goal.speed = speed

        return self._send_goal(self._move_client, goal)

    def _send_goal(self, client: ActionClient, goal) -> bool:
        """Send a goal to an action client and wait for result.

        Args:
            client (ActionClient): The action client to use.
            goal: The goal to send.

        Returns:
            bool: True if the action succeeded, False otherwise.
        """
        if not client.wait_for_server(timeout_sec=2.0):
            self.node.get_logger().error(f"Action server {client._action_name} not available")
            return False

        future = client.send_goal_async(goal)

        # Wait for goal to be accepted
        start_time = self.node.get_clock().now()
        while not future.done():
            if (self.node.get_clock().now() - start_time).nanoseconds / 1e9 > self.config.action_timeout:
                self.node.get_logger().error(f"Timeout waiting for goal acceptance")
                return False
            rclpy.spin_once(self.node, timeout_sec=0.1)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error("Goal was rejected")
            return False

        # Wait for result
        result_future = goal_handle.get_result_async()
        start_time = self.node.get_clock().now()
        while not result_future.done():
            if (self.node.get_clock().now() - start_time).nanoseconds / 1e9 > self.config.action_timeout:
                self.node.get_logger().error(f"Timeout waiting for action result")
                return False
            rclpy.spin_once(self.node, timeout_sec=0.1)

        result = result_future.result()
        if result.result.success:
            self.node.get_logger().info(f"Action completed successfully")
            return True
        else:
            self.node.get_logger().error(f"Action failed: {result.result.error}")
            return False

    def _callback_joint_state(self, msg: JointState):
        """Save the latest joint state values and calculate current width.

        Args:
            msg (JointState): the message containing the joint state.
        """
        if len(msg.position) >= 2:
            # Store joint positions (typically 2 finger joints)
            self._joint_positions = np.array(msg.position[:2])
            # Calculate width from joint positions (sum of both finger positions)
            self._current_width = self._joint_positions[0] + self._joint_positions[1]

    def shutdown(self):
        """Shutdown the node and allow the gripper to be instantiated again."""
        if rclpy.ok():
            rclpy.shutdown()
