"""Provides a client to control the franka robot. It is the easiest way to control the robot using ROS2."""

import threading
from dataclasses import dataclass
from typing import List

import numpy as np
import rclpy
import rclpy.executors
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped, WrenchStamped, TwistStamped
from numpy.typing import NDArray
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from scipy.spatial.transform import Rotation, Slerp
from sensor_msgs.msg import JointState

from arm_client.control.controller_switcher import ControllerSwitcherClient
from arm_client.control.joint_trajectory_controller_client import JointTrajectoryControllerClient
from arm_client.control.parameters_client import ParametersClient
from arm_client.robot_config import FR3Config, RobotConfig
from arm_client.utils.callback_monitor import CallbackMonitor
from arm_interfaces.msg import CartesianTrajectory

import time


@dataclass
class Pose:
    """Compact representation of an SE3 object."""

    position: np.ndarray
    orientation: Rotation

    def copy(self) -> "Pose":
        """Create a copy of this pose."""
        return Pose(self.position.copy(), Rotation.from_quat(self.orientation.as_quat()))

    def __str__(self) -> str:
        """Return a string representation of a Pose."""
        return f"Pos: {np.array2string(self.position, suppress_small=True, precision=2, floatmode='fixed')},\n Orientation: {np.array2string(self.orientation.as_matrix(), suppress_small=True, precision=2, floatmode='fixed')}"

    def __sub__(self, other: "Pose") -> "Pose":
        """Subtract another pose from this pose, i.e. compute the relative pose."""
        return Pose(
            self.position - other.position,
            self.orientation * other.orientation.inv(),
        )

    def __add__(self, other: "Pose") -> "Pose":
        """Add another pose to this pose, i.e. add a relative pose."""
        return Pose(
            self.position + other.position,
            other.orientation * self.orientation,
        )


@dataclass
class Twist:
    """Compact representation of a twist (linear + angular velocity)."""

    linear: np.ndarray
    angular: np.ndarray

    def copy(self) -> "Twist":
        """Create a copy of this twist."""
        return Twist(self.linear.copy(), self.angular.copy())

    def __str__(self) -> str:
        """Return a string representation of a Twist."""
        return f"Linear: {np.array2string(self.linear, suppress_small=True, precision=2, floatmode='fixed')},\n Angular: {np.array2string(self.angular, suppress_small=True, precision=2, floatmode='fixed')}"

    def __sub__(self, other: "Twist") -> "Twist":
        """Subtract another twist from this twist, i.e. compute the relative twist."""
        return Twist(
            self.linear - other.linear,
            self.angular - other.angular,
        )

    def __add__(self, other: "Twist") -> "Twist":
        """Add another twist to this twist, i.e. add a relative twist."""
        return Twist(
            self.linear + other.linear,
            self.angular + other.angular,
        )


class Robot:
    """A high-level interface for controlling robots using ROS2.

    This class provides an easy-to-use interface for controlling robots through ROS2,
    supporting both joint space and Cartesian space control. It handles controller
    switching, trajectory generation, and state monitoring.

    Attributes:
        THREADS_REQUIRED (int): Number of threads required for the ROS2 executor
        node (Node): ROS2 node instance
        config (RobotConfig): Robot configuration parameters
        controller_switcher_client: Client for switching between controllers
        joint_trajectory_controller_client: Client for joint trajectory control
        cartesian_controller_parameters_client: Client for Cartesian controller parameters
    """

    THREADS_REQUIRED = 4

    def __init__(
        self,
        node: Node | None = None,
        namespace: str = "",
        spin_node: bool = True,
        robot_config: RobotConfig | None = None,
        name: str = "robot_client",
        robot_name: str = "fr3",
    ) -> None:
        """Initialize the robot interface.

        Args:
            node (Node, optional): ROS2 node to use. If None, creates a new node.
            namespace (str, optional): ROS2 namespace for the robot.
            spin_node (bool, optional): Whether to spin the node in a separate thread.
            robot_config (RobotConfig, optional): Robot configuration parameters.
            name (str, optional): Name of the robot client node.
        """
        if node is None:
            if not rclpy.ok():
                rclpy.init()
            self.node = rclpy.create_node(name, namespace=namespace)
        else:
            self.node = node

        self.config = robot_config if robot_config else FR3Config()

        self._prefix = f"{namespace}_" if namespace else ""

        self.controller_switcher_client = ControllerSwitcherClient(self.node)
        self.joint_trajectory_controller_client = JointTrajectoryControllerClient(self.node)

        self.cartesian_controller_parameters_client = ParametersClient(
            self.node, target_node=self.config.cartesian_impedance_controller_name
        )
        self.joint_controller_parameters_client = ParametersClient(
            self.node, target_node=self.config.joint_trajectory_controller_name
        )
        self.osc_pd_controller_parameters_client = ParametersClient(self.node, target_node="osc_pd_controller")

        self.joint_space_controller_parameters_client = ParametersClient(
            self.node, target_node="joint_space_controller"
        )

        # Joint space states
        self._q_current = None
        self._q_target = None
        self._dq_current = None
        self._dq_target = None
        self._tau_current = None
        self._tau_target = None

        # Task space states
        self._current_pose = None
        self._current_twist = None

        self._target_pose = None
        self._target_joint = None
        self._target_wrench = None
        self._target_twist = None
        self._current_wrench = None  # added current wrench

        # Flag to disable target_pose publishing during trajectory execution
        self._trajectory_mode_active = False

        self._callback_monitor = CallbackMonitor(
            node=self.node,
            stale_threshold=max(self.config.max_pose_delay, self.config.max_joint_delay),
        )

        self._target_pose_publisher = self.node.create_publisher(
            PoseStamped, self.config.target_pose_topic, qos_profile_system_default
        )
        self._target_trajectory_publisher = self.node.create_publisher(
            CartesianTrajectory, self.config.target_trajectory_topic, qos_profile_system_default
        )
        self._target_wrench_publisher = self.node.create_publisher(
            WrenchStamped, "target_wrench", qos_profile_system_default
        )
        self._target_joint_publisher = self.node.create_publisher(
            JointState, self.config.target_joint_topic, qos_profile_system_default
        )
        self._target_twist_publisher = self.node.create_publisher(
            TwistStamped, "target_twist", qos_profile_system_default
        )

        # Current state subscriptions
        # pose
        self.node.create_subscription(
            PoseStamped,
            self.config.current_pose_topic,
            self._callback_monitor.monitor(f"{namespace.capitalize()} Current Pose", self._callback_current_pose),
            qos_profile_sensor_data,
            callback_group=ReentrantCallbackGroup(),
        )
        # joint states
        self.node.create_subscription(
            JointState,
            self.config.current_joint_topic,
            self._callback_monitor.monitor(f"{namespace.capitalize()} Current Joint", self._callback_current_joint),
            qos_profile_sensor_data,
            callback_group=ReentrantCallbackGroup(),
        )
        # external wrench
        self.node.create_subscription(  # add subscription to wrench in base frame
            WrenchStamped,
            self.config.current_wrench_topic,
            self._callback_monitor.monitor(f"{namespace.capitalize()} Current Wrench", self._callback_current_wrench),
            qos_profile_sensor_data,
            callback_group=ReentrantCallbackGroup(),
        )

        self.node.create_timer(
            1.0 / 100.0,
            self._callback_monitor.monitor(f"{namespace.capitalize()} Target Pose", self._callback_publish_target_pose),
            ReentrantCallbackGroup(),
        )
        self.node.create_timer(
            1.0 / self.config.publish_frequency,
            self._callback_monitor.monitor(
                f"{namespace.capitalize()} Target Joint", self._callback_publish_target_joint
            ),
            ReentrantCallbackGroup(),
        )
        self.node.create_timer(
            1.0 / self.config.publish_frequency,
            self._callback_publish_target_wrench,
            ReentrantCallbackGroup(),
        )
        self.node.create_timer(
            1.0 / self.config.publish_frequency,
            self._callback_publish_target_twist,
            ReentrantCallbackGroup(),
        )

        if spin_node:
            threading.Thread(target=self._spin_node, daemon=True).start()

    def _spin_node(self):
        if not rclpy.ok():
            rclpy.init()
        executor = rclpy.executors.MultiThreadedExecutor(num_threads=self.THREADS_REQUIRED)
        executor.add_node(self.node)
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)

    @property
    def nq(self) -> int:
        """Get the number of joints in the robot.

        Returns:
            int: The number of joints in the robot configuration.
        """
        return len(self.config.joint_names)

    @property
    def end_effector_pose(self) -> Pose:
        """Get the current pose of the end effector.

        Returns:
            Pose: The current pose of the end effector, or None if not available.
        """
        if self._current_pose is None:
            raise RuntimeError(
                "The robot has not received any poses yet. Run wait_until_ready() before running anything else."
            )
        return self._current_pose.copy()

    @property
    def end_effector_wrench(self) -> dict:
        """Get the current wrench applied at the end effector.

        Returns:
            dict: The current wrench applied at the end effector, or None if not available.
        """
        if self._current_wrench is None:
            raise RuntimeError(
                "The robot has not received any wrenches yet. Run wait_until_ready() before running anything else."
            )
        return self._current_wrench.copy()

    @property
    def target_pose(self) -> Pose:
        """Get the target pose of the end effector.

        Returns:
            Pose: The target pose of the end effector, or None if not set.
        """
        if self._target_pose is None:
            raise RuntimeError(
                "The robot has not received any poses yet. Run wait_until_ready() before running anything else."
            )
        return self._target_pose.copy()

    @property
    def q(self) -> NDArray:
        """Get the current joint values of the robot.

        Returns:
            numpy.ndarray: Copy of current joint values, or None if not available.
        """
        if self._q_current is None:
            raise RuntimeError(
                "The robot has not received any joints yet. Run wait_until_ready() before running anything else."
            )
        return self._q_current.copy()

    @property
    def q_target(self) -> NDArray:
        """Get the target joint values of the robot.

        Returns:
            numpy.ndarray: Copy of target joint values, or None if not available.
        """
        if self._q_target is None:
            raise RuntimeError(
                "The robot has not received any joints yet. Run wait_until_ready() before running anything else."
            )
        return self._q_target.copy()

    @property
    def dq(self) -> NDArray:
        """Get the current joint velocities of the robot.

        Returns:
            numpy.ndarray: Copy of current joint velocities, or None if not available.
        """
        if self._dq_current is None:
            raise RuntimeError(
                "The robot has not received any joints yet. Run wait_until_ready() before running anything else."
            )
        return self._dq_current.copy()

    @property
    def dq_target(self) -> NDArray:
        """Get the target joint velocities of the robot.

        Returns:
            numpy.ndarray: Copy of target joint velocities, or None if not available.
        """
        if self._dq_target is None:
            raise RuntimeError(
                "The robot has not received any joints yet. Run wait_until_ready() before running anything else."
            )
        return self._dq_target.copy()

    @property
    def tau(self) -> NDArray:
        """Get the current joint torques of the robot.

        Returns:
            numpy.ndarray: Copy of current joint torques, or None if not available.
        """
        if self._tau_current is None:
            raise RuntimeError(
                "The robot has not received any joints yet. Run wait_until_ready() before running anything else."
            )
        return self._tau_current.copy()

    @property
    def tau_target(self) -> NDArray:
        """Get the target joint torques of the robot.

        Returns:
            numpy.ndarray: Copy of target joint torques, or None if not available.
        """
        if self._tau_target is None:
            raise RuntimeError(
                "The robot has not received any joints yet. Run wait_until_ready() before running anything else."
            )
        return self._tau_target.copy()

    def is_ready(self) -> bool:
        """Check if the robot is ready for operation.

        Returns:
            bool: True if all necessary values for operation are available, False otherwise.
        """
        return (
            self._current_pose is not None
            and self._target_pose is not None
            and self._q_current is not None
            and self._q_target is not None
        )

    def reset_targets(self):
        """Reset all target values to None.

        This method clears the target pose, joint values, and wrench values,
        effectively stopping any ongoing movement or force application.
        """
        self._target_pose = None
        self._q_target = None
        self._target_wrench = None

    def wait_until_ready(self, timeout: float = 10.0, check_frequency: float = 10.0):
        """Wait until the robot is ready for operation.

        Args:
            timeout (float, optional): Maximum time to wait in seconds. Defaults to 10.0.
            check_frequency (float, optional): How often to check readiness in Hz. Defaults to 10.0.

        Raises:
            TimeoutError: If the robot is not ready within the specified timeout.
        """
        rate = self.node.create_rate(check_frequency)
        while not self.is_ready():
            rate.sleep()
            timeout -= 1.0 / check_frequency
            if timeout <= 0:
                raise TimeoutError("Timeout waiting for end-effector pose.")

        print("Robot is ready.")

    def set_target(self, position: List | NDArray | None = None, pose: Pose | None = None):
        """Set the target pose for the end-effector.

        Args:
            position (List | NDArray, optional): Target position as [x, y, z]. If None, uses current orientation.
            pose (Pose, optional): Target pose as SE3 transform. If None, uses position.

        Note:
            Either position or pose must be provided. If both are provided, position overrides
            the translation component of pose.
        """
        target_pose = self._parse_pose_or_position(position, pose)
        self._target_pose = target_pose.copy()
        # Re-enable continuous pose publishing (single-pose mode)
        self._trajectory_mode_active = False

    def set_target_joint(self, q: NDArray):
        """Set the target joint configuration.

        Args:
            q (np.array): Target joint values array of size nq.

        Raises:
            AssertionError: If q is not the same size as the number of joints.
        """
        assert len(q) == self.nq, "Joint state must be of size nq."
        self._q_target = q

    def _callback_publish_target_pose(self):
        """Publish the current target pose if one exists.

        This callback is triggered periodically to publish the target pose
        to the ROS topic for the robot controller.

        Note: Does not publish when trajectory mode is active to avoid
        interfering with trajectory execution.
        """
        if self._target_pose is None or not rclpy.ok() or self._trajectory_mode_active:
            return
        self._target_pose_publisher.publish(self._pose_to_pose_msg(self._target_pose))

    def _callback_publish_target_joint(self):
        """Publish the current target joint configuration if one exists.

        This callback is triggered periodically to publish the target joint values
        to the ROS topic for the robot controller.
        """
        if self._q_target is None or not rclpy.ok():
            return
        self._target_joint_publisher.publish(self._joint_to_joint_msg(self._q_target))

    def _callback_publish_target_wrench(self):
        """Publish the target wrench if one exists.

        This callback is triggered periodically to publish the target wrench (force/torque)
        to the ROS topic for the robot controller.
        """
        if self._target_wrench is None or not rclpy.ok():
            return
        self._target_wrench_publisher.publish(self._wrench_to_wrench_msg(self._target_wrench))

    def _callback_publish_target_twist(self):
        """Publish the current target twist if one exists.

        This callback is triggered periodically to publish the target twist
        to the ROS topic for the robot controller.
        """
        if self._target_twist is None or not rclpy.ok():
            return
        self._target_twist_publisher.publish(self._twist_to_twist_msg(self._target_twist))

    def set_target_wrench(self, force: List | NDArray | None = None, torque: List | NDArray | None = None):
        """Set the target wrench (force/torque) to be applied by the robot.

        Args:
            force (list, optional): Force vector [fx, fy, fz] in N. If None, zeros are used.
            torque (list, optional): Torque vector [tx, ty, tz] in Nm. If None, zeros are used.

        Raises:
            AssertionError: If force or torque vectors are not 3D vectors.
        """
        if force is None:
            force = [0.0, 0.0, 0.0]
        if torque is None:
            torque = [0.0, 0.0, 0.0]

        assert len(force) == 3, "Force must be a 3D vector"
        assert len(torque) == 3, "Torque must be a 3D vector"

        self._target_wrench = {"force": np.array(force), "torque": np.array(torque)}

    def _wrench_to_wrench_msg(self, wrench: dict) -> WrenchStamped:
        """Convert a wrench dictionary to a ROS WrenchStamped message.

        Args:
            wrench (dict): Dictionary containing 'force' and 'torque' numpy arrays.

        Returns:
            WrenchStamped: ROS message containing the wrench data with proper header.
        """
        wrench_msg = WrenchStamped()
        wrench_msg.header.frame_id = self.config.base_frame
        wrench_msg.header.stamp = self.node.get_clock().now().to_msg()
        wrench_msg.wrench.force.x = wrench["force"][0]
        wrench_msg.wrench.force.y = wrench["force"][1]
        wrench_msg.wrench.force.z = wrench["force"][2]
        wrench_msg.wrench.torque.x = wrench["torque"][0]
        wrench_msg.wrench.torque.y = wrench["torque"][1]
        wrench_msg.wrench.torque.z = wrench["torque"][2]
        return wrench_msg

    def _wrench_msg_to_wrench(self, msg: WrenchStamped) -> dict:
        """Convert a ROS WrenchStamped message to a wrench dictionary.

        Args:
            msg (WrenchStamped): ROS message containing the wrench data.

        Returns:
            dict: Dictionary containing 'force' and 'torque' numpy arrays.
        """
        force = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
        torque = np.array([msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z])
        return {"force": force, "torque": torque}

    def _callback_current_wrench(self, msg: WrenchStamped):
        """Update the current wrench from a ROS message.

        This callback is triggered when a new wrench message is received. It updates
        the current wrench.

        Args:
            msg (WrenchStamped): ROS message containing the current wrench.
        """
        self._current_wrench = self._wrench_msg_to_wrench(msg)
        if self._target_wrench is None:
            self._target_wrench = self._current_wrench.copy()

    def _callback_current_pose(self, msg: PoseStamped):
        """Update the current pose from a ROS message.

        This callback is triggered when a new pose message is received. It updates
        the current pose and initializes the target pose if not already set.

        Args:
            msg (PoseStamped): ROS message containing the current pose.
        """
        self._current_pose = self._pose_msg_to_pose(msg)
        if self._target_pose is None:
            self._target_pose = self._current_pose.copy()

    def _callback_current_joint(self, msg: JointState):
        """Update the current joint state (position, velocity and torque) from a ROS message.

        This callback filters the joint state message to only include joints
        that are part of this robot's configuration.

        Args:
            msg (JointState): ROS message containing joint states.
        """
        if self._q_current is None:
            self._q_current = np.zeros(self.nq)
            self._dq_current = np.zeros(self.nq)
            self._tau_current = np.zeros(self.nq)

        # self.node.get_logger().info(f"Current joint state: {msg.name} {msg.position}", throttle_duration_sec=1.0)
        for joint_name, joint_position, joint_velocity, joint_torque in zip(
            msg.name, msg.position, msg.velocity, msg.effort
        ):
            if joint_name not in self.config.joint_names:
                continue

            jnt_idx = self.config.joint_names.index(joint_name)
            self._q_current[jnt_idx] = joint_position
            self._dq_current[jnt_idx] = joint_velocity
            self._tau_current[jnt_idx] = joint_torque

        if self._q_target is None:
            self._q_target = self._q_current.copy()

        if self._dq_target is None:
            self._dq_target = self._dq_current.copy()

        if self._tau_target is None:
            self._tau_target = self._tau_current.copy()

    def move_to(self, position: List | NDArray | None = None, pose: Pose | None = None, speed: float = 0.05):
        """Move the end-effector to a given pose by interpolating linearly between the poses.

        Args:
            position: Position to move to. If None, the pose is used.
            pose: The pose to move to. If None, the position is used.
            speed: The speed of the movement. [m/s]
        """
        if self._current_pose is None:
            raise RuntimeError(
                "The robot has not received any poses yet. Run wait_until_ready() before running anything else."
            )
        desired_pose = self._parse_pose_or_position(position, pose)
        start_pose = self._current_pose
        distance = np.linalg.norm(desired_pose.position - start_pose.position)
        time_to_move = distance / speed

        N = int(time_to_move * self.config.publish_frequency)

        rate = self.node.create_rate(self.config.publish_frequency)

        slerp = Slerp([0, 1], Rotation.concatenate([start_pose.orientation, desired_pose.orientation]))

        for t in np.linspace(0.0, 1.0, N):
            pos = (1 - t) * start_pose.position + t * desired_pose.position
            ori = slerp([t])[0]
            next_pose = Pose(pos, ori)
            self._target_pose = next_pose
            rate.sleep()

        self._target_pose = desired_pose

    def execute_trajectory(
        self,
        waypoints: List[Pose],
        time_from_start: List[float],
        max_linear_velocity: float = -1.0,
        max_angular_velocity: float = -1.0,
    ):
        """Execute a Cartesian trajectory through multiple waypoints.

        This method sends a complete trajectory to the controller, which will
        execute it using quintic (5th order) polynomial interpolation for smooth
        motion with continuous velocity and acceleration.

        Args:
            waypoints: List of Pose objects defining the trajectory waypoints
            time_from_start: Cumulative time (in seconds) to reach each waypoint from trajectory start.
                           Must be same length as waypoints and monotonically increasing.
            max_linear_velocity: Optional override for max linear velocity (m/s).
                               Set to -1.0 to use controller default.
            max_angular_velocity: Optional override for max angular velocity (rad/s).
                                Set to -1.0 to use controller default.

        Example:
            >>> # Create a sinusoidal trajectory
            >>> start_pose = robot.end_effector_pose
            >>> waypoints = []
            >>> times = []
            >>> for i, t in enumerate(np.linspace(0, 2.0, 50)):
            >>>     z = 0.5 + 0.05 * np.sin(2*np.pi*t)
            >>>     pose = Pose(np.array([0.4, 0.0, z]), start_pose.orientation)
            >>>     waypoints.append(pose)
            >>>     times.append(t)
            >>> robot.execute_trajectory(waypoints, times)
        """
        if len(waypoints) != len(time_from_start):
            raise ValueError("waypoints and time_from_start must have the same length")

        if len(waypoints) == 0:
            raise ValueError("waypoints list cannot be empty")

        # Create CartesianTrajectory message
        msg = CartesianTrajectory()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = self.config.base_frame

        # Add waypoints
        for path, time_sec in zip(waypoints, time_from_start):
            pose = path[0]
            twist = path[1]

            point = self._pose_to_pose_msg(pose)
            msg.points.append(point)

            # Convert float time to Duration
            duration = Duration()
            duration.sec = int(time_sec)
            duration.nanosec = int((time_sec % 1.0) * 1e9)
            msg.time_from_start.append(duration)

        # Set velocity limits
        msg.max_linear_velocity = max_linear_velocity
        msg.max_angular_velocity = max_angular_velocity

        # Enable trajectory mode to stop continuous pose publishing
        self._trajectory_mode_active = True

        time.sleep(0.5)  # Small delay to ensure mode switch before publishing

        # Publish trajectory
        self._target_trajectory_publisher.publish(msg)

        self.node.get_logger().info(
            f"Sent trajectory with {len(waypoints)} waypoints, total duration: {time_from_start[-1]:.3f}s"
        )

    def wait_for_trajectory_completion(self, expected_duration: float, timeout_margin: float = 2.0):
        """Wait for trajectory execution to complete.

        Args:
            expected_duration: Expected trajectory duration in seconds
            timeout_margin: Additional time to wait beyond expected duration (seconds)

        Note:
            This is a simple time-based wait. For more precise tracking, consider
            converting to a ROS2 action interface in the future.
        """
        timeout = expected_duration + timeout_margin
        rate = self.node.create_rate(10)  # 10 Hz check rate
        elapsed = 0.0

        while elapsed < timeout:
            rate.sleep()
            elapsed += 0.1

        # Re-enable pose publishing
        self._trajectory_mode_active = False
        self.node.get_logger().info("Trajectory execution completed")

    def home(self, home_config: list[float] | None = None, blocking: bool = True, time_to_home: float | None = None):
        """Home the robot."""
        self.controller_switcher_client.switch_controller("joint_trajectory_controller")
        self.joint_trajectory_controller_client.send_joint_config(
            self.config.joint_names,
            self.config.home_config if home_config is None else home_config,
            self.config.time_to_home if time_to_home is None else time_to_home,
            blocking=blocking,
        )

        # Set to none to avoid publishing the previous target pose after activating the next controller
        self._target_pose = None
        self._q_target = None

        if blocking:
            self.wait_until_ready()

        # if switch_to_default_controller:
        #     self.controller_switcher_client.switch_controller(self.config.default_controller)

    def _pose_msg_to_pose(self, pose: PoseStamped) -> Pose:
        """Convert a ROS2 pose msg to a pose."""
        position = np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])
        orientation = Rotation.from_quat(
            [
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            ]
        )
        return Pose(position, orientation)

    def _pose_to_pose_msg(self, pose: Pose) -> PoseStamped:
        """Convert a pose to a pose message."""
        msg = PoseStamped()
        msg.header.frame_id = self.config.base_frame
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = pose.position
        q = pose.orientation.as_quat()
        (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ) = q
        return msg

    def _joint_to_joint_msg(self, q: NDArray, dq: NDArray | None = None, tau: NDArray | None = None) -> JointState:
        """Convert a pose to a pose message."""
        joint_msg = JointState()
        joint_msg.header.frame_id = self.config.base_frame
        joint_msg.header.stamp = self.node.get_clock().now().to_msg()
        joint_msg.name = [
            joint_name for joint_name in self.config.joint_names
        ]  # [self._prefix + joint_name for joint_name in self.config.joint_names]
        joint_msg.position = q.tolist()
        joint_msg.velocity = dq.tolist() if dq is not None else [0.0] * len(q)
        joint_msg.effort = tau.tolist() if tau is not None else [0.0] * len(q)
        return joint_msg

    def _twist_to_twist_msg(self, twist: Twist) -> TwistStamped:
        msg = TwistStamped()

        msg.header.frame_id = self.config.base_frame
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = twist.linear
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = twist.angular

        return msg

    def _parse_pose_or_position(self, position: List | NDArray | None = None, pose: Pose | None = None) -> Pose:
        """Parse a pose from a desired position or pose.

        This function is a utility to create a pose object from either a position vector or a pose object.
        """
        assert position is not None or pose is not None, "Either position or pose must be provided."

        desired_pose = pose.copy() if pose is not None else self.target_pose
        if position is not None:
            assert len(position) == 3, "Position must be a 3D vector."
            desired_pose.position = np.array(position)

        return desired_pose

    def is_homed(self) -> bool:
        """Check if the robot is homed.

        This method checks if the robot's current joint configuration matches the home configuration.

        Returns:
            bool: True if the robot is homed, False otherwise.
        """
        return np.allclose(self.q, self.config.home_config, atol=1e-1)

    def shutdown(self):
        """Shutdown the node."""
        if rclpy.ok():
            rclpy.shutdown()
