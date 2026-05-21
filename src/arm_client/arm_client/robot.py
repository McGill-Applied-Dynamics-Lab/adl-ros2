"""Provides a client to control the franka robot. It is the easiest way to control the robot using ROS2."""

import threading
import time
from dataclasses import dataclass
from typing import Any, List, Sequence

import numpy as np
import rclpy
import rclpy.executors
from builtin_interfaces.msg import Duration
from franka_msgs.msg import FrankaRobotState
from geometry_msgs.msg import PoseStamped, TwistStamped, WrenchStamped
from numpy.typing import NDArray
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from scipy.spatial.transform import Rotation, Slerp
from sensor_msgs.msg import JointState

from arm_client.control.controller_switcher import ControllerSwitcherClient
from arm_client.control.joint_trajectory_controller_client import JointTrajectoryControllerClient
from arm_client.control.parameters_client import ParametersClient
from arm_client.planning.ik_pyroki import plan_fr3_joint_trajectory, solve_online_planning
from arm_client.planning.types import CartesianWaypoint, PlannedJointTrajectory
from arm_client.planning.visualization import visualize_planned_joint_trajectory
from arm_client.planning.waypoints import generate_linear_waypoints
from arm_client.robot_config import FR3Config, RobotConfig
from arm_client.utils.callback_monitor import CallbackMonitor
from arm_interfaces.msg import CartesianTrajectory


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
    _JOINT_CONTROLLER_KEYWORDS = ("joint_trajectory_controller", "joint_impedance_controller", "joint_space_controller")

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

        self.fr3_pose_controller_parameters_client = ParametersClient(self.node, target_node="fr3_pose_controller")

        # Joint space states
        self._q_current = None
        self._q_target = None
        self._dq_current: None | np.ndarray = None
        self._dq_target: None | np.ndarray = None
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
        self._current_wrench_filtered: dict | None = None
        self._wrench_filter_alpha: float | None = None
        self._tau_ext_current: np.ndarray | None = None

        self._last_pose_update_time: float | None = None
        self._last_joint_update_time: float | None = None
        self._last_twist_update_time: float | None = None
        self._last_wrench_update_time: float | None = None

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
        # Franka robot state — provides tau_ext_hat_filtered (momentum observer external torque estimate)
        self.node.create_subscription(
            FrankaRobotState,
            self.config.franka_robot_state_topic,
            self._callback_monitor.monitor(f"{namespace.capitalize()} Robot State", self._callback_robot_state),
            qos_profile_sensor_data,
            callback_group=ReentrantCallbackGroup(),
        )
        # end-effector twist
        self.node.create_subscription(
            TwistStamped,
            self.config.current_twist_topic,
            self._callback_monitor.monitor(f"{namespace.capitalize()} Current Twist", self._callback_current_twist),
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

        self._rate = self.node.create_rate(100)  # 100 Hz check rate for smooth data collection

        if spin_node:
            threading.Thread(target=self._spin_node, daemon=True).start()

    def _spin_node(self):
        if not rclpy.ok():
            rclpy.init()
        executor = rclpy.executors.MultiThreadedExecutor(num_threads=self.THREADS_REQUIRED)
        executor.add_node(self.node)
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)

    # =======================
    # MARK: Properties
    # =======================

    @property
    def visualizer(self):
        """Lazy-loaded Viser-based robot visualizer."""
        if not hasattr(self, "_visualizer"):
            from arm_client.planning.visualization import RobotVisualizer

            self._visualizer = RobotVisualizer()
        return self._visualizer

    @property
    def pyroki_robot(self):
        """Lazy-loaded Pyroki robot instance."""
        if not hasattr(self, "_pyroki_robot"):
            import pyroki as pk

            from arm_client.planning.ik_pyroki import load_fr3_urdf

            self._pyroki_robot = pk.Robot.from_urdf(load_fr3_urdf())
        return self._pyroki_robot

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
    def end_effector_external_wrench(self) -> dict:
        """Get the external wrench at the end effector, filtered if a filter is configured.

        Returns the low-pass filtered wrench when configure_wrench_filter() has been called
        and data is available; otherwise returns the raw wrench.

        Read from `self.config.current_wrench_topic` (default: "/fr3/franka_robot_state_broadcaster/external_wrench_in_base_frame")

        Returns:
            dict: External wrench with 'force' and 'torque' numpy arrays.
        """
        if self._current_wrench is None:
            raise RuntimeError(
                "The robot has not received any wrenches yet. Run wait_until_ready() before running anything else."
            )
        if self._wrench_filter_alpha is not None and self._current_wrench_filtered is not None:
            return self._current_wrench_filtered.copy()
        return self._current_wrench.copy()

    @property
    def end_effector_external_wrench_raw(self) -> dict:
        """Get the raw (unfiltered) external wrench at the end effector.

        Returns:
            dict: Raw wrench with 'force' and 'torque' numpy arrays.
        """
        if self._current_wrench is None:
            raise RuntimeError(
                "The robot has not received any wrenches yet. Run wait_until_ready() before running anything else."
            )
        return self._current_wrench.copy()

    def configure_wrench_filter(self, alpha: float) -> None:
        """Configure a first-order IIR low-pass filter for end_effector_external_wrench.

        Args:
            alpha: Weight on the new measurement in [0, 1].
                   1.0 = no filtering (pass-through); smaller values give heavier smoothing.
                   Typical: 0.1 ≈ 17 Hz cutoff at 1 kHz update rate.
        """
        self._wrench_filter_alpha = float(np.clip(alpha, 0.0, 1.0))
        self._current_wrench_filtered = None

    @property
    def end_effector_twist(self) -> Twist:
        """Get the current end-effector twist.

        Returns:
            Twist: The current end-effector twist, or None if not available.
        """
        if self._current_twist is None:
            raise RuntimeError(
                "The robot has not received any twists yet. Run wait_until_ready() before running anything else."
            )
        return self._current_twist.copy()

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

        # Returns:
            numpy.ndarray: Copy of current joint torques, or None if not available.
        """
        if self._tau_current is None:
            raise RuntimeError(
                "The robot has not received any joints yet. Run wait_until_ready() before running anything else."
            )
        return self._tau_current.copy()

    @property
    def external_joint_torques(self) -> NDArray:
        """Get the external joint torques from the Franka momentum observer (tau_ext_hat_filtered).

        These are the contact/external torques only — gravity, friction, and control torques
        are excluded. Use this (not tau) as tau_ext in the RIM DynModel.

        Raises:
            RuntimeError: If FrankaRobotState has not been received yet.
        """
        if self._tau_ext_current is None:
            raise RuntimeError("External joint torques not yet received from franka_robot_state_broadcaster.")
        return self._tau_ext_current.copy()

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

    # =======================
    # MARK: General
    # =======================
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

    def wait_until_ready(self, timeout: float = 2.0, check_frequency: float = 10.0):
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

    def _is_joint_controller(self, controller_name: str) -> bool:
        """Return True if the controller name corresponds to a joint-space controller."""
        return any(keyword in controller_name for keyword in self._JOINT_CONTROLLER_KEYWORDS)

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

    # =======================
    # MARK: Callbacks
    # =======================

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

    def _callback_current_wrench(self, msg: WrenchStamped):
        """Update the current wrench from a ROS message.

        This callback is triggered when a new wrench message is received. It updates
        the current wrench.

        Args:
            msg (WrenchStamped): ROS message containing the current wrench.
        """
        self._current_wrench = self._wrench_msg_to_wrench(msg)

        # Flip the franka convention, so it's the external wrench
        self._current_wrench["force"] = -self._current_wrench["force"]
        self._current_wrench["torque"] = -self._current_wrench["torque"]

        if self._wrench_filter_alpha is not None:
            a = self._wrench_filter_alpha
            if self._current_wrench_filtered is None:
                self._current_wrench_filtered = self._current_wrench.copy()
            else:
                self._current_wrench_filtered = {
                    "force": a * self._current_wrench["force"] + (1.0 - a) * self._current_wrench_filtered["force"],
                    "torque": a * self._current_wrench["torque"] + (1.0 - a) * self._current_wrench_filtered["torque"],
                }

        self._last_wrench_update_time = time.time()
        if self._target_wrench is None:
            self._target_wrench = self._current_wrench.copy()

    def _callback_robot_state(self, msg: FrankaRobotState) -> None:
        efforts = msg.tau_ext_hat_filtered.effort
        n = self.nq
        if len(efforts) >= n:
            self._tau_ext_current = np.array(efforts[:n], dtype=float)

    def _callback_current_pose(self, msg: PoseStamped):
        """Update the current pose from a ROS message.

        This callback is triggered when a new pose message is received. It updates
        the current pose and initializes the target pose if not already set.

        Args:
            msg (PoseStamped): ROS message containing the current pose.
        """
        self._current_pose = self._pose_msg_to_pose(msg)
        self._last_pose_update_time = time.time()
        if self._target_pose is None:
            self._target_pose = self._current_pose.copy()

    def _callback_current_twist(self, msg: TwistStamped):
        """Update the current end-effector twist from a ROS message.

        Args:
            msg (TwistStamped): ROS message containing end-effector twist.
        """
        self._current_twist = self._twist_msg_to_twist(msg)
        self._last_twist_update_time = time.time()
        if self._target_twist is None:
            self._target_twist = self._current_twist.copy()

    def _callback_current_joint(self, msg: JointState):
        """Update the current joint state (position, velocity and torque) from a ROS message.

        This callback filters the joint state message to only include joints
        that are part of this robot's configuration.

        Args:
            msg (JointState): ROS message containing joint states.
        """
        if self._q_current is None:
            self._q_current = np.zeros(self.nq)

        if self._dq_current is None:
            self._dq_current = np.zeros(self.nq)

        if self._tau_current is None:
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

        self._last_joint_update_time = time.time()

    def get_state_update_times(self) -> dict[str, float | None]:
        """Return last update timestamps (unix seconds) for key robot streams."""
        return {
            "pose": self._last_pose_update_time,
            "joint": self._last_joint_update_time,
            "twist": self._last_twist_update_time,
            "wrench": self._last_wrench_update_time,
        }

    # =======================
    # MARK: Utilities
    # =======================

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

    def _twist_msg_to_twist(self, msg: TwistStamped) -> Twist:
        """Convert a ROS2 twist msg to a twist."""
        linear = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
        angular = np.array([msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z])
        return Twist(linear=linear, angular=angular)

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

    # =======================
    # MARK: Control
    # =======================

    # ---- Joint Level
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

    def plan_joint_trajectory(
        self,
        waypoints: Sequence[CartesianWaypoint],
        duration: float,
        visualize: bool = False,
        n_points: int | None = None,
        show_progress: bool = True,
        initial_joint_config: NDArray | None = None,
    ) -> PlannedJointTrajectory:
        """Plan a joint trajectory from Cartesian waypoints using FR3 IK.

        Args:
            waypoints: Cartesian path waypoints.
            duration: Trajectory duration in seconds.
            visualize: If True, open preview and require approval.
            n_points: Number of dense IK solve points.
            show_progress: If True, print planner progress bar.
            initial_joint_config: Optional seed for the first IK point.
                If None, uses the current measured robot joint state.
        """
        joint_traj = plan_fr3_joint_trajectory(
            waypoints=list(waypoints),
            duration=duration,
            joint_names=self.config.joint_names,
            target_link_name=self.config.ik_target_link_name,
            n_points=self.config.ik_default_num_points if n_points is None else n_points,
            current_joint_config=self.q
            if initial_joint_config is None
            else np.array(initial_joint_config, dtype=float),
            pos_weight=self.config.ik_position_weight,
            ori_weight=self.config.ik_orientation_weight,
            similarity_weight=self.config.ik_similarity_weight,
            show_progress=show_progress,
        )

        if visualize:
            approved = visualize_planned_joint_trajectory(joint_traj)
            if not approved:
                raise RuntimeError("Trajectory rejected by user.")

        return joint_traj

    def plan_joint_trajectory_sequence(
        self,
        waypoints_list: Sequence[Sequence[CartesianWaypoint]],
        durations: Sequence[float],
        n_points: int | None = None,
        show_progress: bool = True,
    ) -> list[PlannedJointTrajectory]:
        """Plan multiple trajectories with continuous joints.

        Each segment uses the previous segment's final joint configuration as the
        IK initial config for the next one to prevent joint discontinuities.
        """
        if len(waypoints_list) != len(durations):
            raise ValueError("waypoints_list and durations must have the same length")
        if len(waypoints_list) == 0:
            return []

        trajectories: list[PlannedJointTrajectory] = []
        start_joint_cfg = self.q

        for waypoints, duration in zip(waypoints_list, durations):
            traj = self.plan_joint_trajectory(
                waypoints=waypoints,
                duration=duration,
                visualize=False,
                n_points=n_points,
                show_progress=show_progress,
                initial_joint_config=start_joint_cfg,
            )
            trajectories.append(traj)
            start_joint_cfg = traj.joint_positions[-1]

        return trajectories

    def follow_joint_trajectory(
        self,
        trajectory: PlannedJointTrajectory,
        blocking: bool = True,
        settle_time: float = 0.0,
    ):
        """Execute a planned joint trajectory.

        Args:
            trajectory: Planned trajectory to execute.
            blocking: If True, wait for action completion.
            settle_time: Hold time added at start and end in the sent trajectory message.
        """
        active_controller = self.controller_switcher_client.get_active_controller()
        if active_controller != "joint_trajectory_controller":
            self.controller_switcher_client.switch_controller("joint_trajectory_controller")

        trajectory_to_send = self._add_settle_time(trajectory, settle_time) if settle_time > 0.0 else trajectory

        self.joint_trajectory_controller_client.send_joint_trajectory(
            joint_trajectory=trajectory_to_send,
            blocking=blocking,
        )

        if len(trajectory.joint_positions) > 0:
            self._q_target = np.array(trajectory.joint_positions[-1], dtype=float)

    def send_joint_trajectory(
        self,
        trajectory: PlannedJointTrajectory,
    ):
        """Stream a joint trajectory to the controller directly via topic for smooth online execution.

        Args:
            trajectory: Planned trajectory sequence.
        """
        active_controller = self.controller_switcher_client.get_active_controller()
        if active_controller != "joint_trajectory_controller":
            self.controller_switcher_client.switch_controller("joint_trajectory_controller")

        self.joint_trajectory_controller_client.publish_joint_trajectory(
            joint_trajectory=trajectory,
        )

        if len(trajectory.joint_positions) > 0:
            self._q_target = np.array(trajectory.joint_positions[-1], dtype=float)

    def visualize_sequence(
        self,
        steps: Sequence[Any],
        playback_hz: float = 10.0,
        respect_timing: bool = True,
    ) -> bool:
        """Visualize the full joint trajectory sequence before execution.

        This merges all `PlannedJointTrajectory` steps into one continuous trajectory
        and opens a single viewer session for approval.

        Args:
            steps: Sequence containing `PlannedJointTrajectory` and/or callables.
            playback_hz: Playback frequency for visualization.
            respect_timing: If True, preview playback follows trajectory timestamps.

        Returns:
            bool: True if approved, False if rejected.
        """
        full_trajectory = self._build_full_sequence_trajectory(steps)
        return visualize_planned_joint_trajectory(
            full_trajectory,
            playback_hz=playback_hz,
            respect_timing=respect_timing,
        )

    def online_planning(
        self,
        target_position: NDArray,
        target_orientation: NDArray | Rotation,
        trajectory_length: int = 5,
        dt: float = 0.1,
    ) -> NDArray:
        """Continuously solve inverse kinematics online for active teleoperation.

        Args:
            target_position: Desktop/Target space coordinate.
            target_orientation: Target spatial orientation (scipy.spatial.transform.Rotation or wxyz quat array).
            trajectory_length: Number of time steps to look ahead.
            dt: Time step duration.

        Returns:
            NDArray: Joint configuration directly preceding the planner output.
        """

        if not hasattr(self, "_online_planning_sols"):
            # Initialize with copies of current joint configuration
            q_current = np.array(self.q, dtype=float)
            self._online_planning_sols = np.array([q_current.copy() for _ in range(trajectory_length)])

        qk_w = (
            target_orientation.as_quat()[[3, 0, 1, 2]]
            if isinstance(target_orientation, Rotation)
            else target_orientation
        )
        target_w = np.array(qk_w, dtype=float)
        target_p = np.array(target_position, dtype=float)
        start_cfg = np.array(self.q, dtype=float)

        sol_traj, _, _ = solve_online_planning(
            robot=self.pyroki_robot,
            target_link_name=self.config.ik_target_link_name,
            target_position=target_p,
            target_wxyz=target_w,
            timesteps=trajectory_length,
            dt=dt,
            start_cfg=start_cfg,
            prev_sols=self._online_planning_sols,
        )

        # Retain trajectories for the next sequential warmup (may include finger joints internally)
        self._online_planning_sols = sol_traj

        # Return only the joints requested by the arm configuration
        arm_joint_count = len(self.config.joint_names)
        return self._online_planning_sols[0, :arm_joint_count].copy()

    def build_online_planning_step_trajectory(
        self,
        target_position: NDArray,
        target_orientation: NDArray | Rotation,
        trajectory_length: int = 5,
        dt: float = 0.1,
    ) -> PlannedJointTrajectory:
        """Build a single-step `PlannedJointTrajectory` from online planning output.

        This method provides a public API for teleoperation loops to obtain a
        smooth immediate command point without depending on private attributes.
        """
        self.online_planning(
            target_position=target_position,
            target_orientation=target_orientation,
            trajectory_length=trajectory_length,
            dt=dt,
        )

        if not hasattr(self, "_online_planning_sols"):
            raise RuntimeError("Online planning solutions are unavailable.")

        arm_joint_count = len(self.config.joint_names)
        horizon_positions = self._online_planning_sols[:, :arm_joint_count]
        horizon_velocities = np.gradient(horizon_positions, dt, axis=0)
        horizon_accelerations = np.gradient(horizon_velocities, dt, axis=0)

        return PlannedJointTrajectory(
            joint_names=self.config.joint_names,
            time_from_start=[dt],
            joint_positions=horizon_positions[:1],
            joint_velocities=horizon_velocities[:1],
            joint_accelerations=horizon_accelerations[:1],
        )

    def execute_sequence(
        self,
        steps: Sequence[Any],
        stop_on_error: bool = True,
        visualize_before_execution: bool = False,
        playback_hz: float = 10.0,
        respect_timing_in_preview: bool = True,
        settle_time_between_trajectories: float | None = None,
    ):
        """Execute an experiment sequence of trajectories and callables.

        Supported step types:
            - PlannedJointTrajectory
            - callable (e.g. gripper actions)

        Args:
            steps: Ordered sequence to execute.
            stop_on_error: If True, re-raise exceptions and stop execution.
            visualize_before_execution: If True, preview full concatenated trajectory first.
            playback_hz: Playback rate used when `visualize_before_execution=True`.
            respect_timing_in_preview: If True, preview follows trajectory timestamps.
            settle_time_between_trajectories: Optional wait time in seconds between
                consecutive trajectory steps. If None, uses config default.
        """
        if visualize_before_execution:
            approved = self.visualize_sequence(
                steps,
                playback_hz=playback_hz,
                respect_timing=respect_timing_in_preview,
            )
            if not approved:
                raise RuntimeError("Sequence trajectory rejected by user.")

        settle_time = (
            self.config.trajectory_settle_time
            if settle_time_between_trajectories is None
            else settle_time_between_trajectories
        )

        for step in steps:
            try:
                if isinstance(step, PlannedJointTrajectory):
                    self.follow_joint_trajectory(step, blocking=True, settle_time=settle_time)
                elif callable(step):
                    step()
                else:
                    raise TypeError(f"Unsupported sequence step type: {type(step).__name__}")
            except Exception:
                if stop_on_error:
                    raise

    def _add_settle_time(self, trajectory: PlannedJointTrajectory, settle_time: float) -> PlannedJointTrajectory:
        """Add start/end hold points directly in the trajectory message timing.

        This follows the sequence requested for each trajectory:
          1) shift all times by `settle_time`
          2) duplicate first point at `settle_time`
          3) duplicate last point at `duration + 2 * settle_time`
        """
        if settle_time <= 0.0:
            return trajectory
        if len(trajectory.time_from_start) == 0 or len(trajectory.joint_positions) == 0:
            raise ValueError("Trajectory is empty")

        original_times = [float(t) for t in trajectory.time_from_start]
        if any(t2 <= t1 for t1, t2 in zip(original_times, original_times[1:])):
            raise ValueError("Trajectory times must be strictly increasing")

        shifted_times = [t + settle_time for t in original_times]
        first_q = np.array(trajectory.joint_positions[0], dtype=float)
        last_q = np.array(trajectory.joint_positions[-1], dtype=float)
        first_dq = np.array(trajectory.joint_velocities[0], dtype=float)
        last_dq = np.array(trajectory.joint_velocities[-1], dtype=float)
        first_ddq = np.array(trajectory.joint_accelerations[0], dtype=float)
        last_ddq = np.array(trajectory.joint_accelerations[-1], dtype=float)

        # Requested shape:
        #   - duplicate first point at t_settle
        #   - shift all message times by t_settle
        #   - duplicate last point at duration + 2*t_settle
        # If original trajectory starts at t=0, the shifted first point equals t_settle.
        # To keep strictly increasing timestamps for ros2_control, drop that duplicated shifted point.
        eps = 1e-6
        # include_shifted_first = shifted_times[0] > (settle_time + eps)

        new_times = [original_times[0]]
        new_positions: list[np.ndarray] = [first_q]
        new_vels: list[np.ndarray] = [np.zeros_like(first_dq)]
        new_accels: list[np.ndarray] = [np.zeros_like(first_ddq)]

        # if include_shifted_first:
        #     new_times.append(shifted_times[0])
        #     new_positions.append(np.array(trajectory.joint_positions[0], dtype=float))
        #     new_vels.append(np.array(trajectory.joint_velocities[0], dtype=float))
        #     new_accels.append(np.array(trajectory.joint_accelerations[0], dtype=float))

        new_times.extend(shifted_times)

        new_positions.extend(trajectory.joint_positions)
        new_vels.extend(trajectory.joint_velocities)
        new_accels.extend(trajectory.joint_accelerations)

        # Add last time
        new_times.append(new_times[-1] + settle_time)
        new_positions.append(last_q)
        new_vels.append(np.zeros_like(last_dq))
        new_accels.append(np.zeros_like(last_ddq))

        # Final guard against floating-point equality at segment boundaries.
        for i in range(1, len(new_times)):
            if new_times[i] <= new_times[i - 1]:
                new_times[i] = new_times[i - 1] + eps

        return PlannedJointTrajectory(
            joint_names=trajectory.joint_names,
            time_from_start=new_times,
            joint_positions=np.array(new_positions),
            joint_velocities=np.array(new_vels),
            joint_accelerations=np.array(new_accels),
        )

    def _build_full_sequence_trajectory(self, steps: Sequence[Any]) -> PlannedJointTrajectory:
        """Build one continuous trajectory from all trajectory steps in a sequence."""
        traj_steps: list[PlannedJointTrajectory] = [step for step in steps if isinstance(step, PlannedJointTrajectory)]

        if len(traj_steps) == 0:
            raise ValueError("No PlannedJointTrajectory found in sequence.")

        joint_names = traj_steps[0].joint_names
        all_times: list[float] = []
        all_positions: list[np.ndarray] = []
        all_velocities: list[np.ndarray] = []
        all_accelerations: list[np.ndarray] = []

        time_offset = 0.0

        for i, trajectory in enumerate(traj_steps):
            if trajectory.joint_names != joint_names:
                raise ValueError("All trajectory steps must share the same joint_names.")

            if len(trajectory.time_from_start) != len(trajectory.joint_positions):
                raise ValueError("Trajectory time_from_start and joint_positions length mismatch.")

            for j, (t, q, q_dot, q_ddot) in enumerate(
                zip(
                    trajectory.time_from_start,
                    trajectory.joint_positions,
                    trajectory.joint_velocities,
                    trajectory.joint_accelerations,
                )
            ):
                # Avoid duplicate timestamp at segment boundaries.
                if i > 0 and j == 0:
                    continue
                all_times.append(float(t) + time_offset)
                all_positions.append(np.array(q, dtype=float))
                all_velocities.append(np.array(q_dot, dtype=float))
                all_accelerations.append(np.array(q_ddot, dtype=float))

            if len(trajectory.time_from_start) > 0:
                time_offset += float(trajectory.time_from_start[-1])

        return PlannedJointTrajectory(
            joint_names=joint_names,
            time_from_start=all_times,
            joint_positions=np.array(all_positions),
            joint_velocities=np.array(all_velocities),
            joint_accelerations=np.array(all_accelerations),
        )

    # ----- Cartesian
    def move_to(
        self,
        position: List | NDArray | None = None,
        pose: Pose | None = None,
        speed: float = 0.05,
        time_to_move: float | None = None,
    ):
        """Move the end-effector to a target pose.

        Dispatches behavior based on the active controller:
        - Joint controller: plan IK trajectory and execute through joint trajectory action.
        - Other controllers: keep Cartesian interpolation/publishing behavior.

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

        if time_to_move is None:
            time_to_move = float(distance / speed)

        # print(f"[debug] Moving to target pose {desired_pose} with time_to_move: {time_to_move} sec")

        active_controller = self.controller_switcher_client.get_active_controller()
        if active_controller is not None and self._is_joint_controller(active_controller):
            waypoints = generate_linear_waypoints(
                start_position=start_pose.position,
                start_orientation=start_pose.orientation,
                end_position=desired_pose.position,
                end_orientation=desired_pose.orientation,
                num_waypoints=2,
            )
            trajectory = self.plan_joint_trajectory(waypoints, duration=time_to_move, visualize=False)
            self.follow_joint_trajectory(trajectory, blocking=True)
            self._target_pose = desired_pose.copy()
            return

        N = int(time_to_move * self.config.publish_frequency)

        rate = self.node.create_rate(self.config.publish_frequency)

        slerp = Slerp(
            [0, 1], Rotation.from_quat([start_pose.orientation.as_quat(), desired_pose.orientation.as_quat()])
        )

        for t in np.linspace(0.0, 1.0, N):
            pos = (1 - t) * start_pose.position + t * desired_pose.position
            ori = slerp([t])[0]
            next_pose = Pose(pos, ori)
            self._target_pose = next_pose
            rate.sleep()

        self._target_pose = desired_pose

    def execute_cartesian_traj(
        self,
        waypoints: List[tuple[Pose, Twist]],
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

            point = self._pose_to_pose_msg(pose)
            msg.points.append(point)  # msg.points = [*msg.points, point]

            # Convert float time to Duration
            duration = Duration()
            duration.sec = int(time_sec)
            duration.nanosec = int((time_sec % 1.0) * 1e9)
            msg.time_from_start.append(duration)  # msg.time_from_start = [*msg.time_from_start, duration]

        # Set velocity limits
        msg.max_linear_velocity = max_linear_velocity
        msg.max_angular_velocity = max_angular_velocity

        # Enable trajectory mode to stop continuous pose publishing
        self._trajectory_mode_active = True

        time.sleep(0.5)  # Small delay to ensure mode switch before publishing

        # Publish trajectory
        self._target_trajectory_publisher.publish(msg)

        self.node.get_logger().debug(
            f"Sent trajectory with {len(waypoints)} waypoints, total duration: {time_from_start[-1]:.3f}s"
        )

    def wait_for_trajectory_completion(self, expected_duration: float, timeout_margin: float = 2.0):
        """Wait for trajectory execution to complete while allowing state reading.

        This method can be used in a while loop to read robot state during trajectory execution:

        Example:
            >>> while robot.wait_for_trajectory_completion(duration):
            >>>     ee_poses.append(robot.end_effector_pose.copy())
            >>>     ts.append(time.time())

        Args:
            expected_duration: Expected trajectory duration in seconds
            timeout_margin: Additional time to wait beyond expected duration (seconds)

        Returns:
            bool: True if trajectory is still executing, False when complete

        Note:
            This is a simple time-based wait. For more precise tracking, consider
            converting to a ROS2 action interface in the future.
        """
        if not hasattr(self, "_trajectory_start_time"):
            self._trajectory_start_time = self.node.get_clock().now().nanoseconds / 1e9
            self._trajectory_timeout = expected_duration + timeout_margin

        elapsed = self.node.get_clock().now().nanoseconds / 1e9 - self._trajectory_start_time

        if elapsed >= self._trajectory_timeout:
            # Re-enable pose publishing and clean up
            self._trajectory_mode_active = False
            delattr(self, "_trajectory_start_time")
            delattr(self, "_trajectory_timeout")
            self.node.get_logger().debug("Trajectory execution completed")
            return False

        # Sleep briefly to control loop rate
        # self._rate.sleep()
        return True
