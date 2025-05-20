# test/test_gripper_tracking.py
from matplotlib.path import Path
import pytest
import rclpy
import math
import time
import threading
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from robot_arm_interface.fr3_interface import ControlMode, GoalSource
from arm_interfaces.srv import SetControlMode, SetGoalSource

import launch_testing
from launch import LaunchDescription
from launch_ros.actions import Node as LaunchNode
from launch_testing.actions import ReadyToTest

import launch_pytest
import datetime
from pathlib import Path


@launch_pytest.fixture
def generate_test_description():
    # Launch a dummy node to keep the launch alive
    return LaunchDescription(
        [
            LaunchNode(
                package="robot_arm_testing",
                executable="dummy_node",  # You can use any simple node that just spins
                name="dummy_keepalive",
                output="screen",
            ),
            ReadyToTest(),
        ]
    )


class TestGripperTracking:
    @classmethod
    def setup_class(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_gripper_tracking")
        cls.executor = SingleThreadedExecutor()
        cls.executor.add_node(cls.node)
        cls.current_pose = None
        cls.pose_lock = threading.Lock()
        cls.pose_sub = cls.node.create_subscription(
            PoseStamped, "/franka_robot_state_broadcaster/current_pose", cls._pose_callback, 10
        )
        cls.cmd_pub = cls.node.create_publisher(PoseStamped, "/robot_arm/gripper_pose_cmd", 10)
        # Set robot mode and goal source
        cls._set_mode(ControlMode.CART_POSE, GoalSource.TOPIC)
        # Wait for initial pose
        cls._wait_for_pose()

    @classmethod
    def teardown_class(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _pose_callback(cls, msg):
        with cls.pose_lock:
            cls.current_pose = msg

    @classmethod
    def _wait_for_pose(cls, timeout=5.0):
        start = time.time()
        while cls.current_pose is None and (time.time() - start) < timeout:
            rclpy.spin_once(cls.node, timeout_sec=0.1)
        assert cls.current_pose is not None, "Did not receive initial gripper pose."

    @classmethod
    def _set_mode(cls, control_mode, goal_source):
        # Set control mode
        cli = cls.node.create_client(SetControlMode, "/fr3_interface/set_control_mode")
        if not cli.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("SetControlMode service not available")
        req = SetControlMode.Request(control_mode=control_mode.value)
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(cls.node, future)

        # Set goal source
        cli2 = cls.node.create_client(SetGoalSource, "/fr3_interface/set_goal_source")
        if not cli2.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("SetGoalSource service not available")
        req2 = SetGoalSource.Request(goal_source=goal_source.value)
        future2 = cli2.call_async(req2)
        rclpy.spin_until_future_complete(cls.node, future2)

    def test_gripper_tracks_circular_trajectory(self):
        # Get starting pose
        with self.pose_lock:
            start_pose = self.current_pose
        assert start_pose is not None
        center_x = start_pose.pose.position.x
        center_y = start_pose.pose.position.y
        z = start_pose.pose.position.z
        radius = 0.05  # 5cm circle
        duration = 5.0  # seconds
        rate = 50  # Hz
        dt = 1.0 / rate
        n_steps = int(duration * rate)

        errors = []
        desired_xs = []
        desired_ys = []

        actual_xs = []
        actual_ys = []

        # Compute center so that the start position is on the circle (theta=0)
        # Place the center at (x - r, y), so the first target is at the current pose
        circle_center_x = center_x - radius
        circle_center_y = center_y
        for i in range(n_steps):
            theta = 2 * math.pi * (i / n_steps)
            target_x = circle_center_x + radius * math.cos(theta)
            target_y = circle_center_y + radius * math.sin(theta)
            pose = PoseStamped()
            pose.header = Header()
            pose.header.stamp = self.node.get_clock().now().to_msg()
            pose.header.frame_id = start_pose.header.frame_id
            pose.pose.position.x = target_x
            pose.pose.position.y = target_y
            pose.pose.position.z = z
            pose.pose.orientation = start_pose.pose.orientation
            self.cmd_pub.publish(pose)
            desired_xs.append(target_x)
            desired_ys.append(target_y)

            # print(f"Target pose: {target_x:.4f}, {target_y:.4f}, {z:.4f}")

            # Wait for robot to move
            rclpy.spin_once(self.node, timeout_sec=dt)
            with self.pose_lock:
                actual_pose = self.current_pose
            actual_xs.append(actual_pose.pose.position.x)
            actual_ys.append(actual_pose.pose.position.y)
            err = math.sqrt(
                (actual_pose.pose.position.x - target_x) ** 2 + (actual_pose.pose.position.y - target_y) ** 2
            )
            errors.append(err)
            time.sleep(dt)

        mean_err = sum(errors) / len(errors)
        max_err = max(errors)

        # Plot desired vs actual trajectory
        try:
            import matplotlib.pyplot as plt

            plt.figure()
            plt.plot(desired_xs, desired_ys, label="Desired Trajectory", linestyle="--")
            plt.plot(actual_xs, actual_ys, label="Actual Trajectory", marker="o", markersize=2)
            plt.xlabel("X [m]")
            plt.ylabel("Y [m]")
            plt.title("Gripper Trajectory Tracking")
            plt.legend()
            plt.axis("equal")
            plt.tight_layout()

            timestamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
            path = Path(__file__).parent / "plots" / f"gripper_tracking_result_{timestamp}.png"
            plt.savefig(path)
            print(f"Trajectory plot saved to {path}")

        except ImportError:
            print("matplotlib not installed, skipping plot.")

        print(f"Mean tracking error: {mean_err:.4f} m, Max error: {max_err:.4f} m")
        assert mean_err < 0.02, f"Mean tracking error too high: {mean_err}"
        assert max_err < 0.04, f"Max tracking error too high: {max_err}"
