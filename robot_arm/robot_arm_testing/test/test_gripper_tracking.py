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
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters

import launch_pytest
import datetime
from pathlib import Path

import matplotlib.pyplot as plt
import yaml
import numpy as np
import os

"""
Test of gripper tracking accuracy.

To run, make sure the interface is running on the Franka PC. 

Plots are saved in the /plot directory.s
"""


def set_ros2_parameter(node: Node, target_node_name: str, param: str, value):
    """
    Update a parameter on another ROS2 node.

    Args:
        node: The rclpy Node instance (the caller).
        target_node_name: The name of the target node (e.g., '/fr3_interface').
        param: The name of the parameter (str).
        value: The new value for the parameter.
    Returns:
        True if successful, False otherwise.
    """

    # client = node.create_client(
    #     srv_type=node._parameters_client.srv_type, srv_name=f"{target_node_name}/set_parameters"
    # )
    # if not client.wait_for_service(timeout_sec=2.0):
    #     node.get_logger().error(f"Service {target_node_name}/set_parameters not available.")
    #     return False

    # req = client.srv_type.Request()
    # req.parameters = [Parameter(param, Parameter.Type.DOUBLE, value).to_parameter_msg()]

    # future = client.call_async(req)
    # rclpy.spin_until_future_complete(node, future)
    # result = future.result()

    # return result.results[0].successful if result and result.results else False

    client = node.create_client(SetParameters, f"{target_node_name}/set_parameters")
    if not client.wait_for_service(timeout_sec=2.0):
        node.get_logger().error(f"Service {target_node_name}/set_parameters not available.")
        return False

    param_msg = Parameter(param, Parameter.Type.DOUBLE, value).to_parameter_msg()
    req = SetParameters.Request()
    req.parameters = [param_msg]

    future = client.call_async(req)
    import rclpy

    rclpy.spin_until_future_complete(node, future)
    result = future.result()
    return result.results[0].successful if result and result.results else False


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

    def _run_trajectory_test(self, desired_pose_generator, duration, rate, start_pose):
        dt = 1.0 / rate
        n_steps = int(duration * rate)
        errors = []
        desired_xs, desired_ys, desired_zs = [], [], []
        actual_xs, actual_ys, actual_zs = [], [], []
        errors_x, errors_y, errors_z = [], [], []
        for i in range(n_steps):
            target_x, target_y, target_z = desired_pose_generator(i, start_pose)
            pose = PoseStamped()
            pose.header = Header()
            pose.header.stamp = self.node.get_clock().now().to_msg()
            pose.header.frame_id = start_pose.header.frame_id
            pose.pose.position.x = target_x
            pose.pose.position.y = target_y
            pose.pose.position.z = target_z
            pose.pose.orientation = start_pose.pose.orientation
            self.cmd_pub.publish(pose)
            desired_xs.append(target_x)
            desired_ys.append(target_y)
            desired_zs.append(target_z)
            rclpy.spin_once(self.node, timeout_sec=dt)

            with self.pose_lock:
                actual_pose = self.current_pose

            actual_xs.append(actual_pose.pose.position.x)
            actual_ys.append(actual_pose.pose.position.y)
            actual_zs.append(actual_pose.pose.position.z)
            errors_x.append(actual_pose.pose.position.x - target_x)
            errors_y.append(actual_pose.pose.position.y - target_y)
            errors_z.append(actual_pose.pose.position.z - target_z)
            err = math.sqrt(
                (actual_pose.pose.position.x - target_x) ** 2 + (actual_pose.pose.position.y - target_y) ** 2
            )
            errors.append(err)
            time.sleep(dt)

        return {
            "desired_xs": desired_xs,
            "desired_ys": desired_ys,
            "desired_zs": desired_zs,
            "actual_xs": actual_xs,
            "actual_ys": actual_ys,
            "actual_zs": actual_zs,
            "errors_x": errors_x,
            "errors_y": errors_y,
            "errors_z": errors_z,
            "errors": errors,
        }

    def _plot_and_save_results(self, results, axis_labels, plot_prefix, Kp, Kd):
        desired_xs = results["desired_xs"]
        desired_ys = results["desired_ys"]
        desired_zs = results["desired_zs"]
        actual_xs = results["actual_xs"]
        actual_ys = results["actual_ys"]
        actual_zs = results["actual_zs"]
        errors_x = results["errors_x"]
        errors_y = results["errors_y"]
        errors_z = results["errors_z"]
        errors = results["errors"]
        mean_err = sum(errors) / len(errors)
        max_err = max(errors)
        try:
            timestamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
            # Directory structure: results/<trajectory>/<timestamp>/
            result_dir = Path(__file__).parent / "results" / plot_prefix / timestamp
            result_dir.mkdir(parents=True, exist_ok=True)
            # XY plot
            plt.figure()
            plt.plot(desired_xs, desired_ys, label="Desired Trajectory", linestyle="--")
            plt.plot(actual_xs, actual_ys, label="Actual Trajectory", marker="o", markersize=2)
            plt.xlabel(axis_labels[0])
            plt.ylabel(axis_labels[1])
            plt.title(f"{plot_prefix} Trajectory Tracking (XY)")
            plt.legend()
            plt.axis("equal")
            plt.tight_layout()
            path_xy = result_dir / f"{plot_prefix}_xy.png"
            plt.savefig(path_xy)
            print(f"Trajectory XY plot saved to {path_xy}")
            # X
            plt.figure()
            plt.plot(desired_xs, label="Desired X")
            plt.plot(actual_xs, label="Actual X")
            plt.xlabel("Step")
            plt.ylabel("X [m]")
            plt.title(f"{plot_prefix} X Position Tracking")
            plt.legend()
            plt.tight_layout()
            path_x = result_dir / f"{plot_prefix}_x.png"
            plt.savefig(path_x)
            print(f"X position plot saved to {path_x}")
            # Y
            plt.figure()
            plt.plot(desired_ys, label="Desired Y")
            plt.plot(actual_ys, label="Actual Y")
            plt.xlabel("Step")
            plt.ylabel("Y [m]")
            plt.title(f"{plot_prefix} Y Position Tracking")
            plt.legend()
            plt.tight_layout()
            path_y = result_dir / f"{plot_prefix}_y.png"
            plt.savefig(path_y)
            print(f"Y position plot saved to {path_y}")
            # Z
            plt.figure()
            plt.plot(desired_zs, label="Desired Z")
            plt.plot(actual_zs, label="Actual Z")
            plt.xlabel("Step")
            plt.ylabel("Z [m]")
            plt.title(f"{plot_prefix} Z Position Tracking")
            plt.legend()
            plt.tight_layout()
            path_z = result_dir / f"{plot_prefix}_z.png"
            plt.savefig(path_z)
            print(f"Z position plot saved to {path_z}")
            # Error
            plt.figure()
            plt.plot(errors_x, label="Error X")
            plt.plot(errors_y, label="Error Y")
            plt.plot(errors_z, label="Error Z")
            plt.xlabel("Step")
            plt.ylabel("Error [m]")
            plt.title(f"{plot_prefix} Tracking Error (X, Y, Z)")
            plt.legend()
            plt.tight_layout()
            path_err = result_dir / f"{plot_prefix}_error.png"
            plt.savefig(path_err)
            print(f"Error plot saved to {path_err}")
            # YAML summary
            summary = {
                "Kp_gripper_trans": Kp,
                "Kd_gripper_trans": Kd,
                "mean_tracking_error": float(mean_err),
                "max_tracking_error": float(max_err),
            }
            summary_path = result_dir / f"{plot_prefix}_summary.yaml"
            with open(summary_path, "w") as f:
                yaml.dump(summary, f)
            print(f"Summary YAML saved to {summary_path}")
        except ImportError:
            print("matplotlib, numpy, or yaml not installed, skipping plots and summary.")
        print(f"Mean tracking error: {mean_err:.4f} m, Max error: {max_err:.4f} m")
        assert mean_err < 0.02, f"Mean tracking error too high: {mean_err}"
        assert max_err < 0.04, f"Max tracking error too high: {max_err}"

    def test_gripper_tracks_circular_trajectory(self):
        # Set gains
        Kp = 7.0
        Kd = 0.6
        success = set_ros2_parameter(self.node, "/fr3_interface", "Kp_gripper_trans", float(Kp))
        assert success, f"Failed to set Kp_gripper_trans to {Kp}"
        success = set_ros2_parameter(self.node, "/fr3_interface", "Kd_gripper_trans", float(Kd))
        assert success, f"Failed to set Kd_gripper_trans to {Kd}"
        with self.pose_lock:
            start_pose = self.current_pose
        assert start_pose is not None
        center_x = start_pose.pose.position.xself.node
        center_y = start_pose.pose.position.y
        z = start_pose.pose.position.z
        radius = 0.075
        duration = 5.0
        rate = 50
        # Circle center so start is on circle
        circle_center_x = center_x - radius
        circle_center_y = center_y

        def circle_gen(i, pose):
            theta = 2 * math.pi * (i / int(duration * rate))
            return (circle_center_x + radius * math.cos(theta), circle_center_y + radius * math.sin(theta), z)

        results = self._run_trajectory_test(circle_gen, duration, rate, start_pose)
        self._plot_and_save_results(results, ("X [m]", "Y [m]"), "circular", Kp, Kd)

    def test_gripper_tracks_step_x_trajectory(self):
        #! Set gains
        Kp = 7
        Kd = 0.6

        success = set_ros2_parameter(self.node, "/fr3_interface", "Kp_gripper_trans", float(Kp))
        assert success, f"Failed to set Kp_gripper_trans to {Kp}"
        success = set_ros2_parameter(self.node, "/fr3_interface", "Kd_gripper_trans", float(Kd))
        assert success, f"Failed to set Kd_gripper_trans to {Kd}"
        with self.pose_lock:
            start_pose = self.current_pose
        assert start_pose is not None
        x0 = start_pose.pose.position.x
        y0 = start_pose.pose.position.y
        z0 = start_pose.pose.position.z

        #! Parameters
        rate = 20
        dx = 0.0225
        speed = rate * dx
        print(f"Desired EE Vel: {speed:.2f} m/s")

        step_time = 0.5
        hold_time = 2.0

        #! Phase 1: increase X
        duration1 = step_time

        def phase1_gen(i, pose):
            return (x0 + dx * (i + 1), y0, z0)

        results1 = self._run_trajectory_test(phase1_gen, duration1, rate, start_pose)
        #! Phase 2: hold
        duration2 = hold_time

        def phase2_hold_gen(i, pose):
            return (x0 + dx * int(duration1 * rate), y0, z0)

        results2 = self._run_trajectory_test(phase2_hold_gen, duration2, rate, start_pose)

        #! Phase 3: decrease X
        duration3 = step_time

        def phase3_gen(i, pose):
            return (x0 + dx * int(duration1 * rate) - dx * (i + 1), y0, z0)

        results3 = self._run_trajectory_test(phase3_gen, duration3, rate, start_pose)

        #! Phase 4: hold
        duration4 = hold_time

        def phase4_hold_gen(i, pose):
            return (x0, y0, z0)

        results4 = self._run_trajectory_test(phase4_hold_gen, duration4, rate, start_pose)

        #! Concatenate all results
        def concat_lists(*args):
            out = []
            for arr in args:
                out.extend(arr)
            return out

        results = {
            "desired_xs": concat_lists(
                results1["desired_xs"], results2["desired_xs"], results3["desired_xs"], results4["desired_xs"]
            ),
            "desired_ys": concat_lists(
                results1["desired_ys"], results2["desired_ys"], results3["desired_ys"], results4["desired_ys"]
            ),
            "desired_zs": concat_lists(
                results1["desired_zs"], results2["desired_zs"], results3["desired_zs"], results4["desired_zs"]
            ),
            "actual_xs": concat_lists(
                results1["actual_xs"], results2["actual_xs"], results3["actual_xs"], results4["actual_xs"]
            ),
            "actual_ys": concat_lists(
                results1["actual_ys"], results2["actual_ys"], results3["actual_ys"], results4["actual_ys"]
            ),
            "actual_zs": concat_lists(
                results1["actual_zs"], results2["actual_zs"], results3["actual_zs"], results4["actual_zs"]
            ),
            "errors_x": concat_lists(
                results1["errors_x"], results2["errors_x"], results3["errors_x"], results4["errors_x"]
            ),
            "errors_y": concat_lists(
                results1["errors_y"], results2["errors_y"], results3["errors_y"], results4["errors_y"]
            ),
            "errors_z": concat_lists(
                results1["errors_z"], results2["errors_z"], results3["errors_z"], results4["errors_z"]
            ),
            "errors": concat_lists(results1["errors"], results2["errors"], results3["errors"], results4["errors"]),
        }
        self._plot_and_save_results(results, ("X [m]", "Y [m]"), "step_x", Kp, Kd)
