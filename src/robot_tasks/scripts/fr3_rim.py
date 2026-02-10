import rclpy
from rclpy.node import Node

from robot_arm_client.FrankaArm import FrankaArm
from robot_arm_client.teleop_node import TeleopNode
from rclpy.duration import Duration
from rclpy.callback_groups import ReentrantCallbackGroup  # Recommended for Action Server

import argparse
from rclpy.action import ActionClient
from arm_interfaces.action import PegInHole, RlAgent
from arm_interfaces.srv import SetControlMode, GetControlMode, SetGoalSource, GetGoalSource
from robot_arm_interface.fr3_interface import GoalSource, ControlMode

import numpy as np
from numpy import pi as PI

import time
import pinocchio as pin
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import pandas as pd
from datetime import datetime
import signal

# Experiment parameters
START_POSE = np.array([0.4253, 0.0, 0.3])


def _set_robot_mode_async(node: Node, control_mode: ControlMode, goal_source: GoalSource):
    """Helper to asynchronously set control mode and goal source."""
    node.get_logger().info("Setting robot control mode and goal source...")

    node.get_logger().info(
        f"Connecting to services: '/fr3_interface/set_goal_source' and '/fr3_interface/set_control_mode'..."
    )

    fr3_int_set_goal_src = node.create_client(
        SetGoalSource, "/fr3_interface/set_goal_source", callback_group=ReentrantCallbackGroup()
    )
    fr3_int_set_ctrl_mode = node.create_client(
        SetControlMode, "/fr3_interface/set_control_mode", callback_group=ReentrantCallbackGroup()
    )

    # Check if the control mode service is available (non-blocking)
    while not fr3_int_set_goal_src.wait_for_service(timeout_sec=1.0):
        node.get_logger().warn("Service '/fr3_interface/set_goal_source' not available, waiting...")

    while not fr3_int_set_ctrl_mode.wait_for_service(timeout_sec=1.0):
        node.get_logger().warn("Service '/fr3_interface/set_control_mode' not available, waiting...")

    node.get_logger().info("Connected to services.")

    if not fr3_int_set_ctrl_mode.service_is_ready() or not fr3_int_set_goal_src.service_is_ready():
        node.get_logger().error("Control mode or goal source service not ready.")
        return False

    node.get_logger().info(f"Setting control mode and goal source to '{control_mode}' and '{goal_source}'.")
    ctrl_mode_req = SetControlMode.Request(control_mode=control_mode.value)
    goal_src_req = SetGoalSource.Request(goal_source=goal_source.value)

    try:
        # Call services concurrently
        ctrl_future = fr3_int_set_ctrl_mode.call_async(ctrl_mode_req)
        goal_future = fr3_int_set_goal_src.call_async(goal_src_req)

        # Wait for both futures to complete using rclpy.spin_until_future_complete
        rclpy.spin_until_future_complete(node, ctrl_future)
        rclpy.spin_until_future_complete(node, goal_future)

        ctrl_response = ctrl_future.result()
        goal_response = goal_future.result()

        if ctrl_response.success and goal_response.success:
            node.get_logger().info(f"Successfully set control mode to {control_mode} and goal source to {goal_source}.")
            return True
        else:
            node.get_logger().error(
                f"Failed to set modes: ControlMode success={ctrl_response.success}, GoalSource success={goal_response.success}"
            )
            return False

    except Exception as e:
        node.get_logger().error(f"Exception while setting robot modes: {e}")
        return False


def main(args=None):
    parser = argparse.ArgumentParser(description="FR3 Delay RMI ")
    # parser.add_argument("--demo", action="store_true", help="Run the demo sequence")
    # parser.add_argument("--no-teleop", action="store_true", help="Run without teleop capabilities")
    parsed_args, remaining = parser.parse_known_args()

    rclpy.init(args=remaining)

    print("\n\n----- Starting DelayRIM task -----")
    franka_arm = FrankaArm()

    # Create tf2 broadcasters
    node = rclpy.create_node("fr3_lift_tf_broadcaster")
    # tf_broadcaster = TransformBroadcaster(node)

    # Gripper start pose (G is at the tip of the rigid fingers)
    gripper_start_pose_rpy = np.array([PI, 0, 0])  # Orientation
    p_BGstart = START_POSE  # Position in base frame

    #! Moving to start pose
    # Moving to Start pose
    print("Moving to start pose...")
    # franka_arm.gripper_open()

    X_G_start = pin.SE3(pin.rpy.rpyToMatrix(gripper_start_pose_rpy), p_BGstart)

    franka_arm.goto_pose(X_G_start, Duration(seconds=10.0), Kp=1.0, Kd=0.0)

    # Grab the cube
    X_G_cube = pin.SE3(pin.rpy.rpyToMatrix(gripper_start_pose_rpy), p_BGstart - np.array([0.0, 0.0, 0.08]))
    franka_arm.goto_pose(X_G_cube, Duration(seconds=10.0), Kp=1.0, Kd=0.0)

    franka_arm.gripper_close()

    # Press down cube
    print("Pressing down cube...")
    X_G_cube = pin.SE3(pin.rpy.rpyToMatrix(gripper_start_pose_rpy), p_BGstart - np.array([0.0, 0.0, 0.075]))
    franka_arm.goto_pose(X_G_cube, Duration(seconds=10.0), Kp=1.0, Kd=0.0)

    # --- Control mode
    mode_set_ok = _set_robot_mode_async(node, control_mode=ControlMode.CART_VEL, goal_source=GoalSource.TELEOP)
    if not mode_set_ok:
        print("Failed to set control mode and goal source. Exiting...")
        return

    return
    # Start a trial! Call the insert action
    print("--- Starting Delay RIM Experiment ---")

    # Prepare results DataFrame
    in_action = False  # Flag to indicate if we are currently in an action
    results = []
    experiment_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    out_folder = Path("results")
    out_folder.mkdir(exist_ok=True)
    out_name = Path(f"lift_results_{experiment_time.replace(' ', '_').replace(':', '-')}.csv")
    out_path = out_folder / out_name

    def save_results():
        if results:
            df = pd.DataFrame(results)
            df.to_csv(out_path.as_posix(), index=False)
            print(f"Results saved to {out_path}")
        else:
            print("No results to save.")

    try:
        while True:
            ...

    except KeyboardInterrupt:
        print("\nCtrl+C detected. Saving results before exit...")

    finally:
        save_results()
        # Clean up
        # rclpy.shutdown()


if __name__ == "__main__":
    main()
