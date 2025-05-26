import rclpy
from robot_arm_client.FrankaArm import FrankaArm
from robot_arm_client.teleop_node import TeleopNode
from rclpy.duration import Duration
import argparse
from rclpy.action import ActionClient
from arm_interfaces.action import PegInHole, RlAgent

import numpy as np
from numpy import pi as PI

import time
import pinocchio as pin
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

PEG_IN_HAND = True
cube_start_pose = np.array([0.56, 0.0, 0.050])
cube_start_rpy = np.array([0, 0.0, 0.0])
X_BC = pin.SE3(pin.rpy.rpyToMatrix(cube_start_rpy), cube_start_pose)

p_WBsim = np.array([-0.56, 0.0, 0.912])  # World frame position
p_BsimB = np.array([0.0, 0.0, -0.117])  # World frame position

rpy_WB = np.array([0.0, 0.0, 0.0])  # World frame to base frame rpy angles

X_WBsim = pin.SE3(pin.rpy.rpyToMatrix(rpy_WB), p_WBsim)  # World to base transform
X_BsimB = pin.SE3(pin.rpy.rpyToMatrix(rpy_WB), p_BsimB)  # Base to base simulation transform
X_WB = X_WBsim * X_BsimB  # World to base transform in simulation

p_WGstart_W = np.array([0.0, 0.0, 1.01])  # World frame position of gripper start pose
p_BGstart_B = X_WB.inverse() * p_WGstart_W  # Base frame position of gripper start pose


def call_lift_action(node):
    """Send the insert action goal and wait for the result."""
    # Create action client
    action_client = ActionClient(node, RlAgent, "/lift_action")

    # Wait for action server
    print("Waiting for insert action server...")
    if not action_client.wait_for_server(timeout_sec=10.0):
        print("Action server not available after waiting 10 seconds")
        return False, "Action server not available"

    # Create goal message
    goal_msg = RlAgent.Goal()

    # Send goal
    print("Sending goal to insert action server")
    future = action_client.send_goal_async(goal_msg)

    # Wait for goal acceptance
    rclpy.spin_until_future_complete(node, future)
    goal_handle = future.result()

    if not goal_handle.accepted:
        print("Goal rejected by action server")
        return False, "Goal rejected"

    # Get result
    result_future = goal_handle.get_result_async()
    print("Waiting for insertion action to complete...")
    rclpy.spin_until_future_complete(node, result_future)

    result = result_future.result().result
    return result.success, result.message


def main(args=None):
    parser = argparse.ArgumentParser(description="Robot arm control test")
    # parser.add_argument("--demo", action="store_true", help="Run the demo sequence")
    # parser.add_argument("--no-teleop", action="store_true", help="Run without teleop capabilities")
    parsed_args, remaining = parser.parse_known_args()

    rclpy.init(args=remaining)

    #! Agent
    print("----- Starting lift insert task -----")
    franka_arm = FrankaArm()
    # franka_arm.home()

    # # Run with full teleop (joystick) capabilities
    # teleop_node = TeleopNode(franka_arm)

    # # Move joints home
    # franka_arm.goto_home()

    # Moving to Start pose
    print("Moving to start pose...")
    franka_arm.gripper_open()
    # cube_height = 0.025

    # gripper_start_pose_t = np.array([0.56, 0.0, 0.180])
    gripper_start_pose_rpy = np.array([PI, 0, 0])

    X_G_start = pin.SE3(pin.rpy.rpyToMatrix(gripper_start_pose_rpy), p_BGstart_B)
    franka_arm.goto_pose(X_G_start, Duration(seconds=10.0), Kp=1.0, Kd=0.0)

    franka_arm.gripper_open()
    # franka_arm.gripper_close()

    # Start a trial! Call the insert action
    print("--- Starting lift trials ---")
    trial_n = 1

    while True:
        print(f"\n\n--- Trial {trial_n} ---")
        # trial_offset = np.array([*np.random.uniform(-SOCKET_NOISE, SOCKET_NOISE, size=1), 0.0, 0.0])
        # trial_offset = np.array([-0.00, 0.0, 0.0])

        # print(f"Trial offset: {trial_offset}")
        # X_G_start_trial = X_G_start.copy()
        # X_G_start_trial.translation += trial_offset
        # franka_arm.goto_pose(X_G_start_trial, Duration(seconds=10.0))

        print("Starting lift action")
        success, message = call_lift_action(franka_arm)

        # Print result
        if success:
            print(f"Lift successful! Message: {message}")
        else:
            print(f"Lift failed! Message: {message}")

        time.sleep(2.0)

        franka_arm.goto_pose(X_G_start, Duration(seconds=10.0), Kp=1.0, Kd=0.0)
        franka_arm.gripper_open()

        trial_n += 1

    # Clean up
    rclpy.shutdown()


if __name__ == "__main__":
    main()
