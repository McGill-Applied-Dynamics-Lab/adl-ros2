import rclpy
from robot_arm_client.FrankaArm import FrankaArm
from robot_arm_client.teleop_node import TeleopNode
from rclpy.duration import Duration
import argparse

import numpy as np
import time

import pinocchio as pin

PEG_IN_HAND = False
peg_start_pose = np.array([0.30, -0.20, 0.40])
peg_start_rpy = np.array([0.0, 0.0, 0.0])
X_P_start = pin.SE3(pin.rpy.rpyToMatrix(peg_start_rpy), peg_start_pose)

# Socket
socket_start_pose = np.array([0.30, -0.20, 0.40])
socket_start_rpy = np.array([0.0, 0.0, 0.0])
X_S = pin.SE3(pin.rpy.rpyToMatrix(socket_start_rpy), socket_start_pose)


def main(args=None):
    parser = argparse.ArgumentParser(description="Robot arm control test")
    parser.add_argument("--demo", action="store_true", help="Run the demo sequence")
    parser.add_argument("--no-teleop", action="store_true", help="Run without teleop capabilities")
    parsed_args, remaining = parser.parse_known_args()

    rclpy.init(args=remaining)

    print("----- Starting peg insert task -----")
    franka_arm = FrankaArm()

    # # Run with full teleop (joystick) capabilities
    # teleop_node = TeleopNode(franka_arm)

    # # Move joints home
    # franka_arm.goto_home()

    # Move to pick
    if not PEG_IN_HAND:
        print("Picking up peg...")
        franka_arm.goto_pose(X_P_start, Duration(seconds=5.0))

        # Close gripper
        franka_arm.gripper_close()

    # Moving to Start pose
    print("Moving to start pose...")
    gripper_start_pose = np.array([0.30, -0.20, 0.40])
    X_G_start = pin.SE3(pin.rpy.rpyToMatrix(np.array([0, 0, 0])), gripper_start_pose)
    franka_arm.goto_pose(X_G_start, Duration(seconds=5.0))

    # Start a trial!
    print("\nStarting a trial!")
    ...


if __name__ == "__main__":
    main()
