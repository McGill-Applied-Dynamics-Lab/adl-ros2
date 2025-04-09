import rclpy
from robot_arm_client.FrankaArm import FrankaArm
from robot_arm_client.teleop_node import TeleopNode
from rclpy.duration import Duration
import argparse

import numpy as np
from numpy import pi as PI

import time
import pinocchio as pin
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

PEG_IN_HAND = True
peg_start_pose = np.array([0.40, 0.20, 0.050])
peg_start_rpy = np.array([PI, 0.0, 0.0])
X_P_start = pin.SE3(pin.rpy.rpyToMatrix(peg_start_rpy), peg_start_pose)


# Socket
socket_start_pose = np.array([0.6, 0.0, 0.0])
socket_start_rpy = np.array([0.0, 0.0, 0.0])
X_S = pin.SE3(pin.rpy.rpyToMatrix(socket_start_rpy), socket_start_pose)

peg_offset = np.array([0.0, 0.0, -0.04])  # Gripper to peg offset
socket_height = np.array([0.0, 0.0, 0.03])  # Height of the top of socket, from table
peg_socket_offset = 0.01  # Distance between tip of peg and top of socket [cm]

# Agent Params
agent_name = "insert_db2.pt"
package_share_directory = Path(get_package_share_directory("robot_tasks"))
agent_path = package_share_directory / "agents" / agent_name


def main(args=None):
    parser = argparse.ArgumentParser(description="Robot arm control test")
    parser.add_argument("--demo", action="store_true", help="Run the demo sequence")
    parser.add_argument("--no-teleop", action="store_true", help="Run without teleop capabilities")
    parsed_args, remaining = parser.parse_known_args()

    rclpy.init(args=remaining)

    #! Agent
    ctrl_freq = 100  # Hz

    print("----- Starting peg insert task -----")
    franka_arm = FrankaArm()
    # franka_arm.home()

    # # Run with full teleop (joystick) capabilities
    # teleop_node = TeleopNode(franka_arm)

    # # Move joints home
    # franka_arm.goto_home()

    #! Pick up peg
    if not PEG_IN_HAND:
        print("Picking up peg...")
        franka_arm.goto_pose(X_P_start, Duration(seconds=5.0))

        # # Close gripper
        # franka_arm.gripper_close()
        franka_arm.gripper_close()

    # Moving to Start pose
    print("Moving to start pose...")
    gripper_start_pose_t = socket_start_pose - peg_offset + socket_height + np.array([0.0, 0.0, peg_socket_offset])
    gripper_start_pose_rpy = np.array([PI, 0, 0])
    X_G_start = pin.SE3(pin.rpy.rpyToMatrix(gripper_start_pose_rpy), gripper_start_pose_t)
    franka_arm.goto_pose(X_G_start, Duration(seconds=10.0))

    # Start a trial!
    print("\nStarting a trial!")
    ...


if __name__ == "__main__":
    main()
