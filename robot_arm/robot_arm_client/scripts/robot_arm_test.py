import rclpy
from robot_arm_client.FrankaArm import FrankaArm
from robot_arm_client.teleop_node import TeleopNode
from rclpy.duration import Duration
import argparse

import numpy as np


def direct_control_demo(franka_arm):
    """Run a demo of direct control commands"""
    
    # # Move joints home
    # franka_arm.goto_home()

    # # Move joints to a position
    # joints_goal = np.deg2rad([0, 0, 0, -135, 0, 90, 45])
    # franka_arm.goto_joints(joints_goal, Duration(seconds=5.0))

    # # Move to pose
    # pose_goal = np.array([0.30, -0, 0.40])
    # franka_arm.goto_pose(pose_goal, Duration(seconds=5.0))

    # Toggle gripper
    franka_arm.gripper_toggle()
    
    # Wait a moment
    import time
    time.sleep(2.0)
    
    # Toggle gripper again
    franka_arm.gripper_toggle()


def main(args=None):
    parser = argparse.ArgumentParser(description='Robot arm control test')
    parser.add_argument('--demo', action='store_true', help='Run the demo sequence')
    parser.add_argument('--no-teleop', action='store_true', help='Run without teleop capabilities')
    parsed_args, remaining = parser.parse_known_args()

    run_demo = False # parsed_args.demo
    no_teleop = False # parsed_args.no_teleop
    
    rclpy.init(args=remaining)
    
    franka_arm = FrankaArm()

    if run_demo:
        # Just create arm for the demo
        # Run the demo sequence
        direct_control_demo(franka_arm)

    # Run with full teleop (joystick) capabilities
    teleop_node = TeleopNode(franka_arm)
    try:
        print("Starting robot with teleop. Press B to toggle gripper, Y for homing...")
        rclpy.spin(teleop_node)
    except KeyboardInterrupt:
        teleop_node.get_logger().info("KeyboardInterrupt, shutting down.\n")
    finally:
        teleop_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
