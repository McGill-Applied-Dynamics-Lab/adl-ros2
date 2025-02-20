import rclpy
from robot_arm_client.FrankaArm import FrankaArm
from rclpy.duration import Duration

import numpy as np
from spatialmath.pose3d import SE3


def main(args=None):
    rclpy.init(args=args)

    print("Starting robot")
    franka_arm = FrankaArm()

    # # Move joints home
    # franka_arm.goto_home()

    # # Move joints to a position
    # joints_goal = np.deg2rad([0, 0, 0, -135, 0, 90, 45])
    # franka_arm.goto_joints(joints_goal, Duration(seconds=10.0))

    # # Move to pose
    # pose_goal = SE3(0.5, 0.5, 0.5)
    # franka_arm.goto_pose(pose_goal, 5.0)

    # Joint velocities
    joint_vels = [0, 0, 0, 0, 0, 0.0, 0.2]
    duration = 4.0
    franka_arm.goto_joint_vels(joint_vels, unit="rad", duration=Duration(seconds=duration))

    # # EE velocities
    # ee_vel = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    # duration = 2.0
    # franka_arm.goto_ee_vel(ee_vel, duration=Duration(seconds=duration))

    # Clean
    franka_arm.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
