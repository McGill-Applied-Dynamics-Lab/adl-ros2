import rclpy
from robot_arm.FrankaArm import FrankaArm
from rclpy.duration import Duration

import numpy as np
from spatialmath.pose3d import SE3


def main(args=None):
    rclpy.init(args=args)

    print("Starting robot")
    franka_arm = FrankaArm()

    # # Move joints to a position
    # joints_goal = np.deg2rad([0, 0, 0, -135, 0, 90, 45])
    # print(f"Moving joints to goal: {joints_goal}")
    # robot_arm.goto_joints(joints_goal, 5.0)

    # # Move joints home
    # franka_arm.goto_home()

    # # Move to pose
    # pose_goal = SE3(0.5, 0.5, 0.5)
    # robot_arm.goto_pose(pose_goal, 5.0)

    # Joint velocities
    joint_vels = [-40, 0, 0, 0, 0, 0, 0]
    duration = 5.0
    franka_arm.goto_joint_vels(
        joint_vels, unit="deg", duration=Duration(seconds=duration)
    )

    # Clean
    franka_arm.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
