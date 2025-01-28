import rclpy
from robot_arm.FrankaArm import FrankaArm
import numpy as np

from spatialmath.pose3d import SE3


def main(args=None):
    rclpy.init(args=args)

    print("Starting robot")
    robot_arm = FrankaArm()

    # # Move joints to a position
    # joints_goal = np.deg2rad([0, 0, 0, -135, 0, 90, 45])
    # print(f"Moving joints to goal: {joints_goal}")
    # robot_arm.goto_joints(joints_goal, 5.0)

    # Move joints home
    robot_arm.goto_home()

    # # Move to pose
    # pose_goal = SE3(0.5, 0.5, 0.5)
    # robot_arm.goto_pose(pose_goal, 5.0)

    robot_arm.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
