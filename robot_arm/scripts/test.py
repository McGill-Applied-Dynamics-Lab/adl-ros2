import rclpy
from robot_arm.FrankaArm import FrankaArm
import numpy as np


def main(args=None):
    rclpy.init(args=args)

    print("Starting robot")
    robot_arm = FrankaArm()

    joints_goal = np.deg2rad([0, 0, 0, -135, 0, 90, 45])
    print(f"Moving joints to goal: {joints_goal}")
    robot_arm.goto_joints(joints_goal, 5.0)

    # rclpy.spin(robot_arm)

    robot_arm.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
