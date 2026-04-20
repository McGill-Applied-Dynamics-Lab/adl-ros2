#!/usr/bin/env python3
"""Example showing how to use `Robot.execute_sequence()` with IK-planned trajectories."""

import time

import numpy as np
from scipy.spatial.transform import Rotation as R

from arm_client.robot import Pose, Robot
from arm_client.planning.waypoints import (
    generate_linear_waypoints,
    generate_spherical_waypoints,
)

RADIUS = 0.200
THETA = 45
PHI = -225.0


def main():
    robot = Robot(namespace="fr3")
    robot.wait_until_ready()

    print("Switching to joint trajectory controller...")
    robot.controller_switcher_client.switch_controller("joint_trajectory_controller")

    robot.home()

    START_POSITION = np.array([0.40, -0.045, 0.40])
    START_ORIENTATION = R.from_euler("xyz", [-180, 0, 0], degrees=True)

    robot.end_effector_pose

    start_pose = Pose(START_POSITION, START_ORIENTATION)
    robot.move_to(pose=start_pose)
    time.sleep(2.0)

    waypoints_1 = generate_spherical_waypoints(
        start_position=start_pose.position,
        start_orientation=start_pose.orientation,
        radius=RADIUS,
        theta_deg=THETA,
        phi_deg=PHI,
        num_waypoints=50,
    )

    end_pose_1 = Pose(
        position=waypoints_1[-1].position,
        orientation=waypoints_1[-1].orientation,
    )

    waypoints_2 = generate_spherical_waypoints(
        start_position=end_pose_1.position,
        start_orientation=end_pose_1.orientation,
        radius=RADIUS,
        theta_deg=-THETA,
        phi_deg=PHI,
        num_waypoints=50,
    )

    traj1, traj2 = robot.plan_joint_trajectory_sequence(
        [waypoints_1, waypoints_2],
        [2.0, 2.0],
    )

    # robot.execute_sequence(
    #     [
    #         traj1,
    #         traj2,
    #     ],
    #     visualize_before_execution=True,
    #     settle_time_between_trajectories=0.0,
    # )

    robot.follow_joint_trajectory(traj1, settle_time=0.0)
    robot.follow_joint_trajectory(traj2, settle_time=0.0)

    print("Sequence execution complete.")
    robot.shutdown()


if __name__ == "__main__":
    main()
