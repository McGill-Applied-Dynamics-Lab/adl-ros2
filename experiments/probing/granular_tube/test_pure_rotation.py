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


def main():
    robot = Robot(namespace="fr3")
    robot.wait_until_ready()

    print("Switching to joint trajectory controller...")
    robot.controller_switcher_client.switch_controller("joint_trajectory_controller")

    start_pose = robot.end_effector_pose.copy()
    start_orientation = R.from_euler("xyz", [-180, 0, 0], degrees=True)
    rot_vec = np.array([0, 0, np.radians(45.0)])
    rotation = R.from_rotvec(rot_vec)

    waypoints_forward = generate_linear_waypoints(
        start_position=start_pose.position,
        start_orientation=start_orientation,
        end_position=start_pose.position,
        end_orientation=rotation * start_orientation,
        num_waypoints=10,
    )
    end_forward_pose = Pose(
        position=waypoints_forward[-1].position,
        orientation=waypoints_forward[-1].orientation,
    )
    waypoints_reverse = generate_linear_waypoints(
        start_position=start_pose.position,
        start_orientation=end_forward_pose.orientation,
        end_position=start_pose.position,
        end_orientation=start_orientation,
        num_waypoints=10,
    )

    traj1, traj2 = robot.plan_joint_trajectory_sequence(
        [waypoints_forward, waypoints_reverse],
        [1.0, 1.0],
    )

    print("Starting execution.")

    robot.execute_sequence(
        [
            traj1,
            traj2,
        ],
        visualize_before_execution=False,
        settle_time_between_trajectories=0.0,  # minimal delay
    )

    print("Sequence execution complete.")
    robot.shutdown()


if __name__ == "__main__":
    main()
