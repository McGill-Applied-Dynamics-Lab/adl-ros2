#!/usr/bin/env python3
"""Example showing how to use `Robot.execute_sequence()` with IK-planned trajectories."""

import time

import numpy as np

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


    waypoints_2 = generate_linear_waypoints(
        start_position=start_pose.position,
        start_orientation=end_pose_1.orientation,
        end_position=start.position + np.array([0.1, 0.0, 0.0]),
        end_orientation=end_pose_1.orientation,
        num_waypoints=8,
    )
    end_pose_2 = Pose(
        position=waypoints_2[-1].position,
        orientation=waypoints_2[-1].orientation,
    )

    waypoints_3 = generate_spherical_waypoints(
        start_position=end_pose_2.position,
        start_orientation=end_pose_2.orientation,
        radius=0.1,
        theta_deg=-70.0,
        phi_deg=0.0,
        num_waypoints=10,
    )

    # 3) Plan trajectories with chained IK seeds for joint continuity.
    traj1, traj2, traj3 = robot.plan_joint_trajectory_sequence(
        [waypoints_1, waypoints_2, waypoints_3],
        [4.0, 3.0, 4.0],
    )

    # 4) Execute as a sequence.
    # Replace `between_steps_action` with a real gripper call if desired.
    def between_steps_action():
        print("Mid-sequence action (example placeholder, e.g., gripper close)")
        time.sleep(0.5)

    robot.execute_sequence(
        [
            traj1,
            # between_steps_action,
            traj2,
            # between_steps_action,
            traj3,
        ],
        visualize_before_execution=True,
        settle_time_between_trajectories=1.0,
    )

    print("Sequence execution complete.")
    robot.shutdown()


if __name__ == "__main__":
    main()
