#!/usr/bin/env python3
"""Run a fully continuous live grid probe sequence without Teensy synchronization."""

import time

import numpy as np

from arm_client.robot import Pose, Robot

from common import (
    APPROACH_DURATION,
    APPROACH_SKIP_THRESHOLD,
    build_lifted_transition_waypoints,
    generate_smooth_linear_waypoints,
    get_adaptive_waypoint_count,
    get_probe_surface_z,
    load_grid_and_landmarks,
    PROBE_DEPTH,
    PROBE_STEP_SIZE,
    PLUNGE_DURATION,
    precompute_probe_geometry,
    print_progress,
    RETRACT_DURATION,
    RETURN_HOME_DURATION,
    SETTLE_SEC,
    verify_pose_continuity,
    Z_OFFSET,
)


def build_live_sequence_waypoints(
    startup_pose: Pose,
    probe_geometries: list[dict],
) -> tuple[list[list], list[float]]:
    waypoints_list: list[list] = []
    durations: list[float] = []
    nominal_pose = startup_pose.copy()
    total_locations = len(probe_geometries)

    for idx, geometry in enumerate(probe_geometries, start=1):
        approach_pose = geometry["approach_pose"]
        approach_distance = float(np.linalg.norm(approach_pose.position - nominal_pose.position))

        if approach_distance > APPROACH_SKIP_THRESHOLD:
            if idx == 1:
                approach_segments, approach_durations = build_lifted_transition_waypoints(
                    nominal_pose,
                    approach_pose,
                    APPROACH_DURATION,
                )
                waypoints_list.extend(approach_segments)
                durations.extend(approach_durations)
            else:
                approach_waypoint_count = get_adaptive_waypoint_count(
                    nominal_pose,
                    approach_pose,
                )
                waypoints_list.append(
                    generate_smooth_linear_waypoints(
                        nominal_pose,
                        approach_pose,
                        approach_waypoint_count,
                    )
                )
                durations.append(APPROACH_DURATION)

        for depth in PROBE_DEPTH:
            plunge_pose = Pose(
                np.array(
                    [
                        approach_pose.position[0],
                        approach_pose.position[1],
                        approach_pose.position[2] - depth,
                    ],
                    dtype=float,
                ),
                approach_pose.orientation,
            )
            plunge_waypoint_count = get_adaptive_waypoint_count(
                approach_pose,
                plunge_pose,
                step_size=PROBE_STEP_SIZE,
            )
            retract_waypoint_count = get_adaptive_waypoint_count(
                plunge_pose,
                approach_pose,
                step_size=PROBE_STEP_SIZE,
            )
            waypoints_list.append(
                generate_smooth_linear_waypoints(
                    approach_pose,
                    plunge_pose,
                    plunge_waypoint_count,
                )
            )
            durations.append(PLUNGE_DURATION)
            waypoints_list.append(
                generate_smooth_linear_waypoints(
                    plunge_pose,
                    approach_pose,
                    retract_waypoint_count,
                )
            )
            durations.append(RETRACT_DURATION)

        nominal_pose = approach_pose
        print_progress("Sequence build", idx, total_locations)

    return_home_waypoint_count = get_adaptive_waypoint_count(
        nominal_pose,
        startup_pose,
    )
    waypoints_list.append(
        generate_smooth_linear_waypoints(
            nominal_pose,
            startup_pose,
            return_home_waypoint_count,
        )
    )
    durations.append(RETURN_HOME_DURATION)
    verify_pose_continuity(waypoints_list)
    return waypoints_list, durations


def main() -> None:
    set_name = input("Select grid set to run live (train/test) [default: test]: ").strip().lower()
    if set_name not in ["train", "test"]:
        set_name = "test"
        print(f"Invalid selection. Using default: {set_name}")

    landmarks, grid_xy_world, grid_xy_gripper = load_grid_and_landmarks(set_name)
    probe_surface_z = get_probe_surface_z(landmarks)
    probe_geometries = precompute_probe_geometry(grid_xy_world, grid_xy_gripper, probe_surface_z)
    if len(probe_geometries) == 0:
        raise RuntimeError(f"No probe points found for grid set: {set_name}")

    robot = Robot(namespace="fr3")

    try:
        robot.wait_until_ready()
        robot.controller_switcher_client.switch_controller("joint_trajectory_controller")
        time.sleep(0.1)

        startup_pose = robot.end_effector_pose.copy()
        print(f"Loaded {len(probe_geometries)} live grid locations from set: {set_name}")
        print(f"Probe depths: {[round(depth * 1000.0, 3) for depth in PROBE_DEPTH]} mm")
        print("Building one continuous live sequence...")
        waypoints_list, durations = build_live_sequence_waypoints(
            startup_pose,
            probe_geometries,
        )
        print(f"Planning continuous sequence with {len(waypoints_list)} segments...")
        planned_segments = robot.plan_joint_trajectory_sequence(
            waypoints_list=waypoints_list,
            durations=durations,
            show_progress=True,
        )

        input("Press Enter to execute the continuous live probing sequence...")
        robot.execute_sequence(
            planned_segments,
            visualize_before_execution=True,
            settle_time_between_trajectories=0.0,
        )
        time.sleep(SETTLE_SEC)
    finally:
        robot.shutdown()

    print("Done.")


if __name__ == "__main__":
    main()
