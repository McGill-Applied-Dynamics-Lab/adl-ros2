#!/usr/bin/env python3
"""Probe the experiment grid live, one location at a time, with no full-sequence preplanning."""

import time

import numpy as np

from arm_client import CONFIG_DIR
from arm_client.robot import Pose, Robot

from common import (
    APPROACH_LIFT_HEIGHT,
    BASE_ORI,
    get_landmark_home_z,
    get_probe_surface_z,
    load_grid_and_landmarks,
    precompute_probe_geometry,
    PROBE_DEPTH,
    SETTLE_SEC,
    Z_OFFSET,
)


MOVE_SPEED = 0.03
PROBE_SPEED = 0.01


def _move_to_pose(robot: Robot, pose: Pose, speed: float) -> None:
    robot.move_to(pose=pose, speed=speed)
    time.sleep(SETTLE_SEC)


def _ensure_active_controller(robot: Robot, controller_name: str, timeout_sec: float = 5.0) -> None:
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        active = robot.controller_switcher_client.get_active_controller()
        if active == controller_name:
            print(f"Active controller: {active}")
            return
        time.sleep(0.1)
    active = robot.controller_switcher_client.get_active_controller()
    raise RuntimeError(
        f"Expected active controller '{controller_name}', but got '{active}'. "
        "Refusing to continue because that would fall back to IK/joint-trajectory motion."
    )


def main() -> None:
    set_name = input("Select grid set to run live (train/test) [default: test]: ").strip().lower()
    if set_name not in ["train", "test"]:
        set_name = "test"
        print(f"Invalid selection. Using default: {set_name}")

    landmarks, grid_xy_world, grid_xy_gripper = load_grid_and_landmarks(set_name)
    landmark_home_z = get_landmark_home_z(landmarks)
    probe_surface_z = get_probe_surface_z(landmarks)
    probe_geometries = precompute_probe_geometry(grid_xy_world, grid_xy_gripper, probe_surface_z)
    if len(probe_geometries) == 0:
        raise RuntimeError(f"No probe points found for grid set: {set_name}")

    home_pose = Pose(
        np.array([landmarks["x"], landmarks["y"], landmark_home_z], dtype=float),
        BASE_ORI,
    )
    lifted_home_pose = Pose(
        np.array([landmarks["x"], landmarks["y"], landmark_home_z + APPROACH_LIFT_HEIGHT], dtype=float),
        BASE_ORI,
    )

    robot = Robot(namespace="fr3")
    try:
        robot.wait_until_ready()
        robot.controller_switcher_client.switch_controller("fr3_pose_controller")
        robot.fr3_pose_controller_parameters_client.load_param_config(
            file_path=CONFIG_DIR / "controllers" / "fr3_pose" / "probing.yaml"
        )
        _ensure_active_controller(robot, "fr3_pose_controller")

        print(f"Loaded {len(probe_geometries)} live grid locations from set: {set_name}")
        print(f"Probe depths: {[round(depth * 1000.0, 3) for depth in PROBE_DEPTH]} mm")
        print("Moving to lifted landmark home pose...")
        _move_to_pose(robot, lifted_home_pose, MOVE_SPEED)
        print("Descending to landmark home pose...")
        _move_to_pose(robot, home_pose, PROBE_SPEED)

        input("Press Enter to start probing...")

        for idx, geometry in enumerate(probe_geometries, start=1):
            approach_pose = geometry["approach_pose"]
            print(f"\nLocation {idx}/{len(probe_geometries)}")

            if idx == 1:
                lifted_approach_pose = Pose(
                    np.array(
                        [
                            approach_pose.position[0],
                            approach_pose.position[1],
                            approach_pose.position[2] + APPROACH_LIFT_HEIGHT,
                        ],
                        dtype=float,
                    ),
                    approach_pose.orientation,
                )
                print("\tLifting off landmark home pose...")
                _move_to_pose(robot, lifted_home_pose, PROBE_SPEED)
                print("\tMoving across to first approach above the surface...")
                _move_to_pose(robot, lifted_approach_pose, MOVE_SPEED)
                print("\tDescending to first approach pose...")
                _move_to_pose(robot, approach_pose, PROBE_SPEED)
            else:
                print("\tMoving directly to next approach pose...")
                _move_to_pose(robot, approach_pose, MOVE_SPEED)

            for depth_idx, depth in enumerate(PROBE_DEPTH, start=1):
                plunge_pose = Pose(
                    np.array(
                        [
                            approach_pose.position[0],
                            approach_pose.position[1],
                            approach_pose.position[2] - depth,
                        ],
                        dtype=float,
                    ),
                    BASE_ORI,
                )
                print(
                    f"\tDepth {depth_idx}/{len(PROBE_DEPTH)}: {depth * 1000.0:.2f} mm"
                )
                _move_to_pose(robot, plunge_pose, PROBE_SPEED)
                _move_to_pose(robot, approach_pose, PROBE_SPEED)

        print("\nReturning to landmark home pose...")
        _move_to_pose(robot, home_pose, MOVE_SPEED)
    finally:
        robot.shutdown()

    print("Done.")


if __name__ == "__main__":
    main()
