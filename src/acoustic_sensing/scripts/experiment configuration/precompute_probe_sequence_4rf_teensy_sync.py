#!/usr/bin/env python3
"""Precompute and save the full 4RF probing sequence from an assumed startup state."""

from pathlib import Path
import pickle
import time

import numpy as np

from arm_client.robot import Pose, Robot

from common import (
    APPROACH_DURATION,
    APPROACH_LIFT_HEIGHT,
    APPROACH_SKIP_THRESHOLD,
    artifacts_root,
    BASE_ORI,
    build_sequence_waypoints,
    build_lifted_transition_waypoints,
    generate_smooth_linear_waypoints,
    get_adaptive_waypoint_count,
    load_grid_and_landmarks,
    plan_sequence_with_progress,
    PROBE_DEPTH,
    PROBE_STEP_SIZE,
    PLUNGE_DURATION,
    print_progress,
    precompute_probe_geometry,
    RETRACT_DURATION,
    RETURN_HOME_DURATION,
    verify_pose_continuity,
    Z_OFFSET,
    serialize_trajectory,
)


def build_sequence_waypoints_with_probe(
    probe_geometries: list[dict],
    start_pose: Pose,
) -> tuple[list[list], list[float], list[dict], list[int]]:
    waypoints_list = []
    durations = []
    probe_steps: list[dict] = []
    nominal_pose = start_pose.copy()

    for idx, geometry in enumerate(probe_geometries):
        approach_pose = geometry["approach_pose"]
        approach_distance = float(np.linalg.norm(approach_pose.position - nominal_pose.position))
        approach_indices: list[int] = []

        if approach_distance > APPROACH_SKIP_THRESHOLD:
            if idx == 0:
                approach_segments, approach_durations = build_lifted_transition_waypoints(
                    nominal_pose,
                    approach_pose,
                    APPROACH_DURATION,
                )
                approach_indices = list(
                    range(len(waypoints_list), len(waypoints_list) + len(approach_segments))
                )
                waypoints_list.extend(approach_segments)
                durations.extend(approach_durations)
            else:
                approach_waypoint_count = get_adaptive_waypoint_count(
                    nominal_pose,
                    approach_pose,
                )
                approach_indices = [len(waypoints_list)]
                waypoints_list.append(
                    generate_smooth_linear_waypoints(
                        nominal_pose,
                        approach_pose,
                        approach_waypoint_count,
                    )
                )
                durations.append(APPROACH_DURATION)

        probe_segments: list[dict] = []
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

            plunge_idx = len(waypoints_list)
            waypoints_list.append(
                generate_smooth_linear_waypoints(
                    approach_pose,
                    plunge_pose,
                    plunge_waypoint_count,
                )
            )
            durations.append(PLUNGE_DURATION)

            retract_idx = len(waypoints_list)
            waypoints_list.append(
                generate_smooth_linear_waypoints(
                    plunge_pose,
                    approach_pose,
                    retract_waypoint_count,
                )
            )
            durations.append(RETRACT_DURATION)

            probe_segments.append(
                {
                    "depth": float(depth),
                    "plunge_idx": plunge_idx,
                    "retract_idx": retract_idx,
                }
            )

        probe_steps.append(
            {
                "gripper_xy": geometry["gripper_xy"],
                "approach_idx": approach_indices[0] if approach_indices else None,
                "approach_indices": approach_indices,
                "approach_position": np.asarray(approach_pose.position, dtype=float),
                "approach_orientation_quat": approach_pose.orientation.as_quat(),
                "probe_segments": probe_segments,
            }
        )
        nominal_pose = approach_pose

    return_home_waypoint_count = get_adaptive_waypoint_count(
        nominal_pose,
        start_pose,
    )
    return_home_indices = [len(waypoints_list)]
    waypoints_list.append(
        generate_smooth_linear_waypoints(
            nominal_pose,
            start_pose,
            return_home_waypoint_count,
        )
    )
    durations.append(RETURN_HOME_DURATION)
    return waypoints_list, durations, probe_steps, return_home_indices


def main() -> None:
    start_time = time.perf_counter()
    total_stages = 5
    stage = 0

    set_name = input("Select grid set to precompute (train/test) [default: test]: ").strip().lower()
    if set_name not in ["train", "test"]:
        set_name = "test"
        print(f"Invalid selection. Using default: {set_name}")
    include_probe_motion = (
        input("Also precompute plunge/retract probing motion? (y/N): ").strip().lower()
        in {"y", "yes"}
    )
    if include_probe_motion:
        depth_mm = [round(depth * 1000.0, 3) for depth in PROBE_DEPTH]
        step_mm = round(PROBE_STEP_SIZE * 1000.0, 3)
        confirm_depths = input(
            f"Using PROBE_DEPTH={depth_mm} mm and PROBE_STEP_SIZE={step_mm} mm from common.py. Continue? [Y/n]: "
        ).strip().lower()
        if confirm_depths in {"n", "no"}:
            print("Precompute cancelled. Update common.py probe settings first.")
            return

    landmarks, grid_xy_world, grid_xy_gripper = load_grid_and_landmarks(set_name)
    z_surface = landmarks["z"] + Z_OFFSET
    stage += 1
    print_progress("Precompute stages", stage, total_stages)

    robot = Robot(namespace="fr3")
    try:
        probe_geometries = precompute_probe_geometry(
            grid_xy_world,
            grid_xy_gripper,
            z_surface,
        )
        stage += 1
        print_progress("Precompute stages", stage, total_stages)

        home_position = np.array([landmarks["x"], landmarks["y"], z_surface], dtype=float)
        home_pose = Pose(home_position, BASE_ORI)
        startup_position = np.array(
            [landmarks["x"], landmarks["y"], z_surface + APPROACH_LIFT_HEIGHT],
            dtype=float,
        )
        assumed_start_pose = Pose(startup_position, BASE_ORI)
        assumed_start_joint_config = np.array(robot.config.home_config, dtype=float)
        print(
            "Assumed startup pose from lifted landmark home pose: "
            f"{assumed_start_pose.position.tolist()}"
        )
        print(
            "Assumed startup joint configuration from FR3 home_config: "
            f"{assumed_start_joint_config.tolist()}"
        )
        stage += 1
        print_progress("Precompute stages", stage, total_stages)

        print("Building full waypoint sequence...")
        if include_probe_motion:
            waypoints_list, durations, probe_steps, return_home_indices = build_sequence_waypoints_with_probe(
                probe_geometries,
                home_pose,
            )
        else:
            waypoints_list, durations, probe_steps, return_home_indices = build_sequence_waypoints(
                probe_geometries,
                home_pose,
            )
        initial_descent_waypoint_count = get_adaptive_waypoint_count(
            assumed_start_pose,
            home_pose,
            step_size=PROBE_STEP_SIZE,
        )
        initial_descent_waypoints = generate_smooth_linear_waypoints(
            assumed_start_pose,
            home_pose,
            initial_descent_waypoint_count,
        )
        startup_transition_indices = [0]
        waypoints_list.insert(0, initial_descent_waypoints)
        durations.insert(0, APPROACH_DURATION)
        for probe_step in probe_steps:
            probe_step["approach_idx"] = None if probe_step["approach_idx"] is None else int(probe_step["approach_idx"]) + 1
            probe_step["approach_indices"] = [int(idx) + 1 for idx in probe_step["approach_indices"]]
            for probe_segment in probe_step.get("probe_segments", []):
                if "plunge_idx" in probe_segment:
                    probe_segment["plunge_idx"] = int(probe_segment["plunge_idx"]) + 1
                if "retract_idx" in probe_segment:
                    probe_segment["retract_idx"] = int(probe_segment["retract_idx"]) + 1
        return_home_indices = [int(idx) + 1 for idx in return_home_indices]
        verify_pose_continuity(waypoints_list)
        stage += 1
        print_progress("Precompute stages", stage, total_stages)

        print(f"Planning full sequence with {len(waypoints_list)} segments...")
        planned_segments = plan_sequence_with_progress(
            robot,
            waypoints_list,
            durations,
            assumed_start_joint_config,
        )
        stage += 1
        print_progress("Precompute stages", stage, total_stages)
    finally:
        robot.shutdown()

    artifact_dir = artifacts_root()
    artifact_dir.mkdir(parents=True, exist_ok=True)
    if include_probe_motion:
        output_path = artifact_dir / f"precomputed_{set_name.upper()}_4RF_joint_sequence_with_probe_new.pkl"
    else:
        output_path = artifact_dir / f"precomputed_{set_name.upper()}_4RF_joint_sequence_new.pkl"

    payload = {
        "set_name": set_name,
        "landmarks": landmarks,
        "z_offset": Z_OFFSET,
        "includes_probe_motion": include_probe_motion,
        "probe_depths": PROBE_DEPTH if include_probe_motion else [],
        "assumed_start_pose": {
            "position": np.asarray(assumed_start_pose.position, dtype=float),
            "orientation_quat": assumed_start_pose.orientation.as_quat(),
        },
        "assumed_start_joint_config": assumed_start_joint_config,
        "startup_transition_indices": startup_transition_indices,
        "probe_steps": probe_steps,
        "return_home_indices": return_home_indices,
        "planned_segments": [serialize_trajectory(traj) for traj in planned_segments],
    }

    with open(output_path, "wb") as f:
        pickle.dump(payload, f)

    print(f"Saved precomputed sequence to: {output_path}")
    elapsed = time.perf_counter() - start_time
    print(f"Total precompute time: {elapsed:.2f} s ({elapsed / 60.0:.2f} min)")


if __name__ == "__main__":
    main()
