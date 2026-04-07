#!/usr/bin/env python3
"""Execute precomputed approaches and probe each location without RF waiting."""

import pickle
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation

from arm_client.planning.ik_pyroki import plan_fr3_joint_trajectory
from arm_client.robot import Pose, Robot
from arm_client.robot_config import FR3Config

from common import (
    APPROACH_DURATION,
    artifacts_root,
    BRIDGE_DURATION,
    BRIDGE_JOINT_THRESHOLD,
    deserialize_trajectory,
    generate_smooth_linear_waypoints,
    get_adaptive_waypoint_count,
    get_approach_indices,
    get_return_home_indices,
    get_startup_transition_indices,
    load_grid_and_landmarks,
    precompute_probe_geometry,
    print_progress,
    PROBE_DEPTH,
    PROBE_STEP_SIZE,
    PLUNGE_DURATION,
    RETRACT_DURATION,
    SETTLE_SEC,
)


PLANNER_CONFIG = FR3Config()


def _plan_joint_trajectory_background(
    waypoints: list,
    duration: float,
    initial_joint_config: np.ndarray,
):
    return plan_fr3_joint_trajectory(
        waypoints=list(waypoints),
        duration=duration,
        joint_names=PLANNER_CONFIG.joint_names,
        target_link_name=PLANNER_CONFIG.ik_target_link_name,
        n_points=PLANNER_CONFIG.ik_default_num_points,
        current_joint_config=np.array(initial_joint_config, dtype=float),
        pos_weight=PLANNER_CONFIG.ik_position_weight,
        ori_weight=PLANNER_CONFIG.ik_orientation_weight,
        similarity_weight=PLANNER_CONFIG.ik_similarity_weight,
        show_progress=False,
        use_cpu=True,
    )


def _step_has_approach_pose(step: dict) -> bool:
    return "approach_position" in step and "approach_orientation_quat" in step


def _approach_pose_from_step(step: dict) -> Pose:
    return Pose(
        np.asarray(step["approach_position"], dtype=float),
        Rotation.from_quat(np.asarray(step["approach_orientation_quat"], dtype=float)),
    )


def _backfill_approach_pose_metadata(probe_steps: list[dict], payload: dict) -> list[dict]:
    if all(_step_has_approach_pose(step) for step in probe_steps):
        return probe_steps

    _, grid_xy_world, grid_xy_gripper = load_grid_and_landmarks(payload["set_name"])
    z_surface = float(payload["landmarks"]["z"]) + float(payload["z_offset"])
    probe_geometries = precompute_probe_geometry(
        grid_xy_world,
        grid_xy_gripper,
        z_surface,
    )

    if len(probe_geometries) != len(probe_steps):
        raise ValueError(
            "Saved probe step count does not match current grid size. "
            "Cannot reconstruct approach pose metadata."
        )

    patched_steps: list[dict] = []
    for step, geometry in zip(probe_steps, probe_geometries):
        patched = dict(step)
        patched["approach_position"] = np.asarray(geometry["approach_pose"].position, dtype=float)
        patched["approach_orientation_quat"] = geometry["approach_pose"].orientation.as_quat()
        patched_steps.append(patched)

    return patched_steps


def _build_probe_sequence_waypoints(
    probe_steps: list[dict],
    location_idx: int,
) -> tuple[list[list], list[float], bool]:
    step = probe_steps[location_idx]
    approach_pose = _approach_pose_from_step(step)
    sequence_waypoints = []
    sequence_durations = []

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
        sequence_waypoints.append(
            generate_smooth_linear_waypoints(
                approach_pose,
                plunge_pose,
                plunge_waypoint_count,
            )
        )
        sequence_durations.append(PLUNGE_DURATION)
        sequence_waypoints.append(
            generate_smooth_linear_waypoints(
                plunge_pose,
                approach_pose,
                retract_waypoint_count,
            )
        )
        sequence_durations.append(RETRACT_DURATION)

    transitions_to_next = False
    if location_idx + 1 < len(probe_steps) and _step_has_approach_pose(probe_steps[location_idx + 1]):
        next_approach_pose = _approach_pose_from_step(probe_steps[location_idx + 1])
        transition_waypoint_count = get_adaptive_waypoint_count(
            approach_pose,
            next_approach_pose,
        )
        if transition_waypoint_count >= 2:
            sequence_waypoints.append(
                generate_smooth_linear_waypoints(
                    approach_pose,
                    next_approach_pose,
                    transition_waypoint_count,
                )
            )
            sequence_durations.append(APPROACH_DURATION)
            transitions_to_next = True

    return sequence_waypoints, sequence_durations, transitions_to_next


def _get_location_execution_prefix(
    probe_step: dict,
    planned_segments: list,
    at_approach: bool,
) -> list:
    approach_indices = get_approach_indices(probe_step)
    if at_approach or not approach_indices:
        return []
    return [planned_segments[idx] for idx in approach_indices]


def _location_end_segment_indices(probe_steps: list[dict]) -> list[int]:
    end_indices: list[int] = []
    for step in probe_steps:
        segment_indices = list(get_approach_indices(step))
        for probe_segment in step.get("probe_segments", []):
            if "retract_idx" in probe_segment:
                segment_indices.append(int(probe_segment["retract_idx"]))
            elif "plunge_idx" in probe_segment:
                segment_indices.append(int(probe_segment["plunge_idx"]))
        if not segment_indices:
            raise ValueError("Precomputed probe step has no segment indices to report progress from.")
        end_indices.append(max(segment_indices))
    return end_indices


def _execute_full_sequence_with_probe_progress(
    robot: Robot,
    trajectories: list,
    probe_steps: list[dict],
) -> None:
    if len(probe_steps) == 0:
        robot.execute_sequence(
            trajectories,
            visualize_before_execution=False,
            settle_time_between_trajectories=0.0,
        )
        return

    segment_durations = np.array(
        [float(traj.time_from_start[-1]) if len(traj.time_from_start) > 0 else 0.0 for traj in trajectories],
        dtype=float,
    )
    cumulative_durations = np.cumsum(segment_durations)
    location_end_times = [
        float(cumulative_durations[end_idx]) for end_idx in _location_end_segment_indices(probe_steps)
    ]

    errors: list[Exception] = []

    def _runner() -> None:
        try:
            robot.execute_sequence(
                trajectories,
                visualize_before_execution=False,
                settle_time_between_trajectories=0.0,
            )
        except Exception as exc:
            errors.append(exc)

    worker = threading.Thread(target=_runner, daemon=True)
    worker.start()

    total_locations = len(location_end_times)
    completed_locations = 0
    announced_return_home = False
    print_progress("Probing progress", completed_locations, total_locations)
    start_time = time.perf_counter()

    while worker.is_alive():
        elapsed = time.perf_counter() - start_time
        while (
            completed_locations < total_locations
            and elapsed >= location_end_times[completed_locations]
        ):
            completed_locations += 1
            print_progress("Probing progress", completed_locations, total_locations)
        if completed_locations >= total_locations and not announced_return_home:
            print("Returning home...")
            announced_return_home = True
        time.sleep(0.1)

    worker.join(timeout=0.1)
    if errors:
        raise errors[0]

    while completed_locations < total_locations:
        completed_locations += 1
        print_progress("Probing progress", completed_locations, total_locations)


def _preplan_live_probe_sequences(
    robot: Robot,
    probe_steps: list[dict],
    planned_segments: list,
    sequence_start_joint_config: np.ndarray,
) -> tuple[list[list] | None, list[bool] | None]:
    if any(not _step_has_approach_pose(step) for step in probe_steps):
        return None, None

    preplanned_sequences: list[list] = []
    transitions_to_next_flags: list[bool] = []
    current_seed = np.array(sequence_start_joint_config, dtype=float)
    at_approach = False
    total = len(probe_steps)

    for idx, step in enumerate(probe_steps):
        approach_indices = get_approach_indices(step)
        if not at_approach and approach_indices:
            current_seed = np.asarray(planned_segments[approach_indices[-1]].joint_positions[-1], dtype=float)

        sequence_waypoints, sequence_durations, transitions_to_next = _build_probe_sequence_waypoints(
            probe_steps,
            idx,
        )
        local_trajectories = []
        seed = np.array(current_seed, dtype=float)

        for waypoints, duration in zip(sequence_waypoints, sequence_durations):
            traj = _plan_joint_trajectory_background(
                waypoints=waypoints,
                duration=duration,
                initial_joint_config=seed,
            )
            local_trajectories.append(traj)
            seed = traj.joint_positions[-1]

        execution_prefix = _get_location_execution_prefix(step, planned_segments, at_approach)
        preplanned_sequences.append(execution_prefix + local_trajectories)
        transitions_to_next_flags.append(transitions_to_next)
        current_seed = seed
        at_approach = transitions_to_next
        print_progress("Probe planning", idx + 1, total)

    return preplanned_sequences, transitions_to_next_flags


def _plan_live_probe_sequences_background(
    probe_steps: list[dict],
    planned_segments: list,
    sequence_start_joint_config: np.ndarray,
    state: dict,
) -> None:
    try:
        current_seed = np.array(sequence_start_joint_config, dtype=float)
        at_approach = False
        total = len(probe_steps)

        for idx, step in enumerate(probe_steps):
            approach_indices = get_approach_indices(step)
            if not at_approach and approach_indices:
                current_seed = np.asarray(planned_segments[approach_indices[-1]].joint_positions[-1], dtype=float)

            sequence_waypoints, sequence_durations, transitions_to_next = _build_probe_sequence_waypoints(
                probe_steps,
                idx,
            )
            local_trajectories = []
            seed = np.array(current_seed, dtype=float)

            for waypoints, duration in zip(sequence_waypoints, sequence_durations):
                traj = _plan_joint_trajectory_background(
                    waypoints=waypoints,
                    duration=duration,
                    initial_joint_config=seed,
                )
                local_trajectories.append(traj)
                seed = traj.joint_positions[-1]

            with state["condition"]:
                execution_prefix = _get_location_execution_prefix(step, planned_segments, at_approach)
                state["results"][idx] = (execution_prefix + local_trajectories, transitions_to_next)
                state["condition"].notify_all()

            current_seed = seed
            at_approach = transitions_to_next
            print_progress("Probe planning", idx + 1, total)
    except Exception as exc:
        with state["condition"]:
            state["error"] = exc
            state["condition"].notify_all()
    finally:
        with state["condition"]:
            state["done"] = True
            state["condition"].notify_all()


def main() -> None:
    set_name = input("Select precomputed grid set to run (train/test) [default: test]: ").strip().lower()
    if set_name not in ["train", "test"]:
        set_name = "test"
        print(f"Invalid selection. Using default: {set_name}")
    use_precomputed_probe = (
        input("Use precomputed probing waypoints too? (y/N): ").strip().lower()
        in {"y", "yes"}
    )

    if use_precomputed_probe:
        sequence_path = artifacts_root() / f"precomputed_{set_name.upper()}_4RF_joint_sequence_with_probe_new.pkl"
    else:
        sequence_path = artifacts_root() / f"precomputed_{set_name.upper()}_4RF_joint_sequence_new.pkl"
    if not sequence_path.exists():
        raise FileNotFoundError(f"Precomputed sequence not found: {sequence_path}")

    with open(sequence_path, "rb") as f:
        payload = pickle.load(f)

    planned_segments = [deserialize_trajectory(item) for item in payload["planned_segments"]]
    probe_steps = _backfill_approach_pose_metadata(payload["probe_steps"], payload)
    assumed_start_joint_config = np.asarray(payload["assumed_start_joint_config"], dtype=float)
    startup_transition_indices = get_startup_transition_indices(payload)
    return_home_indices = get_return_home_indices(payload, planned_segments)
    sequence_start_joint_config = (
        np.asarray(planned_segments[0].joint_positions[0], dtype=float)
        if len(planned_segments) > 0
        else assumed_start_joint_config
    )

    robot = Robot(namespace="fr3")
    try:
        robot.wait_until_ready()
        robot.controller_switcher_client.switch_controller("joint_trajectory_controller")
        time.sleep(0.1)

        current_q = np.asarray(robot.q, dtype=float)
        joint_error = np.linalg.norm(current_q - sequence_start_joint_config)

        if joint_error > BRIDGE_JOINT_THRESHOLD:
            move_time = max(robot.config.time_to_home, float(joint_error))
            print(
                "Moving to precomputed sequence start joint configuration "
                f"(joint error {joint_error:.4f}, duration {move_time:.2f}s)..."
            )
            robot.home(
                home_config=sequence_start_joint_config.tolist(),
                time_to_home=move_time,
                blocking=True,
            )
            time.sleep(SETTLE_SEC)
        else:
            print("Startup joint move skipped; already close to precomputed sequence start.")

        skipped_approaches = sum(1 for step in probe_steps if not get_approach_indices(step))
        print(
            f"Loaded {len(planned_segments)} precomputed segments from: {sequence_path} "
            f"({skipped_approaches} zero-length approach segments omitted)"
        )

        if use_precomputed_probe:
            if not payload.get("includes_probe_motion", False):
                raise ValueError(
                    "Selected precomputed probing mode, but the loaded artifact does not include "
                    "probe motion. Rerun precompute_probe_sequence_4rf_teensy_sync.py and answer "
                    "'y' to precompute plunge/retract."
                )
            input("Press Enter to start probing...")
            print("Executing full precomputed approach+probe sequence...")
            robot.execute_sequence(
                planned_segments,
                visualize_before_execution=False,
                settle_time_between_trajectories=0.0,
            )
            time.sleep(SETTLE_SEC)
            print("Done.")
            return

        planner_state = {
            "results": {},
            "error": None,
            "done": False,
            "condition": threading.Condition(),
        }
        planner_thread = threading.Thread(
            target=_plan_live_probe_sequences_background,
            args=(
                probe_steps,
                planned_segments,
                sequence_start_joint_config,
                planner_state,
            ),
            daemon=True,
        )
        planner_thread.start()
        print("Planning live probe sequences in background...")

        input("Press Enter to start probing...")

        if startup_transition_indices:
            print("Executing startup descent to landmark home pose...")
            robot.execute_sequence(
                [planned_segments[idx] for idx in startup_transition_indices],
                visualize_before_execution=False,
                settle_time_between_trajectories=0.0,
            )
            time.sleep(SETTLE_SEC)

        at_approach = False
        for i, probe_step in enumerate(probe_steps, start=1):
            print(f"\nLocation {i}/{len(probe_steps)}")

            if planner_state is not None:
                with planner_state["condition"]:
                    announced_wait = False
                    last_ready_count = -1
                    while (i - 1) not in planner_state["results"]:
                        if planner_state["error"] is not None:
                            raise RuntimeError("Background probe planning failed.") from planner_state["error"]
                        if planner_state["done"]:
                            raise RuntimeError(
                                f"Background probe planning finished without producing location {i}."
                            )
                        ready_count = len(planner_state["results"])
                        if ready_count != last_ready_count:
                            print(f"\tBackground probe plans ready: {ready_count}/{len(probe_steps)}")
                            last_ready_count = ready_count
                        if not announced_wait:
                            print("\tWaiting for background probe planning...")
                            announced_wait = True
                        planner_state["condition"].wait(timeout=0.1)

                    execution_sequence, transitions_to_next = planner_state["results"].pop(i - 1)
                at_approach = transitions_to_next
            else:
                execution_sequence = _get_location_execution_prefix(
                    probe_step,
                    planned_segments,
                    at_approach,
                )
                if not execution_sequence:
                    print("\tApproach move skipped; already at current approach pose.")
                approach_pose = robot.end_effector_pose.copy()
                sequence_waypoints = []
                sequence_durations = []

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
                    print(
                        f"\tDepth {depth_idx}/{len(PROBE_DEPTH)}: {depth * 1000.0:.2f} mm "
                        f"(down {plunge_waypoint_count} wp, up {retract_waypoint_count} wp)"
                    )
                    sequence_waypoints.append(
                        generate_smooth_linear_waypoints(
                            approach_pose,
                            plunge_pose,
                            plunge_waypoint_count,
                        )
                    )
                    sequence_durations.append(PLUNGE_DURATION)
                    sequence_waypoints.append(
                        generate_smooth_linear_waypoints(
                            plunge_pose,
                            approach_pose,
                            retract_waypoint_count,
                        )
                    )
                    sequence_durations.append(RETRACT_DURATION)

                at_approach = False
                if i < len(probe_steps):
                    next_step = probe_steps[i]
                    if "approach_position" in next_step and "approach_orientation_quat" in next_step:
                        next_approach_pose = Pose(
                            np.asarray(next_step["approach_position"], dtype=float),
                            Rotation.from_quat(np.asarray(next_step["approach_orientation_quat"], dtype=float)),
                        )
                        transition_waypoint_count = get_adaptive_waypoint_count(
                            approach_pose,
                            next_approach_pose,
                        )
                        print(
                            f"\tTransition to next approach with {transition_waypoint_count} waypoints"
                        )
                        sequence_waypoints.append(
                            generate_smooth_linear_waypoints(
                                approach_pose,
                                next_approach_pose,
                                transition_waypoint_count,
                            )
                        )
                        sequence_durations.append(APPROACH_DURATION)
                        at_approach = True
                    else:
                        print("\tNext approach pose metadata missing; next approach will use precomputed segment.")

                probe_sequence = robot.plan_joint_trajectory_sequence(
                    sequence_waypoints,
                    sequence_durations,
                    show_progress=False,
                )
                execution_sequence = execution_sequence + probe_sequence

            robot.execute_sequence(
                execution_sequence,
                visualize_before_execution=False,
                settle_time_between_trajectories=0.0,
            )
            time.sleep(SETTLE_SEC)

        print("\nReturning home...")
        robot.execute_sequence(
            [planned_segments[idx] for idx in return_home_indices],
            visualize_before_execution=False,
            settle_time_between_trajectories=0.0,
        )
        time.sleep(SETTLE_SEC)
    finally:
        robot.shutdown()
        if "planner_thread" in locals():
            planner_thread.join(timeout=0.1)

    print("Done.")


if __name__ == "__main__":
    main()
