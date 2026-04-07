#!/usr/bin/env python3
"""Run a saved 4RF probing sequence and only plan the startup bridge online."""

import pickle
import time

import numpy as np
import serial

from arm_client.robot import Pose, Robot

from common import (
    acquire_rf_burst,
    artifacts_root,
    BAUD_RATE,
    BRIDGE_DURATION,
    BRIDGE_JOINT_THRESHOLD,
    get_adaptive_waypoint_count,
    get_approach_indices,
    get_return_home_indices,
    get_startup_transition_indices,
    PROBE_DEPTH,
    PROBE_STEP_SIZE,
    PROBE_TIMEOUT_MARGIN,
    SERIAL_PORT,
    SERIAL_TIMEOUT_SEC,
    deserialize_trajectory,
    execute_sequence_and_record,
    generate_smooth_linear_waypoints,
    PLUNGE_DURATION,
    RETRACT_DURATION,
    SETTLE_SEC,
)


def main() -> None:
    set_name = input("Select precomputed grid set to run (train/test) [default: test]: ").strip().lower()
    if set_name not in ["train", "test"]:
        set_name = "test"
        print(f"Invalid selection. Using default: {set_name}")

    sequence_path = artifacts_root() / f"precomputed_{set_name.upper()}_4RF_joint_sequence.pkl"
    if not sequence_path.exists():
        raise FileNotFoundError(f"Precomputed sequence not found: {sequence_path}")

    with open(sequence_path, "rb") as f:
        payload = pickle.load(f)

    planned_segments = [deserialize_trajectory(item) for item in payload["planned_segments"]]
    probe_steps = payload["probe_steps"]
    assumed_start_joint_config = np.asarray(payload["assumed_start_joint_config"], dtype=float)
    startup_transition_indices = get_startup_transition_indices(payload)
    return_home_indices = get_return_home_indices(payload, planned_segments)
    sequence_start_joint_config = (
        np.asarray(planned_segments[0].joint_positions[0], dtype=float)
        if len(planned_segments) > 0
        else assumed_start_joint_config
    )
    saved_start_pose = np.asarray(payload["assumed_start_pose"]["position"], dtype=float)
    saved_landmark_home_pose = np.array(
        [
            payload["landmarks"]["x"],
            payload["landmarks"]["y"],
            payload["landmarks"]["z"] + payload["z_offset"],
        ],
        dtype=float,
    )

    exp_dict = {
        "ts": [],
        "grid_positions": [],
        "probe_depths": [],
        "ee_poses": [],
        "ee_forces": [],
        "rf_data": [],
        "set_name": payload["set_name"],
        "landmarks": payload["landmarks"],
        "z_offset": payload["z_offset"],
        "precomputed_sequence": str(sequence_path),
    }

    robot = Robot(namespace="fr3")
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=SERIAL_TIMEOUT_SEC)

    try:
        robot.wait_until_ready()
        robot.controller_switcher_client.switch_controller("joint_trajectory_controller")
        time.sleep(0.1)

        current_q = np.asarray(robot.q, dtype=float)
        joint_error = np.linalg.norm(current_q - sequence_start_joint_config)

        if joint_error > BRIDGE_JOINT_THRESHOLD:
            move_time = max(BRIDGE_DURATION, float(joint_error))
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

        print(f"Saved startup pose: {saved_start_pose.tolist()}")
        print(f"Saved landmark home pose: {saved_landmark_home_pose.tolist()}")
        print(f"Startup transition indices: {startup_transition_indices}")

        input("Press Enter to start probing...")
        if startup_transition_indices:
            print("Executing startup descent to landmark home pose...")
            robot.execute_sequence(
                [planned_segments[idx] for idx in startup_transition_indices],
                visualize_before_execution=False,
                settle_time_between_trajectories=0.0,
            )
            time.sleep(SETTLE_SEC)
        for i, probe_step in enumerate(probe_steps):
            print(f"\nLocation {i + 1}/{len(probe_steps)}")

            approach_indices = get_approach_indices(probe_step)
            if approach_indices:
                robot.execute_sequence(
                    [planned_segments[idx] for idx in approach_indices],
                    visualize_before_execution=False,
                    settle_time_between_trajectories=0.0,
                )
                time.sleep(SETTLE_SEC)
            else:
                print("\tApproach move skipped; already at assumed start pose.")

            approach_pose = robot.end_effector_pose.copy()
            approach_q = np.asarray(robot.q, dtype=float)

            for depth_idx, depth in enumerate(PROBE_DEPTH):
                print(
                    f"\tDepth {depth_idx + 1}/{len(PROBE_DEPTH)}: "
                    f"{depth * 1000.0:.2f} mm"
                )
                exp_dict["grid_positions"].append(
                    [float(probe_step["gripper_xy"][0]), float(probe_step["gripper_xy"][1])]
                )
                exp_dict["probe_depths"].append(float(depth))

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
                plunge_waypoints = generate_smooth_linear_waypoints(
                    approach_pose,
                    plunge_pose,
                    get_adaptive_waypoint_count(
                        approach_pose,
                        plunge_pose,
                        step_size=PROBE_STEP_SIZE,
                    ),
                )
                retract_waypoints = generate_smooth_linear_waypoints(
                    plunge_pose,
                    approach_pose,
                    get_adaptive_waypoint_count(
                        plunge_pose,
                        approach_pose,
                        step_size=PROBE_STEP_SIZE,
                    ),
                )
                plunge_traj, retract_traj = robot.plan_joint_trajectory_sequence(
                    [plunge_waypoints, retract_waypoints],
                    [PLUNGE_DURATION, RETRACT_DURATION],
                    show_progress=False,
                )

                print("\tExecuting plunge to minimum depth...")
                plunge_ts, plunge_poses, plunge_forces = execute_sequence_and_record(
                    robot,
                    [plunge_traj],
                    timeout_margin=PROBE_TIMEOUT_MARGIN,
                )

                print("\tStarting RF acquisition at minimum depth...")
                rf_start = time.perf_counter()
                rf_frames = acquire_rf_burst(ser)
                rf_wait = time.perf_counter() - rf_start
                print(
                    f"\tRF acquisition complete — {len(rf_frames)} frames "
                    f"in {rf_wait:.3f}s"
                )

                print("\tRetracting after RF completion...")
                retract_ts, retract_poses, retract_forces = execute_sequence_and_record(
                    robot,
                    [retract_traj],
                    timeout_margin=PROBE_TIMEOUT_MARGIN,
                )

                ts = plunge_ts + [t + plunge_ts[-1] + rf_wait for t in retract_ts]
                ee_poses = plunge_poses + retract_poses
                ee_forces = plunge_forces + retract_forces

                exp_dict["ts"].append(ts)
                exp_dict["ee_poses"].append(
                    {
                        "positions": [pose.position for pose in ee_poses],
                        "orientations": [pose.orientation.as_quat() for pose in ee_poses],
                    }
                )
                exp_dict["ee_forces"].append(ee_forces)
                exp_dict["rf_data"].append(rf_frames)

        print("\nReturning home...")
        robot.execute_sequence(
            [planned_segments[idx] for idx in return_home_indices],
            visualize_before_execution=False,
            settle_time_between_trajectories=0.0,
        )
    finally:
        ser.close()
        robot.shutdown()

    results_dir = artifacts_root()
    results_dir.mkdir(parents=True, exist_ok=True)
    counter = 0
    output_path = results_dir / f"{len(probe_steps)}_grid_{set_name.upper()}_4RF_precomputed_run_{counter:02d}.pkl"
    while output_path.exists():
        counter += 1
        output_path = results_dir / f"{len(probe_steps)}_grid_{set_name.upper()}_4RF_precomputed_run_{counter:02d}.pkl"

    with open(output_path, "wb") as f:
        pickle.dump(exp_dict, f)

    print("Done.")
    print(f"Results saved to: {output_path}")


if __name__ == "__main__":
    main()
