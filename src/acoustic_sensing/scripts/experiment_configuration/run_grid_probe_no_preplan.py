#!/usr/bin/env python3
"""Probe the experiment grid live, one location at a time, with no full-sequence preplanning."""

import hashlib
import pickle
import threading
import time

import numpy as np
import serial

from arm_client import CONFIG_DIR
from arm_client.robot import Pose, Robot

from common import (
    acquire_rf_burst,
    APPROACH_LIFT_HEIGHT,
    BASE_ORI,
    BAUD_RATE,
    get_landmark_home_z,
    get_probe_surface_z,
    load_grid_and_landmarks,
    precompute_probe_geometry,
    PROBE_DEPTH,
    results_root,
    SERIAL_PORT,
    SERIAL_TIMEOUT_SEC,
    SETTLE_SEC,
    Z_OFFSET,
)


MOVE_SPEED = 0.03
PROBE_SPEED = 0.01
ROBOT_SAMPLE_PERIOD = 0.01  # seconds between robot state samples during plunge


def _rf_burst_summary(rf_frames: list) -> str:
    frame_shapes = []
    buf = bytearray()
    for frame in rf_frames:
        frame_shapes.append(tuple(len(ch) for ch in frame))
        for ch in frame:
            buf += np.array(ch, dtype=np.int32).tobytes()
    digest = hashlib.sha256(buf).hexdigest()[:16]
    shape_text = ", ".join(str(s) for s in frame_shapes[:3])
    if len(frame_shapes) > 3:
        shape_text += ", ..."
    return f"sha256={digest} frame_shapes=[{shape_text}]"


def _move_to_pose(robot: Robot, pose: Pose, speed: float) -> None:
    robot.move_to(pose=pose, speed=speed)
    time.sleep(SETTLE_SEC)


def _move_and_record(
    robot: Robot, pose: Pose, speed: float
) -> tuple[list[float], list[np.ndarray], list[np.ndarray], list[np.ndarray]]:
    """Move to pose and sample EE state throughout. Returns (ts, positions, orientations, forces)."""
    ts: list[float] = []
    positions: list[np.ndarray] = []
    orientations: list[np.ndarray] = []
    forces: list[np.ndarray] = []
    stop_event = threading.Event()

    def _sampler() -> None:
        t0 = time.perf_counter()
        while not stop_event.is_set():
            ee_pose = robot.end_effector_pose
            ee_force = robot.end_effector_wrench["force"]
            ts.append(time.perf_counter() - t0)
            positions.append(ee_pose.position.copy())
            orientations.append(ee_pose.orientation.as_quat())
            forces.append(ee_force.copy())
            time.sleep(ROBOT_SAMPLE_PERIOD)

    sampler_thread = threading.Thread(target=_sampler, daemon=True)
    sampler_thread.start()
    robot.move_to(pose=pose, speed=speed)
    stop_event.set()
    sampler_thread.join()
    time.sleep(SETTLE_SEC)
    return ts, positions, orientations, forces


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

    exp_dict = {
        "set_name": set_name,
        "landmarks": landmarks,
        "z_offset": Z_OFFSET,
        "probe_depth": PROBE_DEPTH,
        "grid_positions": [],
        "rf_data": [],
        # Per location, per depth: {ts, positions, orientations, forces}
        "robot_data": [],
    }

    robot = Robot(namespace="fr3")
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=SERIAL_TIMEOUT_SEC)
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

        results_dir = results_root()
        results_dir.mkdir(parents=True, exist_ok=True)
        n_total = len(probe_geometries)
        base_filename = f"{n_total}_grid_{set_name.upper()}_RF"
        counter = 0
        out_path = results_dir / f"{base_filename}_{counter:02d}.pkl"
        while out_path.exists():
            counter += 1
            out_path = results_dir / f"{base_filename}_{counter:02d}.pkl"
        print(f"Results will be saved incrementally to: {out_path}")

        # Write file header (metadata only, no location data yet)
        with open(out_path, "wb") as f:
            pickle.dump({
                "__header__": True,
                "set_name": set_name,
                "landmarks": landmarks,
                "z_offset": Z_OFFSET,
                "probe_depth": PROBE_DEPTH,
            }, f)

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

            location_rf: list[list] = []
            location_robot: list[dict] = []
            location_start = time.time()
            teensy_error = False
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
                print(f"\tDepth {depth_idx}/{len(PROBE_DEPTH)}: {depth * 1000.0:.2f} mm")
                ts, positions, orientations, forces = _move_and_record(robot, plunge_pose, PROBE_SPEED)
                print("\t\tAcquiring RF burst...")
                try:
                    rf_frames = acquire_rf_burst(ser, debug=True)
                    print(f"\t\tRF burst complete — {len(rf_frames)} frames collected.")
                    print(f"\t\tRF burst summary: {_rf_burst_summary(rf_frames)}")
                    location_rf.append(rf_frames)
                except RuntimeError as e:
                    print(f"\t\tTEENSY ERROR at depth {depth_idx}: {e} — skipping location.")
                    teensy_error = True
                    break
                location_robot.append({
                    "ts": ts,
                    "positions": positions,
                    "orientations": orientations,
                    "forces": forces,
                })
                _move_to_pose(robot, approach_pose, PROBE_SPEED)

            elapsed = time.time() - location_start
            if teensy_error:
                print(f"\tLocation {idx} skipped due to Teensy error ({elapsed:.1f} s elapsed).")
                _move_to_pose(robot, approach_pose, PROBE_SPEED)
                continue

            exp_dict["grid_positions"].append(geometry["gripper_xy"].tolist())
            exp_dict["rf_data"].append(location_rf)
            exp_dict["robot_data"].append(location_robot)
            with open(out_path, "ab") as f:
                pickle.dump({
                    "grid_position": geometry["gripper_xy"].tolist(),
                    "rf_data": location_rf,
                    "robot_data": location_robot,
                }, f)
            print(f"\tSaved ({idx}/{len(probe_geometries)} locations) → {out_path} [{elapsed:.1f} s]")

        print("\nReturning to landmark home pose...")
        _move_to_pose(robot, lifted_home_pose, MOVE_SPEED)
        print("Descending to landmark home pose...")
        _move_to_pose(robot, home_pose, PROBE_SPEED)
    finally:
        ser.close()
        robot.shutdown()

    print("Done.")


if __name__ == "__main__":
    main()
