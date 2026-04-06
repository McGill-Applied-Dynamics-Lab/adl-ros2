#!/usr/bin/env python3
"""180-degree waveguide probing with full 4RF capture using planned joint trajectories.

This mirrors the behavior of `src/acoustic_sensing/scripts/probing_waveguide_gripper_4rf_6769.py`,
but it avoids `fr3_pose_controller`. All robot motion is planned ahead in joint space and
executed through `joint_trajectory_controller`.
"""

from pathlib import Path
import pickle
import threading
import time

import numpy as np
import serial
from scipy.spatial.transform import Rotation, Slerp

from arm_client.robot import Pose, Robot
from arm_client.planning.types import CartesianWaypoint, PlannedJointTrajectory


SETTLE_SEC = 2.0
LINEAR_WAYPOINTS = 12
PROBE_WAYPOINTS = 21
STATE_SAMPLE_PERIOD = 0.01
PROBE_TIMEOUT_MARGIN = 1.0

Z_OFFSET = 0.0250
PROBE_DEPTH = 0.0090
BASE_ORI = Rotation.from_euler("xyz", [180.0, 0.0, 0.0], degrees=True)

TO_HOME_DURATION = 6.0
APPROACH_DURATION = 4.0
PLUNGE_DURATION = 2.0
RETRACT_DURATION = 2.0
RETURN_HOME_DURATION = 4.0

SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 3000000
SERIAL_TIMEOUT_SEC = 10
CMD_START = bytes([0x43])
CMD_STOP = bytes([0x45])
CHANNEL_MARKERS = ("S0", "S1", "S2", "S3")


def fetch_landmarks(landmark_file: Path, to_fetch: list[str]) -> dict[str, float]:
    """Fetch landmark coordinates from file."""
    values: dict[str, float] = {}
    with open(landmark_file, "r", encoding="utf-8") as f:
        for line in f:
            if ":" not in line:
                continue
            key, val = line.strip().split(":", maxsplit=1)
            key = key.strip()
            if key in to_fetch:
                values[key] = float(val.strip())
    return values


def _stream_reader(ser: serial.Serial, frames: list, stop_event: threading.Event) -> None:
    """Read 4RF frames until the Teensy emits STREAM_END."""
    current_channel = None
    current_samples: list[int] = []
    current_frame: dict[str, list[int]] = {}

    while not stop_event.is_set():
        try:
            raw = ser.readline()
        except serial.SerialException:
            break
        if raw == b"":
            continue

        line = raw.decode("ascii", errors="ignore").strip()

        if line == "STREAM_END":
            if len(current_frame) == len(CHANNEL_MARKERS):
                frames.append([current_frame[m] for m in CHANNEL_MARKERS])
            break

        if line in CHANNEL_MARKERS:
            current_channel = line
            current_samples = []
        elif line == "T":
            if current_channel is not None:
                current_frame[current_channel] = current_samples
                current_channel = None
                if len(current_frame) == len(CHANNEL_MARKERS):
                    frames.append([current_frame[m] for m in CHANNEL_MARKERS])
                    current_frame = {}
        else:
            try:
                current_samples.append(int(line))
            except ValueError:
                pass


def rf_stream_start(ser: serial.Serial) -> tuple[list, threading.Event, threading.Thread]:
    """Send the start-stream command and begin collecting RF frames in the background."""
    frames: list = []
    stop_event = threading.Event()
    ser.reset_input_buffer()
    ser.write(CMD_START)
    thread = threading.Thread(target=_stream_reader, args=(ser, frames, stop_event), daemon=True)
    thread.start()
    return frames, stop_event, thread


def rf_stream_stop(ser: serial.Serial, stop_event: threading.Event, thread: threading.Thread) -> None:
    """Send the stop-stream command and wait for the reader thread to flush."""
    ser.write(CMD_STOP)
    thread.join(timeout=SERIAL_TIMEOUT_SEC)
    stop_event.set()


def generate_smooth_linear_waypoints(
    start_pose: Pose,
    end_pose: Pose,
    num_waypoints: int,
) -> list[CartesianWaypoint]:
    """Generate straight-line Cartesian waypoints with eased start/stop."""
    if num_waypoints < 2:
        raise ValueError("num_waypoints must be >= 2")

    slerp = Slerp(
        [0.0, 1.0],
        Rotation.concatenate([start_pose.orientation, end_pose.orientation]),
    )

    waypoints: list[CartesianWaypoint] = []
    for i in range(num_waypoints):
        s = i / (num_waypoints - 1)
        alpha = 0.5 - 0.5 * np.cos(np.pi * s)
        position = (1.0 - alpha) * start_pose.position + alpha * end_pose.position
        orientation = slerp(alpha)
        waypoints.append(
            CartesianWaypoint(
                position=np.array(position, dtype=float),
                orientation=orientation,
                s=float(alpha),
            )
        )
    return waypoints


def merge_trajectories(trajectories: list[PlannedJointTrajectory]) -> PlannedJointTrajectory:
    """Concatenate planned joint trajectories into one continuous trajectory."""
    if len(trajectories) == 0:
        raise ValueError("At least one trajectory is required")

    joint_names = trajectories[0].joint_names
    all_times: list[float] = []
    all_positions: list[np.ndarray] = []
    time_offset = 0.0

    for i, trajectory in enumerate(trajectories):
        if trajectory.joint_names != joint_names:
            raise ValueError("All trajectories must use the same joint names")
        for j, (t, q) in enumerate(zip(trajectory.time_from_start, trajectory.joint_positions)):
            if i > 0 and j == 0:
                continue
            all_times.append(float(t) + time_offset)
            all_positions.append(np.array(q, dtype=float))
        time_offset += float(trajectory.time_from_start[-1])

    return PlannedJointTrajectory(
        joint_names=joint_names,
        time_from_start=all_times,
        joint_positions=np.array(all_positions, dtype=float),
    )


def plan_linear_trajectory(
    robot: Robot,
    start_pose: Pose,
    end_pose: Pose,
    duration: float,
    num_waypoints: int,
) -> PlannedJointTrajectory:
    """Plan one smooth Cartesian segment in joint space."""
    return robot.plan_joint_trajectory(
        waypoints=generate_smooth_linear_waypoints(start_pose, end_pose, num_waypoints),
        duration=duration,
        visualize=False,
        show_progress=True,
    )


def execute_trajectory_and_record(
    robot: Robot,
    trajectory: PlannedJointTrajectory,
    timeout_margin: float,
) -> tuple[list[float], list[Pose], list[dict]]:
    """Execute a joint trajectory non-blocking while sampling robot state."""
    if len(trajectory.time_from_start) == 0:
        raise ValueError("Trajectory is empty")

    ts: list[float] = []
    ee_poses: list[Pose] = []
    ee_forces: list[dict] = []

    robot.follow_joint_trajectory(trajectory, blocking=False)

    start_time = time.perf_counter()
    timeout = float(trajectory.time_from_start[-1]) + timeout_margin
    while True:
        elapsed = time.perf_counter() - start_time
        if elapsed > timeout:
            break

        ee_poses.append(robot.end_effector_pose.copy())
        ee_forces.append(robot.end_effector_wrench.copy())
        ts.append(elapsed)
        time.sleep(STATE_SAMPLE_PERIOD)

    return ts, ee_poses, ee_forces


def execute_sequence_and_record(
    robot: Robot,
    trajectories: list[PlannedJointTrajectory],
    timeout_margin: float,
) -> tuple[list[float], list[Pose], list[dict]]:
    """Execute a trajectory sequence while sampling robot state."""
    if len(trajectories) == 0:
        raise ValueError("Trajectory sequence is empty")

    ts: list[float] = []
    ee_poses: list[Pose] = []
    ee_forces: list[dict] = []
    error: list[Exception] = []

    def _runner():
        try:
            robot.execute_sequence(
                trajectories,
                visualize_before_execution=False,
                settle_time_between_trajectories=0.0,
            )
        except Exception as exc:
            error.append(exc)

    worker = threading.Thread(target=_runner, daemon=True)
    worker.start()

    start_time = time.perf_counter()
    timeout = sum(float(traj.time_from_start[-1]) for traj in trajectories) + timeout_margin

    while worker.is_alive():
        elapsed = time.perf_counter() - start_time
        if elapsed > timeout:
            break

        ee_poses.append(robot.end_effector_pose.copy())
        ee_forces.append(robot.end_effector_wrench.copy())
        ts.append(elapsed)
        time.sleep(STATE_SAMPLE_PERIOD)

    worker.join(timeout=0.1)
    if error:
        raise error[0]

    return ts, ee_poses, ee_forces


def main() -> None:
    repo_root = Path(__file__).resolve().parents[1]
    acoustic_root = repo_root / "src" / "acoustic_sensing" / "scripts"
    landmark_file = acoustic_root / "results" / "grids" / "landmarks.txt"
    grid_file = acoustic_root / "results" / "grids" / "grids.pkl"

    if not landmark_file.exists():
        raise FileNotFoundError(
            f"Landmark file not found: {landmark_file}. Please run the landmark detection workflow first."
        )
    if not grid_file.exists():
        raise FileNotFoundError(
            f"Grid file not found: {grid_file}. Please run the grid generation workflow first."
        )

    landmarks = fetch_landmarks(landmark_file, ["x", "y", "z"])
    with open(grid_file, "rb") as f:
        grids = pickle.load(f)

    set_name = input("Select grid set (train/test) [default: test]: ").strip().lower()
    if set_name not in ["train", "test"]:
        set_name = "test"
        print(f"Invalid selection. Using default: {set_name}")

    grid_xy_world = np.asarray(grids["WORLD_FRAME"][set_name], dtype=float)
    grid_xy_gripper = np.asarray(grids["GRIPPER_FRAME"][set_name], dtype=float)

    z_surface = landmarks["z"] + Z_OFFSET
    home_position = np.array([landmarks["x"], landmarks["y"], z_surface], dtype=float)
    home_pose = Pose(home_position, BASE_ORI)

    exp_dict = {
        "ts": [],
        "grid_positions": [],
        "ee_poses": [],
        "ee_forces": [],
        "rf_data": [],
        "set_name": set_name,
        "landmarks": landmarks,
        "z_offset": Z_OFFSET,
    }

    robot = Robot(namespace="fr3")
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=SERIAL_TIMEOUT_SEC)

    try:
        robot.wait_until_ready()
        print("Switching to joint trajectory controller...")
        robot.controller_switcher_client.switch_controller("joint_trajectory_controller")

        print("Planning initial move to home pose...")
        initial_home_traj = plan_linear_trajectory(
            robot,
            robot.end_effector_pose.copy(),
            home_pose,
            TO_HOME_DURATION,
            LINEAR_WAYPOINTS,
        )
        robot.follow_joint_trajectory(initial_home_traj, blocking=True)
        time.sleep(SETTLE_SEC)

        input("Press Enter to start probing...")
        for i, (world_xy, gripper_xy) in enumerate(zip(grid_xy_world, grid_xy_gripper)):
            print(f"\nProbe {i + 1}/{len(grid_xy_world)}")
            exp_dict["grid_positions"].append([float(gripper_xy[0]), float(gripper_xy[1])])

            approach_position = np.array([world_xy[0], world_xy[1], z_surface], dtype=float)
            approach_pose = Pose(approach_position, BASE_ORI)
            plunge_pose = Pose(
                np.array(
                    [
                        approach_position[0],
                        approach_position[1],
                        approach_position[2] - (Z_OFFSET + PROBE_DEPTH),
                    ],
                    dtype=float,
                ),
                BASE_ORI,
            )

            print("\tPlanning move to approach pose...")
            approach_traj = plan_linear_trajectory(
                robot,
                robot.end_effector_pose.copy(),
                approach_pose,
                APPROACH_DURATION,
                LINEAR_WAYPOINTS,
            )

            print("\tExecuting move to approach pose...")
            robot.follow_joint_trajectory(approach_traj, blocking=True)
            time.sleep(SETTLE_SEC)

            print("\tPlanning plunge/retract sequence...")
            probe_start_pose = robot.end_effector_pose.copy()
            plunge_waypoints = generate_smooth_linear_waypoints(
                probe_start_pose,
                plunge_pose,
                PROBE_WAYPOINTS,
            )
            retract_waypoints = generate_smooth_linear_waypoints(
                plunge_pose,
                approach_pose,
                PROBE_WAYPOINTS,
            )
            plunge_traj, retract_traj = robot.plan_joint_trajectory_sequence(
                [plunge_waypoints, retract_waypoints],
                [PLUNGE_DURATION, RETRACT_DURATION],
                show_progress=True,
            )
            print("\tStarting RF stream...")
            rf_frames, stop_event, rf_thread = rf_stream_start(ser)

            print("\tExecuting planned probe sequence...")
            ts, ee_poses, ee_forces = execute_sequence_and_record(
                robot,
                [plunge_traj, retract_traj],
                timeout_margin=PROBE_TIMEOUT_MARGIN,
            )

            rf_stream_stop(ser, stop_event, rf_thread)
            print(f"\tRF stream stopped — {len(rf_frames)} frames collected.")

            ee_positions = [pose.position for pose in ee_poses]
            ee_orientations = [pose.orientation.as_quat() for pose in ee_poses]

            exp_dict["ts"].append(ts)
            exp_dict["ee_poses"].append(
                {
                    "positions": ee_positions,
                    "orientations": ee_orientations,
                }
            )
            exp_dict["ee_forces"].append(ee_forces)
            exp_dict["rf_data"].append(rf_frames)
            print("\tProbe complete.")

        print("\nReturning home...")
        return_home_traj = plan_linear_trajectory(
            robot,
            robot.end_effector_pose.copy(),
            home_pose,
            RETURN_HOME_DURATION,
            LINEAR_WAYPOINTS,
        )
        robot.follow_joint_trajectory(return_home_traj, blocking=True)
        time.sleep(SETTLE_SEC)

    finally:
        ser.close()
        robot.shutdown()

    results_dir = acoustic_root / "results"
    results_dir.mkdir(parents=True, exist_ok=True)

    base_filename = f"{len(grid_xy_world)}_grid_{set_name.upper()}_4RF_joint_sequence"
    counter = 0
    filename = f"{base_filename}_{counter:02d}.pkl"
    full_path = results_dir / filename

    while full_path.exists():
        counter += 1
        filename = f"{base_filename}_{counter:02d}.pkl"
        full_path = results_dir / filename

    with open(full_path, "wb") as f:
        pickle.dump(exp_dict, f)

    print("Done.")
    print(f"Results saved to: {full_path}")


if __name__ == "__main__":
    main()
