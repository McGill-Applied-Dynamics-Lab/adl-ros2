#!/usr/bin/env python3
"""180-degree waveguide probing with full 4RF capture using planned joint trajectories.

This mirrors the behavior of `src/acoustic_sensing/scripts/testing/probing_waveguide_gripper_4rf_6769.py`,
but it avoids `fr3_pose_controller`. All robot motion is planned ahead in joint space and
executed through `joint_trajectory_controller`.
"""

from pathlib import Path
import pickle
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from concurrent.futures import as_completed

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
PREPLAN_WORKERS = 8

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


def print_progress(prefix: str, current: int, total: int) -> None:
    """Print a simple single-line progress bar."""
    width = 30
    filled = int(width * current / max(total, 1))
    bar = "#" * filled + "-" * (width - filled)
    print(f"\r{prefix}: [{bar}] {current}/{total}", end="", flush=True)
    if current >= total:
        print()


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
        show_progress=False,
    )


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


def _build_probe_geometry(
    world_xy: np.ndarray,
    gripper_xy: np.ndarray,
    z_surface: float,
) -> dict:
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
    plunge_waypoints = generate_smooth_linear_waypoints(
        approach_pose,
        plunge_pose,
        PROBE_WAYPOINTS,
    )
    retract_waypoints = generate_smooth_linear_waypoints(
        plunge_pose,
        approach_pose,
        PROBE_WAYPOINTS,
    )
    return {
        "gripper_xy": np.array(gripper_xy, dtype=float),
        "approach_pose": approach_pose,
        "plunge_waypoints": plunge_waypoints,
        "retract_waypoints": retract_waypoints,
    }


def precompute_probe_geometry(
    grid_xy_world: np.ndarray,
    grid_xy_gripper: np.ndarray,
    z_surface: float,
) -> list[dict]:
    """Prepare per-probe poses/waypoints in parallel on the PC."""
    total = len(grid_xy_world)
    results: list[dict | None] = [None] * total
    print(f"Precompute workers: {PREPLAN_WORKERS} threads")
    with ThreadPoolExecutor(max_workers=PREPLAN_WORKERS) as executor:
        future_to_idx = {
            executor.submit(_build_probe_geometry, world_xy, gripper_xy, z_surface): i
            for i, (world_xy, gripper_xy) in enumerate(zip(grid_xy_world, grid_xy_gripper))
        }
        completed = 0
        for future in as_completed(future_to_idx):
            idx = future_to_idx[future]
            results[idx] = future.result()
            completed += 1
            print_progress("Geometry precompute", completed, total)
    return [result for result in results if result is not None]


def build_sequence_waypoints(
    probe_geometries: list[dict],
    start_pose: Pose,
) -> tuple[list[list[CartesianWaypoint]], list[float], list[dict], list[CartesianWaypoint]]:
    """Build the full chained waypoint sequence for the whole run."""
    waypoints_list: list[list[CartesianWaypoint]] = []
    durations: list[float] = []
    probe_steps: list[dict] = []
    nominal_pose = start_pose.copy()

    for geometry in probe_geometries:
        approach_waypoints = generate_smooth_linear_waypoints(
            nominal_pose,
            geometry["approach_pose"],
            LINEAR_WAYPOINTS,
        )
        approach_idx = len(waypoints_list)
        plunge_idx = approach_idx + 1
        retract_idx = approach_idx + 2

        waypoints_list.extend(
            [
                approach_waypoints,
                geometry["plunge_waypoints"],
                geometry["retract_waypoints"],
            ]
        )
        durations.extend([APPROACH_DURATION, PLUNGE_DURATION, RETRACT_DURATION])
        probe_steps.append(
            {
                "gripper_xy": geometry["gripper_xy"],
                "approach_idx": approach_idx,
                "plunge_idx": plunge_idx,
                "retract_idx": retract_idx,
            }
        )
        nominal_pose = geometry["approach_pose"]

    return_home_waypoints = generate_smooth_linear_waypoints(
        nominal_pose,
        start_pose,
        LINEAR_WAYPOINTS,
    )
    return waypoints_list, durations, probe_steps, return_home_waypoints


def plan_sequence_with_progress(
    robot: Robot,
    waypoints_list: list[list[CartesianWaypoint]],
    durations: list[float],
) -> list[PlannedJointTrajectory]:
    """Plan a chained sequence segment-by-segment with live progress updates."""
    if len(waypoints_list) != len(durations):
        raise ValueError("waypoints_list and durations must have the same length")

    planned_segments: list[PlannedJointTrajectory] = []
    seed = robot.q
    total = len(waypoints_list)

    for i, (waypoints, duration) in enumerate(zip(waypoints_list, durations)):
        traj = robot.plan_joint_trajectory(
            waypoints=waypoints,
            duration=duration,
            visualize=False,
            show_progress=False,
            initial_joint_config=seed,
        )
        planned_segments.append(traj)
        seed = traj.joint_positions[-1]
        print_progress("Trajectory planning", i + 1, total)

    return planned_segments


def main() -> None:
    project_root = Path(__file__).resolve().parent
    landmark_file = project_root / "results" / "grids" / "landmarks.txt"
    grid_file = project_root / "results" / "grids" / "grids.pkl"

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
        time.sleep(0.1)

        start_pose = robot.end_effector_pose.copy()
        print(f"Captured startup pose for planning: {start_pose.position.tolist()}")
        print(
            "Assuming the robot was already homed and oriented before launching this script."
        )

        print(f"Precomputing geometry for {len(grid_xy_world)} probes...")
        probe_geometries = precompute_probe_geometry(
            grid_xy_world,
            grid_xy_gripper,
            z_surface,
        )

        print("Building full waypoint sequence...")
        waypoints_list, durations, probe_steps, return_home_waypoints = build_sequence_waypoints(
            probe_geometries,
            start_pose,
        )
        waypoints_list.append(return_home_waypoints)
        durations.append(RETURN_HOME_DURATION)

        print(f"Planning full sequence with {len(waypoints_list)} segments...")
        planned_segments = plan_sequence_with_progress(
            robot,
            waypoints_list,
            durations,
        )

        return_home_traj = planned_segments[-1]

        input("Press Enter to start probing...")
        for i, probe_step in enumerate(probe_steps):
            print(f"\nProbe {i + 1}/{len(probe_steps)}")
            exp_dict["grid_positions"].append(
                [float(probe_step["gripper_xy"][0]), float(probe_step["gripper_xy"][1])]
            )

            print("\tExecuting move to approach pose...")
            robot.execute_sequence(
                [planned_segments[probe_step["approach_idx"]]],
                visualize_before_execution=False,
                settle_time_between_trajectories=0.0,
            )

            print("\tStarting RF stream...")
            rf_frames, stop_event, rf_thread = rf_stream_start(ser)

            print("\tExecuting planned probe sequence...")
            ts, ee_poses, ee_forces = execute_sequence_and_record(
                robot,
                [
                    planned_segments[probe_step["plunge_idx"]],
                    planned_segments[probe_step["retract_idx"]],
                ],
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
        robot.execute_sequence(
            [return_home_traj],
            visualize_before_execution=False,
            settle_time_between_trajectories=0.0,
        )

    finally:
        ser.close()
        robot.shutdown()

    results_dir = project_root / "results"
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
