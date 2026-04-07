#!/usr/bin/env python3
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
LINEAR_STEP_SIZE = 0.01
PROBE_WAYPOINTS = 21
PROBE_DEPTH = [0.0020]
PROBE_STEP_SIZE = 0.0005
STATE_SAMPLE_PERIOD = 0.01
PROBE_TIMEOUT_MARGIN = 1.0
PREPLAN_WORKERS = 12
APPROACH_LIFT_HEIGHT = 0.015

Z_OFFSET = 0.0
ACTUATOR_SURFACE_Z_OFFSET = -0.002
BASE_ORI = Rotation.from_euler("xyz", [180.0, 0.0, 0.0], degrees=True)

APPROACH_DURATION = 4.0
PLUNGE_DURATION = 2.0
RETRACT_DURATION = 2.0
RETURN_HOME_DURATION = 4.0
BRIDGE_DURATION = 1.0
BRIDGE_POSITION_THRESHOLD = 0.001
BRIDGE_JOINT_THRESHOLD = 0.05
BRIDGE_STEP_SIZE = 0.005
APPROACH_SKIP_THRESHOLD = 1e-6
POSE_CONTINUITY_POSITION_TOL = 1e-9
POSE_CONTINUITY_ANGLE_TOL = 1e-9

SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 3000000
SERIAL_TIMEOUT_SEC = 10
CMD_START = bytes([0x43])
CHANNEL_MARKERS = ("S0", "S1", "S2", "S3")


def config_root() -> Path:
    return Path(__file__).resolve().parent


def artifacts_root() -> Path:
    return config_root() / "artifacts"


def results_root() -> Path:
    return config_root() / "results"


def resolve_landmark_file() -> Path:
    root = results_root() / "grids"
    preferred = root / "landmarks.txt"
    fallback = root / "landmarks2.txt"
    if preferred.exists():
        return preferred
    if fallback.exists():
        return fallback
    raise FileNotFoundError(
        f"Landmark file not found. Expected one of: {preferred}, {fallback}"
    )


def print_progress(prefix: str, current: int, total: int) -> None:
    width = 30
    filled = int(width * current / max(total, 1))
    bar = "#" * filled + "-" * (width - filled)
    print(f"\r{prefix}: [{bar}] {current}/{total}", end="", flush=True)
    if current >= total:
        print()


def fetch_landmarks(landmark_file: Path, to_fetch: list[str]) -> dict[str, float]:
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


def load_grid_and_landmarks(set_name: str) -> tuple[dict[str, float], np.ndarray, np.ndarray]:
    root = results_root()
    landmark_file = resolve_landmark_file()
    grid_file = root / "grids" / "grids.pkl"

    if not grid_file.exists():
        raise FileNotFoundError(f"Grid file not found: {grid_file}")

    print(f"Using landmark file: {landmark_file}")
    landmarks = fetch_landmarks(landmark_file, ["x", "y", "z"])
    with open(grid_file, "rb") as f:
        grids = pickle.load(f)

    return (
        landmarks,
        np.asarray(grids["WORLD_FRAME"][set_name], dtype=float),
        np.asarray(grids["GRIPPER_FRAME"][set_name], dtype=float),
    )


def get_landmark_home_z(landmarks: dict[str, float]) -> float:
    return float(landmarks["z"]) + Z_OFFSET


def get_probe_surface_z(landmarks: dict[str, float]) -> float:
    return get_landmark_home_z(landmarks) + ACTUATOR_SURFACE_Z_OFFSET


def generate_smooth_linear_waypoints(
    start_pose: Pose,
    end_pose: Pose,
    num_waypoints: int,
) -> list[CartesianWaypoint]:
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


def get_adaptive_waypoint_count(
    start_pose: Pose,
    end_pose: Pose,
    step_size: float = LINEAR_STEP_SIZE,
    min_waypoints: int = 3,
) -> int:
    distance = float(np.linalg.norm(end_pose.position - start_pose.position))
    return max(min_waypoints, int(np.ceil(distance / step_size)) + 1)


def _pose_delta(
    start_pose: Pose,
    end_pose: Pose,
) -> tuple[float, float]:
    position_error = float(np.linalg.norm(end_pose.position - start_pose.position))
    rotation_delta = start_pose.orientation.inv() * end_pose.orientation
    angle_error = float(np.linalg.norm(rotation_delta.as_rotvec()))
    return position_error, angle_error


def build_lifted_transition_waypoints(
    start_pose: Pose,
    end_pose: Pose,
    total_duration: float,
    lift_height: float = APPROACH_LIFT_HEIGHT,
) -> tuple[list[list[CartesianWaypoint]], list[float]]:
    lifted_start = Pose(
        np.array(
            [start_pose.position[0], start_pose.position[1], start_pose.position[2] + lift_height],
            dtype=float,
        ),
        start_pose.orientation,
    )
    lifted_end = Pose(
        np.array(
            [end_pose.position[0], end_pose.position[1], end_pose.position[2] + lift_height],
            dtype=float,
        ),
        end_pose.orientation,
    )

    segment_specs = [
        (start_pose, lifted_start, PROBE_STEP_SIZE),
        (lifted_start, lifted_end, LINEAR_STEP_SIZE),
        (lifted_end, end_pose, PROBE_STEP_SIZE),
    ]
    segment_waypoints: list[list[CartesianWaypoint]] = []
    segment_lengths: list[float] = []

    for seg_start, seg_end, step_size in segment_specs:
        position_error, angle_error = _pose_delta(seg_start, seg_end)
        if (
            position_error <= APPROACH_SKIP_THRESHOLD
            and angle_error <= POSE_CONTINUITY_ANGLE_TOL
        ):
            continue
        segment_waypoints.append(
            generate_smooth_linear_waypoints(
                seg_start,
                seg_end,
                get_adaptive_waypoint_count(seg_start, seg_end, step_size=step_size),
            )
        )
        segment_lengths.append(max(position_error, 1e-9))

    if not segment_waypoints:
        return [], []

    total_length = float(sum(segment_lengths))
    segment_durations = [
        total_duration * (segment_length / total_length) for segment_length in segment_lengths
    ]
    return segment_waypoints, segment_durations


def plan_linear_trajectory(
    robot: Robot,
    start_pose: Pose,
    end_pose: Pose,
    duration: float,
    num_waypoints: int,
    initial_joint_config: np.ndarray | None = None,
) -> PlannedJointTrajectory:
    return robot.plan_joint_trajectory(
        waypoints=generate_smooth_linear_waypoints(start_pose, end_pose, num_waypoints),
        duration=duration,
        visualize=False,
        show_progress=False,
        initial_joint_config=initial_joint_config,
    )


def _build_probe_geometry(
    world_xy: np.ndarray,
    gripper_xy: np.ndarray,
    z_surface: float,
) -> dict:
    approach_position = np.array([world_xy[0], world_xy[1], z_surface], dtype=float)
    approach_pose = Pose(approach_position, BASE_ORI)
    return {
        "gripper_xy": np.array(gripper_xy, dtype=float),
        "approach_pose": approach_pose,
    }


def precompute_probe_geometry(
    grid_xy_world: np.ndarray,
    grid_xy_gripper: np.ndarray,
    z_surface: float,
) -> list[dict]:
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
) -> tuple[list[list[CartesianWaypoint]], list[float], list[dict], list[int]]:
    waypoints_list: list[list[CartesianWaypoint]] = []
    durations: list[float] = []
    probe_steps: list[dict] = []
    nominal_pose = start_pose.copy()

    for idx, geometry in enumerate(probe_geometries):
        approach_distance = float(
            np.linalg.norm(geometry["approach_pose"].position - nominal_pose.position)
        )
        approach_indices: list[int] = []

        if approach_distance > APPROACH_SKIP_THRESHOLD:
            if idx == 0:
                approach_segments, approach_durations = build_lifted_transition_waypoints(
                    nominal_pose,
                    geometry["approach_pose"],
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
                    geometry["approach_pose"],
                )
                approach_indices = [len(waypoints_list)]
                waypoints_list.append(
                    generate_smooth_linear_waypoints(
                        nominal_pose,
                        geometry["approach_pose"],
                        approach_waypoint_count,
                    )
                )
                durations.append(APPROACH_DURATION)

        probe_steps.append(
            {
                "gripper_xy": geometry["gripper_xy"],
                "approach_idx": approach_indices[0] if approach_indices else None,
                "approach_indices": approach_indices,
                "approach_position": np.asarray(geometry["approach_pose"].position, dtype=float),
                "approach_orientation_quat": geometry["approach_pose"].orientation.as_quat(),
            }
        )
        nominal_pose = geometry["approach_pose"]

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


def verify_pose_continuity(
    waypoints_list: list[list[CartesianWaypoint]],
) -> None:
    for idx in range(len(waypoints_list) - 1):
        end_wp = waypoints_list[idx][-1]
        start_wp = waypoints_list[idx + 1][0]
        pos_error = float(np.linalg.norm(end_wp.position - start_wp.position))
        rot_delta = end_wp.orientation.inv() * start_wp.orientation
        angle_error = float(np.linalg.norm(rot_delta.as_rotvec()))

        if pos_error > POSE_CONTINUITY_POSITION_TOL or angle_error > POSE_CONTINUITY_ANGLE_TOL:
            raise ValueError(
                "Pose continuity check failed between segments "
                f"{idx} and {idx + 1}: position error={pos_error:.3e} m, "
                f"angle error={angle_error:.3e} rad"
            )


def get_approach_indices(step: dict) -> list[int]:
    if "approach_indices" in step:
        return [int(idx) for idx in step["approach_indices"]]
    if step.get("approach_idx") is None:
        return []
    return [int(step["approach_idx"])]


def get_return_home_indices(payload: dict, planned_segments: list[PlannedJointTrajectory]) -> list[int]:
    if "return_home_indices" in payload:
        return [int(idx) for idx in payload["return_home_indices"]]
    if not planned_segments:
        return []
    return [len(planned_segments) - 1]


def get_startup_transition_indices(payload: dict) -> list[int]:
    return [int(idx) for idx in payload.get("startup_transition_indices", [])]


def plan_sequence_with_progress(
    robot: Robot,
    waypoints_list: list[list[CartesianWaypoint]],
    durations: list[float],
    initial_joint_config: np.ndarray,
) -> list[PlannedJointTrajectory]:
    if len(waypoints_list) != len(durations):
        raise ValueError("waypoints_list and durations must have the same length")
    planned_segments: list[PlannedJointTrajectory] = []
    seed = np.array(initial_joint_config, dtype=float)
    total = len(waypoints_list)

    for i, (waypoints, duration) in enumerate(zip(waypoints_list, durations), start=1):
        traj = robot.plan_joint_trajectory(
            waypoints=waypoints,
            duration=duration,
            visualize=False,
            show_progress=False,
            initial_joint_config=seed,
        )
        planned_segments.append(traj)
        seed = traj.joint_positions[-1]
        print_progress("Trajectory planning", i, total)

    return planned_segments


def serialize_trajectory(traj: PlannedJointTrajectory) -> dict:
    return {
        "joint_names": list(traj.joint_names),
        "time_from_start": [float(t) for t in traj.time_from_start],
        "joint_positions": np.asarray(traj.joint_positions, dtype=float),
    }


def deserialize_trajectory(data: dict) -> PlannedJointTrajectory:
    return PlannedJointTrajectory(
        joint_names=list(data["joint_names"]),
        time_from_start=[float(t) for t in data["time_from_start"]],
        joint_positions=np.asarray(data["joint_positions"], dtype=float),
    )


def acquire_rf_burst(ser: serial.Serial) -> list:
    """Send 67, read frames until Teensy returns 69, then return collected frames."""
    frames: list = []
    current_channel = None
    current_samples: list[int] = []
    current_frame: dict[str, list[int]] = {}

    ser.reset_input_buffer()
    ser.write(CMD_START)

    while True:
        try:
            raw = ser.readline()
        except serial.SerialException as exc:
            raise RuntimeError("Serial connection to Teensy failed during RF acquisition") from exc

        if raw == b"":
            continue

        line = raw.decode("ascii", errors="ignore").strip()

        # Firmware returns one byte 0x45 ('E', 69) after the fixed burst completes.
        if raw == bytes([0x45]) or line == "E":
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

    return frames


def execute_sequence_and_record(
    robot: Robot,
    trajectories: list[PlannedJointTrajectory],
    timeout_margin: float,
) -> tuple[list[float], list[Pose], list[dict]]:
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
