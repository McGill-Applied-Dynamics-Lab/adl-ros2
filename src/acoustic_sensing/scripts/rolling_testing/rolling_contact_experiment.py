#!/usr/bin/env python3
"""Rolling contact experiment: slide the 4-waveguide finray sensor along -Y while compressed.

Hardware setup
--------------
- FR3 robot with finray gripper in 90-degree custom adapter (orientation [180, 0, 0]).
- Roller of diameter ROLLER_DIAMETER_M mounted horizontally with its axis along robot X.
- Set CONTACT_POSITION_M in rolling_config.py to the XYZ of the cylinder surface at
  the top (φ=0). The approach pose is derived automatically as contact + APPROACH_HEIGHT_M
  above. If CONTACT_POSITION_M is None, jog the robot to the contact point first.

Motion sequence per trial
--------------------------
(1) approach_pose   ← CONTACT_POSITION_M + APPROACH_HEIGHT_M above, above cylinder
         │ descend straight down APPROACH_HEIGHT_M at APPROACH_SPEED
(2) contact_pose    ← sensor touching cylinder surface at φ = 0
         │ descend COMPRESS_DEPTH_M more (along world −Z at φ=0) at APPROACH_SPEED
(3) compressed_start_pose  ← sensor pre-loaded into cylinder surface
         │ [RF stream starts here]
         │ slide in -Y at constant Z/orientation at TRIAL_SPEED
(4) compressed_end_pose    ← end of slide, still pre-loaded
         │ [RF stream stops]
         │ decompress: move COMPRESS_DEPTH_M upward in world Z
(5) surface_end_pose       ← back on the surface at the final Y position
         │ return to approach_pose at RETURN_SPEED (fast, not recording)
(6) approach_pose   ← ready for next trial

Slide geometry
--------------
The script slides across the configured test length in world -Y:

    y_total = FINRAY_LENGTH_M * (ROLL_LENGTH_PERCENT / 100)

Then it linearly interpolates from the compressed start pose to that final -Y
offset while keeping:
- end-effector orientation fixed at [180, 0, 0]
- Z fixed at contact_z − COMPRESS_DEPTH_M

Output
------
- Pickle file in results/ with RF frames + timestamps per trial.
- PNG plots: mean contact location vs. time, and per-waveguide breakdown.

Notes
-----
- Motion is planned with chained IK and executed through `joint_trajectory_controller`.
- Adjust CONTACT_POSITION_M, ROLLER_DIAMETER_M, COMPRESS_DEPTH_M,
  FINRAY_LENGTH_M, ROLL_LENGTH_PERCENT, interpolation spacings, and ROLL_SPEEDS_M_S to match your setup.
"""

import pickle
import threading
import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import serial
from scipy.signal import hilbert
from scipy.spatial.transform import Rotation

from arm_client.planning.types import CartesianWaypoint
from arm_client.planning.types import PlannedJointTrajectory
from arm_client.robot import Pose, Robot
from arm_client.planning.waypoints import generate_linear_waypoints

from rolling_config import (
    APPROACH_HEIGHT_M,
    APPROACH_POINT_SPACING_M,
    APPROACH_SPEED_M_S,
    BAUD_RATE,
    BASE_ORI_EULER_DEG,
    COMPRESS_POINT_SPACING_M,
    COMPRESS_DEPTH_M,
    CONTACT_POSITION_M,
    CONTACT_POINT_SPACING_M,
    CONTROLLER_NAME,
    DECOMPRESS_POINT_SPACING_M,
    DRY_RUN,
    EE_SAMPLE_PERIOD_SEC,
    FINRAY_LENGTH_M,
    FIRST_WAYPOINT_MAX_JOINT_SPEED_RAD_S,
    IK_ORIENTATION_WEIGHT,
    IK_POSITION_WEIGHT,
    IK_SIMILARITY_WEIGHT,
    ROLL_LENGTH_PERCENT,
    ROLLER_DIAMETER_M,
    RETURN_POINT_SPACING_M,
    RETURN_SPEED_M_S,
    ROBOT_NAMESPACE,
    ROLL_SPEEDS_M_S,
    SERIAL_PORT,
    SERIAL_TIMEOUT_SEC,
    SETTLE_SEC,
    SLIDE_POINT_SPACING_M,
)

# Gripper base orientation: 90-degree custom adapter, face pointing down.
BASE_ORI = Rotation.from_euler("xyz", BASE_ORI_EULER_DEG, degrees=True)

CMD_START = bytes([0x43])
CMD_STOP = bytes([0x45])
CHANNEL_MARKERS = ("S0", "S1", "S2", "S3")
STREAM_END = "STREAM_END"
EXPECTED_RF_SAMPLES = int((178804.2 / 343.0) * 2.0 * 0.6 * 1.2)

# ---------------------------------------------------------------------------
# Results directory
# ---------------------------------------------------------------------------

RESULTS_DIR = Path(__file__).resolve().parent / "results"


# ---------------------------------------------------------------------------
# RF streaming helpers
# ---------------------------------------------------------------------------

def _stream_reader(
    ser: serial.Serial,
    frames: list,
    stop_event: threading.Event,
    first_frame_event: threading.Event,
    t0: float,
) -> None:
    """Background thread: collect 4RF frames and timestamp them.

    Each entry in *frames* is (frame_data, relative_timestamp_s) where
    frame_data = [ch0_samples, ch1_samples, ch2_samples, ch3_samples].
    """
    current_channel = None
    current_samples: list[int] = []
    current_frame: dict[str, list[int]] = {}
    line_buffer = bytearray()

    def _handle_line(raw: bytes) -> None:
        nonlocal current_channel, current_samples, current_frame
        line = raw.decode("ascii", errors="ignore").strip()
        if not line:
            return
        if line == STREAM_END:
            stop_event.set()
            return
        if line in CHANNEL_MARKERS:
            current_channel = line
            current_samples = []
        elif line == "T":
            if current_channel is not None:
                current_frame[current_channel] = current_samples
                current_channel = None
                if len(current_frame) == len(CHANNEL_MARKERS):
                    if any(len(current_frame[m]) != EXPECTED_RF_SAMPLES for m in CHANNEL_MARKERS):
                        current_frame = {}
                        return
                    ts = time.perf_counter() - t0
                    frames.append(([current_frame[m] for m in CHANNEL_MARKERS], ts))
                    first_frame_event.set()
                    current_frame = {}
        else:
            try:
                current_samples.append(int(line))
            except ValueError:
                pass

    while not stop_event.is_set():
        try:
            raw = ser.read(min(max(ser.in_waiting, 1), 4096))
        except serial.SerialException:
            break
        if raw == b"":
            continue
        line_buffer.extend(raw)
        while True:
            try:
                idx = line_buffer.index(ord("\n"))
            except ValueError:
                break
            _handle_line(bytes(line_buffer[:idx]))
            del line_buffer[: idx + 1]


def rf_stream_start(ser: serial.Serial) -> tuple[list, threading.Event, threading.Event, threading.Thread, float]:
    """Send CMD_START and begin collecting RF frames in background.

    Returns (frames, stop_event, first_frame_event, thread, t0).
    """
    frames: list = []
    stop_event = threading.Event()
    first_frame_event = threading.Event()
    t0 = time.perf_counter()
    try:
        ser.reset_input_buffer()
        written = ser.write(CMD_START)
    except serial.SerialException as err:
        raise RuntimeError(f"Failed to send start byte 67 to Teensy: {err}") from err
    if written != len(CMD_START):
        raise RuntimeError(f"Failed to send full start byte to Teensy: wrote {written} bytes")
    thread = threading.Thread(
        target=_stream_reader, args=(ser, frames, stop_event, first_frame_event, t0), daemon=True
    )
    thread.start()
    return frames, stop_event, first_frame_event, thread, t0


def rf_stream_stop(
    ser: serial.Serial,
    stop_event: threading.Event,
    thread: threading.Thread,
) -> None:
    """Send CMD_STOP and wait for the reader to flush."""
    try:
        written = ser.write(CMD_STOP)
    except serial.SerialException as err:
        raise RuntimeError(f"Failed to send stop byte 69 to Teensy: {err}") from err
    if written != len(CMD_STOP):
        raise RuntimeError(f"Failed to send full stop byte to Teensy: wrote {written} bytes")
    stop_event.set()
    thread.join(timeout=SERIAL_TIMEOUT_SEC)


# ---------------------------------------------------------------------------
# Contact location estimation
# ---------------------------------------------------------------------------

def estimate_contact_location_per_frame(frame_data: list) -> np.ndarray:
    """Envelope-peak sample index for each of the 4 waveguides. Shape (4,)."""
    locs = np.empty(4)
    for ch, samples in enumerate(frame_data):
        arr = np.array(samples, dtype=float)
        if arr.size == 0:
            locs[ch] = np.nan
        else:
            locs[ch] = float(np.argmax(np.abs(hilbert(arr))))
    return locs


# ---------------------------------------------------------------------------
# Controller setup helper
# ---------------------------------------------------------------------------

def _setup_joint_trajectory_controller(robot: Robot) -> None:
    robot.controller_switcher_client.switch_controller(CONTROLLER_NAME)
    deadline = time.time() + 5.0
    while time.time() < deadline:
        if robot.controller_switcher_client.get_active_controller() == CONTROLLER_NAME:
            print(f"Active controller: {CONTROLLER_NAME}")
            return
        time.sleep(0.1)
    raise RuntimeError(
        f"Controller switch to '{CONTROLLER_NAME}' did not complete within 5 s."
    )


def _duration_for_move(start_pose: Pose, end_pose: Pose, speed: float) -> float:
    distance = float(np.linalg.norm(end_pose.position - start_pose.position))
    return max(distance / speed, 0.25)


def _path_length(pose_path: list[Pose]) -> float:
    if len(pose_path) < 2:
        return 0.0
    return float(
        sum(
            np.linalg.norm(next_pose.position - prev_pose.position)
            for prev_pose, next_pose in zip(pose_path[:-1], pose_path[1:])
        )
    )


def _dense_ik_points_for_path(
    pose_path: list[Pose],
    min_points: int,
    max_spacing_m: float = 0.002,
) -> int:
    distance = _path_length(pose_path)
    dense_count = max(2, int(np.ceil(distance / max_spacing_m)) + 1)
    return max(len(pose_path), dense_count, min_points)


def _linear_pose_path(start_pose: Pose, end_pose: Pose, spacing_m: float) -> list[Pose]:
    distance = float(np.linalg.norm(end_pose.position - start_pose.position))
    n_points = max(2, int(np.ceil(distance / spacing_m)) + 1)
    waypoints = generate_linear_waypoints(
        start_position=start_pose.position,
        start_orientation=start_pose.orientation,
        end_position=end_pose.position,
        end_orientation=end_pose.orientation,
        num_waypoints=n_points,
    )
    return [Pose(w.position, w.orientation) for w in waypoints]


def _waypoints_from_pose_path(pose_path: list[Pose]) -> list[CartesianWaypoint]:
    if len(pose_path) < 2:
        raise ValueError("pose_path must contain at least two poses")
    cumulative = [0.0]
    for prev_pose, next_pose in zip(pose_path[:-1], pose_path[1:]):
        step = float(np.linalg.norm(next_pose.position - prev_pose.position))
        cumulative.append(cumulative[-1] + step)
    total_distance = cumulative[-1]
    if total_distance <= 1e-9:
        s_values = [0.0 for _ in cumulative]
    else:
        s_values = [dist / total_distance for dist in cumulative]
    return [
        CartesianWaypoint(
            position=np.array(pose.position, dtype=float),
            orientation=pose.orientation,
            s=s,
        )
        for pose, s in zip(pose_path, s_values)
    ]




def _anchor_first_waypoint_to_current_joints(
    trajectory: PlannedJointTrajectory,
    current_q: np.ndarray,
    max_joint_speed_rad_s: float,
) -> PlannedJointTrajectory:
    if len(trajectory.joint_positions) == 0:
        return trajectory

    positions = np.array(trajectory.joint_positions, dtype=float, copy=True)
    times = [float(t) for t in trajectory.time_from_start]
    positions[0] = np.array(current_q, dtype=float)

    if len(positions) >= 2:
        nominal_dt = max(times[1] - times[0], 1e-4)
        dq = np.abs(positions[1] - positions[0])
        required_dt = max(float(np.max(dq) / max_joint_speed_rad_s), nominal_dt)
        if required_dt > nominal_dt:
            shift = required_dt - nominal_dt
            for i in range(1, len(times)):
                times[i] += shift

    return PlannedJointTrajectory(
        joint_names=list(trajectory.joint_names),
        time_from_start=times,
        joint_positions=positions,
    )


def _sample_ee_poses(
    robot: Robot,
    ee_poses: list[dict],
    stop_event: threading.Event,
    t0: float,
    period_s: float = EE_SAMPLE_PERIOD_SEC,
) -> None:
    while not stop_event.is_set():
        try:
            ee = robot.end_effector_pose
        except RuntimeError:
            time.sleep(period_s)
            continue
        ee_poses.append({
            "t": time.perf_counter() - t0,
            "position": ee.position.tolist(),
            "orientation_quat": ee.orientation.as_quat().tolist(),
            "orientation_euler_deg_xyz": ee.orientation.as_euler("xyz", degrees=True).tolist(),
        })
        time.sleep(period_s)


def _pose_xyz_rpy_str(pose: Pose) -> str:
    pos = np.array(pose.position, dtype=float)
    rpy = pose.orientation.as_euler("xyz", degrees=True)
    return (
        f"xyz=({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}) m, "
        f"rpy=({rpy[0]:.2f}, {rpy[1]:.2f}, {rpy[2]:.2f}) deg"
    )


def _print_segment_plan(name: str, start_pose: Pose, end_pose: Pose) -> None:
    print(f"    {name} start wanted: {_pose_xyz_rpy_str(start_pose)}")
    print(f"    {name} end wanted:   {_pose_xyz_rpy_str(end_pose)}")


def _print_segment_actual(name: str, actual_pose: Pose) -> None:
    print(f"    {name} actual:       {_pose_xyz_rpy_str(actual_pose)}")


def _raise_robot_not_ready(err: TimeoutError, dry_run: bool) -> None:
    mode = "dry-run" if dry_run else "normal run"
    raise RuntimeError(
        f"Robot state did not become available in {mode}. "
        "This script can skip RF serial I/O in dry-run mode, but it still requires live FR3 ROS state "
        "to execute motion. Expected topics under the robot namespace are "
        "'franka_robot_state_broadcaster/current_pose' and 'joint_states'. "
        f"Original error: {err}"
    ) from err


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

def plot_mean_contact(trials: list[dict], output_path: Path) -> None:
    """Mean contact location (avg. of 4 waveguides) vs. time, one curve per speed."""
    fig, ax = plt.subplots(figsize=(10, 5))
    for trial in trials:
        frames = trial["frames"]
        if not frames:
            continue
        ts = np.array([f[1] for f in frames])
        mean_locs = np.array([
            np.nanmean(estimate_contact_location_per_frame(f[0]))
            for f in frames
        ])
        speed_mm = trial["speed_m_s"] * 1000.0
        ax.plot(ts, mean_locs, label=f"{speed_mm:.1f} mm/s")

    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Mean contact location [sample index]")
    ax.set_title("Rolling contact: mean contact location vs. time")
    handles, labels = ax.get_legend_handles_labels()
    if handles:
        ax.legend(title="Roll speed")
    ax.grid(True)
    fig.tight_layout()
    fig.savefig(output_path.with_suffix(".png"), dpi=150)
    print(f"  Plot saved: {output_path.with_suffix('.png')}")
    plt.close(fig)


def plot_per_waveguide(trials: list[dict], output_path: Path) -> None:
    """4-panel plot: contact location per waveguide vs. time, all speeds overlaid."""
    n_trials = len(trials)
    colors = plt.cm.viridis(np.linspace(0.0, 0.9, n_trials))
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), sharex=False)
    axes = axes.flatten()

    for ch, ax in enumerate(axes):
        for ti, trial in enumerate(trials):
            frames = trial["frames"]
            if not frames:
                continue
            ts = np.array([f[1] for f in frames])
            locs = np.array([estimate_contact_location_per_frame(f[0])[ch] for f in frames])
            ax.plot(ts, locs, color=colors[ti],
                    label=f"{trial['speed_m_s'] * 1000:.1f} mm/s")
        ax.set_title(f"Waveguide {ch} (S{ch})")
        ax.set_xlabel("Time [s]")
        ax.set_ylabel("Contact location [sample]")
        ax.grid(True)
        if ch == 0:
            handles, labels = ax.get_legend_handles_labels()
            if handles:
                ax.legend(title="Speed", fontsize=7)

    fig.suptitle("Rolling contact: per-waveguide contact location vs. time")
    fig.tight_layout()
    out = output_path.parent / (output_path.stem + "_per_waveguide.png")
    fig.savefig(out, dpi=150)
    print(f"  Per-waveguide plot saved: {out}")
    plt.close(fig)


def _next_trial_dir(results_dir: Path) -> Path:
    counter = 0
    trial_dir = results_dir / f"trial_{counter:02d}"
    while trial_dir.exists():
        counter += 1
        trial_dir = results_dir / f"trial_{counter:02d}"
    trial_dir.mkdir(parents=True, exist_ok=False)
    return trial_dir


def _roll_distance_m() -> float:
    return FINRAY_LENGTH_M * (ROLL_LENGTH_PERCENT / 100.0)


def _rf_capture_summary(frames: list, expected_duration_s: float) -> dict:
    if not frames:
        return {
            "frame_count": 0,
            "first_frame_t_s": None,
            "last_frame_t_s": None,
            "observed_duration_s": 0.0,
            "coverage_ratio": 0.0,
        }

    ts = np.array([frame[1] for frame in frames], dtype=float)
    observed_duration = float(ts[-1] - ts[0]) if len(ts) >= 2 else 0.0
    coverage_ratio = observed_duration / expected_duration_s if expected_duration_s > 0 else 0.0
    return {
        "frame_count": len(frames),
        "first_frame_t_s": float(ts[0]),
        "last_frame_t_s": float(ts[-1]),
        "observed_duration_s": observed_duration,
        "coverage_ratio": coverage_ratio,
    }


def _save_trial_result(
    trial_dir: Path,
    speed_m_s: float,
    trial_payload: dict,
    session_metadata: dict,
) -> Path:
    speed_mm_s = speed_m_s * 1000.0
    out_path = trial_dir / f"rolling_speed_{speed_mm_s:06.1f}_mm_s.pkl"
    with open(out_path, "wb") as f:
        pickle.dump(
            {
                "trial": trial_payload,
                "session": session_metadata,
            },
            f,
        )
    print(f"  Result saved: {out_path}")
    plot_mean_contact([trial_payload], out_path)
    plot_per_waveguide([trial_payload], out_path)
    return out_path


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    trial_dir = _next_trial_dir(RESULTS_DIR)
    print(f"Saving this session under: {trial_dir}")

    roll_distance_m = _roll_distance_m()
    R = ROLLER_DIAMETER_M / 2.0
    d = COMPRESS_DEPTH_M
    R_eff = R - d

    # Pre-compute expected slide displacement for operator confirmation
    y_disp_mm = roll_distance_m * 1000.0
    roller_rotation_deg = np.degrees(roll_distance_m / max(R, 1e-9))

    # ── Dry-run prompt ────────────────────────────────────────────────────────
    dry_run = DRY_RUN
    answer = input("Dry run (skip RF only; robot state + motion still required)? [y/N]: ").strip().lower()
    if answer in {"y", "yes"}:
        dry_run = True
    if dry_run:
        print("  DRY RUN — serial port will not be opened; RF frames will be empty.")

    robot = Robot(namespace=ROBOT_NAMESPACE)
    ser = (
        serial.Serial(
            SERIAL_PORT,
            BAUD_RATE,
            timeout=SERIAL_TIMEOUT_SEC,
            write_timeout=SERIAL_TIMEOUT_SEC,
        )
        if not dry_run
        else None
    )

    trials: list[dict] = []
    active_stop_event: threading.Event | None = None
    active_thread: threading.Thread | None = None

    session_metadata = {
        "roller_diameter_m": ROLLER_DIAMETER_M,
        "roller_radius_m": R,
        "compress_depth_m": d,
        "approach_height_m": APPROACH_HEIGHT_M,
        "finray_length_m": FINRAY_LENGTH_M,
        "roll_length_percent": ROLL_LENGTH_PERCENT,
        "roll_distance_m": roll_distance_m,
        "roller_rotation_deg_nominal": float(roller_rotation_deg),
        "slide_point_spacing_m": SLIDE_POINT_SPACING_M,
        "approach_point_spacing_m": APPROACH_POINT_SPACING_M,
        "contact_point_spacing_m": CONTACT_POINT_SPACING_M,
        "compress_point_spacing_m": COMPRESS_POINT_SPACING_M,
        "decompress_point_spacing_m": DECOMPRESS_POINT_SPACING_M,
        "return_point_spacing_m": RETURN_POINT_SPACING_M,
        "speeds_m_s": ROLL_SPEEDS_M_S,
        "base_ori_euler_deg": BASE_ORI_EULER_DEG,
        "robot_namespace": ROBOT_NAMESPACE,
        "controller_name": CONTROLLER_NAME,
        "expected_rf_samples": EXPECTED_RF_SAMPLES,
        "trial_dir": str(trial_dir),
    }

    try:
        try:
            robot.wait_until_ready()
        except TimeoutError as err:
            _raise_robot_not_ready(err, dry_run)
        robot.config.ik_position_weight = IK_POSITION_WEIGHT
        robot.config.ik_orientation_weight = IK_ORIENTATION_WEIGHT
        robot.config.ik_similarity_weight = IK_SIMILARITY_WEIGHT
        _setup_joint_trajectory_controller(robot)

        # ── Phase 0: resolve contact pose and approach pose ──────────────────
        if CONTACT_POSITION_M is not None:
            _contact_pose = Pose(
                np.array(CONTACT_POSITION_M, dtype=float),
                BASE_ORI,
            )
            print("\n=== Rolling contact experiment ===")
            print(f"Contact position (from config): {CONTACT_POSITION_M}")
        else:
            # Fallback: robot must already be jogged to the contact point.
            _contact_pose = robot.end_effector_pose.copy()
            print("\n=== Rolling contact experiment ===")
            print(f"Contact position (captured at startup): {_contact_pose.position.tolist()}")
            print("  (Set CONTACT_POSITION_M in rolling_config.py to skip manual jogging.)")

        approach_pose = Pose(
            _contact_pose.position + np.array([0.0, 0.0, APPROACH_HEIGHT_M]),
            BASE_ORI,
        )
        compressed_z = _contact_pose.position[2] - COMPRESS_DEPTH_M

        n_slide_intervals = max(1, int(np.ceil(_roll_distance_m() / SLIDE_POINT_SPACING_M)))

        print(f"Approach pose:   {approach_pose.position.tolist()}  "
              f"(contact + {APPROACH_HEIGHT_M*1000:.0f} mm above)")
        print(f"Compressed z (nominal): {compressed_z:.4f} m  (config contact − {d*1000:.1f} mm; actual derived after descend)")
        print(f"Roller diameter: {ROLLER_DIAMETER_M*1000:.1f} mm  |  radius: {R*1000:.1f} mm")
        print(f"Effective radius under compression: {R_eff*1000:.1f} mm")
        print(f"Fin-ray length tested: {ROLL_LENGTH_PERCENT:.1f}% of {FINRAY_LENGTH_M*1000:.1f} mm")
        print(f"Slide travel:    {y_disp_mm:.1f} mm in -Y  ({n_slide_intervals} planned intervals)")
        print(f"Implied no-slip roller rotation: {roller_rotation_deg:.1f} deg")
        print(f"Compressed Z (nominal): {compressed_z:.4f} m  (actual planned per-trial after measuring contact)")
        print("Orientation:     fixed at [180, 0, 0] deg")
        print(
            "IK weights:      "
            f"pos={robot.config.ik_position_weight:.3f}, "
            f"ori={robot.config.ik_orientation_weight:.3f}, "
            f"similarity={robot.config.ik_similarity_weight:.3f}"
        )
        print(f"Speeds:          {[s*1000 for s in ROLL_SPEEDS_M_S]} mm/s")

        input("\nPress Enter to begin sliding trials...")

        for trial_idx, speed in enumerate(ROLL_SPEEDS_M_S):
            print(f"\n─── Trial {trial_idx + 1}/{len(ROLL_SPEEDS_M_S)}  "
                  f"speed = {speed * 1000:.1f} mm/s ───")
            input("Press Enter to start this rolling session...")

            # ── 1. Plan approach → descend (contact trajectory planned later, after
            #       measuring actual contact pose, so targets match where robot lands) ──────
            current_pose = robot.end_effector_pose.copy()

            descend_path     = _linear_pose_path(approach_pose, _contact_pose, CONTACT_POINT_SPACING_M)
            descend_duration = _duration_for_move(approach_pose, _contact_pose, APPROACH_SPEED_M_S)

            needs_pre_approach = np.linalg.norm(current_pose.position - approach_pose.position) > 1e-3
            if needs_pre_approach:
                pre_approach_path     = _linear_pose_path(current_pose, approach_pose, APPROACH_POINT_SPACING_M)
                pre_approach_duration = _duration_for_move(current_pose, approach_pose, RETURN_SPEED_M_S)
                pre_approach_traj, descend_traj = robot.plan_joint_trajectory_sequence(
                    [_waypoints_from_pose_path(pre_approach_path),
                     _waypoints_from_pose_path(descend_path)],
                    [pre_approach_duration, descend_duration],
                )
                pre_approach_traj = _anchor_first_waypoint_to_current_joints(
                    pre_approach_traj, np.array(robot.q, dtype=float), FIRST_WAYPOINT_MAX_JOINT_SPEED_RAD_S,
                )
            else:
                pre_approach_traj = None
                [descend_traj] = robot.plan_joint_trajectory_sequence(
                    [_waypoints_from_pose_path(descend_path)],
                    [descend_duration],
                )
                descend_traj = _anchor_first_waypoint_to_current_joints(
                    descend_traj, np.array(robot.q, dtype=float), FIRST_WAYPOINT_MAX_JOINT_SPEED_RAD_S,
                )

            print(f"  Descend: {descend_duration:.1f} s")
            if needs_pre_approach:
                _print_segment_plan("pre-approach", current_pose, approach_pose)
            _print_segment_plan("descend", approach_pose, _contact_pose)

            # ── 2. Execute: pre-approach → descend  (stops OK here) ──────────────────────
            # After each stop we re-anchor the next trajectory to the actual measured joint
            # state, so the controller never sees a first-waypoint mismatch.
            if pre_approach_traj is not None:
                robot.execute_sequence(
                    [pre_approach_traj],
                    visualize_before_execution=False,
                    settle_time_between_trajectories=SETTLE_SEC,
                )
                _print_segment_actual("pre-approach", robot.end_effector_pose.copy())
                descend_traj = _anchor_first_waypoint_to_current_joints(
                    descend_traj, np.array(robot.q, dtype=float), FIRST_WAYPOINT_MAX_JOINT_SPEED_RAD_S,
                )

            robot.execute_sequence(
                [descend_traj],
                visualize_before_execution=False,
                settle_time_between_trajectories=SETTLE_SEC,
            )
            measured_contact_pose = robot.end_effector_pose.copy()
            _print_segment_actual("descend", measured_contact_pose)

            # ── 3. Plan contact phase from MEASURED contact position ──────────────────────
            # Planning here (after descend) means compress/slide/decompress targets are
            # derived from where the robot actually landed, not the config nominal.
            # Dense IK points ensure straight-line Cartesian tracking during the slide.
            desired_compressed_start = Pose(
                measured_contact_pose.position + np.array([0.0, 0.0, -COMPRESS_DEPTH_M]),
                BASE_ORI,
            )
            desired_compressed_end = Pose(
                desired_compressed_start.position + np.array([0.0, -roll_distance_m, 0.0]),
                BASE_ORI,
            )
            desired_surface_end = Pose(
                desired_compressed_end.position + np.array([0.0, 0.0, COMPRESS_DEPTH_M]),
                BASE_ORI,
            )

            compress_duration   = _duration_for_move(measured_contact_pose, desired_compressed_start, APPROACH_SPEED_M_S)
            slide_duration      = _duration_for_move(desired_compressed_start, desired_compressed_end, speed)
            decompress_duration = _duration_for_move(desired_compressed_end, desired_surface_end, APPROACH_SPEED_M_S)
            rf_window_duration  = compress_duration + slide_duration + decompress_duration

            compress_path   = _linear_pose_path(measured_contact_pose, desired_compressed_start, COMPRESS_POINT_SPACING_M)
            slide_path      = _linear_pose_path(desired_compressed_start, desired_compressed_end, SLIDE_POINT_SPACING_M)
            decompress_path = _linear_pose_path(desired_compressed_end, desired_surface_end, DECOMPRESS_POINT_SPACING_M)
            full_contact_path = compress_path + slide_path[1:] + decompress_path[1:]

            n_contact_ik_pts = _dense_ik_points_for_path(full_contact_path, robot.config.ik_default_num_points, max_spacing_m=0.004)
            [contact_traj] = robot.plan_joint_trajectory_sequence(
                [_waypoints_from_pose_path(full_contact_path)],
                [rf_window_duration],
                n_points=n_contact_ik_pts,
            )
            contact_traj = _anchor_first_waypoint_to_current_joints(
                contact_traj, np.array(robot.q, dtype=float), FIRST_WAYPOINT_MAX_JOINT_SPEED_RAD_S,
            )

            print(
                f"  Contact IK pts: {n_contact_ik_pts}  |  "
                f"compress {compress_duration:.1f} s  |  "
                f"slide {slide_duration:.1f} s  |  "
                f"decompress {decompress_duration:.1f} s"
            )
            _print_segment_plan("compress",   measured_contact_pose, desired_compressed_start)
            _print_segment_plan("slide",      desired_compressed_start, desired_compressed_end)
            _print_segment_plan("decompress", desired_compressed_end, desired_surface_end)

            boundary_states: dict[str, dict] = {
                "contact": {
                    "position": measured_contact_pose.position.tolist(),
                    "orientation_quat": measured_contact_pose.orientation.as_quat().tolist(),
                    "orientation_euler_deg_xyz": measured_contact_pose.orientation.as_euler("xyz", degrees=True).tolist(),
                    "q": robot.q.tolist(),
                },
                "compressed_start": {"position": desired_compressed_start.position.tolist(),
                                     "orientation_quat": desired_compressed_start.orientation.as_quat().tolist(),
                                     "orientation_euler_deg_xyz": desired_compressed_start.orientation.as_euler("xyz", degrees=True).tolist(),
                                     "q": None},
                "compressed_end":   {"position": desired_compressed_end.position.tolist(),
                                     "orientation_quat": desired_compressed_end.orientation.as_quat().tolist(),
                                     "orientation_euler_deg_xyz": desired_compressed_end.orientation.as_euler("xyz", degrees=True).tolist(),
                                     "q": None},
            }

            # ── 4. Execute: contact phase (no stops — single trajectory) ──────────────────
            # RF capture bookends are plain callables; execute_sequence calls them
            # synchronously before/after the trajectory within the same call.
            frames: list = []
            ee_poses_trial: list[dict] = []
            sample_stop_event: threading.Event | None = None
            sample_thread: threading.Thread | None = None
            first_frame_event: threading.Event | None = None
            slide_t0: float | None = None

            def start_rf_capture() -> None:
                nonlocal frames, active_stop_event, active_thread, sample_stop_event, sample_thread, first_frame_event, slide_t0
                slide_t0 = time.perf_counter()
                if dry_run:
                    print(
                        f"  Dry-run RF window at {speed*1000:.1f} mm/s "
                        "(compress + slide + decompress; RF skipped)..."
                    )
                else:
                    print("  Contact reached; sending 67 to Teensy...")
                    frames, stop_event, first_frame_event, thread, _ = rf_stream_start(ser)
                    active_stop_event = stop_event
                    active_thread = thread
                    if not first_frame_event.wait(timeout=SERIAL_TIMEOUT_SEC):
                        raise TimeoutError(
                            f"No RF frame received from Teensy within {SERIAL_TIMEOUT_SEC} s after sending 67."
                        )
                    print("  First RF frame received; starting compress/slide/decompress sequence...")
                sample_stop_event = threading.Event()
                sample_thread = threading.Thread(
                    target=_sample_ee_poses,
                    args=(robot, ee_poses_trial, sample_stop_event, slide_t0),
                    daemon=True,
                )
                sample_thread.start()

            def stop_rf_capture() -> None:
                nonlocal active_stop_event, active_thread, sample_stop_event, sample_thread, first_frame_event
                if sample_stop_event is not None:
                    sample_stop_event.set()
                if sample_thread is not None:
                    sample_thread.join(timeout=1.0)
                sample_stop_event = None
                sample_thread = None
                first_frame_event = None
                if dry_run:
                    print(
                        f"  Dry-run RF window complete: {rf_window_duration:.2f} s, "
                        "no RF frames recorded"
                    )
                else:
                    if active_stop_event is not None and active_thread is not None:
                        print("  Decompress complete; sending 69 to Teensy to stop RF capture...")
                        rf_stream_stop(ser, active_stop_event, active_thread)
                    active_stop_event = None
                    active_thread = None
                    print(f"  RF stream stopped: {len(frames)} frames in {rf_window_duration:.2f} s")

            robot.execute_sequence(
                [start_rf_capture, contact_traj, stop_rf_capture],
                visualize_before_execution=False,
                settle_time_between_trajectories=0.0,
            )

            measured_post_contact = robot.end_effector_pose.copy()
            _print_segment_actual("contact (full)", measured_post_contact)
            boundary_states["surface_end"] = {
                "position": measured_post_contact.position.tolist(),
                "orientation_quat": measured_post_contact.orientation.as_quat().tolist(),
                "orientation_euler_deg_xyz": measured_post_contact.orientation.as_euler("xyz", degrees=True).tolist(),
                "q": robot.q.tolist(),
            }

            # Slide-only EE window extracted from the full contact recording by time
            ee_poses_slide_only: list[dict] = (
                [ep for ep in ee_poses_trial
                 if compress_duration <= ep["t"] <= compress_duration + slide_duration]
                if slide_t0 is not None else []
            )

            # ── 5. Return to approach (re-planned from measured state for accuracy) ────────
            return_path     = _linear_pose_path(measured_post_contact, approach_pose, RETURN_POINT_SPACING_M)
            return_duration = _duration_for_move(return_path[0], return_path[-1], RETURN_SPEED_M_S)
            _print_segment_plan("return", measured_post_contact, approach_pose)
            [return_traj] = robot.plan_joint_trajectory_sequence(
                [_waypoints_from_pose_path(return_path)],
                [return_duration],
                n_points=_dense_ik_points_for_path(return_path, robot.config.ik_default_num_points),
            )
            return_traj = _anchor_first_waypoint_to_current_joints(
                return_traj, np.array(robot.q, dtype=float), FIRST_WAYPOINT_MAX_JOINT_SPEED_RAD_S,
            )
            robot.execute_sequence(
                [return_traj],
                visualize_before_execution=False,
                settle_time_between_trajectories=SETTLE_SEC,
            )
            _print_segment_actual("return", robot.end_effector_pose.copy())

            # ── 6. Print slide-step summary ───────────────────────────────────────────────
            slide_poses = slide_path[1:]
            if slide_t0 is not None:
                for step_idx, slide_pose in enumerate(slide_poses):
                    y_offset_mm = (slide_pose.position[1] - measured_contact_pose.position[1]) * 1000.0
                    t_now = compress_duration + slide_duration * (step_idx + 1) / len(slide_poses)
                    print(
                        f"    step {step_idx + 1:2d}/{len(slide_poses)}  "
                        f"Δy={y_offset_mm:5.1f} mm  "
                        f"y={slide_pose.position[1]:.4f} m  "
                        f"z={slide_pose.position[2]:.4f} m  "
                        f"t≈{t_now:.2f} s  "
                        f"frames={len(frames)}"
                    )

            # ── 7. Save results ───────────────────────────────────────────────────────────
            planned_trajs_for_save = [
                traj
                for traj in [pre_approach_traj, descend_traj, contact_traj, return_traj]
                if traj is not None
            ]
            trial_payload = {
                "speed_m_s": speed,
                "roll_distance_m": roll_distance_m,
                "roller_rotation_deg_nominal": float(roller_rotation_deg),
                "frames": list(frames),
                "ee_poses": ee_poses_trial,
                "ee_poses_slide_only": ee_poses_slide_only,
                "rf_window_duration_s": rf_window_duration,
                "slide_duration_s": slide_duration,
                "compress_duration_s": compress_duration,
                "decompress_duration_s": decompress_duration,
                "approach_position": approach_pose.position.tolist(),
                "contact_position": measured_contact_pose.position.tolist(),
                "compressed_start_position": desired_compressed_start.position.tolist(),
                "compressed_end_position": desired_compressed_end.position.tolist(),
                "surface_end_position": desired_surface_end.position.tolist(),
                "boundary_states": boundary_states,
                "planned_slide_poses": [
                    {
                        "position": pose.position.tolist(),
                        "orientation_quat": pose.orientation.as_quat().tolist(),
                        "orientation_euler_deg_xyz": pose.orientation.as_euler("xyz", degrees=True).tolist(),
                    }
                    for pose in slide_path
                ],
                "planned_joint_trajectories": [
                    {
                        "joint_names": traj.joint_names,
                        "time_from_start": list(traj.time_from_start),
                        "joint_positions": traj.joint_positions.tolist(),
                    }
                    for traj in planned_trajs_for_save
                ],
            }
            trial_payload["rf_capture_summary"] = _rf_capture_summary(frames, rf_window_duration)
            trials.append(trial_payload)
            _save_trial_result(trial_dir, speed, trial_payload, session_metadata)
            if not dry_run:
                coverage = trial_payload["rf_capture_summary"]["coverage_ratio"]
                print(
                    "  RF coverage check: "
                    f"{coverage * 100.0:.1f}% of expected capture window "
                    f"({trial_payload['rf_capture_summary']['frame_count']} frames)"
                )
                if coverage < 0.9:
                    print("  Warning: RF coverage looks short; check Teensy stop timing or dropped frames.")

    finally:
        if ser is not None and active_stop_event is not None and active_thread is not None:
            rf_stream_stop(ser, active_stop_event, active_thread)
        if ser is not None:
            ser.close()
        robot.shutdown()

    print(f"\nSession complete. Saved {len(trials)} rolling files under: {trial_dir}")


if __name__ == "__main__":
    main()
