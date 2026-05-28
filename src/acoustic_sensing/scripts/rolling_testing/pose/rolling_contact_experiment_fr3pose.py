#!/usr/bin/env python3
"""Rolling contact experiment using fr3_pose_controller (Cartesian pose streaming).

Same experiment as rolling_contact_experiment.py but replaces the
joint_trajectory_controller / IK planning pipeline with direct Cartesian pose
streaming via fr3_pose_controller.

Advantages over joint_trajectory version:
- No IK solve, no joint-space interpolation → Z held by the controller natively
- No pre-planning delay before each trial
- Smoother compress → slide → decompress continuity (single streaming loop)
- Wall-clock pose computation guarantees consistent speed regardless of scheduling jitter

Motion sequence per trial
--------------------------
(1) approach_pose   ← CONTACT_POSITION_M + APPROACH_HEIGHT_M above
         │ move_to (fr3_pose, APPROACH_SPEED_M_S)
(2) contact_pose    ← sensor touching cylinder surface
         │ [measure actual contact pose]
         │ stream compress + slide + decompress at 100 Hz (no stops)
         │ [RF capture wraps the full stream]
(5) surface_end_pose ← back up COMPRESS_DEPTH_M after decompress
         │ move_to (fr3_pose, RETURN_SPEED_M_S)
(6) approach_pose   ← ready for next trial
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

from arm_client import CONFIG_DIR
from arm_client.robot import Pose, Robot

from rolling_config import (
    APPROACH_HEIGHT_M,
    APPROACH_SPEED_M_S,
    BAUD_RATE,
    BASE_ORI_EULER_DEG,
    COMPRESS_DEPTH_M,
    CONTACT_POSITION_M,
    DRY_RUN,
    EE_SAMPLE_PERIOD_SEC,
    FINRAY_LENGTH_M,
    N_REPEATS,
    RETURN_SPEED_M_S,
    ROBOT_NAMESPACE,
    ROLL_END_OFFSET_M,
    ROLL_LENGTH_PERCENT,
    ROLLER_DIAMETER_M,
    ROLL_SPEEDS_M_S,
    SERIAL_PORT,
    SERIAL_TIMEOUT_SEC,
    SETTLE_SEC,
    STATIONARY_BASELINE_FRAMES,
    STATIONARY_BASELINE_TIMEOUT_SEC,
    BASELINE_EXTRA_HEIGHT_M,
    CONTACT_HOLD_SEC,
    PRE_COMPRESS_HOLD_SEC,
    POST_SLIDE_HOLD_SEC,
    MAX_CARTESIAN_SPEED_M_S,
    MIN_TRAJ_DURATION_S,
    MIN_WAYPOINT_DT_S,
)

FR3_POSE_CONTROLLER = "fr3_pose_controller"
FR3_POSE_CONFIG = "probing.yaml"

# Number of waypoints sent in each CartesianTrajectory message.
# The controller interpolates between them with quintic polynomials internally.
CONTACT_WAYPOINTS = 50

# execute_trajectory sleeps 0.5s internally before publishing, so the message
# header timestamp is ~0.5s old when it arrives at the controller.  Offsetting
# all waypoint times by EXEC_TRAJ_TIME_OFFSET (slightly > 0.5s) keeps the first
# waypoint in the future, avoiding the abrupt acceleration at trajectory start.
EXEC_TRAJ_TIME_OFFSET = 0.6

BASE_ORI = Rotation.from_euler("xyz", BASE_ORI_EULER_DEG, degrees=True)

CMD_START = bytes([0x43])
CMD_STOP = bytes([0x45])
CHANNEL_MARKERS = ("S0", "S1", "S2", "S3")
STREAM_END = "STREAM_END"
EXPECTED_RF_SAMPLES = 1000

RESULTS_DIR = Path(__file__).resolve().parent / "results"


# ---------------------------------------------------------------------------
# RF streaming (identical to joint_trajectory version)
# ---------------------------------------------------------------------------

def _stream_reader(ser, frames, stop_event, first_frame_event, t0):
    current_channel = None
    current_samples = []
    current_frame = {}
    line_buffer = bytearray()

    def _handle_line(raw):
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


def rf_stream_start(ser):
    frames = []
    stop_event = threading.Event()
    first_frame_event = threading.Event()
    t0 = time.perf_counter()
    ser.reset_input_buffer()
    written = ser.write(CMD_START)
    if written != len(CMD_START):
        raise RuntimeError(f"Failed to send full start byte: wrote {written} bytes")
    thread = threading.Thread(
        target=_stream_reader, args=(ser, frames, stop_event, first_frame_event, t0), daemon=True
    )
    thread.start()
    return frames, stop_event, first_frame_event, thread, t0


def rf_stream_stop(ser, stop_event, thread):
    written = ser.write(CMD_STOP)
    if written != len(CMD_STOP):
        raise RuntimeError(f"Failed to send full stop byte: wrote {written} bytes")
    # Let the reader thread drain in-flight bytes (incl. STREAM_END) before
    # exiting.  STREAM_END itself sets stop_event inside _stream_reader.
    thread.join(timeout=SERIAL_TIMEOUT_SEC)
    if thread.is_alive():
        stop_event.set()
        thread.join(timeout=1.0)


def capture_stationary_frames(ser, n_frames, timeout_s=STATIONARY_BASELINE_TIMEOUT_SEC):
    """Stream RF until n_frames have been collected, then stop."""
    frames, stop_event, _first_frame_event, thread, _t0 = rf_stream_start(ser)
    deadline = time.perf_counter() + timeout_s
    try:
        while len(frames) < n_frames:
            if time.perf_counter() > deadline:
                raise TimeoutError(
                    f"Only received {len(frames)}/{n_frames} stationary RF frames "
                    f"within {timeout_s:.1f} s."
                )
            time.sleep(0.02)
    finally:
        rf_stream_stop(ser, stop_event, thread)
    return frames[:n_frames]


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _roll_distance_m():
    """Effective roll distance, with the end-of-sensor safety margin removed."""
    return max(0.0, FINRAY_LENGTH_M * (ROLL_LENGTH_PERCENT / 100.0) - ROLL_END_OFFSET_M)


def _duration_for_move(start_pose, end_pose, speed):
    """Duration for a straight-line Cartesian segment.

    Clamps the requested speed at MAX_CARTESIAN_SPEED_M_S and floors the
    duration at MIN_TRAJ_DURATION_S so degenerate (near-zero-distance) moves
    don't collapse into a sub-second window — which is what triggered the
    `joint_acceleration_discontinuity` reflex previously.
    """
    distance = float(np.linalg.norm(end_pose.position - start_pose.position))
    speed_capped = min(max(speed, 1e-6), MAX_CARTESIAN_SPEED_M_S)
    return max(distance / speed_capped, MIN_TRAJ_DURATION_S)


def _wait_for_move(start_pose, end_pose, speed, extra_sec=1.0):
    """Sleep long enough for a `move_to(end_pose, speed=speed)` to finish.

    `robot.move_to` returns before the trajectory completes.  If the next
    command lands while the previous one is still running, the controller
    logs "Ignoring new pose - trajectory in progress" and ends up with
    EE velocity != 0 when the next trajectory's ramp-up starts — which
    trips `joint_acceleration_discontinuity`.  Wait long enough for the
    planned motion (+ a safety margin) before issuing the next command.
    """
    distance = float(np.linalg.norm(end_pose.position - start_pose.position))
    duration = distance / max(speed, 1e-6)
    time.sleep(max(duration + extra_sec, SETTLE_SEC))


def _pose_xyz_rpy_str(pose):
    pos = np.array(pose.position, dtype=float)
    rpy = pose.orientation.as_euler("xyz", degrees=True)
    return (
        f"xyz=({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}) m, "
        f"rpy=({rpy[0]:.2f}, {rpy[1]:.2f}, {rpy[2]:.2f}) deg"
    )


def _build_contact_trajectory(
    segment_pairs,
    durations,
    n_waypoints=CONTACT_WAYPOINTS,
    time_offset=EXEC_TRAJ_TIME_OFFSET,
    segment_velocities=None,
):
    """Build (waypoints, times) for execute_cartesian_traj.

    Distributes n_waypoints evenly across the total duration, interpolating
    linearly across segment boundaries. The fr3_pose controller uses quintic
    polynomial interpolation between these waypoints internally.

    time_offset shifts all timestamps forward to compensate for execute_cartesian_traj's
    internal 0.5s pre-publish sleep: without the offset the first waypoint is already
    in the past when the controller receives the message, causing an abrupt start.

    Args:
        segment_pairs: List of (start_pose, end_pose) for each segment.
        durations: Duration [s] for each segment.
        n_waypoints: Total number of waypoints to send.
        time_offset: Seconds to add to all waypoint times (default EXEC_TRAJ_TIME_OFFSET).
        segment_velocities: Optional list of Twist | None, one entry per segment.
            When non-None for a segment, every waypoint in that segment carries the
            given constant velocity hint.  The controller uses this to avoid stopping
            at intermediate waypoints, producing smooth constant-speed motion.
            Waypoints with None hints receive zero-velocity boundary conditions.

    Returns:
        (waypoints, times): waypoints is List[tuple[Pose, Twist | None]], times is List[float].
    """
    t_bounds = [0.0]
    for dur in durations:
        t_bounds.append(t_bounds[-1] + dur)
    total_duration = t_bounds[-1]

    # Cap waypoint density so consecutive waypoints stay >= MIN_WAYPOINT_DT_S
    # apart.  Otherwise the controller races through dense waypoints and the
    # apparent inter-waypoint speed spikes, which triggers the
    # joint_acceleration_discontinuity reflex.
    max_n_waypoints = max(2, int(total_duration / MIN_WAYPOINT_DT_S) + 1)
    n_waypoints = max(2, min(n_waypoints, max_n_waypoints))

    waypoints = []
    times = []

    for t in np.linspace(0.0, total_duration, n_waypoints):
        # Find which segment this time falls in.
        seg = len(segment_pairs) - 1
        for i in range(len(segment_pairs)):
            if t <= t_bounds[i + 1]:
                seg = i
                break

        seg_duration = t_bounds[seg + 1] - t_bounds[seg]
        t_frac = np.clip((t - t_bounds[seg]) / seg_duration, 0.0, 1.0) if seg_duration > 0 else 1.0

        sp, ep = segment_pairs[seg]
        pos = (1.0 - t_frac) * np.array(sp.position) + t_frac * np.array(ep.position)
        q = (1.0 - t_frac) * sp.orientation.as_quat() + t_frac * ep.orientation.as_quat()
        ori = Rotation.from_quat(q / np.linalg.norm(q))

        vel = (segment_velocities[seg] if segment_velocities is not None else None)
        waypoints.append((Pose(pos, ori), vel))
        times.append(float(t) + time_offset)

    return waypoints, times


def _sample_ee_poses(robot, ee_poses, stop_event, t0, period_s=EE_SAMPLE_PERIOD_SEC):
    prev_t = None
    prev_pos = None
    prev_vel = None
    while not stop_event.is_set():
        try:
            ee = robot.end_effector_pose
            wrench = robot.end_effector_wrench
        except RuntimeError:
            time.sleep(period_s)
            continue
        t = time.perf_counter() - t0
        pos = ee.position.tolist()

        dt = t - prev_t if prev_t is not None else None
        if dt and dt > 0 and prev_pos is not None:
            vel = [(pos[i] - prev_pos[i]) / dt for i in range(3)]
            acc = [(vel[i] - prev_vel[i]) / dt for i in range(3)] if prev_vel is not None else None
        else:
            vel = None
            acc = None

        ee_poses.append({
            "t": t,
            "position": pos,
            "orientation_quat": ee.orientation.as_quat().tolist(),
            "orientation_euler_deg_xyz": ee.orientation.as_euler("xyz", degrees=True).tolist(),
            "force_xyz": wrench["force"].tolist(),
            "torque_xyz": wrench["torque"].tolist(),
            "velocity_xyz": vel,
            "acceleration_xyz": acc,
        })

        prev_t = t
        prev_pos = pos
        prev_vel = vel
        time.sleep(period_s)


def _sample_cmd_poses(robot, cmd_poses, stop_event, t0, period_s=EE_SAMPLE_PERIOD_SEC):
    prev_t = None
    prev_pos = None
    prev_vel = None
    while not stop_event.is_set():
        try:
            ee = robot.target_pose
            wrench = robot.end_effector_wrench
        except RuntimeError:
            time.sleep(period_s)
            continue
        t = time.perf_counter() - t0
        pos = ee.position.tolist()

        dt = t - prev_t if prev_t is not None else None
        if dt and dt > 0 and prev_pos is not None:
            vel = [(pos[i] - prev_pos[i]) / dt for i in range(3)]
            acc = [(vel[i] - prev_vel[i]) / dt for i in range(3)] if prev_vel is not None else None
        else:
            vel = None
            acc = None

        cmd_poses.append({
            "t": t,
            "position": pos,
            "orientation_quat": ee.orientation.as_quat().tolist(),
            "orientation_euler_deg_xyz": ee.orientation.as_euler("xyz", degrees=True).tolist(),
            "force_xyz": wrench["force"].tolist(),
            "torque_xyz": wrench["torque"].tolist(),
            "velocity_xyz": vel,
            "acceleration_xyz": acc,
        })

        prev_t = t
        prev_pos = pos
        prev_vel = vel
        time.sleep(period_s)


def _ensure_rf_active(frames, timeout_s: float = 5.0) -> None:
    """Block until at least one new RF frame arrives within *timeout_s* seconds.

    Called immediately before the slide trajectory to guarantee the Teensy is
    actively streaming data.  Raises TimeoutError if the stream has stalled.
    """
    count_before = len(frames)
    deadline = time.perf_counter() + timeout_s
    while time.perf_counter() < deadline:
        if len(frames) > count_before:
            return
        time.sleep(0.05)
    raise TimeoutError(
        f"RF stream stalled: no new frame in the last {timeout_s:.0f} s. "
        "Slide aborted to avoid recording without sensor data."
    )


def estimate_contact_location_per_frame(frame_data):
    locs = np.empty(4)
    for ch, samples in enumerate(frame_data):
        arr = np.array(samples, dtype=float)
        locs[ch] = float(np.argmax(np.abs(hilbert(arr)))) if arr.size > 0 else np.nan
    return locs


def _rf_capture_summary(frames, expected_duration_s):
    if not frames:
        return {"frame_count": 0, "first_frame_t_s": None, "last_frame_t_s": None,
                "observed_duration_s": 0.0, "coverage_ratio": 0.0}
    ts = np.array([f[1] for f in frames], dtype=float)
    observed = float(ts[-1] - ts[0]) if len(ts) >= 2 else 0.0
    return {
        "frame_count": len(frames),
        "first_frame_t_s": float(ts[0]),
        "last_frame_t_s": float(ts[-1]),
        "observed_duration_s": observed,
        "coverage_ratio": observed / expected_duration_s if expected_duration_s > 0 else 0.0,
    }


def _next_trial_dir(results_dir):
    """Return the next available trial directory path without creating it."""
    counter = 0
    trial_dir = results_dir / f"trial_{counter:02d}"
    while trial_dir.exists():
        counter += 1
        trial_dir = results_dir / f"trial_{counter:02d}"
    return trial_dir


def _save_trial_result(trial_dir, speed_m_s, trial_payload, session_metadata, repeat_idx=0):
    trial_dir.mkdir(parents=True, exist_ok=True)
    speed_mm_s = speed_m_s * 1000.0
    out_path = trial_dir / f"rolling_speed_{speed_mm_s:06.1f}_mm_s_rep_{repeat_idx:02d}.pkl"
    with open(out_path, "wb") as f:
        pickle.dump({"trial": trial_payload, "session": session_metadata}, f)
    print(f"  Result saved: {out_path}")
    _plot_mean_contact([trial_payload], out_path)
    _plot_per_waveguide([trial_payload], out_path)
    return out_path


def _plot_mean_contact(trials, output_path):
    fig, ax = plt.subplots(figsize=(10, 5))
    for trial in trials:
        frames = trial["frames"]
        if not frames:
            continue
        ts = np.array([f[1] for f in frames])
        mean_locs = np.array([np.nanmean(estimate_contact_location_per_frame(f[0])) for f in frames])
        ax.plot(ts, mean_locs, label=f"{trial['speed_m_s']*1000:.1f} mm/s")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Mean contact location [sample index]")
    ax.set_title("Rolling contact (fr3_pose): mean contact location vs. time")
    handles, _ = ax.get_legend_handles_labels()
    if handles:
        ax.legend(title="Roll speed")
    ax.grid(True)
    fig.tight_layout()
    fig.savefig(output_path.with_suffix(".png"), dpi=150)
    print(f"  Plot saved: {output_path.with_suffix('.png')}")
    plt.close(fig)


def _plot_per_waveguide(trials, output_path):
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
            ax.plot(ts, locs, color=colors[ti], label=f"{trial['speed_m_s']*1000:.1f} mm/s")
        ax.set_title(f"Waveguide {ch} (S{ch})")
        ax.set_xlabel("Time [s]")
        ax.set_ylabel("Contact location [sample]")
        ax.grid(True)
        if ch == 0:
            handles, _ = ax.get_legend_handles_labels()
            if handles:
                ax.legend(title="Speed", fontsize=7)
    fig.suptitle("Rolling contact (fr3_pose): per-waveguide contact location vs. time")
    fig.tight_layout()
    out = output_path.parent / (output_path.stem + "_per_waveguide.png")
    fig.savefig(out, dpi=150)
    print(f"  Per-waveguide plot saved: {out}")
    plt.close(fig)


# ---------------------------------------------------------------------------
# Controller setup
# ---------------------------------------------------------------------------

def _setup_fr3_pose_controller(robot):
    robot.controller_switcher_client.switch_controller(FR3_POSE_CONTROLLER)
    robot.fr3_pose_controller_parameters_client.load_param_config(
        file_path=CONFIG_DIR / "controllers" / "fr3_pose" / FR3_POSE_CONFIG
    )
    deadline = time.time() + 5.0
    while time.time() < deadline:
        if robot.controller_switcher_client.get_active_controller() == FR3_POSE_CONTROLLER:
            print(f"Active controller: {FR3_POSE_CONTROLLER}")
            return
        time.sleep(0.1)
    raise RuntimeError(f"Controller switch to '{FR3_POSE_CONTROLLER}' did not complete within 5 s.")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    trial_dir = _next_trial_dir(RESULTS_DIR)
    print(f"Results will be saved under: {trial_dir}  (created on first completed roll)")

    roll_distance_m = _roll_distance_m()
    R = ROLLER_DIAMETER_M / 2.0
    d = COMPRESS_DEPTH_M
    R_eff = R - d
    y_disp_mm = roll_distance_m * 1000.0
    roller_rotation_deg = np.degrees(roll_distance_m / max(R, 1e-9))

    dry_run = DRY_RUN
    answer = input("Dry run (skip RF only; robot state + motion still required)? [y/N]: ").strip().lower()
    if answer in {"y", "yes"}:
        dry_run = True
    if dry_run:
        print("  DRY RUN — serial port will not be opened; RF frames will be empty.")

    robot = Robot(namespace=ROBOT_NAMESPACE)
    ser = (
        serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=SERIAL_TIMEOUT_SEC, write_timeout=SERIAL_TIMEOUT_SEC)
        if not dry_run else None
    )

    trials = []
    active_stop_event = None
    active_thread = None

    session_metadata = {
        "controller": FR3_POSE_CONTROLLER,
        "roller_diameter_m": ROLLER_DIAMETER_M,
        "compress_depth_m": d,
        "approach_height_m": APPROACH_HEIGHT_M,
        "finray_length_m": FINRAY_LENGTH_M,
        "roll_length_percent": ROLL_LENGTH_PERCENT,
        "roll_distance_m": roll_distance_m,
        "roller_rotation_deg_nominal": float(roller_rotation_deg),
        "speeds_m_s": ROLL_SPEEDS_M_S,
        "base_ori_euler_deg": BASE_ORI_EULER_DEG,
        "robot_namespace": ROBOT_NAMESPACE,
        "expected_rf_samples": EXPECTED_RF_SAMPLES,
        "trial_dir": str(trial_dir),
    }

    try:
        robot.wait_until_ready()
        _setup_fr3_pose_controller(robot)

        # ── Phase 0a: resolve contact and approach poses ─────────────────────
        if CONTACT_POSITION_M is not None:
            _contact_pose = Pose(np.array(CONTACT_POSITION_M, dtype=float), BASE_ORI)
            print(f"\n=== Rolling contact experiment (fr3_pose) ===")
            print(f"Contact position (from config): {CONTACT_POSITION_M}")
        else:
            _contact_pose = robot.end_effector_pose.copy()
            print(f"\n=== Rolling contact experiment (fr3_pose) ===")
            print(f"Contact position (captured at startup): {_contact_pose.position.tolist()}")

        approach_pose = Pose(
            _contact_pose.position + np.array([0.0, 0.0, APPROACH_HEIGHT_M]),
            BASE_ORI,
        )
        baseline_pose = Pose(
            _contact_pose.position + np.array(
                [0.0, 0.0, APPROACH_HEIGHT_M + BASELINE_EXTRA_HEIGHT_M]
            ),
            BASE_ORI,
        )

        print(f"Baseline pose:   {baseline_pose.position.tolist()}  "
              f"(approach + {BASELINE_EXTRA_HEIGHT_M*1000:.0f} mm)")
        print(f"Approach pose:   {approach_pose.position.tolist()}")
        print(f"Compress depth:  {d*1000:.1f} mm")
        print(f"Roller diameter: {ROLLER_DIAMETER_M*1000:.1f} mm  |  R_eff: {R_eff*1000:.1f} mm")
        print(f"Slide travel:    {y_disp_mm:.1f} mm in +Y")
        print(f"Roller rotation: {roller_rotation_deg:.1f} deg")
        print(f"Speeds:          {[s*1000 for s in ROLL_SPEEDS_M_S]} mm/s")
        print(f"Trajectory waypoints: {CONTACT_WAYPOINTS} (quintic interpolation by controller)")

        # ── Phase 0b: move to baseline pose and capture stationary RF ────────
        print("\nMoving to baseline pose for stationary RF capture...")
        _pre_baseline_pose = robot.end_effector_pose.copy()
        robot.move_to(pose=baseline_pose, speed=RETURN_SPEED_M_S)
        _wait_for_move(_pre_baseline_pose, baseline_pose, RETURN_SPEED_M_S)
        print(f"  at baseline: {_pose_xyz_rpy_str(robot.end_effector_pose.copy())}")

        static_rf_payload: dict = {
            "frames": [],
            "n_frames_requested": STATIONARY_BASELINE_FRAMES,
            "ee_pose": None,
            "expected_rf_samples": EXPECTED_RF_SAMPLES,
        }
        if STATIONARY_BASELINE_FRAMES > 0:
            baseline_ee_pose = robot.end_effector_pose.copy()
            static_rf_payload["ee_pose"] = {
                "position": baseline_ee_pose.position.tolist(),
                "orientation_quat": baseline_ee_pose.orientation.as_quat().tolist(),
                "orientation_euler_deg_xyz": baseline_ee_pose.orientation.as_euler("xyz", degrees=True).tolist(),
            }
            print(f"Capturing {STATIONARY_BASELINE_FRAMES} stationary RF frames...")
            if dry_run:
                print("  DRY RUN — skipping stationary RF capture.")
            else:
                static_rf_payload["frames"] = capture_stationary_frames(ser, STATIONARY_BASELINE_FRAMES)
                print(f"  Stationary baseline captured: {len(static_rf_payload['frames'])} frames")
            session_metadata["stationary_baseline_frames"] = STATIONARY_BASELINE_FRAMES
            session_metadata["baseline_extra_height_m"] = BASELINE_EXTRA_HEIGHT_M

        # ── Phase 0c: descend to approach pose ───────────────────────────────
        print("\nMoving to approach pose...")
        _pre_approach_pose = robot.end_effector_pose.copy()
        robot.move_to(pose=approach_pose, speed=RETURN_SPEED_M_S)
        _wait_for_move(_pre_approach_pose, approach_pose, RETURN_SPEED_M_S)
        print(f"  at approach: {_pose_xyz_rpy_str(robot.end_effector_pose.copy())}")

        input("\nPress Enter to begin sliding trials...")

        trial_plan = [
            (speed_idx, rep_idx, speed)
            for speed_idx, speed in enumerate(ROLL_SPEEDS_M_S)
            for rep_idx in range(N_REPEATS)
        ]

        for trial_idx, (speed_idx, rep_idx, speed) in enumerate(trial_plan):
            print(f"\n─── Trial {trial_idx + 1}/{len(trial_plan)}  "
                  f"speed = {speed * 1000:.1f} mm/s  "
                  f"(speed {speed_idx + 1}/{len(ROLL_SPEEDS_M_S)}, "
                  f"rep {rep_idx + 1}/{N_REPEATS}) ───")
            input("Press Enter to start this rolling session...")

            # ── 1. Descend to contact ────────────────────────────────────────
            print("  Descending to contact...")
            _pre_descend_pose = robot.end_effector_pose.copy()
            robot.move_to(pose=_contact_pose, speed=APPROACH_SPEED_M_S)
            _wait_for_move(_pre_descend_pose, _contact_pose, APPROACH_SPEED_M_S)
            print(f"  Contact point reached; holding {CONTACT_HOLD_SEC:.1f} s to settle...")
            time.sleep(CONTACT_HOLD_SEC)
            measured_contact_pose = robot.end_effector_pose.copy()
            print(f"  Contact actual: {_pose_xyz_rpy_str(measured_contact_pose)}")

            # ── 2. Compute contact-phase poses from MEASURED contact position ─
            desired_compressed_start = Pose(
                measured_contact_pose.position + np.array([0.0, 0.0, -COMPRESS_DEPTH_M]),
                BASE_ORI,
            )
            desired_compressed_end = Pose(
                desired_compressed_start.position + np.array([0.0, roll_distance_m, 0.0]),
                BASE_ORI,
            )
            desired_surface_end = Pose(
                desired_compressed_end.position + np.array([0.0, 0.0, COMPRESS_DEPTH_M]),
                BASE_ORI,
            )

            compress_duration   = _duration_for_move(measured_contact_pose, desired_compressed_start, APPROACH_SPEED_M_S)
            slide_duration      = _duration_for_move(desired_compressed_start, desired_compressed_end, speed)
            decompress_duration = _duration_for_move(desired_compressed_end, desired_surface_end, APPROACH_SPEED_M_S)
            # RF window covers compress + settle + slide + decompress
            # Each execute_trajectory call wall time = motion_duration + 0.5s internal sleep
            # + EXEC_TRAJ_TIME_OFFSET + 2.0s timeout_margin (from wait_for_trajectory_completion).
            exec_traj_overhead = (0.5 + EXEC_TRAJ_TIME_OFFSET + 2.0) * 3  # 3 traj phases (compress + slide + decompress)
            rf_window_duration  = compress_duration + SETTLE_SEC + slide_duration + decompress_duration + exec_traj_overhead

            # Build trajectories for all three phases.
            # time_offset shifts waypoints forward by EXEC_TRAJ_TIME_OFFSET so the
            # first waypoint is still in the future when the controller receives the message.
            compress_wps, compress_times = _build_contact_trajectory(
                [(measured_contact_pose, desired_compressed_start)],
                [compress_duration],
            )
            slide_wps, slide_times = _build_contact_trajectory(
                [(desired_compressed_start, desired_compressed_end)],
                [slide_duration],
            )
            decompress_wps, decompress_times = _build_contact_trajectory(
                [(desired_compressed_end, desired_surface_end)],
                [decompress_duration],
            )

            print(f"  compress {compress_duration:.1f} s  |  "
                  f"settle {SETTLE_SEC:.1f} s  |  "
                  f"slide {slide_duration:.1f} s ({CONTACT_WAYPOINTS} wps)  |  "
                  f"decompress {decompress_duration:.1f} s")
            print(f"  compress target: {_pose_xyz_rpy_str(desired_compressed_start)}")
            print(f"  slide end:       {_pose_xyz_rpy_str(desired_compressed_end)}")
            print(f"  surface end:     {_pose_xyz_rpy_str(desired_surface_end)}")

            # ── 3. RF + EE sampling setup ─────────────────────────────────────
            frames = []
            ee_poses_trial = []
            cmd_poses_trial = []
            sample_stop_event = None
            sample_thread = None
            cmd_thread = None
            slide_t0 = None

            # ── 4. Execute compress → settle → slide → decompress ─────────────
            # All three phases use execute_cartesian_traj (quintic interpolation).

            # Start RF before compress
            slide_t0 = time.perf_counter()
            if dry_run:
                print(f"  Dry-run: compress → settle → slide → decompress ({rf_window_duration:.1f} s, RF skipped)...")
            else:
                print("  Contact reached; sending 67 to Teensy...")
                frames, stop_ev, first_frame_ev, thread, _ = rf_stream_start(ser)
                active_stop_event = stop_ev
                active_thread = thread
                if not first_frame_ev.wait(timeout=SERIAL_TIMEOUT_SEC):
                    raise TimeoutError(f"No RF frame received within {SERIAL_TIMEOUT_SEC} s.")
                print("  First RF frame received; compressing...")

            # Start EE sampler
            sample_stop_event = threading.Event()
            sample_thread = threading.Thread(
                target=_sample_ee_poses,
                args=(robot, ee_poses_trial, sample_stop_event, slide_t0),
                daemon=True,
            )
            cmd_thread = threading.Thread(
                target=_sample_cmd_poses,
                args=(robot, cmd_poses_trial, sample_stop_event, slide_t0),
                daemon=True,
            )
            sample_thread.start()
            cmd_thread.start()

            # Hold before kicking off compress so the operator can verify pose
            # and the EEF/sampler threads have produced a baseline reading.
            if PRE_COMPRESS_HOLD_SEC > 0:
                print(f"  Holding {PRE_COMPRESS_HOLD_SEC:.1f} s before starting compress...")
                time.sleep(PRE_COMPRESS_HOLD_SEC)

            # Step 1: Compress — smooth descent to compressed Z
            print("  Starting compress...")
            robot.execute_cartesian_traj(compress_wps, compress_times)
            while robot.wait_for_trajectory_completion(compress_duration + EXEC_TRAJ_TIME_OFFSET):
                pass
            print(f"  Compress done: {_pose_xyz_rpy_str(robot.end_effector_pose.copy())}")

            # Step 2: Settle — hold at compressed Z before sliding
            time.sleep(SETTLE_SEC)

            # Step 3: Slide — quintic-interpolated Y translation
            # Guard: confirm the Teensy is still streaming before moving.
            if not dry_run:
                print("  Waiting for active RF stream before slide…")
                _ensure_rf_active(frames)
                print(f"  RF active ({len(frames)} frames so far) — sliding.")
            print("  Sliding...")
            robot.execute_cartesian_traj(slide_wps, slide_times)
            while robot.wait_for_trajectory_completion(slide_duration + EXEC_TRAJ_TIME_OFFSET):
                pass
            print(f"  Slide done: {_pose_xyz_rpy_str(robot.end_effector_pose.copy())}")

            # Hold between slide and decompress so the EE comes to a full stop
            # before changing direction (+Y → +Z).  Skipping this trips
            # Franka's joint_acceleration_discontinuity reflex at higher
            # slide speeds.
            if POST_SLIDE_HOLD_SEC > 0:
                print(f"  Holding {POST_SLIDE_HOLD_SEC:.1f} s after slide before decompress...")
                time.sleep(POST_SLIDE_HOLD_SEC)

            # Step 4: Decompress — smooth ascent back to surface
            robot.execute_cartesian_traj(decompress_wps, decompress_times)
            while robot.wait_for_trajectory_completion(decompress_duration + EXEC_TRAJ_TIME_OFFSET):
                pass
            print(f"  Decompress done: {_pose_xyz_rpy_str(robot.end_effector_pose.copy())}")

            # Stop EE sampler
            sample_stop_event.set()
            sample_thread.join(timeout=1.0)
            cmd_thread.join(timeout=1.0)

            # Stop RF
            if not dry_run:
                print("  Sending 69 to Teensy to stop RF...")
                rf_stream_stop(ser, active_stop_event, active_thread)
                active_stop_event = None
                active_thread = None
                print(f"  RF stream stopped: {len(frames)} frames in {rf_window_duration:.2f} s")
            else:
                print(f"  Dry-run complete: {rf_window_duration:.2f} s")

            measured_post_contact = robot.end_effector_pose.copy()

            # Slide-only EE window.
            # Time-based windowing is unreliable because stick-slip causes the
            # robot to lag behind the commanded trajectory and continue moving in Y
            # after the 25 s trajectory nominally ends.  Instead, filter by Z
            # position: the robot is in the slide phase whenever its Z is within
            # 1 mm of the compressed depth, which cleanly separates slide from the
            # compress (Z descending) and decompress (Z ascending) phases.
            compressed_z = desired_compressed_start.position[2]
            Z_TOL_M = 0.001
            ee_poses_slide_only = [
                ep for ep in ee_poses_trial
                if abs(ep["position"][2] - compressed_z) < Z_TOL_M
            ]

            # ── 5. Return to approach ─────────────────────────────────────────
            print("  Returning to approach pose...")
            _pre_return_pose = robot.end_effector_pose.copy()
            robot.move_to(pose=approach_pose, speed=RETURN_SPEED_M_S)
            _wait_for_move(_pre_return_pose, approach_pose, RETURN_SPEED_M_S)
            print(f"  return actual: {_pose_xyz_rpy_str(robot.end_effector_pose.copy())}")

            # ── 6. Save ───────────────────────────────────────────────────────
            trial_payload = {
                "speed_m_s": speed,
                "roll_distance_m": roll_distance_m,
                "roll_end_offset_m": ROLL_END_OFFSET_M,
                "roller_diameter_m": ROLLER_DIAMETER_M,
                "roller_radius_m": R,
                "roller_rotation_deg_nominal": float(roller_rotation_deg),
                "frames": list(frames),
                "ee_poses": ee_poses_trial,
                "ee_poses_slide_only": ee_poses_slide_only,
                "commanded_ee_poses": cmd_poses_trial,
                "rf_window_duration_s": rf_window_duration,
                "slide_duration_s": slide_duration,
                "compress_duration_s": compress_duration,
                "decompress_duration_s": decompress_duration,
                "approach_position": approach_pose.position.tolist(),
                "contact_position": measured_contact_pose.position.tolist(),
                "compressed_start_position": desired_compressed_start.position.tolist(),
                "compressed_end_position": desired_compressed_end.position.tolist(),
                "surface_end_position": desired_surface_end.position.tolist(),
                "boundary_states": {
                    "contact": {
                        "position": measured_contact_pose.position.tolist(),
                        "orientation_quat": measured_contact_pose.orientation.as_quat().tolist(),
                        "orientation_euler_deg_xyz": measured_contact_pose.orientation.as_euler("xyz", degrees=True).tolist(),
                    },
                    "compressed_start": {
                        "position": desired_compressed_start.position.tolist(),
                        "orientation_quat": desired_compressed_start.orientation.as_quat().tolist(),
                        "orientation_euler_deg_xyz": desired_compressed_start.orientation.as_euler("xyz", degrees=True).tolist(),
                    },
                    "compressed_end": {
                        "position": desired_compressed_end.position.tolist(),
                        "orientation_quat": desired_compressed_end.orientation.as_quat().tolist(),
                        "orientation_euler_deg_xyz": desired_compressed_end.orientation.as_euler("xyz", degrees=True).tolist(),
                    },
                    "surface_end": {
                        "position": measured_post_contact.position.tolist(),
                        "orientation_quat": measured_post_contact.orientation.as_quat().tolist(),
                        "orientation_euler_deg_xyz": measured_post_contact.orientation.as_euler("xyz", degrees=True).tolist(),
                    },
                },
            }
            trial_payload["rf_capture_summary"] = _rf_capture_summary(frames, rf_window_duration)
            trial_payload["static_rf"] = static_rf_payload
            trial_payload["repeat_idx"] = rep_idx
            trial_payload["speed_idx"] = speed_idx
            trials.append(trial_payload)
            _save_trial_result(trial_dir, speed, trial_payload, session_metadata, repeat_idx=rep_idx)

            if not dry_run:
                coverage = trial_payload["rf_capture_summary"]["coverage_ratio"]
                print(f"  RF coverage: {coverage*100:.1f}%  ({trial_payload['rf_capture_summary']['frame_count']} frames)")
                if coverage < 0.9:
                    print("  Warning: RF coverage looks short.")

    finally:
        if ser is not None and active_stop_event is not None and active_thread is not None:
            rf_stream_stop(ser, active_stop_event, active_thread)
        if ser is not None:
            ser.close()
        robot.shutdown()

    print(f"\nSession complete. Saved {len(trials)} rolling files under: {trial_dir}")


if __name__ == "__main__":
    main()
