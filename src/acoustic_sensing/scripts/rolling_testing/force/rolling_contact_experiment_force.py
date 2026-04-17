#!/usr/bin/env python3
"""Rolling contact experiment with force-controlled slide phase.

Compress and decompress are position-controlled (identical to the fr3pose version).
The slide phase uses a closed-loop force controller: Y advances at a constant speed
while Z is adjusted proportionally to the Fz error so the contact force stays at
a constant force in [FORCE_LOWER_N, FORCE_UPPER_N] = [-11, -10] N throughout the roll.

Sign convention: end_effector_wrench["force"][2] is NEGATIVE when the robot presses
into the surface.  More negative = harder contact.

Dead-band PID force control (runs at FORCE_CTRL_HZ):
    fz > FORCE_UPPER_N (-10N): f_error = fz - FORCE_UPPER_N  → press down
    fz < FORCE_LOWER_N (-11N): f_error = fz - FORCE_LOWER_N  → lift up
    FORCE_LOWER_N ≤ fz ≤ FORCE_UPPER_N: f_error = 0          → hold Z
    vz_cmd = -(Kp*f_error + Ki*∫f_error + Kd*df_error/dt) / dt

Kp is estimated online from the local fin-ray stiffness (varies along slide direction):
    k_est  = Δfz / Δz   over a STIFFNESS_WINDOW sliding window  [N/m]
    Kp     = clip(1 / k_est, KP_MIN, KP_MAX)   [m/N]

Y and Z are decoupled — Y always advances at SLIDE_SPEED_M_S, Z corrects
independently (capped at MAX_VZ_SPEED_M_S) so force corrections never stall Y:
    vy_cmd = SLIDE_SPEED_M_S                      (constant, always)
    vz_cmd = -(PID output) / dt                    (capped to ±MAX_VZ_SPEED_M_S)
    y_cmd += vy_cmd * dt
    z_cmd += vz_cmd * dt   (clamped to z_initial ± MAX_Z_CORRECTION_M)
    robot.set_target(pose=Pose([x_fixed, y_cmd, z_cmd], BASE_ORI))

Loop terminates when:
    - SLIDE_TIMEOUT_S elapsed
    - Y travel exceeds roll_distance_m
    - contact force drops below CONTACT_FORCE_THRESHOLD_N (contact lost)

Motion sequence per trial
--------------------------
(1) approach_pose   ← CONTACT_POSITION_M + APPROACH_HEIGHT_M above
         │ move_to (APPROACH_SPEED_M_S)
(2) contact_pose    ← sensor touching cylinder surface
         │ execute_cartesian_traj compress (position-controlled)
(3) compressed_start_pose  ← COMPRESS_DEPTH_M below contact
         │ settle SETTLE_SEC
         │ force-controlled slide loop (set_target at FORCE_CTRL_HZ)
         │   Y: y_start + SLIDE_SPEED_M_S * t
         │   Z: adjusted by Kp * Fz error
(4) compressed_end_pose  ← after slide
         │ execute_cartesian_traj decompress (position-controlled)
(5) surface_end_pose  ← back up COMPRESS_DEPTH_M
         │ move_to approach (RETURN_SPEED_M_S)
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

from rolling_force_config import ( # type: ignore
    APPROACH_HEIGHT_M,
    APPROACH_SPEED_M_S,
    BAUD_RATE,
    BASE_ORI_EULER_DEG,
    COMPRESS_DEPTH_M,
    CONTACT_FORCE_THRESHOLD_N,
    CONTACT_POSITION_M,
    DRY_RUN,
    EE_SAMPLE_PERIOD_SEC,
    FINRAY_LENGTH_M,
    FORCE_ACQUIRE_MAX_DEPTH_M,
    FORCE_ACQUIRE_SPEED_M_S,
    FORCE_ACQUIRE_TIMEOUT_S,
    FORCE_CTRL_HZ,
    FORCE_LOWER_N,
    FORCE_UPPER_N,
    KD_FORCE,
    KI_FORCE,
    KI_WINDUP_CLAMP_M,
    KP_INITIAL,
    KP_MAX,
    KP_MIN,
    MAX_SLIDE_DISTANCE_M,
    MAX_VZ_SPEED_M_S,
    MAX_Z_CORRECTION_M,
    RETURN_SPEED_M_S,
    ROBOT_NAMESPACE,
    ROLL_LENGTH_PERCENT,
    ROLLER_DIAMETER_M,
    SERIAL_PORT,
    SERIAL_TIMEOUT_SEC,
    SETTLE_SEC,
    SLIDE_SPEED_M_S,
    SLIDE_TIMEOUT_S,
    STIFFNESS_MIN_DZ_M,
    STIFFNESS_WINDOW,
)

FR3_POSE_CONTROLLER = "fr3_pose_controller"
FR3_POSE_CONFIG = "probing.yaml"

CONTACT_WAYPOINTS = 50
EXEC_TRAJ_TIME_OFFSET = 0.6

BASE_ORI = Rotation.from_euler("xyz", BASE_ORI_EULER_DEG, degrees=True)

CMD_START = bytes([0x43])
CMD_STOP = bytes([0x45])
CHANNEL_MARKERS = ("S0", "S1", "S2", "S3")
STREAM_END = "STREAM_END"
EXPECTED_RF_SAMPLES = 1000

RESULTS_DIR = Path(__file__).resolve().parent / "results"


# ---------------------------------------------------------------------------
# RF streaming
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
    stop_event.set()
    thread.join(timeout=SERIAL_TIMEOUT_SEC)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _roll_distance_m():
    if MAX_SLIDE_DISTANCE_M is not None:
        return float(MAX_SLIDE_DISTANCE_M)
    return FINRAY_LENGTH_M * (ROLL_LENGTH_PERCENT / 100.0)


def _duration_for_move(start_pose, end_pose, speed):
    distance = float(np.linalg.norm(end_pose.position - start_pose.position))
    return max(distance / speed, 0.25)


def _pose_xyz_rpy_str(pose):
    pos = np.array(pose.position, dtype=float)
    rpy = pose.orientation.as_euler("xyz", degrees=True)
    return (
        f"xyz=({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}) m, "
        f"rpy=({rpy[0]:.2f}, {rpy[1]:.2f}, {rpy[2]:.2f}) deg"
    )


def _build_contact_trajectory(segment_pairs, durations, n_waypoints=CONTACT_WAYPOINTS,
                               time_offset=EXEC_TRAJ_TIME_OFFSET):
    t_bounds = [0.0]
    for dur in durations:
        t_bounds.append(t_bounds[-1] + dur)
    total_duration = t_bounds[-1]

    waypoints = []
    times = []

    for t in np.linspace(0.0, total_duration, n_waypoints):
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

        waypoints.append((Pose(pos, ori), None))
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
    prev_t = prev_pos = prev_vel = None
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
            vel = acc = None
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
        prev_t = t; prev_pos = pos; prev_vel = vel
        time.sleep(period_s)


def _ensure_rf_active(frames, timeout_s: float = 5.0) -> None:
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
    counter = 0
    trial_dir = results_dir / f"trial_{counter:02d}"
    while trial_dir.exists():
        counter += 1
        trial_dir = results_dir / f"trial_{counter:02d}"
    return trial_dir


def _save_trial_result(trial_dir, trial_payload, session_metadata):
    trial_dir.mkdir(parents=True, exist_ok=True)
    speed_mm_s = SLIDE_SPEED_M_S * 1000.0
    out_path = trial_dir / f"rolling_force_{FORCE_LOWER_N:.0f}_{FORCE_UPPER_N:.0f}N_{speed_mm_s:.1f}mm_s.pkl"
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
        ax.plot(ts, mean_locs, label=f"Fz=[{trial['force_lower_n']:.0f},{trial['force_upper_n']:.0f}] N  {SLIDE_SPEED_M_S*1000:.1f} mm/s")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Mean contact location [sample index]")
    ax.set_title("Force-controlled rolling: mean contact location vs. time")
    handles, _ = ax.get_legend_handles_labels()
    if handles:
        ax.legend()
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
            ax.plot(ts, locs, color=colors[ti],
                    label=f"Fz=[{trial['force_lower_n']:.0f},{trial['force_upper_n']:.0f}] N  {SLIDE_SPEED_M_S*1000:.1f} mm/s")
        ax.set_title(f"Waveguide {ch} (S{ch})")
        ax.set_xlabel("Time [s]")
        ax.set_ylabel("Contact location [sample]")
        ax.grid(True)
        if ch == 0:
            handles, _ = ax.get_legend_handles_labels()
            if handles:
                ax.legend(fontsize=7)
    fig.suptitle("Force-controlled rolling: per-waveguide contact location vs. time")
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
# Force-controlled slide
# ---------------------------------------------------------------------------

def _force_controlled_slide(robot, compressed_start_pose, roll_distance_m,
                              ee_poses_slide_force, t0):
    """Run the force-controlled slide loop.

    Advances Y at SLIDE_SPEED_M_S while keeping fz in [FORCE_LOWER_N, FORCE_UPPER_N].
    Records per-step telemetry in ee_poses_slide_force.

    Returns:
        actual_end_pose (Pose): final measured EEF pose after the slide.
        termination_reason (str): one of "timeout", "distance", "contact_lost", "dry_run".
    """
    x_fixed = float(compressed_start_pose.position[0])
    y_start = float(compressed_start_pose.position[1])
    z_initial = float(compressed_start_pose.position[2])
    z_cmd = z_initial
    y_cmd = y_start                              # accumulated incrementally each step

    z_min = z_initial - MAX_Z_CORRECTION_M   # lower limit  (deeper into surface)
    z_max = z_initial + MAX_Z_CORRECTION_M   # upper limit  (away from surface)

    dt = 1.0 / FORCE_CTRL_HZ
    t0_slide = time.perf_counter()
    elapsed = 0.0
    termination_reason = "timeout"

    # ── Force acquisition: press slowly until fz <= FORCE_UPPER_N ───────────
    # The position-controlled compress may leave the sensor with insufficient
    # force. We press down at FORCE_ACQUIRE_SPEED_M_S until the target band is
    # reached, then rebase z_initial and the Z correction window around that depth.
    z_acquire = z_initial
    z_acquire_limit = z_initial - FORCE_ACQUIRE_MAX_DEPTH_M
    t_acq_start = time.perf_counter()
    acq_dt = 1.0 / FORCE_CTRL_HZ
    acq_step = FORCE_ACQUIRE_SPEED_M_S * acq_dt   # Z decrement per step [m]

    try:
        fz_now = float(robot.end_effector_wrench["force"][2])
    except RuntimeError:
        fz_now = 0.0

    if fz_now > FORCE_UPPER_N:
        print("  Acquiring contact force...", end="", flush=True)
        while True:
            if time.perf_counter() - t_acq_start > FORCE_ACQUIRE_TIMEOUT_S:
                raise TimeoutError(
                    f"Force acquisition timed out after {FORCE_ACQUIRE_TIMEOUT_S:.0f} s "
                    f"(last Fz={fz_now:.3f} N, target ≤ {FORCE_UPPER_N:.1f} N)."
                )
            try:
                fz_now = float(robot.end_effector_wrench["force"][2])
            except RuntimeError:
                pass
            depth_mm = (z_acquire - z_initial) * 1000.0
            print(
                f"\r  Acquiring: Fz={fz_now:+7.3f}N  target≤{FORCE_UPPER_N:.1f}N  "
                f"depth={depth_mm:+6.2f}mm / -{FORCE_ACQUIRE_MAX_DEPTH_M*1000:.0f}mm",
                end="", flush=True,
            )
            if fz_now <= FORCE_UPPER_N:
                break
            z_acquire = float(np.clip(z_acquire - acq_step, z_acquire_limit, z_initial))
            robot.set_target(
                pose=Pose(np.array([x_fixed, y_start, z_acquire]),
                          compressed_start_pose.orientation)
            )
            time.sleep(acq_dt)
        acq_depth_mm = (z_acquire - z_initial) * 1000.0
        print(f"\n  Force acquired: Fz={fz_now:+.3f}N  extra depth={acq_depth_mm:+.2f}mm")
    else:
        print(f"  Force already in band: Fz={fz_now:+.3f}N — no acquisition needed.")

    # Rebase the Z correction window around the acquired depth
    z_initial = z_acquire
    z_cmd = z_acquire
    z_min = z_initial - MAX_Z_CORRECTION_M
    z_max = z_initial + MAX_Z_CORRECTION_M

    # Adaptive Kp: sliding window of recent (z_cmd, fz) for stiffness estimation.
    kp = KP_INITIAL
    z_window  = []   # list of z_cmd values
    fz_window = []   # list of fz values

    # PID state
    integral_z = 0.0    # accumulated I term [m]
    prev_f_error = None # previous f_error for derivative

    print(f"  Force-controlled slide (adaptive-PID): "
          f"band=[{FORCE_LOWER_N:.1f}, {FORCE_UPPER_N:.1f}] N, "
          f"Kp=adaptive (init={KP_INITIAL:.4f}, [{KP_MIN:.5f},{KP_MAX:.4f}] m/N), "
          f"Ki={KI_FORCE:.5f} m/(N·s)  Kd={KD_FORCE:.5f} m·s/N, "
          f"total speed={SLIDE_SPEED_M_S*1000:.1f} mm/s, "
          f"max_dist={roll_distance_m*1000:.1f} mm, timeout={SLIDE_TIMEOUT_S:.0f} s")

    next_tick = time.perf_counter()   # absolute time for the next iteration start
    actual_dt = dt                    # measured real step duration (for integration)

    while True:
        t_iter_start = time.perf_counter()
        elapsed = t_iter_start - t0_slide
        y_traveled = y_cmd - y_start

        # Termination: distance reached
        if y_traveled >= roll_distance_m:
            termination_reason = "distance"
            break

        # Termination: timeout
        if elapsed >= SLIDE_TIMEOUT_S:
            termination_reason = "timeout"
            break

        # Read wrench
        try:
            wrench = robot.end_effector_wrench
            fz = float(wrench["force"][2])
        except RuntimeError:
            fz = (FORCE_UPPER_N + FORCE_LOWER_N) / 2.0  # no update — assume mid-band

        # Termination: contact lost (fz becomes less negative than threshold)
        if fz > CONTACT_FORCE_THRESHOLD_N:
            termination_reason = "contact_lost"
            print(f"\n  Contact lost: Fz={fz:.3f} N > threshold {CONTACT_FORCE_THRESHOLD_N:.2f} N")
            break

        # ── Adaptive Kp: update stiffness estimate from sliding window ──────
        z_window.append(z_cmd)
        fz_window.append(fz)
        if len(z_window) > STIFFNESS_WINDOW:
            z_window.pop(0)
            fz_window.pop(0)

        if len(z_window) >= 2:
            dz  = z_window[-1]  - z_window[0]
            dfz = fz_window[-1] - fz_window[0]
            if abs(dz) >= STIFFNESS_MIN_DZ_M and dz != 0.0:
                k_est = dfz / dz
                if k_est > 0.0:
                    kp = float(np.clip(1.0 / k_est, KP_MIN, KP_MAX))

        # ── Dead-band PID Z correction ──────────────────────────────────────
        # Error is relative to the nearest band edge; zero inside the band.
        if fz > FORCE_UPPER_N:
            f_error = fz - FORCE_UPPER_N   # positive → press down
        elif fz < FORCE_LOWER_N:
            f_error = fz - FORCE_LOWER_N   # negative → lift up
        else:
            f_error = 0.0                  # inside band → hold Z

        # Use actual_dt for integration so jitter doesn't corrupt the accumulators
        integral_z += f_error * actual_dt
        integral_z = float(np.clip(integral_z, -KI_WINDUP_CLAMP_M, KI_WINDUP_CLAMP_M))

        if prev_f_error is not None:
            d_f_error = (f_error - prev_f_error) / actual_dt
        else:
            d_f_error = 0.0
        prev_f_error = f_error

        vz_cmd = -(kp * f_error + KI_FORCE * integral_z + KD_FORCE * d_f_error) / actual_dt
        vz_cmd = float(np.clip(vz_cmd, -MAX_VZ_SPEED_M_S, MAX_VZ_SPEED_M_S))

        # Y always advances at full speed — decoupled from Z correction.
        vy_cmd = SLIDE_SPEED_M_S

        # Advance positions using actual elapsed step time for accuracy
        z_cmd = float(np.clip(z_cmd + vz_cmd * actual_dt, z_min, z_max))
        y_cmd += vy_cmd * actual_dt

        # Stream new target pose
        target_pose = Pose(np.array([x_fixed, y_cmd, z_cmd]), compressed_start_pose.orientation)
        robot.set_target(pose=target_pose)

        # Log telemetry
        t_now = time.perf_counter() - t0
        try:
            ee = robot.end_effector_pose
            actual_pos = ee.position.tolist()
        except RuntimeError:
            actual_pos = [x_fixed, y_cmd, z_cmd]

        total_speed = float(np.sqrt(vy_cmd ** 2 + vz_cmd ** 2))
        z_corr_mm = (z_cmd - z_initial) * 1000.0

        ee_poses_slide_force.append({
            "t": t_now,
            "elapsed_slide_s": elapsed,
            "y_traveled_m": y_traveled,
            "y_cmd": y_cmd,
            "z_cmd": z_cmd,
            "vy_cmd": vy_cmd,
            "vz_cmd": vz_cmd,
            "total_speed_cmd": total_speed,
            "fz_measured": fz,
            "f_error": f_error,
            "in_band": (FORCE_LOWER_N <= fz <= FORCE_UPPER_N),
            "z_correction_m": z_cmd - z_initial,
            "kp": kp,
            "integral_z": integral_z,
            "d_f_error": d_f_error,
            "position_actual": actual_pos,
        })

        # ── Live debug line (overwrites in place) ───────────────────────────
        band_str = "OK " if FORCE_LOWER_N <= fz <= FORCE_UPPER_N else ("DN " if fz > FORCE_UPPER_N else "UP ")
        print(
            f"\r  [{band_str}] "
            f"Fz={fz:+7.3f}N  err={f_error:+6.3f}N  "
            f"Z={z_corr_mm:+6.3f}mm  "
            f"Y={y_traveled*1000:6.1f}/{roll_distance_m*1000:.0f}mm  "
            f"vy={vy_cmd*1000:4.2f}mm/s  vz={vz_cmd*1000:+5.2f}mm/s  "
            f"spd={total_speed*1000:.2f}mm/s  "
            f"Kp={kp:.5f}",
            end="", flush=True,
        )

        # ── Precise loop timing ─────────────────────────────────────────────
        # Schedule the next tick at a fixed absolute time so computation
        # overhead doesn't accumulate into speed error.
        next_tick += dt
        sleep_remaining = next_tick - time.perf_counter()
        if sleep_remaining > 0:
            time.sleep(sleep_remaining)

        # Measure actual step duration for use in next iteration's integrators
        actual_dt = time.perf_counter() - t_iter_start

    print()  # newline after the live debug line

    # Return the COMMANDED end-of-slide pose (last set_target), not the actual.
    # The Franka reference continues from the previous trajectory's endpoint; passing
    # the actual pose (which lags commanded by surface compliance) causes a reference
    # step when the next execute_cartesian_traj fires, triggering the reflex.
    commanded_end_pose = Pose(
        np.array([x_fixed, y_cmd, z_cmd]),
        compressed_start_pose.orientation,
    )

    try:
        actual_end_pose = robot.end_effector_pose.copy()
        print(f"  Slide done ({termination_reason}): "
              f"Y traveled {(y_cmd - y_start) * 1000:.1f} mm in {elapsed:.1f} s  "
              f"  commanded: {_pose_xyz_rpy_str(commanded_end_pose)}  "
              f"actual: {_pose_xyz_rpy_str(actual_end_pose)}")
    except RuntimeError:
        print(f"  Slide done ({termination_reason}): "
              f"Y traveled {(y_cmd - y_start) * 1000:.1f} mm in {elapsed:.1f} s  "
              f"  commanded: {_pose_xyz_rpy_str(commanded_end_pose)}")

    return commanded_end_pose, termination_reason


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
        "experiment_type": "force_controlled_slide",
        "roller_diameter_m": ROLLER_DIAMETER_M,
        "compress_depth_m": d,
        "approach_height_m": APPROACH_HEIGHT_M,
        "finray_length_m": FINRAY_LENGTH_M,
        "roll_length_percent": ROLL_LENGTH_PERCENT,
        "roll_distance_m": roll_distance_m,
        "roller_rotation_deg_nominal": float(roller_rotation_deg),
        "force_upper_n": FORCE_UPPER_N,
        "force_lower_n": FORCE_LOWER_N,
        "force_acquire_max_depth_m": FORCE_ACQUIRE_MAX_DEPTH_M,
        "force_acquire_speed_m_s": FORCE_ACQUIRE_SPEED_M_S,
        "kp_initial": KP_INITIAL,
        "kp_min": KP_MIN,
        "kp_max": KP_MAX,
        "ki_force": KI_FORCE,
        "kd_force": KD_FORCE,
        "ki_windup_clamp_m": KI_WINDUP_CLAMP_M,
        "stiffness_window": STIFFNESS_WINDOW,
        "max_z_correction_m": MAX_Z_CORRECTION_M,
        "max_vz_speed_m_s": MAX_VZ_SPEED_M_S,
        "contact_force_threshold_n": CONTACT_FORCE_THRESHOLD_N,
        "force_ctrl_hz": FORCE_CTRL_HZ,
        "slide_speed_m_s": SLIDE_SPEED_M_S,
        "slide_timeout_s": SLIDE_TIMEOUT_S,
        "base_ori_euler_deg": BASE_ORI_EULER_DEG,
        "robot_namespace": ROBOT_NAMESPACE,
        "expected_rf_samples": EXPECTED_RF_SAMPLES,
        "trial_dir": str(trial_dir),
    }

    try:
        robot.wait_until_ready()
        _setup_fr3_pose_controller(robot)

        # ── Resolve contact and approach poses ──────────────────────────────
        if CONTACT_POSITION_M is not None:
            _contact_pose = Pose(np.array(CONTACT_POSITION_M, dtype=float), BASE_ORI)
            print(f"\n=== Rolling contact experiment (force-controlled slide) ===")
            print(f"Contact position (from config): {CONTACT_POSITION_M}")
        else:
            _contact_pose = robot.end_effector_pose.copy()
            print(f"\n=== Rolling contact experiment (force-controlled slide) ===")
            print(f"Contact position (captured at startup): {_contact_pose.position.tolist()}")

        approach_pose = Pose(
            _contact_pose.position + np.array([0.0, 0.0, APPROACH_HEIGHT_M]),
            BASE_ORI,
        )

        print(f"Approach pose:      {approach_pose.position.tolist()}")
        print(f"Compress depth:     {d*1000:.1f} mm  (position-controlled)")
        print(f"Roller diameter:    {ROLLER_DIAMETER_M*1000:.1f} mm  |  R_eff: {R_eff*1000:.1f} mm")
        print(f"Slide distance:     {y_disp_mm:.1f} mm in +Y")
        print(f"Slide speed:        {SLIDE_SPEED_M_S*1000:.1f} mm/s")
        print(f"Force band:         [{FORCE_LOWER_N:.1f}, {FORCE_UPPER_N:.1f}] N")
        print(f"Kp:                 adaptive (init={KP_INITIAL:.4f}, range=[{KP_MIN:.5f}, {KP_MAX:.4f}] m/N)")
        print(f"Max Z correction:   ±{MAX_Z_CORRECTION_M*1000:.1f} mm")
        print(f"Force ctrl rate:    {FORCE_CTRL_HZ:.0f} Hz")
        print(f"Slide timeout:      {SLIDE_TIMEOUT_S:.0f} s")
        print(f"Roller rotation:    {roller_rotation_deg:.1f} deg")

        # ── Move to approach pose ────────────────────────────────────────────
        current_pose = robot.end_effector_pose.copy()
        if np.linalg.norm(current_pose.position - approach_pose.position) > 1e-3:
            print("\nMoving to approach pose...")
            robot.move_to(pose=approach_pose, speed=RETURN_SPEED_M_S)
            time.sleep(SETTLE_SEC)
            print(f"  at approach: {_pose_xyz_rpy_str(robot.end_effector_pose.copy())}")

        input("\nPress Enter to begin force-controlled rolling trial...")

        trial_num = 0
        while True:
            trial_num += 1
            print(f"\n─── Trial {trial_num} ───")
            answer = input("Press Enter to start this trial, or 'q' to quit: ").strip().lower()
            if answer in {"q", "quit"}:
                print("Stopping trials.")
                break

            # ── 1. Descend to contact ────────────────────────────────────────
            print("  Descending to contact...")
            robot.move_to(pose=_contact_pose, speed=APPROACH_SPEED_M_S)
            time.sleep(SETTLE_SEC)
            measured_contact_pose = robot.end_effector_pose.copy()
            print(f"  Contact actual: {_pose_xyz_rpy_str(measured_contact_pose)}")

            # ── 2. Compute phase poses from MEASURED contact ─────────────────
            desired_compressed_start = Pose(
                measured_contact_pose.position + np.array([0.0, 0.0, -COMPRESS_DEPTH_M]),
                BASE_ORI,
            )

            compress_duration = _duration_for_move(measured_contact_pose, desired_compressed_start,
                                                    APPROACH_SPEED_M_S)
            exec_traj_overhead_compress = 0.5 + EXEC_TRAJ_TIME_OFFSET + 2.0

            compress_wps, compress_times = _build_contact_trajectory(
                [(measured_contact_pose, desired_compressed_start)],
                [compress_duration],
            )

            print(f"  compress {compress_duration:.1f} s  |  "
                  f"settle {SETTLE_SEC:.1f} s  |  "
                  f"force slide timeout {SLIDE_TIMEOUT_S:.0f} s")
            print(f"  compress target: {_pose_xyz_rpy_str(desired_compressed_start)}")

            # ── 3. RF + EE sampling setup ─────────────────────────────────────
            frames = []
            ee_poses_trial = []
            cmd_poses_trial = []
            ee_poses_slide_force = []
            sample_stop_event = None
            sample_thread = None
            cmd_thread = None

            slide_t0 = time.perf_counter()

            # Start RF before compress
            if dry_run:
                print("  Dry-run: compress → settle → force-slide → decompress (RF skipped)...")
            else:
                print("  Contact reached; sending start to Teensy...")
                frames, stop_ev, first_frame_ev, thread, _ = rf_stream_start(ser)
                active_stop_event = stop_ev
                active_thread = thread
                if not first_frame_ev.wait(timeout=SERIAL_TIMEOUT_SEC):
                    raise TimeoutError(f"No RF frame received within {SERIAL_TIMEOUT_SEC} s.")
                print("  First RF frame received; compressing...")

            # Start EE sampler and commanded pose sampler
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

            # ── 4. Compress (position-controlled) ────────────────────────────
            robot.execute_cartesian_traj(compress_wps, compress_times)
            while robot.wait_for_trajectory_completion(compress_duration + EXEC_TRAJ_TIME_OFFSET):
                pass
            print(f"  Compress done: {_pose_xyz_rpy_str(robot.end_effector_pose.copy())}")

            # Re-read actual compressed pose to seed the force-control loop
            actual_compressed_start = robot.end_effector_pose.copy()

            # ── 5. Settle ────────────────────────────────────────────────────
            time.sleep(SETTLE_SEC)

            # ── 6. RF guard before slide ──────────────────────────────────────
            if not dry_run:
                print("  Waiting for active RF stream before slide…")
                _ensure_rf_active(frames)
                print(f"  RF active ({len(frames)} frames so far) — sliding.")

            # ── 7. Force-controlled slide ─────────────────────────────────────
            actual_slide_end_pose, termination_reason = _force_controlled_slide(
                robot, actual_compressed_start, roll_distance_m,
                ee_poses_slide_force, slide_t0,
            )

            # ── 8. Decompress (position-controlled) from commanded slide end ──
            print("  Settling after slide...")
            time.sleep(SETTLE_SEC)
            # Keep actual_slide_end_pose as the COMMANDED end (from _force_controlled_slide).
            # Reading the actual pose here would cause a reference step in the next trajectory.
            desired_surface_end = Pose(
                actual_slide_end_pose.position + np.array([0.0, 0.0, COMPRESS_DEPTH_M]),
                BASE_ORI,
            )
            decompress_duration = _duration_for_move(actual_slide_end_pose, desired_surface_end,
                                                      APPROACH_SPEED_M_S)
            decompress_wps, decompress_times = _build_contact_trajectory(
                [(actual_slide_end_pose, desired_surface_end)],
                [decompress_duration],
            )

            robot.execute_cartesian_traj(decompress_wps, decompress_times)
            while robot.wait_for_trajectory_completion(decompress_duration + EXEC_TRAJ_TIME_OFFSET):
                pass
            print(f"  Decompress done: {_pose_xyz_rpy_str(robot.end_effector_pose.copy())}")
            print("  Settling after decompress...")
            time.sleep(1.0)

            # Stop EE sampler
            sample_stop_event.set()
            sample_thread.join(timeout=1.0)

            # Stop RF
            rf_window_duration = time.perf_counter() - slide_t0
            if not dry_run:
                print("  Sending stop to Teensy...")
                rf_stream_stop(ser, active_stop_event, active_thread)
                active_stop_event = None
                active_thread = None
                print(f"  RF stream stopped: {len(frames)} frames in {rf_window_duration:.2f} s")
            else:
                print(f"  Dry-run complete: {rf_window_duration:.2f} s")

            measured_post_contact = robot.end_effector_pose.copy()

            # Slide-only EE window (Z within 1 mm of compressed depth)
            compressed_z = desired_compressed_start.position[2]
            Z_TOL_M = 0.001
            ee_poses_slide_only = [
                ep for ep in ee_poses_trial
                if abs(ep["position"][2] - compressed_z) < Z_TOL_M
            ]

            # ── 9. Return to approach (ascend first, then sweep back) ────────────
            current_after_decompress = robot.end_effector_pose.copy()
            lift_pose = Pose(
                np.array([
                    current_after_decompress.position[0],
                    current_after_decompress.position[1],
                    approach_pose.position[2],
                ]),
                BASE_ORI,
            )
            print("  Ascending to approach height...")
            lift_duration = _duration_for_move(current_after_decompress, lift_pose, APPROACH_SPEED_M_S)
            lift_wps, lift_times = _build_contact_trajectory(
                [(current_after_decompress, lift_pose)],
                [lift_duration],
            )
            robot.execute_cartesian_traj(lift_wps, lift_times)
            while robot.wait_for_trajectory_completion(lift_duration + EXEC_TRAJ_TIME_OFFSET):
                pass
            time.sleep(SETTLE_SEC)

            print("  Returning to approach pose...")
            current_after_lift = robot.end_effector_pose.copy()
            return_duration = _duration_for_move(current_after_lift, approach_pose, RETURN_SPEED_M_S)
            return_wps, return_times = _build_contact_trajectory(
                [(current_after_lift, approach_pose)],
                [return_duration],
            )
            robot.execute_cartesian_traj(return_wps, return_times)
            while robot.wait_for_trajectory_completion(return_duration + EXEC_TRAJ_TIME_OFFSET):
                pass
            time.sleep(SETTLE_SEC)
            print(f"  return actual: {_pose_xyz_rpy_str(robot.end_effector_pose.copy())}")

            # ── 10. Force control diagnostics ────────────────────────────────
            if ee_poses_slide_force:
                fz_vals = [s["fz_measured"] for s in ee_poses_slide_force]
                in_band = sum(1 for s in ee_poses_slide_force if s["in_band"])
                pct_in_band = 100.0 * in_band / len(ee_poses_slide_force)
                print(f"  Fz: mean={np.mean(fz_vals):.3f} N  "
                      f"std={np.std(fz_vals):.3f} N  "
                      f"min={np.min(fz_vals):.3f} N  "
                      f"max={np.max(fz_vals):.3f} N  "
                      f"(band=[{FORCE_LOWER_N:.1f},{FORCE_UPPER_N:.1f}] N  in-band {pct_in_band:.1f}%)")
                z_corr_vals = [s["z_correction_m"] for s in ee_poses_slide_force]
                print(f"  Z correction: "
                      f"mean={np.mean(z_corr_vals)*1000:.3f} mm  "
                      f"max={np.max(z_corr_vals)*1000:.3f} mm  "
                      f"min={np.min(z_corr_vals)*1000:.3f} mm")

            # ── 11. Save ──────────────────────────────────────────────────────
            trial_payload = {
                "trial_num": trial_num,
                "force_upper_n": FORCE_UPPER_N,
                "force_lower_n": FORCE_LOWER_N,
                "slide_speed_m_s": SLIDE_SPEED_M_S,
                "kp_initial": KP_INITIAL,
                "max_z_correction_m": MAX_Z_CORRECTION_M,
                "termination_reason": termination_reason,
                "roll_distance_m": roll_distance_m,
                "roller_rotation_deg_nominal": float(roller_rotation_deg),
                "frames": list(frames),
                "ee_poses": ee_poses_trial,
                "commanded_ee_poses": cmd_poses_trial,
                "ee_poses_slide_only": ee_poses_slide_only,
                "ee_poses_slide_force": ee_poses_slide_force,
                "rf_window_duration_s": rf_window_duration,
                "compress_duration_s": compress_duration,
                "decompress_duration_s": decompress_duration,
                "approach_position": approach_pose.position.tolist(),
                "contact_position": measured_contact_pose.position.tolist(),
                "compressed_start_position": desired_compressed_start.position.tolist(),
                "compressed_start_actual": actual_compressed_start.position.tolist(),
                "compressed_end_actual": actual_slide_end_pose.position.tolist(),
                "surface_end_position": desired_surface_end.position.tolist(),
                "boundary_states": {
                    "contact": {
                        "position": measured_contact_pose.position.tolist(),
                        "orientation_quat": measured_contact_pose.orientation.as_quat().tolist(),
                        "orientation_euler_deg_xyz": measured_contact_pose.orientation.as_euler("xyz", degrees=True).tolist(),
                    },
                    "compressed_start": {
                        "position": actual_compressed_start.position.tolist(),
                        "orientation_quat": actual_compressed_start.orientation.as_quat().tolist(),
                        "orientation_euler_deg_xyz": actual_compressed_start.orientation.as_euler("xyz", degrees=True).tolist(),
                    },
                    "compressed_end": {
                        "position": actual_slide_end_pose.position.tolist(),
                        "orientation_quat": actual_slide_end_pose.orientation.as_quat().tolist(),
                        "orientation_euler_deg_xyz": actual_slide_end_pose.orientation.as_euler("xyz", degrees=True).tolist(),
                    },
                    "surface_end": {
                        "position": measured_post_contact.position.tolist(),
                        "orientation_quat": measured_post_contact.orientation.as_quat().tolist(),
                        "orientation_euler_deg_xyz": measured_post_contact.orientation.as_euler("xyz", degrees=True).tolist(),
                    },
                },
            }
            trial_payload["rf_capture_summary"] = _rf_capture_summary(frames, rf_window_duration)
            trials.append(trial_payload)
            _save_trial_result(trial_dir, trial_payload, session_metadata)

            if not dry_run:
                coverage = trial_payload["rf_capture_summary"]["coverage_ratio"]
                print(f"  RF coverage: {coverage*100:.1f}%  "
                      f"({trial_payload['rf_capture_summary']['frame_count']} frames)")
                if coverage < 0.9:
                    print("  Warning: RF coverage looks short.")

    finally:
        if ser is not None and active_stop_event is not None and active_thread is not None:
            rf_stream_stop(ser, active_stop_event, active_thread)
        if ser is not None:
            ser.close()
        robot.shutdown()

    print(f"\nSession complete. Saved {len(trials)} force-controlled rolling files under: {trial_dir}")


if __name__ == "__main__":
    main()
