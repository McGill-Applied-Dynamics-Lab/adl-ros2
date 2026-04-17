#!/usr/bin/env python3
"""Bar orientation experiment: press sensor onto bar, hold, retract, record angle.

Hardware setup
--------------
- FR3 robot with finray gripper in 90-degree custom adapter (orientation [180, 0, 0]).
- Horizontal bar mounted below the gripper, rotatable to discrete angles.
- Set CONTACT_POSITION_M in orientation_config.py to the XYZ where the sensor
  just touches the bar at φ=0. If None, jog the robot to contact first.

Motion sequence per trial
--------------------------
(1) approach_pose  ← CONTACT_POSITION_M + APPROACH_HEIGHT_M above
        │ descend at APPROACH_SPEED to contact
(2) contact_pose   ← sensor just touching bar
        │ send 67 to Teensy → RF streaming starts
        │ compress COMPRESS_DEPTH_M down at APPROACH_SPEED
(3) compressed_pose ← sensor preloaded into bar
        │ hold HOLD_AT_CONTACT_SEC
        │ lift back to contact_pose at APPROACH_SPEED
(4) contact_pose again
        │ send 69 to Teensy → RF streaming stops
        │ return to approach_pose at RETURN_SPEED
(5) approach_pose  ← prompt for bar angle, then press Enter for next trial

Output
------
- Pickle files in results/trial_NN/depth_Dmm_angle_A.adeg_NN.pkl
  where D = COMPRESS_DEPTH_M in mm and A = angle entered by user.
"""

import pickle
import threading
import time
from pathlib import Path

import numpy as np
import serial
from scipy.spatial.transform import Rotation

from arm_client import CONFIG_DIR
from arm_client.robot import Pose, Robot

from orientation_config import (
    APPROACH_HEIGHT_M,
    APPROACH_SPEED_M_S,
    BASE_ORI_EULER_DEG,
    BAUD_RATE,
    COMPRESS_DEPTH_M,
    CONTACT_POSITION_M,
    CONTROLLER_NAME,
    DRY_RUN,
    EE_SAMPLE_PERIOD_SEC,
    HOLD_AT_CONTACT_SEC,
    HOLD_FORCE_CONTROL,
    HOLD_FORCE_CTRL_HZ,
    HOLD_FORCE_DEADBAND_N,
    HOLD_FORCE_KP,
    HOLD_FORCE_MAX_Z_CORR_M,
    HOLD_SETTLE_SEC,
    HOLD_TORQUE_ADAPT,
    N_REPEATS,
    HOLD_TORQUE_DEADBAND_NM,
    HOLD_TORQUE_KP_DEG_PER_NM,
    HOLD_TORQUE_MAX_CORR_DEG,
    PROBING_CONFIG_REL,
    RETURN_SPEED_M_S,
    ROBOT_NAMESPACE,
    SERIAL_PORT,
    SERIAL_TIMEOUT_SEC,
    WAVEGUIDE_LATERAL_POSITIONS_MM as _WAVEGUIDE_LATERAL_POSITIONS_MM,
    WAVEGUIDE_SAMPLE_PITCH_MM,
)

PROBING_CONFIG = CONFIG_DIR / PROBING_CONFIG_REL
BASE_ORI = Rotation.from_euler("xyz", BASE_ORI_EULER_DEG, degrees=True)
WAVEGUIDE_LATERAL_POSITIONS_MM = np.array(_WAVEGUIDE_LATERAL_POSITIONS_MM, dtype=float)

CMD_START = bytes([0x43])
CMD_STOP = bytes([0x45])
CHANNEL_MARKERS = ("S0", "S1", "S2", "S3")
STREAM_END = "STREAM_END"

RESULTS_DIR = Path(__file__).resolve().parent / "results"


# ---------------------------------------------------------------------------
# RF streaming
# ---------------------------------------------------------------------------

def _stream_reader(
    ser: serial.Serial,
    frames: list,
    stop_event: threading.Event,
    first_frame_event: threading.Event,
    t0: float,
) -> None:
    """Background thread: collect 4-channel RF frames and timestamp them.

    Each entry appended to *frames* is (frame_data, relative_timestamp_s)
    where frame_data = [ch0_samples, ch1_samples, ch2_samples, ch3_samples].
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


def rf_stream_start(
    ser: serial.Serial,
) -> tuple[list, threading.Event, threading.Event, threading.Thread, float]:
    """Send 67 (CMD_START) and begin collecting RF frames in background.

    Returns (frames, stop_event, first_frame_event, thread, t0).
    """
    frames: list = []
    stop_event = threading.Event()
    first_frame_event = threading.Event()
    t0 = time.perf_counter()
    ser.reset_input_buffer()
    written = ser.write(CMD_START)
    if written != len(CMD_START):
        raise RuntimeError(f"Failed to send start byte to Teensy: wrote {written} bytes")
    thread = threading.Thread(
        target=_stream_reader,
        args=(ser, frames, stop_event, first_frame_event, t0),
        daemon=True,
    )
    thread.start()
    return frames, stop_event, first_frame_event, thread, t0


def rf_stream_stop(
    ser: serial.Serial,
    stop_event: threading.Event,
    thread: threading.Thread,
) -> None:
    """Send 69 (CMD_STOP) and wait for the reader thread to flush."""
    written = ser.write(CMD_STOP)
    if written != len(CMD_STOP):
        raise RuntimeError(f"Failed to send stop byte to Teensy: wrote {written} bytes")
    stop_event.set()
    thread.join(timeout=SERIAL_TIMEOUT_SEC)


# ---------------------------------------------------------------------------
# Controller setup
# ---------------------------------------------------------------------------

def _setup_pose_controller(robot: Robot) -> None:
    robot.controller_switcher_client.switch_controller(CONTROLLER_NAME)
    robot.fr3_pose_controller_parameters_client.load_param_config(file_path=PROBING_CONFIG)
    deadline = time.time() + 5.0
    while time.time() < deadline:
        if robot.controller_switcher_client.get_active_controller() == CONTROLLER_NAME:
            print(f"Active controller: {CONTROLLER_NAME}")
            return
        time.sleep(0.1)
    raise RuntimeError(
        f"Controller switch to '{CONTROLLER_NAME}' did not complete within 5 s."
    )


def _move(robot: Robot, pose: Pose, speed: float, settle: float = 0.3) -> None:
    robot.move_to(pose=pose, speed=speed)
    time.sleep(settle)


def _cartesian_move(robot: Robot, start: Pose, end: Pose, speed: float, settle: float = 0.0) -> None:
    """Execute a smooth Cartesian straight-line move using quintic interpolation."""
    distance = float(np.linalg.norm(end.position - start.position))
    duration = max(distance / speed, 0.1)
    robot.execute_cartesian_traj(
        [(start,), (end,)],
        [0.0, duration],
        max_linear_velocity=speed,
    )
    while robot.wait_for_trajectory_completion(duration):
        time.sleep(0.02)
    time.sleep(settle)


# ---------------------------------------------------------------------------
# EEF sampling
# ---------------------------------------------------------------------------

def _make_ee_sampler(get_pose, get_wrench):
    """Return a background-thread function that records pose/wrench/vel/acc."""
    def _sampler(poses: list, stop_event: threading.Event,
                 t0: float, period_s: float = EE_SAMPLE_PERIOD_SEC) -> None:
        prev_t = prev_pos = prev_vel = None
        while not stop_event.is_set():
            try:
                ee = get_pose()
                wrench = get_wrench()
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
            poses.append({
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
    return _sampler


def _sample_ee_poses(robot: Robot, ee_poses: list, stop_event: threading.Event,
                     t0: float, period_s: float = EE_SAMPLE_PERIOD_SEC) -> None:
    """Background thread: record measured EEF pose, wrench, velocity and acceleration."""
    _make_ee_sampler(
        lambda: robot.end_effector_pose,
        lambda: robot.end_effector_wrench,
    )(ee_poses, stop_event, t0, period_s)


def _sample_cmd_poses(robot: Robot, cmd_poses: list, stop_event: threading.Event,
                      t0: float, period_s: float = EE_SAMPLE_PERIOD_SEC) -> None:
    """Background thread: record commanded target pose, wrench, velocity and acceleration."""
    _make_ee_sampler(
        lambda: robot.target_pose,
        lambda: robot.end_effector_wrench,  # no separate commanded wrench topic
    )(cmd_poses, stop_event, t0, period_s)


# ---------------------------------------------------------------------------
# Results helpers
# ---------------------------------------------------------------------------

def _alloc_trial_dir(results_dir: Path, depth_mm: float) -> Path:
    """Return the next available trial_NN_depthXmm path without creating it yet."""
    counter = 0
    trial_dir = results_dir / f"trial_{counter:02d}_depth{depth_mm:.1f}mm"
    while trial_dir.exists():
        counter += 1
        trial_dir = results_dir / f"trial_{counter:02d}_depth{depth_mm:.1f}mm"
    return trial_dir


def _next_result_path(trial_dir: Path, angle_deg: float) -> Path:
    counter = 0
    path = trial_dir / f"angle_{angle_deg:.1f}_{counter:02d}.pkl"
    while path.exists():
        counter += 1
        path = trial_dir / f"angle_{angle_deg:.1f}_{counter:02d}.pkl"
    return path


# ---------------------------------------------------------------------------
# Hold-phase force controller
# ---------------------------------------------------------------------------

def _hold_force_controlled(
    robot: Robot,
    compressed_pose: Pose,
    hold_sec: float,
    t0: float,
) -> list:
    """Hold at compressed_pose for hold_sec seconds with Z force correction.

    Sign convention (matches rolling experiment):
        Fz < 0 when pressing into the surface; more negative = harder contact.
        f_error = fz - fz_target
          positive → contact is being lost → press down (z_cmd decreases)
          negative → over-pressed          → lift up   (z_cmd increases)
        delta_z = -Kp * f_error
    """
    x = float(compressed_pose.position[0])
    y = float(compressed_pose.position[1])
    z_initial = float(compressed_pose.position[2])
    z_cmd = z_initial
    z_min = z_initial - HOLD_FORCE_MAX_Z_CORR_M
    z_max = z_initial + HOLD_FORCE_MAX_Z_CORR_M

    # Decompose initial orientation: rx is kept fixed, ry/rz are free to adapt.
    init_euler = compressed_pose.orientation.as_euler("xyz", degrees=True)
    rx_fixed = float(init_euler[0])   # ~180°, never changed
    ry_cmd   = float(init_euler[1])   # pitch — will adapt to surface torque
    rz_cmd   = float(init_euler[2])   # yaw   — will adapt to surface torque
    ry_min, ry_max = ry_cmd - HOLD_TORQUE_MAX_CORR_DEG, ry_cmd + HOLD_TORQUE_MAX_CORR_DEG
    rz_min, rz_max = rz_cmd - HOLD_TORQUE_MAX_CORR_DEG, rz_cmd + HOLD_TORQUE_MAX_CORR_DEG

    dt = 1.0 / HOLD_FORCE_CTRL_HZ
    telemetry: list = []

    # Explicitly hand off from trajectory mode to set_target mode, then settle
    # so the position controller reaches the compressed depth before we snapshot Fz.
    robot.set_target(pose=compressed_pose)
    print(f"  Settling {HOLD_SETTLE_SEC:.2f} s at compressed depth before force snapshot…")
    time.sleep(HOLD_SETTLE_SEC)

    try:
        wrench0 = robot.end_effector_wrench
        fz_target = float(wrench0["force"][2])
    except RuntimeError:
        fz_target = 0.0

    torque_adapt_str = (
        f"Kto={HOLD_TORQUE_KP_DEG_PER_NM:.2f} deg/Nm  "
        f"db=±{HOLD_TORQUE_DEADBAND_NM:.3f} Nm  "
        f"max=±{HOLD_TORQUE_MAX_CORR_DEG:.1f}°"
        if HOLD_TORQUE_ADAPT else "OFF"
    )
    print(
        f"  Hold ctrl: Fz_target={fz_target:+.3f} N  Kp={HOLD_FORCE_KP:.5f} m/N  "
        f"db_F=±{HOLD_FORCE_DEADBAND_N:.2f} N  max_z=±{HOLD_FORCE_MAX_Z_CORR_M*1000:.1f} mm  "
        f"{HOLD_FORCE_CTRL_HZ:.0f} Hz\n"
        f"  Torque adapt: {torque_adapt_str}  "
        f"init_ori=[rx={rx_fixed:.1f}° ry={ry_cmd:.2f}° rz={rz_cmd:.2f}°]"
    )

    t_start = time.perf_counter()
    next_tick = t_start
    t_last_print = t_start

    while time.perf_counter() - t_start < hold_sec:
        t_iter = time.perf_counter()

        try:
            wrench = robot.end_effector_wrench
            fz = float(wrench["force"][2])
            ty = float(wrench["torque"][1])
            tz = float(wrench["torque"][2])
        except RuntimeError:
            fz, ty, tz = fz_target, 0.0, 0.0

        # ── Z force correction ──────────────────────────────────────────────
        f_error = fz - fz_target
        f_error_ctrl = 0.0 if abs(f_error) < HOLD_FORCE_DEADBAND_N else f_error
        z_cmd = float(np.clip(z_cmd - HOLD_FORCE_KP * f_error_ctrl, z_min, z_max))

        # ── Rotational compliance (Ry / Rz from Ty / Tz torques) ───────────
        if HOLD_TORQUE_ADAPT:
            ty_ctrl = 0.0 if abs(ty) < HOLD_TORQUE_DEADBAND_NM else ty
            tz_ctrl = 0.0 if abs(tz) < HOLD_TORQUE_DEADBAND_NM else tz
            ry_cmd = float(np.clip(
                ry_cmd + HOLD_TORQUE_KP_DEG_PER_NM * ty_ctrl, ry_min, ry_max))
            rz_cmd = float(np.clip(
                rz_cmd + HOLD_TORQUE_KP_DEG_PER_NM * tz_ctrl, rz_min, rz_max))

        ori_cmd = Rotation.from_euler("xyz", [rx_fixed, ry_cmd, rz_cmd], degrees=True)
        robot.set_target(pose=Pose(np.array([x, y, z_cmd]), ori_cmd))

        z_corr_mm = (z_cmd - z_initial) * 1000.0
        telemetry.append({
            "t": t_iter - t0,
            "elapsed_hold_s": t_iter - t_start,
            "fz_measured": fz,
            "fz_target": fz_target,
            "f_error": f_error,
            "z_cmd": z_cmd,
            "z_correction_m": z_cmd - z_initial,
            "in_band_force": abs(f_error) < HOLD_FORCE_DEADBAND_N,
            "torque_y": ty,
            "torque_z": tz,
            "ry_cmd_deg": ry_cmd,
            "rz_cmd_deg": rz_cmd,
        })

        if t_iter - t_last_print >= 0.5:
            band_str = "IN " if abs(f_error) < HOLD_FORCE_DEADBAND_N else ("DN " if f_error > 0 else "UP ")
            print(
                f"\r  [{band_str}] t={t_iter-t_start:.1f}/{hold_sec:.1f}s  "
                f"Fz={fz:+.3f}N err={f_error:+.3f}N z={z_corr_mm:+.2f}mm  "
                f"Ty={ty:+.3f}Nm Tz={tz:+.3f}Nm  "
                f"ry={ry_cmd:+.2f}° rz={rz_cmd:+.2f}°",
                end="", flush=True,
            )
            t_last_print = t_iter

        next_tick += dt
        sleep_remaining = next_tick - time.perf_counter()
        if sleep_remaining > 0:
            time.sleep(sleep_remaining)

    print()

    fz_vals    = [s["fz_measured"] for s in telemetry]
    z_corr_vals = [s["z_correction_m"] for s in telemetry]
    ry_vals    = [s["ry_cmd_deg"] for s in telemetry]
    rz_vals    = [s["rz_cmd_deg"] for s in telemetry]
    in_band_pct = 100.0 * sum(s["in_band_force"] for s in telemetry) / max(len(telemetry), 1)
    print(
        f"  Hold done: Fz mean={np.mean(fz_vals):+.3f} N  std={np.std(fz_vals):.3f} N  "
        f"z_corr=[{min(z_corr_vals)*1000:+.2f},{max(z_corr_vals)*1000:+.2f}] mm  "
        f"in-band {in_band_pct:.1f}%\n"
        f"  Orientation settled: ry=[{min(ry_vals):+.2f},{max(ry_vals):+.2f}]°  "
        f"rz=[{min(rz_vals):+.2f},{max(rz_vals):+.2f}]°"
    )
    return telemetry


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    trial_dir: Path | None = None  # created lazily on first save

    dry_run = DRY_RUN
    answer = input("Dry run (skip RF only; robot motion still required)? [y/N]: ").strip().lower()
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

    active_stop_event: threading.Event | None = None
    active_thread: threading.Thread | None = None

    try:
        robot.wait_until_ready()
        _setup_pose_controller(robot)

        # ── Resolve contact and derived poses ────────────────────────────────
        if CONTACT_POSITION_M is not None:
            contact_pose = Pose(np.array(CONTACT_POSITION_M, dtype=float), BASE_ORI)
            print(f"Contact position (from config): {CONTACT_POSITION_M}")
        else:
            contact_pose = robot.end_effector_pose.copy()
            print(f"Contact position (captured at startup): {contact_pose.position.tolist()}")
            print("  (Set CONTACT_POSITION_M in orientation_config.py to skip manual jogging.)")

        approach_pose = Pose(
            contact_pose.position + np.array([0.0, 0.0, APPROACH_HEIGHT_M]),
            BASE_ORI,
        )
        compressed_pose = Pose(
            contact_pose.position + np.array([0.0, 0.0, -COMPRESS_DEPTH_M]),
            BASE_ORI,
        )

        depth_mm = COMPRESS_DEPTH_M * 1000.0
        print(f"\nApproach pose:    {approach_pose.position.tolist()}")
        print(f"Contact pose:     {contact_pose.position.tolist()}")
        print(f"Compressed pose:  {compressed_pose.position.tolist()}")
        print(f"Compress depth:   {depth_mm:.1f} mm")
        print(f"Hold time:        {HOLD_AT_CONTACT_SEC:.1f} s")
        print(f"Approach speed:   {APPROACH_SPEED_M_S * 1000:.1f} mm/s")
        print(f"Return speed:     {RETURN_SPEED_M_S * 1000:.1f} mm/s")

        # Move to approach pose before starting trials
        print("\nMoving to approach pose...")
        _move(robot, approach_pose, RETURN_SPEED_M_S, settle=0.5)

        input("\nPress Enter to begin trials (Ctrl+C to quit at any prompt)...")

        while True:
            print("\n═══ New angle session ════════════════════════════════════════")
            angle_str = input("  Enter bar angle [degrees]: ").strip()
            angle_deg = float(angle_str)

            if trial_dir is None:
                trial_dir = _alloc_trial_dir(RESULTS_DIR, depth_mm)
                trial_dir.mkdir(parents=True, exist_ok=False)
                print(f"  Session folder: {trial_dir}")

            for repeat_idx in range(N_REPEATS):
                print(f"\n─── Repeat {repeat_idx + 1} / {N_REPEATS}  (angle {angle_deg:+.1f}°) ─────────────")

                # 1. Descend to contact
                print("  Descending to contact...")
                _move(robot, contact_pose, APPROACH_SPEED_M_S, settle=0.3)
                measured_contact = robot.end_effector_pose.copy()
                measured_compressed = Pose(
                    measured_contact.position + np.array([0.0, 0.0, -COMPRESS_DEPTH_M]),
                    BASE_ORI,
                )
                print(f"  Contact actual: {measured_contact.position.tolist()}")

                # 2. Start EEF sampler and RF streaming
                frames: list = []
                ee_poses: list = []
                cmd_poses: list = []
                stop_event: threading.Event | None = None
                thread: threading.Thread | None = None
                t0 = time.perf_counter()
                sample_stop_event = threading.Event()
                sample_thread = threading.Thread(
                    target=_sample_ee_poses,
                    args=(robot, ee_poses, sample_stop_event, t0),
                    daemon=True,
                )
                cmd_thread = threading.Thread(
                    target=_sample_cmd_poses,
                    args=(robot, cmd_poses, sample_stop_event, t0),
                    daemon=True,
                )
                sample_thread.start()
                cmd_thread.start()
                if dry_run:
                    print("  DRY RUN — skipping 67.")
                else:
                    print("  Sending 67 → Teensy RF streaming started.")
                    frames, stop_event, first_frame_event, thread, t0 = rf_stream_start(ser)
                    active_stop_event = stop_event
                    active_thread = thread
                    if not first_frame_event.wait(timeout=SERIAL_TIMEOUT_SEC):
                        rf_stream_stop(ser, stop_event, thread)
                        active_stop_event = None
                        active_thread = None
                        raise TimeoutError(
                            f"No RF frame received from Teensy within {SERIAL_TIMEOUT_SEC} s."
                        )
                    print("  First RF frame received.")

                # 3. Compress
                print(f"  Compressing {depth_mm:.1f} mm...")
                t_compress_start = time.perf_counter() - t0
                _cartesian_move(robot, measured_contact, measured_compressed, APPROACH_SPEED_M_S)
                t_compress_end = time.perf_counter() - t0
                actual_compressed = robot.end_effector_pose.copy()

                # 4. Hold
                print(f"  Holding for {HOLD_AT_CONTACT_SEC:.1f} s...")
                hold_force_telemetry: list = []
                if HOLD_FORCE_CONTROL and not dry_run:
                    hold_force_telemetry = _hold_force_controlled(
                        robot, actual_compressed, HOLD_AT_CONTACT_SEC, t0)
                else:
                    time.sleep(HOLD_AT_CONTACT_SEC)
                t_hold_end = time.perf_counter() - t0

                # 5. Lift
                actual_after_hold = robot.end_effector_pose.copy()
                print("  Lifting to contact level...")
                t_lift_start = time.perf_counter() - t0
                _cartesian_move(robot, actual_after_hold, measured_contact, APPROACH_SPEED_M_S, settle=0.3)
                t_lift_end = time.perf_counter() - t0

                # 6. Stop RF and sampler
                sample_stop_event.set()
                sample_thread.join(timeout=1.0)
                if dry_run:
                    print("  DRY RUN — skipping 69.")
                else:
                    rf_stream_stop(ser, stop_event, thread)
                    active_stop_event = None
                    active_thread = None
                    print(f"  Sending 69 → RF stopped. {len(frames)} frames.")

                # 7. Return to approach
                print("  Returning to approach pose...")
                _move(robot, approach_pose, RETURN_SPEED_M_S, settle=0.5)

                # 8. Save
                out_path = _next_result_path(trial_dir, angle_deg)
                payload = {
                    "frames": list(frames),
                    "t0": t0,
                    "angle_deg": angle_deg,
                    "repeat_idx": repeat_idx,
                    "n_repeats": N_REPEATS,
                    "compress_depth_m": COMPRESS_DEPTH_M,
                    "hold_at_contact_sec": HOLD_AT_CONTACT_SEC,
                    "hold_force_telemetry": hold_force_telemetry,
                    "hold_force_control_enabled": HOLD_FORCE_CONTROL and not dry_run,
                    "contact_position": measured_contact.position.tolist(),
                    "approach_position": approach_pose.position.tolist(),
                    "compressed_position": measured_compressed.position.tolist(),
                    "commanded_contact_pose": {
                        "position": measured_contact.position.tolist(),
                        "orientation_quat": measured_contact.orientation.as_quat().tolist(),
                        "orientation_euler_deg_xyz": measured_contact.orientation.as_euler("xyz", degrees=True).tolist(),
                    },
                    "commanded_approach_pose": {
                        "position": approach_pose.position.tolist(),
                        "orientation_quat": approach_pose.orientation.as_quat().tolist(),
                        "orientation_euler_deg_xyz": approach_pose.orientation.as_euler("xyz", degrees=True).tolist(),
                    },
                    "commanded_compressed_pose": {
                        "position": measured_compressed.position.tolist(),
                        "orientation_quat": measured_compressed.orientation.as_quat().tolist(),
                        "orientation_euler_deg_xyz": measured_compressed.orientation.as_euler("xyz", degrees=True).tolist(),
                    },
                    "waveguide_lateral_positions_mm": WAVEGUIDE_LATERAL_POSITIONS_MM.tolist(),
                    "waveguide_sample_pitch_mm": WAVEGUIDE_SAMPLE_PITCH_MM,
                    "base_ori_euler_deg": BASE_ORI_EULER_DEG,
                    "ee_poses": ee_poses,
                    "commanded_ee_poses": cmd_poses,
                    "commanded_segments": [
                        {"phase": "compress",
                         "t_start_s": t_compress_start, "t_end_s": t_compress_end,
                         "start_pose": {
                             "position": measured_contact.position.tolist(),
                             "orientation_quat": measured_contact.orientation.as_quat().tolist(),
                             "orientation_euler_deg_xyz": measured_contact.orientation.as_euler("xyz", degrees=True).tolist(),
                         },
                         "end_pose": {
                             "position": measured_compressed.position.tolist(),
                             "orientation_quat": measured_compressed.orientation.as_quat().tolist(),
                             "orientation_euler_deg_xyz": measured_compressed.orientation.as_euler("xyz", degrees=True).tolist(),
                         }},
                        {"phase": "hold",
                         "t_start_s": t_compress_end, "t_end_s": t_hold_end,
                         "start_pose": {
                             "position": measured_compressed.position.tolist(),
                             "orientation_quat": measured_compressed.orientation.as_quat().tolist(),
                             "orientation_euler_deg_xyz": measured_compressed.orientation.as_euler("xyz", degrees=True).tolist(),
                         },
                         "end_pose": {
                             "position": measured_compressed.position.tolist(),
                             "orientation_quat": measured_compressed.orientation.as_quat().tolist(),
                             "orientation_euler_deg_xyz": measured_compressed.orientation.as_euler("xyz", degrees=True).tolist(),
                         }},
                        {"phase": "lift",
                         "t_start_s": t_lift_start, "t_end_s": t_lift_end,
                         "start_pose": {
                             "position": measured_compressed.position.tolist(),
                             "orientation_quat": measured_compressed.orientation.as_quat().tolist(),
                             "orientation_euler_deg_xyz": measured_compressed.orientation.as_euler("xyz", degrees=True).tolist(),
                         },
                         "end_pose": {
                             "position": measured_contact.position.tolist(),
                             "orientation_quat": measured_contact.orientation.as_quat().tolist(),
                             "orientation_euler_deg_xyz": measured_contact.orientation.as_euler("xyz", degrees=True).tolist(),
                         }},
                    ],
                }
                with open(out_path, "wb") as f:
                    pickle.dump(payload, f)
                print(f"  Saved [{repeat_idx + 1}/{N_REPEATS}]: {out_path}")

            input(f"\n  All {N_REPEATS} repeats done (angle {angle_deg:+.1f}°). "
                  "Rotate bar, then press Enter for next angle (Ctrl+C to stop)...")

    finally:
        if not dry_run and ser is not None:
            if active_stop_event is not None and active_thread is not None:
                rf_stream_stop(ser, active_stop_event, active_thread)
            ser.close()
        robot.shutdown()


if __name__ == "__main__":
    main()
