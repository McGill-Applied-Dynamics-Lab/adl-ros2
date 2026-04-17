#!/usr/bin/env python3
"""Rolling contact experiment with spline feedforward slide phase.

Identical to rolling_contact_experiment_force.py for all phases except the slide:
  - Approach, compress, RF start/stop, decompress, settle, ascent, return are unchanged.
  - The slide phase follows a pre-learned Z(Y) spline (from build_depth_trajectory.py)
    instead of a closed-loop force controller.  Y advances at SLIDE_SPEED_M_S while
    Z tracks the spline at each control step.  Fz is still monitored for contact loss.

Motion sequence per trial
--------------------------
(1) approach_pose
         │ move_to (APPROACH_SPEED_M_S)
(2) contact_pose
         │ execute_cartesian_traj compress (position-controlled)
(3) compressed_start_pose
         │ press to spline start Z (position-controlled, FORCE_ACQUIRE_SPEED_M_S)
         │ settle SETTLE_SEC
         │ spline-tracked slide (set_target at FORCE_CTRL_HZ)
         │   Y: y_start + SLIDE_SPEED_M_S * t
         │   Z: np.interp(y_traveled, spline_y_mm, spline_z_world_m)
(4) compressed_end_pose
         │ settle 1.0 s
         │ execute_cartesian_traj decompress
(5) surface_end_pose
         │ execute_cartesian_traj ascend
         │ execute_cartesian_traj return
(6) approach_pose
"""

import pickle
import threading
import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import serial
from scipy.spatial.transform import Rotation

from arm_client import CONFIG_DIR
from arm_client.robot import Pose, Robot

from rolling_force_config import (  # type: ignore
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
    MAX_SLIDE_DISTANCE_M,
    RETURN_SPEED_M_S,
    ROBOT_NAMESPACE,
    ROLL_LENGTH_PERCENT,
    ROLLER_DIAMETER_M,
    SERIAL_PORT,
    SERIAL_TIMEOUT_SEC,
    SETTLE_SEC,
    SLIDE_SPEED_M_S,
    SLIDE_TIMEOUT_S,
)

FR3_POSE_CONTROLLER = "fr3_pose_controller"
FR3_POSE_CONFIG     = "probing.yaml"

CONTACT_WAYPOINTS    = 50
EXEC_TRAJ_TIME_OFFSET = 0.6

BASE_ORI = Rotation.from_euler("xyz", BASE_ORI_EULER_DEG, degrees=True)

CMD_START = bytes([0x43])
CMD_STOP  = bytes([0x45])

RESULTS_DIR    = Path(__file__).resolve().parent / "results"
DEPTH_PROFILES = Path(__file__).resolve().parent / "depth_profiles"

# ── RF samples (from force experiment) ───────────────────────────────────────
try:
    from rolling_contact_experiment_force import (  # type: ignore
        EXPECTED_RF_SAMPLES,
        _rf_capture_summary,
        _sample_ee_poses,
        _sample_cmd_poses,
        _ensure_rf_active,
        rf_stream_start,
        rf_stream_stop,
        _next_trial_dir,
        _save_trial_result,
        _build_contact_trajectory,
        _duration_for_move,
        _pose_xyz_rpy_str,
        _setup_fr3_pose_controller,
        _roll_distance_m,
    )
except ImportError as e:
    raise ImportError(
        "rolling_contact_experiment_force.py must be in the same directory."
    ) from e


# ---------------------------------------------------------------------------
# Spline loading
# ---------------------------------------------------------------------------

def _load_latest_spline() -> tuple[np.ndarray, np.ndarray]:
    """Return (y_mm, z_world_m) from the most recently saved depth spline .npz."""
    npzs = sorted(DEPTH_PROFILES.glob("*_depth_spline.npz"),
                  key=lambda p: p.stat().st_mtime)
    if not npzs:
        raise FileNotFoundError(
            f"No depth_spline.npz found in {DEPTH_PROFILES}. "
            "Run build_depth_trajectory.py first."
        )
    path = npzs[-1]
    data = np.load(path)
    y_mm       = data["y_mm"]          # shape (N,)  in mm
    z_world_mm = data["z_world_mm"]    # shape (N,)  in mm (absolute world frame)
    print(f"  Spline loaded: {path.name}")
    print(f"    Y: {y_mm[0]:.1f}–{y_mm[-1]:.1f} mm  |  "
          f"Z world: {z_world_mm.min():.1f}–{z_world_mm.max():.1f} mm")
    return y_mm, z_world_mm / 1000.0   # convert Z to metres


# ---------------------------------------------------------------------------
# Spline-tracked slide
# ---------------------------------------------------------------------------

def _spline_slide(robot, compressed_start_pose, roll_distance_m,
                  spline_y_mm, spline_z_world_m,
                  ee_poses_slide, t0):
    """Slide while tracking the pre-learned Z(Y) spline.

    Pre-computes all waypoints from the spline and executes them as a single
    execute_cartesian_traj call (quintic interpolation, smooth velocity profile).
    A parallel thread monitors Fz and records telemetry at FORCE_CTRL_HZ.
    Contact loss is flagged but does not abort the trajectory (safe — robot
    follows the spline through air if contact is lost).

    Returns:
        actual_end_pose (Pose)
        termination_reason (str): "distance" | "contact_lost"
    """
    x_fixed = float(compressed_start_pose.position[0])
    y_start = float(compressed_start_pose.position[1])
    current_z = float(compressed_start_pose.position[2])
    z_spline_start = float(np.interp(0.0, spline_y_mm, spline_z_world_m))

    # ── Press to spline start Z (smooth, position-controlled) ────────────────
    if abs(current_z - z_spline_start) > 5e-4:
        print(f"  Pressing to spline start Z ({z_spline_start*1000:.2f} mm) "
              f"from current Z ({current_z*1000:.2f} mm)...")
        press_end = Pose(np.array([x_fixed, y_start, z_spline_start]),
                         compressed_start_pose.orientation)
        press_dur = max(abs(z_spline_start - current_z) / FORCE_ACQUIRE_SPEED_M_S, 0.5)
        press_wps, press_times = _build_contact_trajectory(
            [(compressed_start_pose, press_end)], [press_dur])
        robot.execute_cartesian_traj(press_wps, press_times)
        while robot.wait_for_trajectory_completion(press_dur + EXEC_TRAJ_TIME_OFFSET):
            pass
        time.sleep(SETTLE_SEC)
        print(f"  At spline start Z.")

    # ── Build full slide trajectory from spline ───────────────────────────────
    slide_duration = roll_distance_m / SLIDE_SPEED_M_S
    n_wps = max(int(slide_duration * 8), 20)  # 8 waypoints/s; quintic interp fills the rest

    y_frac    = np.linspace(0.0, roll_distance_m, n_wps)
    y_world   = y_start + y_frac
    z_world   = np.interp(y_frac * 1000.0, spline_y_mm, spline_z_world_m)
    wps_times = np.linspace(0.0, slide_duration, n_wps) + EXEC_TRAJ_TIME_OFFSET

    waypoints = [
        (Pose(np.array([x_fixed, float(y_world[i]), float(z_world[i])]),
              compressed_start_pose.orientation), None)
        for i in range(n_wps)
    ]

    print(f"  Spline slide: speed={SLIDE_SPEED_M_S*1000:.1f} mm/s  "
          f"dist={roll_distance_m*1000:.1f} mm  duration={slide_duration:.1f} s  "
          f"waypoints={n_wps}")

    # ── Telemetry + contact-loss monitor (parallel thread) ───────────────────
    termination_flag = {"reason": "distance"}
    monitor_stop = threading.Event()
    t0_slide = time.perf_counter()

    def _monitor():
        dt = 1.0 / FORCE_CTRL_HZ
        while not monitor_stop.is_set():
            t_now = time.perf_counter()
            elapsed = t_now - t0_slide
            try:
                fz         = float(robot.end_effector_wrench["force"][2])
                actual_pos = robot.end_effector_pose.position.tolist()
            except RuntimeError:
                time.sleep(dt)
                continue

            y_traveled    = actual_pos[1] - y_start
            y_traveled_mm = y_traveled * 1000.0
            z_target      = float(np.interp(y_traveled_mm, spline_y_mm, spline_z_world_m))

            ee_poses_slide.append({
                "t":               time.perf_counter() - t0,
                "elapsed_slide_s": elapsed,
                "y_traveled_m":    y_traveled,
                "z_cmd":           z_target,
                "z_spline_target": z_target,
                "z_error_m":       actual_pos[2] - z_target,
                "vy_cmd":          SLIDE_SPEED_M_S,
                "fz_measured":     fz,
                "position_actual": actual_pos,
            })

            if fz > CONTACT_FORCE_THRESHOLD_N:
                termination_flag["reason"] = "contact_lost"
                print(f"\n  Contact lost: Fz={fz:.3f} N  Y={y_traveled_mm:.1f} mm")

            print(
                f"\r  Fz={fz:+7.3f}N  "
                f"Z={z_target*1000:+7.2f}mm  "
                f"Y={y_traveled_mm:6.1f}/{roll_distance_m*1000:.0f}mm",
                end="", flush=True,
            )
            time.sleep(dt)

    monitor_thread = threading.Thread(target=_monitor, daemon=True)
    monitor_thread.start()

    # ── Execute trajectory ────────────────────────────────────────────────────
    robot.execute_cartesian_traj(waypoints, wps_times.tolist())
    while robot.wait_for_trajectory_completion(slide_duration + EXEC_TRAJ_TIME_OFFSET + 1.0):
        pass

    monitor_stop.set()
    monitor_thread.join(timeout=1.0)
    print()

    try:
        actual_end_pose = robot.end_effector_pose.copy()
    except RuntimeError:
        actual_end_pose = Pose(
            np.array([x_fixed, y_start + roll_distance_m, float(z_world[-1])]),
            compressed_start_pose.orientation)

    print(f"  Slide done ({termination_flag['reason']}): "
          f"actual: {_pose_xyz_rpy_str(actual_end_pose)}")
    return actual_end_pose, termination_flag["reason"]


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    trial_dir = _next_trial_dir(RESULTS_DIR)
    print(f"Results will be saved under: {trial_dir}  (created on first completed roll)")

    spline_y_mm, spline_z_world_m = _load_latest_spline()

    roll_distance_m      = _roll_distance_m()
    R                    = ROLLER_DIAMETER_M / 2.0
    d                    = COMPRESS_DEPTH_M
    R_eff                = R - d
    y_disp_mm            = roll_distance_m * 1000.0
    roller_rotation_deg  = np.degrees(roll_distance_m / max(R, 1e-9))

    dry_run = DRY_RUN
    answer = input("Dry run? [y/N]: ").strip().lower()
    if answer in {"y", "yes"}:
        dry_run = True

    robot = Robot(namespace=ROBOT_NAMESPACE)
    ser = (
        serial.Serial(SERIAL_PORT, BAUD_RATE,
                      timeout=SERIAL_TIMEOUT_SEC, write_timeout=SERIAL_TIMEOUT_SEC)
        if not dry_run else None
    )

    trials = []
    active_stop_event = None
    active_thread     = None

    session_metadata = {
        "controller":            FR3_POSE_CONTROLLER,
        "experiment_type":       "spline_slide",
        "roller_diameter_m":     ROLLER_DIAMETER_M,
        "compress_depth_m":      d,
        "approach_height_m":     APPROACH_HEIGHT_M,
        "finray_length_m":       FINRAY_LENGTH_M,
        "roll_length_percent":   ROLL_LENGTH_PERCENT,
        "roll_distance_m":       roll_distance_m,
        "slide_speed_m_s":       SLIDE_SPEED_M_S,
        "slide_timeout_s":       SLIDE_TIMEOUT_S,
        "contact_force_threshold_n": CONTACT_FORCE_THRESHOLD_N,
        "force_ctrl_hz":         FORCE_CTRL_HZ,
        "base_ori_euler_deg":    BASE_ORI_EULER_DEG,
        "robot_namespace":       ROBOT_NAMESPACE,
        "expected_rf_samples":   EXPECTED_RF_SAMPLES,
        "spline_y_mm":           spline_y_mm.tolist(),
        "spline_z_world_m":      spline_z_world_m.tolist(),
        "trial_dir":             str(trial_dir),
    }

    try:
        robot.wait_until_ready()
        _setup_fr3_pose_controller(robot)

        _contact_pose = Pose(np.array(CONTACT_POSITION_M, dtype=float), BASE_ORI)
        approach_pose = Pose(
            _contact_pose.position + np.array([0.0, 0.0, APPROACH_HEIGHT_M]),
            BASE_ORI,
        )

        print(f"\n=== Rolling contact experiment (spline slide) ===")
        print(f"Contact position:   {CONTACT_POSITION_M}")
        print(f"Slide distance:     {y_disp_mm:.1f} mm in +Y  at {SLIDE_SPEED_M_S*1000:.1f} mm/s")
        print(f"Spline Z range:     {spline_z_world_m.min()*1000:.1f}–{spline_z_world_m.max()*1000:.1f} mm (world)")

        current_pose = robot.end_effector_pose.copy()
        if np.linalg.norm(current_pose.position - approach_pose.position) > 1e-3:
            print("\nMoving to approach pose...")
            robot.move_to(pose=approach_pose, speed=RETURN_SPEED_M_S)
            time.sleep(SETTLE_SEC)

        input("\nPress Enter to begin spline-tracked rolling trial...")

        trial_num = 0
        while True:
            trial_num += 1
            print(f"\n─── Trial {trial_num} ───")
            answer = input("Press Enter to start, or 'q' to quit: ").strip().lower()
            if answer in {"q", "quit"}:
                break

            # ── 1. Descend to contact ────────────────────────────────────────
            print("  Descending to contact...")
            robot.move_to(pose=_contact_pose, speed=APPROACH_SPEED_M_S)
            time.sleep(SETTLE_SEC)
            measured_contact_pose = robot.end_effector_pose.copy()

            # ── 2. Compress ──────────────────────────────────────────────────
            desired_compressed_start = Pose(
                measured_contact_pose.position + np.array([0.0, 0.0, -COMPRESS_DEPTH_M]),
                BASE_ORI,
            )
            compress_duration = _duration_for_move(
                measured_contact_pose, desired_compressed_start, APPROACH_SPEED_M_S)
            compress_wps, compress_times = _build_contact_trajectory(
                [(measured_contact_pose, desired_compressed_start)], [compress_duration])

            frames          = []
            ee_poses_trial  = []
            cmd_poses_trial = []
            ee_poses_slide  = []
            slide_t0        = time.perf_counter()

            # ── 3. RF start ──────────────────────────────────────────────────
            if not dry_run:
                print("  Sending start to Teensy...")
                frames, stop_ev, first_frame_ev, thread, _ = rf_stream_start(ser)
                active_stop_event = stop_ev
                active_thread     = thread
                if not first_frame_ev.wait(timeout=SERIAL_TIMEOUT_SEC):
                    raise TimeoutError("No RF frame received.")
                print("  First RF frame received; compressing...")

            sample_stop_event = threading.Event()
            sample_thread = threading.Thread(
                target=_sample_ee_poses,
                args=(robot, ee_poses_trial, sample_stop_event, slide_t0), daemon=True)
            cmd_thread = threading.Thread(
                target=_sample_cmd_poses,
                args=(robot, cmd_poses_trial, sample_stop_event, slide_t0), daemon=True)
            sample_thread.start()
            cmd_thread.start()

            robot.execute_cartesian_traj(compress_wps, compress_times)
            while robot.wait_for_trajectory_completion(compress_duration + EXEC_TRAJ_TIME_OFFSET):
                pass
            print(f"  Compress done: {_pose_xyz_rpy_str(robot.end_effector_pose.copy())}")
            actual_compressed_start = robot.end_effector_pose.copy()

            # ── 4. Settle ────────────────────────────────────────────────────
            time.sleep(SETTLE_SEC)

            # ── 5. RF guard ──────────────────────────────────────────────────
            if not dry_run:
                print("  Waiting for active RF stream before slide…")
                _ensure_rf_active(frames)
                print(f"  RF active ({len(frames)} frames so far) — sliding.")

            # ── 6. Spline slide ──────────────────────────────────────────────
            actual_slide_end_pose, termination_reason = _spline_slide(
                robot, actual_compressed_start, roll_distance_m,
                spline_y_mm, spline_z_world_m,
                ee_poses_slide, slide_t0,
            )

            # ── 7. Decompress ────────────────────────────────────────────────
            print("  Settling after slide...")
            time.sleep(SETTLE_SEC)
            actual_slide_end_pose = robot.end_effector_pose.copy()
            desired_surface_end = Pose(
                actual_slide_end_pose.position + np.array([0.0, 0.0, COMPRESS_DEPTH_M]),
                BASE_ORI,
            )
            decompress_duration = _duration_for_move(
                actual_slide_end_pose, desired_surface_end, APPROACH_SPEED_M_S)
            decompress_wps, decompress_times = _build_contact_trajectory(
                [(actual_slide_end_pose, desired_surface_end)], [decompress_duration])

            robot.execute_cartesian_traj(decompress_wps, decompress_times)
            while robot.wait_for_trajectory_completion(decompress_duration + EXEC_TRAJ_TIME_OFFSET):
                pass
            print(f"  Decompress done: {_pose_xyz_rpy_str(robot.end_effector_pose.copy())}")
            print("  Settling after decompress...")
            time.sleep(1.0)

            # ── 8. Stop EE sampler + RF ──────────────────────────────────────
            sample_stop_event.set()
            sample_thread.join(timeout=1.0)

            rf_window_duration = time.perf_counter() - slide_t0
            if not dry_run:
                print("  Sending stop to Teensy...")
                rf_stream_stop(ser, active_stop_event, active_thread)
                active_stop_event = None
                active_thread     = None
                print(f"  RF stream stopped: {len(frames)} frames in {rf_window_duration:.2f} s")

            measured_post_contact = robot.end_effector_pose.copy()

            compressed_z     = desired_compressed_start.position[2]
            ee_poses_slide_only = [
                ep for ep in ee_poses_trial
                if abs(ep["position"][2] - compressed_z) < 0.001
            ]

            # ── 9. Ascend then return ────────────────────────────────────────
            current_after_decompress = robot.end_effector_pose.copy()
            lift_pose = Pose(
                np.array([current_after_decompress.position[0],
                          current_after_decompress.position[1],
                          approach_pose.position[2]]),
                BASE_ORI,
            )
            print("  Ascending to approach height...")
            lift_duration = _duration_for_move(current_after_decompress, lift_pose, APPROACH_SPEED_M_S)
            lift_wps, lift_times = _build_contact_trajectory(
                [(current_after_decompress, lift_pose)], [lift_duration])
            robot.execute_cartesian_traj(lift_wps, lift_times)
            while robot.wait_for_trajectory_completion(lift_duration + EXEC_TRAJ_TIME_OFFSET):
                pass
            time.sleep(SETTLE_SEC)

            print("  Returning to approach pose...")
            current_after_lift = robot.end_effector_pose.copy()
            return_duration = _duration_for_move(current_after_lift, approach_pose, RETURN_SPEED_M_S)
            return_wps, return_times = _build_contact_trajectory(
                [(current_after_lift, approach_pose)], [return_duration])
            robot.execute_cartesian_traj(return_wps, return_times)
            while robot.wait_for_trajectory_completion(return_duration + EXEC_TRAJ_TIME_OFFSET):
                pass
            time.sleep(SETTLE_SEC)
            print(f"  Return actual: {_pose_xyz_rpy_str(robot.end_effector_pose.copy())}")

            # ── 10. Diagnostics ──────────────────────────────────────────────
            if ee_poses_slide:
                fz_vals = [s["fz_measured"] for s in ee_poses_slide]
                print(f"  Fz: mean={np.mean(fz_vals):.3f} N  "
                      f"std={np.std(fz_vals):.3f} N  "
                      f"min={np.min(fz_vals):.3f} N  "
                      f"max={np.max(fz_vals):.3f} N")

            # ── 11. Save ─────────────────────────────────────────────────────
            trial_payload = {
                "trial_num":              trial_num,
                "slide_speed_m_s":        SLIDE_SPEED_M_S,
                "termination_reason":     termination_reason,
                "roll_distance_m":        roll_distance_m,
                "roller_rotation_deg_nominal": float(roller_rotation_deg),
                "frames":                 list(frames),
                "ee_poses":               ee_poses_trial,
                "commanded_ee_poses":     cmd_poses_trial,
                "ee_poses_slide_only":    ee_poses_slide_only,
                "ee_poses_slide_force":   ee_poses_slide,   # same key for read_force_results.py compat
                "rf_window_duration_s":   rf_window_duration,
                "compress_duration_s":    compress_duration,
                "decompress_duration_s":  decompress_duration,
                "spline_y_mm":            spline_y_mm.tolist(),
                "spline_z_world_m":       spline_z_world_m.tolist(),
                "approach_position":      approach_pose.position.tolist(),
                "contact_position":       measured_contact_pose.position.tolist(),
                "compressed_start_actual": actual_compressed_start.position.tolist(),
                "compressed_end_actual":  actual_slide_end_pose.position.tolist(),
                "surface_end_position":   desired_surface_end.position.tolist(),
            }
            trial_payload["rf_capture_summary"] = _rf_capture_summary(
                frames, rf_window_duration)
            trials.append(trial_payload)

            # Save with spline prefix so results don't mix with force trials
            out_stem = (
                f"rolling_spline_"
                f"{SLIDE_SPEED_M_S*1000:.1f}mm_s"
            )
            trial_dir.mkdir(parents=True, exist_ok=True)
            out_path = trial_dir / f"{out_stem}.pkl"
            with open(out_path, "wb") as f:
                pickle.dump({"trial": trial_payload, "session": session_metadata}, f)
            print(f"  Saved: {out_path}")

    finally:
        if ser is not None and active_stop_event is not None and active_thread is not None:
            rf_stream_stop(ser, active_stop_event, active_thread)
        if ser is not None:
            ser.close()
        robot.shutdown()

    print(f"\nSession complete. {len(trials)} spline-tracked trials saved under: {trial_dir}")


if __name__ == "__main__":
    main()
