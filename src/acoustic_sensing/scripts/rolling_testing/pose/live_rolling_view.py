#!/usr/bin/env python3
"""Live viewer + experiment runner for rolling contact (fr3_pose version).

Run this INSTEAD OF rolling_contact_experiment_fr3pose.py to get a live
pyqtgraph window showing all 4 RF channels, roller position, and every
EEF metric while the experiment is in progress.

Usage
-----
    python live_rolling_view.py [--dry-run] [--port /dev/ttyACM0]

The "Continue ▶" button in the toolbar replaces every "Press Enter…" prompt
from the original script.
"""
from __future__ import annotations

import argparse
import pickle
import sys
import threading
import time
from pathlib import Path

import numpy as np
import serial
from scipy.signal import hilbert
from scipy.spatial.transform import Rotation

from arm_client import CONFIG_DIR
from arm_client.robot import Pose, Robot

from rolling_config import (
    APPROACH_HEIGHT_M, APPROACH_SPEED_M_S, BAUD_RATE,
    BASE_ORI_EULER_DEG, COMPRESS_DEPTH_M, CONTACT_POSITION_M,
    DRY_RUN as _CFG_DRY_RUN, EE_SAMPLE_PERIOD_SEC,
    FINRAY_LENGTH_M, N_REPEATS, RETURN_SPEED_M_S, ROBOT_NAMESPACE,
    ROLL_END_OFFSET_M,
    ROLL_LENGTH_PERCENT, ROLLER_DIAMETER_M, ROLL_SPEEDS_M_S,
    SERIAL_PORT, SERIAL_TIMEOUT_SEC, SETTLE_SEC,
    STATIONARY_BASELINE_FRAMES, STATIONARY_BASELINE_TIMEOUT_SEC,
    BASELINE_EXTRA_HEIGHT_M,
    CONTACT_HOLD_SEC, PRE_COMPRESS_HOLD_SEC, POST_SLIDE_HOLD_SEC,
    LIVE_AUTO_RUN,
    MAX_CARTESIAN_SPEED_M_S, MIN_TRAJ_DURATION_S, MIN_WAYPOINT_DT_S,
)

# ---------------------------------------------------------------------------
# Qt / pyqtgraph — loaded lazily
# ---------------------------------------------------------------------------
pg = QtCore = QtWidgets = None


def _load_qt() -> None:
    global pg, QtCore, QtWidgets
    try:
        import pyqtgraph as _pg
        from pyqtgraph.Qt import QtCore as _Qc, QtWidgets as _Qw
    except ImportError as exc:
        raise SystemExit(
            "live_rolling_view.py requires pyqtgraph and a Qt binding.\n"
            "Install: pip install pyqtgraph PyQt5"
        ) from exc
    pg = _pg
    QtCore = _Qc
    QtWidgets = _Qw
    if not hasattr(QtCore, "Signal"):
        QtCore.Signal = QtCore.pyqtSignal  # PyQt5/6 compat


# ---------------------------------------------------------------------------
# Shared state
# ---------------------------------------------------------------------------
class _State:
    def __init__(self):
        self.latest_rf: list | None = None      # [4 × np.ndarray float32]
        self.rf_frame_count: int = 0
        self.latest_ee: dict = {}
        self.phase: str = "idle"                # compress/slide/decompress/idle/done
        self.slide_start_y: float = 0.0
        self.roll_distance_m: float = 0.0
        self.speed_m_s: float = 0.0
        self.trial_label: str = "—"
        self.status: str = "Initialising…"
        self.done: bool = False
        # User-prompt flow
        self.waiting_for_user: bool = False
        self.continue_event: threading.Event = threading.Event()


_state = _State()

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
FR3_POSE_CONTROLLER = "fr3_pose_controller"
FR3_POSE_CONFIG = "probing.yaml"
CONTACT_WAYPOINTS = 50
EXEC_TRAJ_TIME_OFFSET = 0.6
BASE_ORI = Rotation.from_euler("xyz", BASE_ORI_EULER_DEG, degrees=True)
CMD_START = bytes([0x43])
CMD_STOP  = bytes([0x45])
CHANNEL_MARKERS = ("S0", "S1", "S2", "S3")
STREAM_END = "STREAM_END"
EXPECTED_RF_SAMPLES = 1000
RESULTS_DIR = Path(__file__).resolve().parent / "results"

_CH_COLORS = [
    (0,   212, 255),   # S0 – cyan
    (255, 112,  67),   # S1 – orange
    (179, 136, 255),   # S2 – purple
    (105, 240, 174),   # S3 – green
]
WG_LATERAL_MM = [-4.5, -1.5, 1.5, 4.5]

# ---------------------------------------------------------------------------
# RF streaming
# ---------------------------------------------------------------------------
def _stream_reader(ser, frames, stop_event, first_frame_event, t0):
    cur_ch = None
    cur_smp: list = []
    cur_frame: dict = {}
    buf = bytearray()

    def _handle(raw: bytes) -> None:
        nonlocal cur_ch, cur_smp, cur_frame
        line = raw.decode("ascii", errors="ignore").strip()
        if not line:
            return
        if line == STREAM_END:
            stop_event.set()
            return
        if line in CHANNEL_MARKERS:
            cur_ch = line; cur_smp = []; return
        if line == "T":
            if cur_ch is not None:
                cur_frame[cur_ch] = cur_smp
                cur_ch = None
                if len(cur_frame) == len(CHANNEL_MARKERS):
                    if any(len(cur_frame[m]) != EXPECTED_RF_SAMPLES for m in CHANNEL_MARKERS):
                        cur_frame = {}; return
                    arrs = [np.asarray(cur_frame[m], dtype=np.float32) for m in CHANNEL_MARKERS]
                    frames.append((arrs, time.perf_counter() - t0))
                    _state.latest_rf = arrs
                    _state.rf_frame_count += 1
                    first_frame_event.set()
                    cur_frame = {}
            return
        if cur_ch is not None:
            try: cur_smp.append(int(line))
            except ValueError: pass

    while not stop_event.is_set():
        try:
            raw = ser.read(min(max(ser.in_waiting, 1), 4096))
        except serial.SerialException:
            break
        if not raw:
            continue
        buf.extend(raw)
        while True:
            try: idx = buf.index(ord("\n"))
            except ValueError: break
            _handle(bytes(buf[:idx]))
            del buf[:idx + 1]


def _rf_start(ser):
    frames = []
    stop_ev = threading.Event()
    first_ev = threading.Event()
    t0 = time.perf_counter()
    ser.reset_input_buffer()
    ser.write(CMD_START); ser.flush()
    thr = threading.Thread(
        target=_stream_reader, args=(ser, frames, stop_ev, first_ev, t0), daemon=True)
    thr.start()
    return frames, stop_ev, first_ev, thr, t0


def _rf_stop(ser, stop_ev, thr):
    ser.write(CMD_STOP); ser.flush()
    # Don't force stop_ev: let the reader thread drain the remaining in-flight
    # bytes and the STREAM_END marker first.  STREAM_END itself sets stop_ev
    # inside `_handle`, so the thread exits naturally.  If the Teensy never
    # emits STREAM_END (e.g. firmware mismatch), fall back to forcing it.
    thr.join(timeout=SERIAL_TIMEOUT_SEC)
    if thr.is_alive():
        stop_ev.set()
        thr.join(timeout=1.0)


def _capture_stationary(ser, n_frames, timeout_s=STATIONARY_BASELINE_TIMEOUT_SEC):
    """Stream RF until n_frames have been collected, then stop. Returns the frames."""
    frames, stop_ev, _first_ev, thr, _t0 = _rf_start(ser)
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
        _rf_stop(ser, stop_ev, thr)
    return frames[:n_frames]


# ---------------------------------------------------------------------------
# EEF sampler
# ---------------------------------------------------------------------------
def _sample_ee(robot, ee_poses, stop_ev, t0, period_s=EE_SAMPLE_PERIOD_SEC):
    prev_t = prev_pos = prev_vel = None
    while not stop_ev.is_set():
        try:
            ee = robot.end_effector_pose
            wr = robot.end_effector_wrench
        except RuntimeError:
            time.sleep(period_s); continue
        t = time.perf_counter() - t0
        pos = ee.position.tolist()
        dt = t - prev_t if prev_t is not None else None
        vel = ([(pos[i] - prev_pos[i]) / dt for i in range(3)]
               if (dt and dt > 0 and prev_pos is not None) else None)
        acc = ([(vel[i] - prev_vel[i]) / dt for i in range(3)]
               if (vel is not None and prev_vel is not None) else None)
        entry = {
            "t": t, "position": pos,
            "orientation_quat": ee.orientation.as_quat().tolist(),
            "orientation_euler_deg_xyz": ee.orientation.as_euler("xyz", degrees=True).tolist(),
            "force_xyz": wr["force"].tolist(),
            "torque_xyz": wr["torque"].tolist(),
            "velocity_xyz": vel, "acceleration_xyz": acc,
        }
        ee_poses.append(entry)
        _state.latest_ee = entry
        prev_t = t; prev_pos = pos; prev_vel = vel
        time.sleep(period_s)


# ---------------------------------------------------------------------------
# RF active guard
# ---------------------------------------------------------------------------
def _ensure_rf_active(frames, timeout_s: float = 5.0) -> None:
    """Block until at least one new RF frame arrives within *timeout_s* seconds.

    Called immediately before the slide to confirm the Teensy is still
    streaming.  Raises TimeoutError if the stream has stalled.
    """
    count_before = len(frames)
    deadline = time.perf_counter() + timeout_s
    while time.perf_counter() < deadline:
        if len(frames) > count_before:
            return
        time.sleep(0.05)
    raise TimeoutError(
        f"RF stream stalled: no new frame in {timeout_s:.0f} s. "
        "Slide aborted."
    )


# ---------------------------------------------------------------------------
# Experiment helpers (identical to rolling_contact_experiment_fr3pose.py)
# ---------------------------------------------------------------------------
def _roll_dist():
    """Effective roll distance, with the end-of-sensor safety margin removed."""
    return max(0.0, FINRAY_LENGTH_M * ROLL_LENGTH_PERCENT / 100.0 - ROLL_END_OFFSET_M)


def _move_dur(a, b, speed):
    """Cartesian segment duration, with hard speed cap and minimum floor.

    Clamps requested speed at MAX_CARTESIAN_SPEED_M_S and floors the result
    at MIN_TRAJ_DURATION_S — prevents near-zero-distance moves from
    collapsing into the dense-waypoint regime that triggered Franka's
    joint_acceleration_discontinuity reflex.
    """
    speed_capped = min(max(speed, 1e-6), MAX_CARTESIAN_SPEED_M_S)
    dist = float(np.linalg.norm(b.position - a.position))
    return max(dist / speed_capped, MIN_TRAJ_DURATION_S)


def _wait_for_move(start_pose, end_pose, speed, extra_sec=1.0):
    """Sleep long enough for a `move_to(end_pose, speed=speed)` to actually finish.

    `robot.move_to` returns before the trajectory completes.  If the next
    command (another move_to, or execute_cartesian_traj) lands while the
    previous trajectory is still running, the controller logs
    "Ignoring new pose - trajectory in progress" and lands in a state where
    EE velocity != 0 when the next trajectory's ramp-up starts — which
    triggers `joint_acceleration_discontinuity`.  Wait based on the planned
    distance + a safety margin so the controller is guaranteed idle before
    the next command.
    """
    dist = float(np.linalg.norm(end_pose.position - start_pose.position))
    dur = dist / max(speed, 1e-6)
    time.sleep(max(dur + extra_sec, SETTLE_SEC))


def _build_traj(seg_pairs, durations, n_wps=CONTACT_WAYPOINTS, t_off=EXEC_TRAJ_TIME_OFFSET, seg_vels=None):
    bounds = [0.0]
    for d in durations: bounds.append(bounds[-1] + d)
    total = bounds[-1]
    # Cap waypoint density: never closer than MIN_WAYPOINT_DT_S in time.
    max_n_wps = max(2, int(total / MIN_WAYPOINT_DT_S) + 1)
    n_wps = max(2, min(n_wps, max_n_wps))
    wps, times = [], []
    for t in np.linspace(0.0, total, n_wps):
        seg = len(seg_pairs) - 1
        for i in range(len(seg_pairs)):
            if t <= bounds[i + 1]: seg = i; break
        sdur = bounds[seg + 1] - bounds[seg]
        tf = float(np.clip((t - bounds[seg]) / sdur, 0.0, 1.0)) if sdur > 0 else 1.0
        sp, ep = seg_pairs[seg]
        pos = (1 - tf) * np.array(sp.position) + tf * np.array(ep.position)
        q = (1 - tf) * sp.orientation.as_quat() + tf * ep.orientation.as_quat()
        ori = Rotation.from_quat(q / np.linalg.norm(q))
        wps.append((Pose(pos, ori), seg_vels[seg] if seg_vels else None))
        times.append(float(t) + t_off)
    return wps, times


def _rf_summary(frames, dur):
    if not frames: return {"frame_count": 0, "coverage_ratio": 0.0}
    ts = np.array([f[1] for f in frames])
    obs = float(ts[-1] - ts[0]) if len(ts) >= 2 else 0.0
    return {"frame_count": len(frames), "coverage_ratio": obs / dur if dur > 0 else 0.0}


def _next_trial_dir():
    c = 0
    d = RESULTS_DIR / f"trial_{c:02d}"
    while d.exists(): c += 1; d = RESULTS_DIR / f"trial_{c:02d}"
    return d


def _save(trial_dir, speed, trial_payload, session_meta, repeat_idx=0):
    trial_dir.mkdir(parents=True, exist_ok=True)
    mm = speed * 1000.0
    out = trial_dir / f"rolling_speed_{mm:06.1f}_mm_s_rep_{repeat_idx:02d}.pkl"
    with open(out, "wb") as f:
        pickle.dump({"trial": trial_payload, "session": session_meta}, f)
    _state.status = f"Saved: {out.name}"
    print(f"  Saved: {out}")


def _setup_controller(robot):
    robot.controller_switcher_client.switch_controller(FR3_POSE_CONTROLLER)
    robot.fr3_pose_controller_parameters_client.load_param_config(
        file_path=CONFIG_DIR / "controllers" / "fr3_pose" / FR3_POSE_CONFIG)
    deadline = time.time() + 5.0
    while time.time() < deadline:
        if robot.controller_switcher_client.get_active_controller() == FR3_POSE_CONTROLLER:
            return
        time.sleep(0.1)
    raise RuntimeError("Controller switch timed out.")


# ---------------------------------------------------------------------------
# Reconnect helpers — when fr3-launch crashes (typically from a reflex), allow
# the operator to relaunch it and have this script pick the connection back up
# without restarting.
# ---------------------------------------------------------------------------
def _wait_for_robot_alive(robot, max_wait_sec=600.0):
    """Block until `robot.wait_until_ready` succeeds. Returns True/False.

    Polls every ~2 s for the robot state topics to come back online so the
    operator can `ros2 launch ... fr3-launch ...` on the robot host without
    killing this script.
    """
    print("\n*** Robot connection lost. Waiting for FR3 to come back online... ***")
    print(f"    Relaunch fr3-launch on the robot host (will retry for {max_wait_sec:.0f} s).")
    deadline = time.perf_counter() + max_wait_sec
    while time.perf_counter() < deadline:
        _state.status = "Robot offline — waiting for fr3-launch to come back…"
        try:
            robot.wait_until_ready(timeout=2.0)
            print("    Robot back online.")
            return True
        except (TimeoutError, RuntimeError):
            time.sleep(1.0)
    print("    Timed out waiting for robot.")
    return False


def _recover_robot(robot):
    """Wait for the robot to come back, then re-apply fr3_pose_controller setup."""
    if not _wait_for_robot_alive(robot):
        return False
    try:
        _setup_controller(robot)
    except Exception as err:
        print(f"    Controller re-setup after reconnect failed: {err}")
        return False
    _state.status = "Robot reconnected and pose controller re-armed."
    return True


def _stop_threads_safely(sample_stop_event, sample_thread, ser, active_stop_event, active_thread, dry_run):
    """Best-effort shutdown of background threads + RF stream after a fault."""
    if sample_stop_event is not None:
        sample_stop_event.set()
    if sample_thread is not None:
        try:
            sample_thread.join(timeout=1.0)
        except Exception:
            pass
    if (not dry_run and ser is not None
            and active_stop_event is not None and active_thread is not None):
        try:
            _rf_stop(ser, active_stop_event, active_thread)
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Wait helper — pauses worker thread until GUI button is clicked
# ---------------------------------------------------------------------------
def _wait_user(msg: str) -> None:
    """Surface a prompt in the UI.

    In auto mode (LIVE_AUTO_RUN=True) the worker thread auto-advances after a
    short pause instead of blocking on the Continue button — the message is
    still shown so the operator can see what step is starting.
    """
    _state.status = msg
    if LIVE_AUTO_RUN:
        time.sleep(0.5)
        return
    _state.waiting_for_user = True
    _state.continue_event.clear()
    _state.continue_event.wait()
    _state.waiting_for_user = False


# ---------------------------------------------------------------------------
# Experiment worker thread
# ---------------------------------------------------------------------------
def _run_experiment(dry_run: bool, robot: Robot, ser) -> None:
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    trial_dir = _next_trial_dir()
    roll_dist = _roll_dist()
    _state.roll_distance_m = roll_dist
    d = COMPRESS_DEPTH_M

    session_meta = {
        "controller": FR3_POSE_CONTROLLER,
        "roller_diameter_m": ROLLER_DIAMETER_M,
        "compress_depth_m": d,
        "finray_length_m": FINRAY_LENGTH_M,
        "roll_length_percent": ROLL_LENGTH_PERCENT,
        "roll_distance_m": roll_dist,
        "speeds_m_s": ROLL_SPEEDS_M_S,
        "expected_rf_samples": EXPECTED_RF_SAMPLES,
        "trial_dir": str(trial_dir),
    }

    active_sev = active_thr = None

    try:
        _state.status = "Waiting for robot…"
        robot.wait_until_ready()
        _setup_controller(robot)

        contact = (Pose(np.array(CONTACT_POSITION_M, dtype=float), BASE_ORI)
                   if CONTACT_POSITION_M else robot.end_effector_pose.copy())
        approach = Pose(contact.position + np.array([0.0, 0.0, APPROACH_HEIGHT_M]), BASE_ORI)
        baseline = Pose(
            contact.position + np.array(
                [0.0, 0.0, APPROACH_HEIGHT_M + BASELINE_EXTRA_HEIGHT_M]
            ),
            BASE_ORI,
        )

        # ── Move to baseline pose (approach + 50 mm) and capture static RF ────
        _state.status = (
            f"Moving to baseline pose (approach + {BASELINE_EXTRA_HEIGHT_M*1000:.0f} mm)…"
        )
        _wait_start_pose = robot.end_effector_pose.copy()
        robot.move_to(pose=baseline, speed=RETURN_SPEED_M_S)
        _wait_for_move(_wait_start_pose, baseline, RETURN_SPEED_M_S)

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
            _state.status = f"Capturing {STATIONARY_BASELINE_FRAMES} stationary RF frames…"
            if dry_run:
                print("  DRY RUN — skipping stationary RF capture.")
            else:
                static_rf_payload["frames"] = _capture_stationary(ser, STATIONARY_BASELINE_FRAMES)
                print(f"  Stationary baseline captured: {len(static_rf_payload['frames'])} frames")
            session_meta["stationary_baseline_frames"] = STATIONARY_BASELINE_FRAMES
            session_meta["baseline_extra_height_m"] = BASELINE_EXTRA_HEIGHT_M

        _state.status = "Moving to approach pose…"
        _wait_start_pose = robot.end_effector_pose.copy()
        robot.move_to(pose=approach, speed=RETURN_SPEED_M_S)
        _wait_for_move(_wait_start_pose, approach, RETURN_SPEED_M_S)

        trial_plan = [
            (speed_idx, rep_idx, speed)
            for speed_idx, speed in enumerate(ROLL_SPEEDS_M_S)
            for rep_idx in range(N_REPEATS)
        ]
        _wait_user(f"At approach. Click Continue to begin {len(trial_plan)} trials.")

        def _execute_trial(speed_idx, rep_idx, speed):
            """Run a single trial. Raises on failure for the retry loop to catch."""
            nonlocal active_sev, active_thr
            s_stop_event = None
            s_thread = None
            try:
                # Descend to contact
                _state.phase = "compress"
                _state.status = "Descending to contact…"
                _pre_descend_pose = robot.end_effector_pose.copy()
                robot.move_to(pose=contact, speed=APPROACH_SPEED_M_S)
                _wait_for_move(_pre_descend_pose, contact, APPROACH_SPEED_M_S)
                if CONTACT_HOLD_SEC > 0:
                    _state.status = (
                        f"Contact point reached; holding {CONTACT_HOLD_SEC:.1f} s to settle…"
                    )
                    time.sleep(CONTACT_HOLD_SEC)
                meas_contact = robot.end_effector_pose.copy()

                # Trajectory poses
                c_start = Pose(meas_contact.position + [0.0, 0.0, -d], BASE_ORI)
                c_end   = Pose(c_start.position + [0.0, roll_dist, 0.0], BASE_ORI)
                c_surf  = Pose(c_end.position + [0.0, 0.0, d], BASE_ORI)

                cmp_dur  = _move_dur(meas_contact, c_start, APPROACH_SPEED_M_S)
                sld_dur  = _move_dur(c_start, c_end, speed)
                dcmp_dur = _move_dur(c_end, c_surf, APPROACH_SPEED_M_S)
                rf_dur   = cmp_dur + SETTLE_SEC + sld_dur + dcmp_dur + (0.5 + EXEC_TRAJ_TIME_OFFSET + 2.0) * 3

                cmp_wps,  cmp_t  = _build_traj([(meas_contact, c_start)], [cmp_dur])
                sld_wps,  sld_t  = _build_traj([(c_start, c_end)], [sld_dur])
                dcmp_wps, dcmp_t = _build_traj([(c_end, c_surf)], [dcmp_dur])

                _state.slide_start_y = float(c_start.position[1])

                # Start RF + EEF sampling
                frames = []
                ee_trial = []
                t0 = time.perf_counter()
                if not dry_run:
                    frames, active_sev, first_ev, active_thr, t0 = _rf_start(ser)
                    _state.status = "Waiting for first RF frame…"
                    if not first_ev.wait(timeout=SERIAL_TIMEOUT_SEC):
                        raise TimeoutError("No RF frame from Teensy.")
                s_stop_event = threading.Event()
                s_thread = threading.Thread(
                    target=_sample_ee, args=(robot, ee_trial, s_stop_event, t0), daemon=True)
                s_thread.start()

                if PRE_COMPRESS_HOLD_SEC > 0:
                    _state.status = (
                        f"Holding {PRE_COMPRESS_HOLD_SEC:.1f} s before starting compress…"
                    )
                    time.sleep(PRE_COMPRESS_HOLD_SEC)

                # Compress
                _state.status = "Starting compress…"
                _state.phase = "compress"
                robot.execute_cartesian_traj(cmp_wps, cmp_t)
                while robot.wait_for_trajectory_completion(cmp_dur + EXEC_TRAJ_TIME_OFFSET): pass
                time.sleep(SETTLE_SEC)

                if not dry_run:
                    _state.status = "Confirming RF stream is active before slide…"
                    _ensure_rf_active(frames)

                # Slide
                _state.status = f"Sliding at {speed*1000:.1f} mm/s…"
                _state.phase = "slide"
                robot.execute_cartesian_traj(sld_wps, sld_t)
                while robot.wait_for_trajectory_completion(sld_dur + EXEC_TRAJ_TIME_OFFSET): pass

                if POST_SLIDE_HOLD_SEC > 0:
                    _state.status = (
                        f"Slide done; holding {POST_SLIDE_HOLD_SEC:.1f} s before decompress…"
                    )
                    time.sleep(POST_SLIDE_HOLD_SEC)

                # Decompress
                _state.status = "Decompressing…"
                _state.phase = "decompress"
                robot.execute_cartesian_traj(dcmp_wps, dcmp_t)
                while robot.wait_for_trajectory_completion(dcmp_dur + EXEC_TRAJ_TIME_OFFSET): pass

                # Stop samplers (success path)
                s_stop_event.set(); s_thread.join(timeout=1.0)
                s_stop_event = None; s_thread = None
                if not dry_run:
                    _rf_stop(ser, active_sev, active_thr)
                    active_sev = active_thr = None

                # Z-based slide-only filter
                cz = float(c_start.position[2])
                slide_only = [ep for ep in ee_trial if abs(ep["position"][2] - cz) < 0.001]

                # Return to approach
                _state.phase = "idle"
                _state.status = "Returning to approach…"
                _pre_return_pose = robot.end_effector_pose.copy()
                robot.move_to(pose=approach, speed=RETURN_SPEED_M_S)
                _wait_for_move(_pre_return_pose, approach, RETURN_SPEED_M_S)

                # Save
                payload = {
                    "speed_m_s": speed, "roll_distance_m": roll_dist,
                    "roll_end_offset_m": ROLL_END_OFFSET_M,
                    "roller_diameter_m": ROLLER_DIAMETER_M,
                    "roller_radius_m": ROLLER_DIAMETER_M / 2.0,
                    "frames": list(frames),
                    "ee_poses": ee_trial, "ee_poses_slide_only": slide_only,
                    "rf_window_duration_s": rf_dur, "slide_duration_s": sld_dur,
                    "compress_duration_s": cmp_dur, "decompress_duration_s": dcmp_dur,
                    "approach_position": approach.position.tolist(),
                    "contact_position": meas_contact.position.tolist(),
                    "compressed_start_position": c_start.position.tolist(),
                    "compressed_end_position": c_end.position.tolist(),
                    "surface_end_position": c_surf.position.tolist(),
                    "rf_capture_summary": _rf_summary(frames, rf_dur),
                    "static_rf": static_rf_payload,
                    "repeat_idx": rep_idx,
                    "speed_idx": speed_idx,
                }
                _save(trial_dir, speed, payload, session_meta, repeat_idx=rep_idx)
                print(f"  RF coverage: {payload['rf_capture_summary']['coverage_ratio']*100:.1f}%")
            finally:
                # Best-effort cleanup of background threads so a retry doesn't
                # leak workers reading from a dead robot/teensy.
                _stop_threads_safely(s_stop_event, s_thread, ser, active_sev, active_thr, dry_run)
                if active_sev is not None or active_thr is not None:
                    active_sev = None
                    active_thr = None

        for t_idx, (speed_idx, rep_idx, speed) in enumerate(trial_plan):
            _state.speed_m_s = speed
            _state.trial_label = (
                f"Trial {t_idx+1}/{len(trial_plan)}  "
                f"{speed*1000:.1f} mm/s  "
                f"(speed {speed_idx+1}/{len(ROLL_SPEEDS_M_S)}, rep {rep_idx+1}/{N_REPEATS})"
            )
            _state.phase = "idle"
            _wait_user(f"{_state.trial_label} — click Continue to start.")

            attempt = 0
            while True:
                attempt += 1
                try:
                    _execute_trial(speed_idx, rep_idx, speed)
                    break  # success → next trial
                except Exception as err:
                    print(f"\n*** Trial {t_idx+1} attempt {attempt} failed: {err} ***")
                    _state.status = (
                        f"Trial {t_idx+1} failed (attempt {attempt}). "
                        "Recovering…"
                    )
                    if not _recover_robot(robot):
                        print("    Recovery failed; aborting session.")
                        raise
                    # After fr3-launch is back, move to approach pose before retrying
                    try:
                        _pre_back_pose = robot.end_effector_pose.copy()
                        robot.move_to(pose=approach, speed=RETURN_SPEED_M_S)
                        _wait_for_move(_pre_back_pose, approach, RETURN_SPEED_M_S)
                    except Exception as err2:
                        print(f"    Failed to return to approach after recovery: {err2}")
                        raise
                    print(f"  Retrying trial {t_idx+1} (attempt {attempt + 1})…\n")
                    _state.status = f"Retrying trial {t_idx+1}…"

        _state.status = f"Session complete. Results: {trial_dir}"

    finally:
        if ser is not None and active_sev is not None and active_thr is not None:
            _rf_stop(ser, active_sev, active_thr)
        if ser is not None:
            ser.close()
        robot.shutdown()
        _state.done = True
        _state.phase = "done"


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="Live rolling experiment viewer.")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--port", default=SERIAL_PORT)
    args = parser.parse_args()

    _load_qt()

    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")

    # ── Window defined here so Qt classes are available ──────────────────────
    class _Window(QtWidgets.QMainWindow):
        def __init__(self):
            super().__init__()
            self.setWindowTitle("Live Rolling Experiment")
            pg.setConfigOption("background", "#12121e")
            pg.setConfigOption("foreground", "#d4d4f0")

            central = QtWidgets.QWidget()
            self.setCentralWidget(central)
            outer = QtWidgets.QHBoxLayout(central)
            outer.setContentsMargins(4, 4, 4, 4)
            outer.setSpacing(4)

            # ── Left: 4 RF plots ───────────────────────────────────────────
            left_gl = pg.GraphicsLayoutWidget()
            outer.addWidget(left_gl, stretch=3)
            self._rf_curves = []
            x_idx = np.arange(EXPECTED_RF_SAMPLES)
            _first_rf_plot = None
            for i, (name, clr) in enumerate(zip(CHANNEL_MARKERS, _CH_COLORS)):
                p = left_gl.addPlot(row=0, col=i, title=name)
                p.setLabel("bottom", "Sample index")
                if i == 0:
                    p.setLabel("left", "ADC")
                    _first_rf_plot = p
                else:
                    p.setXLink(_first_rf_plot)
                    p.getAxis("left").setStyle(showValues=False)
                p.setXRange(350, EXPECTED_RF_SAMPLES - 1, padding=0)
                p.setYRange(0, 4095, padding=0.02)
                p.showGrid(x=True, y=True, alpha=0.25)
                crv = p.plot(x_idx, np.zeros(EXPECTED_RF_SAMPLES),
                             pen=pg.mkPen(color=clr, width=1))
                self._rf_curves.append(crv)

            # ── Right column ───────────────────────────────────────────────
            rw = QtWidgets.QWidget()
            rv = QtWidgets.QVBoxLayout(rw)
            rv.setContentsMargins(0, 0, 0, 0)
            rv.setSpacing(4)
            outer.addWidget(rw, stretch=2)

            # ── Right top: Contact position viz ────────────────────────────
            top_gl = pg.GraphicsLayoutWidget()
            rv.addWidget(top_gl, stretch=3)
            cp = top_gl.addPlot(title="Contact Position")
            cp.setXRange(-6.0, 6.0, padding=0)
            cp.setYRange(-0.05, 1.05, padding=0)
            cp.setLabel("left", "sensor pos (base→tip)")
            cp.setLabel("bottom", "lateral [mm]")
            cp.showGrid(x=False, y=True, alpha=0.3)
            self._cp = cp

            # Sensor outline
            cp.plot([-5, -5, 5, 5, -5], [0, 1, 1, 0, 0],
                    pen=pg.mkPen("#4a90e2", width=1.5))
            # Waveguide guide lines
            for xi, clr in zip(WG_LATERAL_MM, _CH_COLORS):
                cp.plot([xi, xi], [0, 1],
                        pen=pg.mkPen(color=clr, width=0.8, style=QtCore.Qt.DashLine))
            # Labels
            for xi, name in zip(WG_LATERAL_MM, CHANNEL_MARKERS):
                t = pg.TextItem(name, color="#7070a0", anchor=(0.5, 1.1))
                t.setPos(xi, 0)
                cp.addItem(t)

            # Rolling contact band
            self._band = pg.LinearRegionItem(
                values=[0.0, 0.05], orientation="horizontal",
                movable=False,
                brush=pg.mkBrush(255, 60, 60, 110),
                pen=pg.mkPen("#ff3b30", width=1.5),
            )
            cp.addItem(self._band)

            # Per-channel Hilbert peak scatter
            self._peaks = pg.ScatterPlotItem(
                x=WG_LATERAL_MM, y=[0.5] * 4, size=12,
                brush=[pg.mkBrush(*c) for c in _CH_COLORS],
                pen=pg.mkPen("w", width=0.6),
            )
            cp.addItem(self._peaks)

            # Phase label
            self._phase_txt = pg.TextItem("IDLE", color="#ffcc44", anchor=(0.5, 0))
            self._phase_txt.setPos(0, 1.0)
            cp.addItem(self._phase_txt)

            # ── Right bottom: EEF metrics ───────────────────────────────────
            self._metrics = QtWidgets.QLabel("Waiting for EEF data…")
            self._metrics.setStyleSheet(
                "font-family: monospace; font-size: 8pt; color: #d4d4f0;"
                "background: #1c1c30; padding: 8px; "
                "border: 1px solid #2e2e50; border-radius: 4px;"
            )
            self._metrics.setAlignment(
                QtCore.Qt.AlignTop | QtCore.Qt.AlignLeft)
            rv.addWidget(self._metrics, stretch=2)

            # ── Toolbar ────────────────────────────────────────────────────
            tb = self.addToolBar("Controls")
            tb.setMovable(False)
            tb.setStyleSheet("background:#1c1c30; border:none; spacing:6px;")

            self._btn = QtWidgets.QPushButton("  Continue ▶  ")
            self._btn.setStyleSheet(
                "QPushButton{background:#2e4070;color:#d4d4f0;padding:4px 14px;"
                "border-radius:4px;font-weight:bold;font-size:9pt;}"
                "QPushButton:hover{background:#3a5090;}"
                "QPushButton:disabled{background:#222230;color:#555;}"
            )
            self._btn.setEnabled(False)
            self._btn.clicked.connect(self._on_continue)
            tb.addWidget(self._btn)

            self._status_lbl = QtWidgets.QLabel("  Starting…")
            self._status_lbl.setStyleSheet("color:#7070a0;font-size:8pt;")
            tb.addWidget(self._status_lbl)

            # ── Timer ──────────────────────────────────────────────────────
            timer = QtCore.QTimer(self)
            timer.timeout.connect(self._update)
            timer.start(50)  # 20 Hz

            self._done_shown = False

        def _on_continue(self):
            self._btn.setEnabled(False)
            _state.continue_event.set()

        def _update(self):
            if _state.waiting_for_user and not self._btn.isEnabled():
                self._btn.setEnabled(True)
            self._status_lbl.setText(f"  {_state.status}")

            # RF curves
            if _state.latest_rf is not None:
                for ch, crv in enumerate(self._rf_curves):
                    crv.setData(_state.latest_rf[ch])
                peaks_norm = []
                for frm in _state.latest_rf:
                    env = np.abs(hilbert(frm.astype(float)))
                    peaks_norm.append(float(np.argmax(env)) / (EXPECTED_RF_SAMPLES - 1))
                self._peaks.setData(x=WG_LATERAL_MM, y=peaks_norm)

            # Contact band
            if _state.phase == "slide" and _state.roll_distance_m > 0:
                ee = _state.latest_ee
                if ee:
                    cy = ee.get("position", [0, 0, 0])[1]
                    prog = float(np.clip(
                        (cy - _state.slide_start_y) / _state.roll_distance_m, 0, 1))
                    self._band.setRegion([max(0, prog - 0.025), min(1, prog + 0.025)])
            elif _state.phase == "compress":
                self._band.setRegion([0.0, 0.04])
            elif _state.phase == "decompress":
                self._band.setRegion([0.96, 1.0])

            PH_CLR = {
                "compress":   "#ffcc44", "slide": "#44ff99",
                "decompress": "#ff9944", "idle":  "#7070a0", "done": "#7070a0",
            }
            self._phase_txt.setColor(PH_CLR.get(_state.phase, "#d4d4f0"))
            self._phase_txt.setText(_state.phase.upper())
            self.setWindowTitle(
                f"Live Rolling Experiment — {_state.trial_label} — {_state.phase}")

            # EEF metrics
            ee = _state.latest_ee
            if ee:
                pos = ee.get("position", [float("nan")] * 3)
                rot = ee.get("orientation_euler_deg_xyz", [float("nan")] * 3)
                frc = ee.get("force_xyz", [float("nan")] * 3)
                vel = ee.get("velocity_xyz") or [float("nan")] * 3
                acc = ee.get("acceleration_xyz") or [float("nan")] * 3

                def _f(v):  return f"{v:+.2f}" if v == v else "  n/a"
                def _fv(v): return f"{v:+.5f}" if v == v else "   n/a  "
                def _fa(v): return f"{v:+.3f}" if v == v else "  n/a"

                self._metrics.setText(
                    "<pre style='margin:0;padding:0'>"
                    f"{_state.trial_label}  [{_state.phase}]  RF: {_state.rf_frame_count}\n\n"
                    f"<span style='color:#ff4444'>measured pose</span>\n"
                    f" x  {pos[0]:+.4f} m    Fx  {_f(frc[0])} N\n"
                    f" y  {pos[1]:+.4f} m    Fy  {_f(frc[1])} N\n"
                    f" z  {pos[2]:+.4f} m    Fz  {_f(frc[2])} N\n\n"
                    f"rx  {rot[0]:+.2f}°      vx  {_fv(vel[0])} m/s\n"
                    f"ry  {rot[1]:+.2f}°      vy  {_fv(vel[1])} m/s\n"
                    f"rz  {rot[2]:+.2f}°      vz  {_fv(vel[2])} m/s\n\n"
                    f"               ax  {_fa(acc[0])} m/s²\n"
                    f"               ay  {_fa(acc[1])} m/s²\n"
                    f"               az  {_fa(acc[2])} m/s²"
                    "</pre>"
                )

            if _state.done and not self._done_shown:
                self._done_shown = True
                QtWidgets.QMessageBox.information(
                    self, "Done", f"Experiment complete!\n\n{_state.status}")

    window = _Window()
    window.resize(1480, 940)
    window.show()

    # Start experiment thread
    dry_run = args.dry_run or _CFG_DRY_RUN
    robot = Robot(namespace=ROBOT_NAMESPACE)
    ser = (
        serial.Serial(args.port, BAUD_RATE,
                      timeout=SERIAL_TIMEOUT_SEC, write_timeout=SERIAL_TIMEOUT_SEC)
        if not dry_run else None
    )
    exp_thr = threading.Thread(
        target=_run_experiment, args=(dry_run, robot, ser), daemon=True)
    exp_thr.start()

    exec_fn = getattr(app, "exec", None) or app.exec_
    sys.exit(exec_fn())


if __name__ == "__main__":
    main()
