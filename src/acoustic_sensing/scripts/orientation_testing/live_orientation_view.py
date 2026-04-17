#!/usr/bin/env python3
"""Live viewer + experiment runner for orientation (bar-angle) testing.

Run this INSTEAD OF orientation_experiment.py to get a live pyqtgraph
window showing all 4 RF channels, the angular contact diagram, and every
EEF metric while the experiment is in progress.

Usage
-----
    python live_orientation_view.py [--dry-run] [--port /dev/ttyACM0]

The GUI replaces all terminal prompts:
  • "Continue ▶" toolbar button  → replaces "Press Enter…"
  • Angle input dialog (opens automatically before each trial)  → you enter
    the bar angle BEFORE pressing the sensor in, so the expected contact
    line is shown live during the hold phase.
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

from orientation_config import (
    APPROACH_HEIGHT_M, APPROACH_SPEED_M_S, BASE_ORI_EULER_DEG,
    BAUD_RATE, COMPRESS_DEPTH_M, CONTACT_POSITION_M,
    CONTROLLER_NAME, DRY_RUN as _CFG_DRY_RUN, EE_SAMPLE_PERIOD_SEC,
    HOLD_AT_CONTACT_SEC, N_REPEATS, PROBING_CONFIG_REL, RETURN_SPEED_M_S,
    ROBOT_NAMESPACE, SERIAL_PORT, SERIAL_TIMEOUT_SEC,
    WAVEGUIDE_LATERAL_POSITIONS_MM as _WG_MM,
    WAVEGUIDE_SAMPLE_PITCH_MM,
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
            "live_orientation_view.py requires pyqtgraph and a Qt binding.\n"
            "Install: pip install pyqtgraph PyQt5"
        ) from exc
    pg = _pg
    QtCore = _Qc
    QtWidgets = _Qw
    if not hasattr(QtCore, "Signal"):
        QtCore.Signal = QtCore.pyqtSignal


# ---------------------------------------------------------------------------
# Shared state
# ---------------------------------------------------------------------------
class _State:
    def __init__(self):
        self.latest_rf: list | None = None      # [4 × np.ndarray float32]
        self.rf_frame_count: int = 0
        self.latest_ee: dict = {}
        self.latest_cmd: dict = {}
        self.phase: str = "idle"                # idle/compress/hold/decompress/done
        self.angle_deg: float = 0.0
        self.status: str = "Initialising…"
        self.done: bool = False
        # User-prompt flow
        self.waiting_for_user: bool = False
        self.continue_event: threading.Event = threading.Event()
        self.waiting_for_angle: bool = False
        self.angle_event: threading.Event = threading.Event()


_state = _State()

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
PROBING_CONFIG = CONFIG_DIR / PROBING_CONFIG_REL
BASE_ORI = Rotation.from_euler("xyz", BASE_ORI_EULER_DEG, degrees=True)
WG_MM = np.array(_WG_MM, dtype=float)
CMD_START = bytes([0x43])
CMD_STOP  = bytes([0x45])
CHANNEL_MARKERS = ("S0", "S1", "S2", "S3")
STREAM_END = "STREAM_END"
EXPECTED_RF_SAMPLES = 1000      # default; actual frame size may differ
RESULTS_DIR = Path(__file__).resolve().parent / "results"
DEPTH_MM = COMPRESS_DEPTH_M * 1000.0

_CH_COLORS = [
    (0,   212, 255),   # S0 – cyan
    (255, 112,  67),   # S1 – orange
    (179, 136, 255),   # S2 – purple
    (105, 240, 174),   # S3 – green
]

# ---------------------------------------------------------------------------
# RF streaming
# ---------------------------------------------------------------------------
def _stream_reader(ser, frames, stop_ev, first_ev, t0):
    cur_ch = None
    cur_smp: list = []
    cur_frame: dict = {}
    buf = bytearray()
    n_smp = EXPECTED_RF_SAMPLES  # updated on first complete frame

    def _handle(raw: bytes) -> None:
        nonlocal cur_ch, cur_smp, cur_frame, n_smp
        line = raw.decode("ascii", errors="ignore").strip()
        if not line:
            return
        if line == STREAM_END:
            stop_ev.set(); return
        if line in CHANNEL_MARKERS:
            cur_ch = line; cur_smp = []; return
        if line == "T":
            if cur_ch is not None:
                cur_frame[cur_ch] = cur_smp; cur_ch = None
                if len(cur_frame) == len(CHANNEL_MARKERS):
                    lengths = [len(cur_frame[m]) for m in CHANNEL_MARKERS]
                    if len(set(lengths)) != 1:
                        cur_frame = {}; return
                    n_smp = lengths[0]
                    arrs = [np.asarray(cur_frame[m], dtype=np.float32) for m in CHANNEL_MARKERS]
                    frames.append((arrs, time.perf_counter() - t0))
                    _state.latest_rf = arrs
                    _state.rf_frame_count += 1
                    first_ev.set()
                    cur_frame = {}
            return
        if cur_ch is not None:
            try: cur_smp.append(int(line))
            except ValueError: pass

    while not stop_ev.is_set():
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
    # Wait for the reader to see STREAM_END (which sets stop_ev naturally).
    # Only force-set after timeout so the Teensy has time to process the stop.
    thr.join(timeout=SERIAL_TIMEOUT_SEC)
    stop_ev.set()


# ---------------------------------------------------------------------------
# EEF sampler
# ---------------------------------------------------------------------------
def _make_sampler(get_pose, get_wrench):
    def _sampler(poses, stop_ev, t0, period_s=EE_SAMPLE_PERIOD_SEC):
        prev_t = prev_pos = prev_vel = None
        while not stop_ev.is_set():
            try:
                ee = get_pose()
                wr = get_wrench()
            except RuntimeError:
                time.sleep(period_s); continue
            t = time.perf_counter() - t0
            pos = ee.position.tolist()
            dt = t - prev_t if prev_t is not None else None
            vel = ([(pos[i] - prev_pos[i]) / dt for i in range(3)]
                   if (dt and dt > 0 and prev_pos is not None) else None)
            acc = ([(vel[i] - prev_vel[i]) / dt for i in range(3)]
                   if (vel is not None and prev_vel is not None) else None)
            poses.append({
                "t": t, "position": pos,
                "orientation_quat": ee.orientation.as_quat().tolist(),
                "orientation_euler_deg_xyz": ee.orientation.as_euler("xyz", degrees=True).tolist(),
                "force_xyz": wr["force"].tolist(),
                "torque_xyz": wr["torque"].tolist(),
                "velocity_xyz": vel, "acceleration_xyz": acc,
            })
            prev_t = t; prev_pos = pos; prev_vel = vel
            time.sleep(period_s)
    return _sampler


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


def _sample_cmd(robot, cmd_poses, stop_ev, t0, period_s=EE_SAMPLE_PERIOD_SEC):
    prev_t = prev_pos = prev_vel = None
    while not stop_ev.is_set():
        try:
            ee = robot.target_pose
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
        cmd_poses.append(entry)
        _state.latest_cmd = entry
        prev_t = t; prev_pos = pos; prev_vel = vel
        time.sleep(period_s)


# ---------------------------------------------------------------------------
# Controller helpers
# ---------------------------------------------------------------------------
def _setup_controller(robot: Robot) -> None:
    robot.controller_switcher_client.switch_controller(CONTROLLER_NAME)
    robot.fr3_pose_controller_parameters_client.load_param_config(
        file_path=PROBING_CONFIG)
    deadline = time.time() + 5.0
    while time.time() < deadline:
        if robot.controller_switcher_client.get_active_controller() == CONTROLLER_NAME:
            return
        time.sleep(0.1)
    raise RuntimeError("Controller switch timed out.")


def _cartesian_move(robot, start, end, speed, settle=0.0):
    dist = float(np.linalg.norm(end.position - start.position))
    dur  = max(dist / speed, 0.1)
    robot.execute_cartesian_traj([(start,), (end,)], [0.0, dur],
                                 max_linear_velocity=speed)
    while robot.wait_for_trajectory_completion(dur):
        time.sleep(0.02)
    time.sleep(settle)


# ---------------------------------------------------------------------------
# Results helpers
# ---------------------------------------------------------------------------
def _alloc_trial_dir():
    c = 0
    d = RESULTS_DIR / f"trial_{c:02d}_depth{DEPTH_MM:.1f}mm"
    while d.exists():
        c += 1
        d = RESULTS_DIR / f"trial_{c:02d}_depth{DEPTH_MM:.1f}mm"
    return d


def _next_result_paths(trial_dir, angle_deg):
    full_dir   = trial_dir / "full_exp"
    frames_dir = trial_dir / "20frames"
    full_dir.mkdir(parents=True, exist_ok=True)
    frames_dir.mkdir(parents=True, exist_ok=True)
    c = 0
    full_p   = full_dir   / f"angle_{angle_deg:.1f}_{c:02d}.pkl"
    frames_p = frames_dir / f"20_angle_{angle_deg:.1f}_{c:02d}.pkl"
    while full_p.exists() or frames_p.exists():
        c += 1
        full_p   = full_dir   / f"angle_{angle_deg:.1f}_{c:02d}.pkl"
        frames_p = frames_dir / f"20_angle_{angle_deg:.1f}_{c:02d}.pkl"
    return full_p, frames_p


# ---------------------------------------------------------------------------
# User-prompt helpers
# ---------------------------------------------------------------------------
def _wait_user(msg: str) -> None:
    _state.status = msg
    _state.waiting_for_user = True
    _state.continue_event.clear()
    _state.continue_event.wait()
    _state.waiting_for_user = False


def _request_angle() -> float:
    """Ask the GUI for the bar angle; blocks until the dialog is answered."""
    _state.waiting_for_angle = True
    _state.angle_event.clear()
    _state.angle_event.wait()
    _state.waiting_for_angle = False
    return _state.angle_deg


# ---------------------------------------------------------------------------
# Experiment worker thread
# ---------------------------------------------------------------------------
def _run_experiment(dry_run: bool, robot: Robot, ser) -> None:
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    trial_dir: Path | None = None
    active_sev = active_thr = None

    try:
        _state.status = "Waiting for robot…"
        robot.wait_until_ready()
        _setup_controller(robot)

        contact = (Pose(np.array(CONTACT_POSITION_M, dtype=float), BASE_ORI)
                   if CONTACT_POSITION_M else robot.end_effector_pose.copy())
        approach = Pose(contact.position + np.array([0.0, 0.0, APPROACH_HEIGHT_M]), BASE_ORI)

        _state.status = "Moving to approach pose…"
        robot.move_to(pose=approach, speed=RETURN_SPEED_M_S)
        time.sleep(0.5)

        _wait_user("At approach pose. Set bar angle, then click Continue.")

        while True:
            # Ask for angle once — all N_REPEATS presses share the same angle entry.
            angle_deg = _request_angle()
            _state.angle_deg = angle_deg

            if trial_dir is None:
                trial_dir = _alloc_trial_dir()
                trial_dir.mkdir(parents=True, exist_ok=False)

            for repeat_idx in range(N_REPEATS):
                _state.status = (
                    f"Angle {angle_deg:+.1f}° — repeat {repeat_idx + 1}/{N_REPEATS}"
                    " — descending to contact…"
                )

                # Descend to contact
                _state.phase = "compress"
                robot.move_to(pose=contact, speed=APPROACH_SPEED_M_S)
                time.sleep(0.3)
                meas_contact = robot.end_effector_pose.copy()
                meas_compressed = Pose(
                    meas_contact.position + np.array([0.0, 0.0, -COMPRESS_DEPTH_M]),
                    BASE_ORI,
                )

                # Start EEF sampler and commanded pose sampler
                frames: list = []
                ee_poses: list = []
                cmd_poses: list = []
                t0 = time.perf_counter()
                s_stop = threading.Event()
                s_thr = threading.Thread(
                    target=_sample_ee, args=(robot, ee_poses, s_stop, t0), daemon=True)
                c_thr = threading.Thread(
                    target=_sample_cmd, args=(robot, cmd_poses, s_stop, t0), daemon=True)
                s_thr.start()
                c_thr.start()

                # Compress
                _state.status = (
                    f"Compressing {DEPTH_MM:.1f} mm… [{repeat_idx + 1}/{N_REPEATS}]"
                )
                _state.phase = "compress"
                t_compress_start = time.perf_counter() - t0
                _cartesian_move(robot, meas_contact, meas_compressed, APPROACH_SPEED_M_S)
                t_compress_end = time.perf_counter() - t0

                # Acquire exactly 20 RF cycles at compressed depth
                _state.phase = "hold"
                RF_CYCLES = 20
                t_rf_start = t_rf_end = t_compress_end
                if dry_run:
                    print("  DRY RUN — skipping RF.")
                    t_hold_end = time.perf_counter() - t0
                else:
                    t_rf_start = time.perf_counter() - t0
                    frames, active_sev, first_ev, active_thr, _rf_t0 = _rf_start(ser)
                    _state.status = (
                        f"Waiting for first RF frame… [{repeat_idx + 1}/{N_REPEATS}]"
                    )
                    if not first_ev.wait(timeout=SERIAL_TIMEOUT_SEC):
                        _rf_stop(ser, active_sev, active_thr)
                        active_sev = active_thr = None
                        raise TimeoutError("No RF frame received.")
                    _state.status = (
                        f"Collecting {RF_CYCLES} RF frames… [{repeat_idx + 1}/{N_REPEATS}]"
                    )
                    deadline = time.time() + 30.0
                    while len(frames) < RF_CYCLES and time.time() < deadline:
                        time.sleep(0.01)
                    _rf_stop(ser, active_sev, active_thr)
                    active_sev = active_thr = None
                    frames = frames[:RF_CYCLES]
                    t_rf_end = time.perf_counter() - t0
                    t_hold_end = t_rf_end
                    print(f"  RF stopped: {len(frames)} frames collected.")

                # Lift back to contact
                _state.status = (
                    f"Lifting to contact level… [{repeat_idx + 1}/{N_REPEATS}]"
                )
                _state.phase = "decompress"
                t_lift_start = time.perf_counter() - t0
                _cartesian_move(robot, meas_compressed, meas_contact, APPROACH_SPEED_M_S,
                                settle=0.3)
                t_lift_end = time.perf_counter() - t0

                # Stop samplers
                s_stop.set(); s_thr.join(timeout=1.0); c_thr.join(timeout=1.0)

                # Return to approach
                _state.phase = "idle"
                _state.status = (
                    f"Returning to approach… [{repeat_idx + 1}/{N_REPEATS}]"
                )
                robot.move_to(pose=approach, speed=RETURN_SPEED_M_S)
                time.sleep(0.5)

                # Save
                full_out, frames_out = _next_result_paths(trial_dir, angle_deg)

                _pose_dict = lambda p: {
                    "position": p.position.tolist(),
                    "orientation_quat": p.orientation.as_quat().tolist(),
                    "orientation_euler_deg_xyz": p.orientation.as_euler(
                        "xyz", degrees=True).tolist(),
                }
                common = {
                    "angle_deg": angle_deg,
                    "repeat_idx": repeat_idx,
                    "n_repeats": N_REPEATS,
                    "compress_depth_m": COMPRESS_DEPTH_M,
                    "contact_position": meas_contact.position.tolist(),
                    "approach_position": approach.position.tolist(),
                    "compressed_position": meas_compressed.position.tolist(),
                    "commanded_contact_pose": _pose_dict(meas_contact),
                    "commanded_approach_pose": _pose_dict(approach),
                    "commanded_compressed_pose": _pose_dict(meas_compressed),
                    "waveguide_lateral_positions_mm": WG_MM.tolist(),
                    "waveguide_sample_pitch_mm": WAVEGUIDE_SAMPLE_PITCH_MM,
                    "base_ori_euler_deg": BASE_ORI_EULER_DEG,
                    "t_rf_start_s": t_rf_start,
                    "t_rf_end_s": t_rf_end,
                }
                segments = [
                    {"phase": "compress",
                     "t_start_s": t_compress_start, "t_end_s": t_compress_end,
                     "start_pose": _pose_dict(meas_contact),
                     "end_pose": _pose_dict(meas_compressed)},
                    {"phase": "rf_acquisition",
                     "t_start_s": t_rf_start, "t_end_s": t_rf_end,
                     "start_pose": _pose_dict(meas_compressed),
                     "end_pose": _pose_dict(meas_compressed)},
                    {"phase": "lift",
                     "t_start_s": t_lift_start, "t_end_s": t_lift_end,
                     "start_pose": _pose_dict(meas_compressed),
                     "end_pose": _pose_dict(meas_contact)},
                ]

                # Full experiment file
                full_payload = {
                    **common,
                    "t0": t0,
                    "frames": list(frames),
                    "ee_poses": ee_poses,
                    "commanded_ee_poses": cmd_poses,
                    "commanded_segments": segments,
                }
                with open(full_out, "wb") as f:
                    pickle.dump(full_payload, f)

                # 20-frames focused file — RF + EEF captured during RF window only
                rf_ee  = [p for p in ee_poses  if t_rf_start <= p["t"] <= t_rf_end]
                rf_cmd = [p for p in cmd_poses if t_rf_start <= p["t"] <= t_rf_end]
                frames_payload = {
                    **common,
                    "frames": list(frames),
                    "ee_poses_during_rf": rf_ee,
                    "commanded_ee_poses_during_rf": rf_cmd,
                }
                with open(frames_out, "wb") as f:
                    pickle.dump(frames_payload, f)

                _state.status = (
                    f"Saved [{repeat_idx + 1}/{N_REPEATS}]: {full_out.name}"
                )
                print(f"  Saved full : {full_out}")
                print(f"  Saved 20fr : {frames_out}")

            _wait_user(
                f"All {N_REPEATS} repeats done (angle {angle_deg:+.1f}°). "
                "Rotate bar, then click Continue for next angle."
            )

    finally:
        if ser is not None and active_sev is not None and active_thr is not None:
            _rf_stop(ser, active_sev, active_thr)
        if ser is not None:
            ser.close()
        robot.shutdown()
        _state.done = True
        _state.phase = "done"


# ---------------------------------------------------------------------------
# Angular diagram geometry helpers
# ---------------------------------------------------------------------------
def _bar_line_xy(angle_deg: float, n_samples: int, pitch_mm: float,
                 x_extent: float = 35.0):
    """Return (x_arr, y_mm_arr) for the bar line. Both axes in mm so
    the visual slope equals the physical angle when aspect is locked."""
    phi = np.radians(angle_deg)
    xs = np.array([-x_extent, x_extent])
    ys = (n_samples / 2.0) * pitch_mm + xs * np.tan(phi)
    return xs, ys


def _expected_samples(angle_deg: float, wg_mm: np.ndarray,
                      n_samples: int, pitch_mm: float) -> np.ndarray:
    """Return y positions in mm for each waveguide at the given angle."""
    phi = np.radians(angle_deg)
    return (n_samples / 2.0) * pitch_mm + wg_mm * np.tan(phi)


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="Live orientation experiment viewer.")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--port", default=SERIAL_PORT)
    args = parser.parse_args()

    _load_qt()

    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")

    class _Window(QtWidgets.QMainWindow):
        def __init__(self):
            super().__init__()
            self.setWindowTitle("Live Orientation Experiment")
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
                p.setXRange(500, EXPECTED_RF_SAMPLES - 1, padding=0)
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

            # ── Right top: Angular diagram ─────────────────────────────────
            top_gl = pg.GraphicsLayoutWidget()
            rv.addWidget(top_gl, stretch=3)

            _D = EXPECTED_RF_SAMPLES * WAVEGUIDE_SAMPLE_PITCH_MM  # total depth in mm
            _D_MID = _D / 2.0

            ap = top_gl.addPlot(title="Bar Angle: —")
            ap.setAspectLocked(True)
            ap.setXRange(-6.0, 6.0, padding=0)
            ap.setYRange(-5, _D + 5, padding=0)
            ap.setLabel("bottom", "lateral [mm]")
            ap.setLabel("left", "depth [mm]")
            ap.showGrid(x=True, y=True, alpha=0.25)
            self._ap = ap

            # Sensor body outline
            ap.plot(
                [-5, -5, 5, 5, -5],
                [0, _D, _D, 0, 0],
                pen=pg.mkPen("#4a90e2", width=1.5),
            )

            # Waveguide lines (vertical)
            for xi, clr in zip(WG_MM, _CH_COLORS):
                ap.plot([xi, xi], [0, _D],
                        pen=pg.mkPen(color=clr, width=0.9,
                                     style=QtCore.Qt.DashLine))

            # Bar line (angle-dependent, updated live)
            bx, by = _bar_line_xy(0.0, EXPECTED_RF_SAMPLES, WAVEGUIDE_SAMPLE_PITCH_MM)
            self._bar_curve = ap.plot(bx, by,
                                      pen=pg.mkPen("#ffcc44", width=2.5))

            # Gripper-axis arrow (static)
            arr = pg.ArrowItem(pos=(0, _D + 3),
                               angle=-90, tipAngle=30, headLen=12,
                               pen=pg.mkPen("#7070a0", width=1.2),
                               brush=pg.mkBrush("#7070a0"))
            ap.addItem(arr)
            lbl_grip = pg.TextItem("gripper\naxis", color="#7070a0",
                                   anchor=(0.5, 0))
            lbl_grip.setPos(1.0, _D + 1)
            ap.addItem(lbl_grip)

            # Expected contact markers (open circles — one per channel)
            exp_pts = _expected_samples(0.0, WG_MM, EXPECTED_RF_SAMPLES,
                                        WAVEGUIDE_SAMPLE_PITCH_MM)
            self._exp_scatter = pg.ScatterPlotItem(
                x=WG_MM, y=exp_pts, size=12, symbol="o",
                brush=pg.mkBrush(None),
                pen=[pg.mkPen(clr, width=2) for clr in _CH_COLORS],
            )
            ap.addItem(self._exp_scatter)

            # Measured contact markers (filled circles — updated per frame)
            self._meas_scatter = pg.ScatterPlotItem(
                x=WG_MM, y=exp_pts, size=8, symbol="o",
                brush=[pg.mkBrush(*clr) for clr in _CH_COLORS],
                pen=pg.mkPen("w", width=0.5),
            )
            ap.addItem(self._meas_scatter)

            # Angle label
            self._angle_lbl = pg.TextItem(
                "+0.0°", color="#ffcc44", anchor=(0.0, 0.5))
            self._angle_lbl.setPos(0.6, _D_MID)
            ap.addItem(self._angle_lbl)

            # Phase label
            self._phase_txt = pg.TextItem("IDLE", color="#7070a0", anchor=(0.5, 0))
            self._phase_txt.setPos(0, -3)
            ap.addItem(self._phase_txt)

            self._last_angle = None   # track to avoid redundant updates

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
            self._angle_dialog_shown = False

        def _on_continue(self):
            self._btn.setEnabled(False)
            _state.continue_event.set()

        def _update(self):
            # Continue button
            if _state.waiting_for_user and not self._btn.isEnabled():
                self._btn.setEnabled(True)
            self._status_lbl.setText(f"  {_state.status}")

            # Angle dialog (shown once per waiting_for_angle pulse)
            if _state.waiting_for_angle and not self._angle_dialog_shown:
                self._angle_dialog_shown = True
                angle, ok = QtWidgets.QInputDialog.getDouble(
                    self, "Bar angle",
                    "Set the physical bar angle, then enter it here [°]:",
                    _state.angle_deg, -90.0, 90.0, 1,
                )
                _state.angle_deg = angle if ok else _state.angle_deg
                _state.angle_event.set()
                self._angle_dialog_shown = False

            # RF curves — resize x-axis if frame length differs
            if _state.latest_rf is not None:
                n = len(_state.latest_rf[0])
                xd = self._rf_curves[0].xData
                if xd is None or len(xd) != n:
                    for crv in self._rf_curves:
                        crv.getViewBox().setXRange(500, n - 1, padding=0)
                for ch, crv in enumerate(self._rf_curves):
                    crv.setData(_state.latest_rf[ch])

            # Angular diagram updates
            angle = _state.angle_deg
            n_smp = (len(_state.latest_rf[0]) if _state.latest_rf else EXPECTED_RF_SAMPLES)

            if angle != self._last_angle:
                self._last_angle = angle
                bx, by = _bar_line_xy(angle, n_smp, WAVEGUIDE_SAMPLE_PITCH_MM)
                self._bar_curve.setData(bx, by)
                exp_pts_mm = _expected_samples(angle, WG_MM, n_smp,
                                               WAVEGUIDE_SAMPLE_PITCH_MM)
                self._exp_scatter.setData(x=WG_MM, y=exp_pts_mm)
                self._angle_lbl.setText(f"{angle:+.1f}°")
                self._ap.setTitle(f"Bar Angle: {angle:+.1f}°")

            # Measured Hilbert peaks (live) — search echo region, convert to mm
            if _state.latest_rf is not None and _state.phase in ("compress", "hold", "decompress"):
                meas_pts = []
                for frm in _state.latest_rf:
                    seg = frm[EXPECTED_RF_SAMPLES // 2:]
                    env = np.abs(hilbert(seg.astype(float)))
                    peak_sample = EXPECTED_RF_SAMPLES // 2 + int(np.argmax(env))
                    meas_pts.append(peak_sample * WAVEGUIDE_SAMPLE_PITCH_MM)
                self._meas_scatter.setData(x=WG_MM, y=meas_pts)

            # Phase label
            PH_CLR = {
                "compress":   "#ffcc44", "hold":   "#44ff99",
                "decompress": "#ff9944", "idle":   "#7070a0",
                "done":       "#7070a0",
            }
            self._phase_txt.setColor(PH_CLR.get(_state.phase, "#d4d4f0"))
            self._phase_txt.setText(_state.phase.upper())
            self.setWindowTitle(
                f"Live Orientation Experiment — {_state.phase} "
                f"— angle {_state.angle_deg:+.1f}°")

            # EEF metrics
            ee  = _state.latest_ee
            cmd = _state.latest_cmd
            if ee:
                pos = ee.get("position", [float("nan")] * 3)
                rot = ee.get("orientation_euler_deg_xyz", [float("nan")] * 3)
                frc = ee.get("force_xyz", [float("nan")] * 3)
                vel = ee.get("velocity_xyz") or [float("nan")] * 3
                cpos = cmd.get("position", [float("nan")] * 3) if cmd else [float("nan")] * 3
                crot = cmd.get("orientation_euler_deg_xyz", [float("nan")] * 3) if cmd else [float("nan")] * 3

                def _f(v):  return f"{v:+.2f}" if v == v else "  n/a"
                def _fv(v): return f"{v:+.5f}" if v == v else "   n/a  "
                def _ferr(m, c): return f"{(m-c)*1000:+.2f}" if (m == m and c == c) else "  n/a"
                def _rerr(m, c): return f"{m-c:+.3f}" if (m == m and c == c) else " n/a"

                self._metrics.setText(
                    "<pre style='margin:0;padding:0'>"
                    f"angle {_state.angle_deg:+.1f}°  [{_state.phase}]"
                    f"  RF: {_state.rf_frame_count}\n\n"
                    f"     <span style='color:#ff4444'>meas</span>        cmd       err\n"
                    f" x  {pos[0]:+.4f}  {cpos[0]:+.4f}  {_ferr(pos[0],cpos[0])} mm\n"
                    f" y  {pos[1]:+.4f}  {cpos[1]:+.4f}  {_ferr(pos[1],cpos[1])} mm\n"
                    f" z  {pos[2]:+.4f}  {cpos[2]:+.4f}  {_ferr(pos[2],cpos[2])} mm\n\n"
                    f"rx  {rot[0]:+.2f}°  {crot[0]:+.2f}°  {_rerr(rot[0],crot[0])}°\n"
                    f"ry  {rot[1]:+.2f}°  {crot[1]:+.2f}°  {_rerr(rot[1],crot[1])}°\n"
                    f"rz  {rot[2]:+.2f}°  {crot[2]:+.2f}°  {_rerr(rot[2],crot[2])}°\n\n"
                    f"Fx {_f(frc[0])} N   vx {_fv(vel[0])} m/s\n"
                    f"Fy {_f(frc[1])} N   vy {_fv(vel[1])} m/s\n"
                    f"Fz {_f(frc[2])} N   vz {_fv(vel[2])} m/s"
                    "</pre>"
                )

            if _state.done and not self._done_shown:
                self._done_shown = True
                QtWidgets.QMessageBox.information(
                    self, "Done",
                    f"Experiment finished.\n\n{_state.status}")

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
