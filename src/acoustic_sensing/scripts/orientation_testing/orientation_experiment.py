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
    HOLD_AT_CONTACT_SEC,
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
# Results helpers
# ---------------------------------------------------------------------------

def _alloc_trial_dir(results_dir: Path) -> Path:
    """Return the next available trial_NN path without creating it yet."""
    counter = 0
    trial_dir = results_dir / f"trial_{counter:02d}"
    while trial_dir.exists():
        counter += 1
        trial_dir = results_dir / f"trial_{counter:02d}"
    return trial_dir


def _next_result_path(trial_dir: Path, depth_mm: float, angle_deg: float) -> Path:
    counter = 0
    path = trial_dir / f"depth_{depth_mm:.1f}mm_angle_{angle_deg:.1f}deg_{counter:02d}.pkl"
    while path.exists():
        counter += 1
        path = trial_dir / f"depth_{depth_mm:.1f}mm_angle_{angle_deg:.1f}deg_{counter:02d}.pkl"
    return path


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
            print("\n─── New trial ───────────────────────────────────────────────")

            # 1. Descend to contact, then measure actual EEF pose
            print("  Descending to contact...")
            _move(robot, contact_pose, APPROACH_SPEED_M_S, settle=0.3)
            measured_contact = robot.end_effector_pose.copy()
            measured_compressed = Pose(
                measured_contact.position + np.array([0.0, 0.0, -COMPRESS_DEPTH_M]),
                BASE_ORI,
            )
            print(f"  Contact actual: {measured_contact.position.tolist()}")

            # 2. Start RF streaming (send 67)
            frames: list = []
            stop_event: threading.Event | None = None
            thread: threading.Thread | None = None
            t0 = time.perf_counter()
            if dry_run:
                print("  DRY RUN — skipping 67, no RF frames will be recorded.")
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

            # 3. Compress (smooth Cartesian trajectory from measured contact)
            print(f"  Compressing {depth_mm:.1f} mm into target...")
            _cartesian_move(robot, measured_contact, measured_compressed, APPROACH_SPEED_M_S)

            # 4. Hold
            print(f"  Holding for {HOLD_AT_CONTACT_SEC:.1f} s...")
            time.sleep(HOLD_AT_CONTACT_SEC)

            # 5. Lift back to contact (smooth Cartesian trajectory from measured compressed)
            print("  Lifting to contact level...")
            _cartesian_move(robot, measured_compressed, measured_contact, APPROACH_SPEED_M_S, settle=0.3)

            # 6. Stop RF streaming (send 69)
            if dry_run:
                print("  DRY RUN — skipping 69, no RF frames recorded.")
            else:
                rf_stream_stop(ser, stop_event, thread)
                active_stop_event = None
                active_thread = None
                print(f"  Sending 69 → RF streaming stopped. {len(frames)} frames collected.")

            # 7. Return to approach pose
            print("  Returning to approach pose...")
            _move(robot, approach_pose, RETURN_SPEED_M_S, settle=0.5)

            # 8. Prompt for angle
            angle_str = input("\n  Enter bar angle for this trial [degrees]: ").strip()
            angle_deg = float(angle_str)

            # 9. Save (create trial dir on first save)
            if trial_dir is None:
                trial_dir = _alloc_trial_dir(RESULTS_DIR)
                trial_dir.mkdir(parents=True, exist_ok=False)
                print(f"  Session folder: {trial_dir}")
            out_path = _next_result_path(trial_dir, depth_mm, angle_deg)
            payload = {
                "frames": list(frames),
                "t0": t0,
                "angle_deg": angle_deg,
                "compress_depth_m": COMPRESS_DEPTH_M,
                "hold_at_contact_sec": HOLD_AT_CONTACT_SEC,
                "contact_position": measured_contact.position.tolist(),
                "approach_position": approach_pose.position.tolist(),
                "compressed_position": measured_compressed.position.tolist(),
                "waveguide_lateral_positions_mm": WAVEGUIDE_LATERAL_POSITIONS_MM.tolist(),
                "waveguide_sample_pitch_mm": WAVEGUIDE_SAMPLE_PITCH_MM,
                "base_ori_euler_deg": BASE_ORI_EULER_DEG,
            }
            with open(out_path, "wb") as f:
                pickle.dump(payload, f)
            print(f"  Saved: {out_path}")

            input("  Press Enter for next trial (Ctrl+C to stop)...")

    finally:
        if not dry_run and ser is not None:
            if active_stop_event is not None and active_thread is not None:
                rf_stream_stop(ser, active_stop_event, active_thread)
            ser.close()
        robot.shutdown()


if __name__ == "__main__":
    main()
