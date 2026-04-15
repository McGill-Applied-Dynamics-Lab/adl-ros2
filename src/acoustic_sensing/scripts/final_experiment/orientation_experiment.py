#!/usr/bin/env python3
"""Bar orientation experiment: estimate bar angle from 4-waveguide contact distribution.

Hardware setup
--------------
- FR3 robot with finray gripper in 90-degree custom adapter (orientation [180, 0, 0]).
- Horizontal bar mounted to the robot table directly below the gripper.
- Bar is attached to a 3D-printed adapter that lets it be rotated to discrete angles.
- Before running: jog the robot so the sensor is directly above the bar at the
  desired plunge height. The XY position is captured automatically on startup and
  kept fixed for all trials.

Experiment
----------
- For each of the 6 bar orientations (0°, 15°, 30°, 45°, 60°, 75°):
    1. User manually rotates the bar adapter to the target orientation.
    2. Robot plunges straight down by PLUNGE_DEPTH_M.
    3. RF burst is acquired from all 4 waveguides while at contact.
    4. Robot retracts to approach height.
- Contact location per waveguide is estimated from the RF burst (envelope peak).
- Bar angle is estimated by fitting a line through the 4 contact points:
    angle_est = arctan(d_contact / d_lateral)
  where d_contact is the variation in contact location across waveguides and
  d_lateral is the lateral spacing between waveguides.
- Signed angular error (estimated − true) is computed for each orientation.

Output
------
- Pickle file in results/ with RF data, contact locations, and angle estimates.
- Two PNG plots:
    1. True vs. estimated angle (scatter + identity line).
    2. Signed error vs. true angle (bar chart).

Calibration parameters
-----------------------
Adjust WAVEGUIDE_LATERAL_POSITIONS_MM and WAVEGUIDE_SAMPLE_PITCH_MM to match
your sensor.  Incorrect values shift the absolute angle estimates but preserve
relative ordering and qualitative trends.

Notes
-----
- Motion uses fr3_pose_controller (robot.move_to) — no joint-space trajectory planner.
- EE XY position never changes between trials; only Z plunges.
"""

import pickle
import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import serial
from scipy.signal import hilbert
from scipy.spatial.transform import Rotation

from arm_client import CONFIG_DIR
from arm_client.robot import Pose, Robot

# ---------------------------------------------------------------------------
# Experiment parameters – adjust for your setup
# ---------------------------------------------------------------------------

BAR_ORIENTATIONS_DEG: list[float] = [0.0, 15.0, 30.0, 45.0, 60.0, 75.0]
PLUNGE_DEPTH_M = 0.005           # how far to plunge into contact (metres)
APPROACH_SPEED_M_S = 0.01        # speed for plunge and retract moves
SETTLE_AT_CONTACT_SEC = 1.5      # settle time at contact before RF acquisition

CONTROLLER_NAME = "fr3_pose_controller"
PROBING_CONFIG = CONFIG_DIR / "controllers" / "fr3_pose" / "probing.yaml"

# Gripper orientation: 90-degree adapter, face pointing down.
BASE_ORI = Rotation.from_euler("xyz", [180.0, 0.0, 0.0], degrees=True)

# ---------------------------------------------------------------------------
# Sensor geometry calibration
# ---------------------------------------------------------------------------
# Lateral positions of the 4 waveguides across the sensor face (mm).
# Origin is at the sensor centre-line; positive is in the +X sensor direction.
# Update these to match your actual gripper geometry.
WAVEGUIDE_LATERAL_POSITIONS_MM = np.array([-4.5, -1.5, 1.5, 4.5], dtype=float)

# Physical distance along a waveguide represented by one sample index step (mm).
# Calibrate from a known contact position (e.g. plunge onto a point contact and
# measure the physical depth vs. argmax index).  A rough default is fine for
# qualitative comparisons.
WAVEGUIDE_SAMPLE_PITCH_MM = 0.1  # mm per sample index

# ---------------------------------------------------------------------------
# Teensy / RF parameters
# ---------------------------------------------------------------------------

SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 3_000_000
SERIAL_TIMEOUT_SEC = 30
CMD_START = bytes([0x43])
CHANNEL_MARKERS = ("S0", "S1", "S2", "S3")
RF_BURST_TIMEOUT_SEC = 30.0

# ---------------------------------------------------------------------------
# Results directory
# ---------------------------------------------------------------------------

RESULTS_DIR = Path(__file__).resolve().parent / "results"


# ---------------------------------------------------------------------------
# RF acquisition (single burst)
# ---------------------------------------------------------------------------

def acquire_rf_burst(ser: serial.Serial) -> list:
    """Send CMD_START, collect all frames until Teensy sends 0x45, return frames.

    Each frame is a list of 4 channel arrays:
    ``[ch0_samples, ch1_samples, ch2_samples, ch3_samples]``.
    """
    frames: list = []
    current_channel = None
    current_samples: list[int] = []
    current_frame: dict[str, list[int]] = {}
    saw_frame_data = False
    line_buffer = bytearray()

    def _handle_line(raw: bytes) -> None:
        nonlocal current_channel, current_samples, current_frame, saw_frame_data
        line = raw.decode("ascii", errors="ignore").strip()
        if not line:
            return
        if line in CHANNEL_MARKERS:
            saw_frame_data = True
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

    ser.reset_input_buffer()
    ser.write(CMD_START)
    ser.flush()

    original_timeout = ser.timeout
    ser.timeout = 0.05
    deadline = time.monotonic() + RF_BURST_TIMEOUT_SEC
    try:
        while True:
            raw = ser.read(min(max(ser.in_waiting, 1), 4096))
            if raw == b"":
                if time.monotonic() > deadline:
                    raise RuntimeError(
                        f"RF burst timed out after {RF_BURST_TIMEOUT_SEC:.0f} s."
                    )
                continue
            if saw_frame_data:
                finish_idx = raw.find(bytes([0x45]))
                if finish_idx >= 0:
                    line_buffer.extend(raw[:finish_idx])
                    while True:
                        try:
                            idx = line_buffer.index(ord("\n"))
                        except ValueError:
                            break
                        _handle_line(bytes(line_buffer[:idx]))
                        del line_buffer[: idx + 1]
                    if len(current_frame) == len(CHANNEL_MARKERS):
                        frames.append([current_frame[m] for m in CHANNEL_MARKERS])
                    break
            else:
                raw = raw.replace(bytes([0x45]), b"")
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
    finally:
        ser.timeout = original_timeout

    return frames


# ---------------------------------------------------------------------------
# Contact location estimation
# ---------------------------------------------------------------------------

def estimate_contact_locations(rf_frames: list) -> np.ndarray:
    """Return mean envelope-peak sample index per waveguide, averaged over all frames.

    Returns array of shape (4,).  NaN if no frames or channel is empty.
    """
    if not rf_frames:
        return np.full(4, np.nan)

    per_ch: list[list[float]] = [[] for _ in range(4)]
    for frame in rf_frames:
        for ch, samples in enumerate(frame):
            arr = np.array(samples, dtype=float)
            if arr.size == 0:
                continue
            env = np.abs(hilbert(arr))
            per_ch[ch].append(float(np.argmax(env)))

    return np.array([
        float(np.mean(vals)) if vals else np.nan
        for vals in per_ch
    ])


# ---------------------------------------------------------------------------
# Bar angle estimation
# ---------------------------------------------------------------------------

def estimate_bar_angle_deg(
    contact_locs_samples: np.ndarray,
    lateral_positions_mm: np.ndarray = WAVEGUIDE_LATERAL_POSITIONS_MM,
    sample_pitch_mm: float = WAVEGUIDE_SAMPLE_PITCH_MM,
) -> float:
    """Estimate bar orientation angle from contact locations on the 4 waveguides.

    Model: a bar at angle theta creates a linear contact-location gradient across
    the waveguides:
        contact_location_mm(x) = contact_centre_mm + tan(theta) * x
    where x is the lateral waveguide position.

    Args:
        contact_locs_samples: Mean contact location per waveguide [sample index].
        lateral_positions_mm: Lateral waveguide positions [mm].
        sample_pitch_mm: Physical distance per sample index [mm].

    Returns:
        Estimated bar angle in degrees.  NaN if fewer than 2 valid channels.
    """
    valid = np.isfinite(contact_locs_samples)
    if valid.sum() < 2:
        return float("nan")

    contact_locs_mm = contact_locs_samples[valid] * sample_pitch_mm
    x_mm = lateral_positions_mm[valid]
    slope, _ = np.polyfit(x_mm, contact_locs_mm, 1)  # slope [mm_contact / mm_lateral]
    return float(np.degrees(np.arctan(slope)))


# ---------------------------------------------------------------------------
# Controller setup helper
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


def _move(robot: Robot, pose: Pose, speed: float, settle: float = 0.5) -> None:
    robot.move_to(pose=pose, speed=speed)
    time.sleep(settle)


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

def plot_angle_results(
    orientations_deg: list[float],
    estimated_deg: list[float],
    output_path: Path,
) -> None:
    """Two-panel figure: (left) true vs. estimated angle; (right) signed error."""
    true_arr = np.array(orientations_deg)
    est_arr = np.array(estimated_deg, dtype=float)
    error_arr = est_arr - true_arr

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    # Left: scatter + identity line
    valid = np.isfinite(est_arr)
    ax1.plot([true_arr.min(), true_arr.max()],
             [true_arr.min(), true_arr.max()],
             "k--", linewidth=1, label="Ideal")
    ax1.scatter(true_arr[valid], est_arr[valid], s=60, zorder=5, label="Measured")
    ax1.set_xlabel("True bar angle [°]")
    ax1.set_ylabel("Estimated bar angle [°]")
    ax1.set_title("True vs. estimated bar angle")
    ax1.legend()
    ax1.grid(True)

    # Right: signed error bar chart
    x = np.arange(len(orientations_deg))
    ax2.bar(x, error_arr, color=["steelblue" if e >= 0 else "tomato" for e in error_arr])
    ax2.axhline(0, color="k", linewidth=0.8)
    ax2.set_xticks(x)
    ax2.set_xticklabels([f"{a:.0f}°" for a in orientations_deg])
    ax2.set_xlabel("True bar angle")
    ax2.set_ylabel("Error (estimated − true) [°]")
    ax2.set_title("Angle estimation error")
    ax2.grid(True, axis="y")

    # RMSE annotation
    rmse = float(np.sqrt(np.nanmean(error_arr**2)))
    ax2.set_title(f"Angle estimation error  (RMSE = {rmse:.2f}°)")

    fig.tight_layout()
    out = output_path.with_suffix(".png")
    fig.savefig(out, dpi=150)
    print(f"Plot saved: {out}")
    plt.show()


def plot_contact_locations(
    orientations_deg: list[float],
    all_contact_locs: list[np.ndarray],
    output_path: Path,
) -> None:
    """Plot contact locations across waveguides for each bar orientation."""
    n_ori = len(orientations_deg)
    colors = plt.cm.plasma(np.linspace(0.0, 0.9, n_ori))
    fig, ax = plt.subplots(figsize=(8, 5))

    for idx, (ori, locs) in enumerate(zip(orientations_deg, all_contact_locs)):
        valid = np.isfinite(locs)
        ax.plot(
            WAVEGUIDE_LATERAL_POSITIONS_MM[valid],
            locs[valid],
            marker="o",
            color=colors[idx],
            label=f"{ori:.0f}°",
        )

    ax.set_xlabel("Waveguide lateral position [mm]")
    ax.set_ylabel("Contact location [sample index]")
    ax.set_title("Contact locations across waveguides per bar orientation")
    ax.legend(title="Bar angle", bbox_to_anchor=(1.02, 1), loc="upper left")
    ax.grid(True)
    fig.tight_layout()
    out = output_path.parent / (output_path.stem + "_contact_locs.png")
    fig.savefig(out, dpi=150, bbox_inches="tight")
    print(f"Contact location plot saved: {out}")
    plt.show()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)

    robot = Robot(namespace="fr3")
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=SERIAL_TIMEOUT_SEC)

    results: list[dict] = []
    approach_position_saved: list[float] = []

    try:
        robot.wait_until_ready()
        _setup_pose_controller(robot)

        # Capture approach pose (EE directly above bar, XY fixed for all trials)
        approach_pose = robot.end_effector_pose.copy()
        approach_position_saved = approach_pose.position.tolist()
        plunge_pose = Pose(
            approach_pose.position + np.array([0.0, 0.0, -PLUNGE_DEPTH_M]),
            approach_pose.orientation,
        )
        print(f"Approach pose: {approach_pose.position.tolist()}")
        print(f"Plunge depth: {PLUNGE_DEPTH_M * 1000:.1f} mm")
        print(f"Bar orientations: {BAR_ORIENTATIONS_DEG} degrees")
        print(f"Waveguide lateral positions: {WAVEGUIDE_LATERAL_POSITIONS_MM.tolist()} mm")
        print()

        for trial_idx, bar_angle_deg in enumerate(BAR_ORIENTATIONS_DEG):
            print(f"--- Orientation {trial_idx + 1}/{len(BAR_ORIENTATIONS_DEG)}: "
                  f"{bar_angle_deg:.0f}° ---")
            input(f"  Set bar to {bar_angle_deg:.0f}° and press Enter to plunge...")

            # Plunge to contact
            print("  Plunging to contact...")
            _move(robot, plunge_pose, APPROACH_SPEED_M_S, settle=SETTLE_AT_CONTACT_SEC)

            # Acquire RF burst at contact
            print("  Acquiring RF burst...")
            try:
                rf_frames = acquire_rf_burst(ser)
                print(f"  RF burst complete — {len(rf_frames)} frames collected.")
            except RuntimeError as exc:
                print(f"  RF acquisition failed: {exc}  Skipping orientation.")
                _move(robot, approach_pose, APPROACH_SPEED_M_S)
                results.append({
                    "bar_angle_deg": bar_angle_deg,
                    "rf_frames": [],
                    "contact_locs_samples": np.full(4, np.nan),
                    "estimated_angle_deg": float("nan"),
                })
                continue

            # Retract
            print("  Retracting...")
            _move(robot, approach_pose, APPROACH_SPEED_M_S)

            # Estimate contact locations and bar angle
            contact_locs = estimate_contact_locations(rf_frames)
            estimated_angle = estimate_bar_angle_deg(contact_locs)
            error_deg = estimated_angle - bar_angle_deg

            print(f"  Contact locations [samples]: {np.round(contact_locs, 1).tolist()}")
            print(f"  Estimated angle: {estimated_angle:.2f}°  "
                  f"(true: {bar_angle_deg:.1f}°, error: {error_deg:+.2f}°)")

            results.append({
                "bar_angle_deg": bar_angle_deg,
                "rf_frames": rf_frames,
                "contact_locs_samples": contact_locs,
                "estimated_angle_deg": estimated_angle,
                "error_deg": error_deg,
            })

    finally:
        ser.close()
        robot.shutdown()

    # -------------------------------------------------------------------------
    # Summary
    # -------------------------------------------------------------------------
    print("\n=== Summary ===")
    print(f"{'True [°]':>10}  {'Estimated [°]':>14}  {'Error [°]':>10}")
    errors = []
    for r in results:
        est = r.get("estimated_angle_deg", float("nan"))
        err = r.get("error_deg", float("nan"))
        print(f"{r['bar_angle_deg']:10.1f}  {est:14.2f}  {err:+10.2f}")
        if np.isfinite(err):
            errors.append(err)
    if errors:
        rmse = float(np.sqrt(np.mean(np.array(errors) ** 2)))
        print(f"\nRMSE: {rmse:.2f}°")
        print(f"Max absolute error: {max(abs(e) for e in errors):.2f}°")

    # -------------------------------------------------------------------------
    # Save results
    # -------------------------------------------------------------------------
    counter = 0
    out_path = RESULTS_DIR / f"orientation_experiment_{counter:02d}.pkl"
    while out_path.exists():
        counter += 1
        out_path = RESULTS_DIR / f"orientation_experiment_{counter:02d}.pkl"

    exp_dict = {
        "results": results,
        "bar_orientations_deg": BAR_ORIENTATIONS_DEG,
        "plunge_depth_m": PLUNGE_DEPTH_M,
        "waveguide_lateral_positions_mm": WAVEGUIDE_LATERAL_POSITIONS_MM.tolist(),
        "waveguide_sample_pitch_mm": WAVEGUIDE_SAMPLE_PITCH_MM,
        "approach_position": approach_position_saved,
    }
    with open(out_path, "wb") as f:
        pickle.dump(exp_dict, f)
    print(f"\nResults saved: {out_path}")

    # -------------------------------------------------------------------------
    # Plots
    # -------------------------------------------------------------------------
    orientations = [r["bar_angle_deg"] for r in results]
    estimated = [r.get("estimated_angle_deg", float("nan")) for r in results]
    contact_locs_all = [r["contact_locs_samples"] for r in results]

    plot_angle_results(orientations, estimated, out_path)
    plot_contact_locations(orientations, contact_locs_all, out_path)
    print("Done.")


if __name__ == "__main__":
    main()
