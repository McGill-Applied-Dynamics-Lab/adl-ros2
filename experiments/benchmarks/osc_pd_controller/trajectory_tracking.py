"""
Trajectory tracking benchmark for the OSC PD controller.

Four trajectories characterize tracking accuracy and stability across gain settings:

  1. Circle (x-y):    5 cm radius at 1 rad/s — coupled tracking, path fidelity, orientation stability.
  2. Step Z:          5 cm step — rise time, overshoot, settling time (2 % band), SS error.
  3. Sine Z:          ±3 cm at 0.5 Hz — amplitude attenuation and phase lag (via FFT at cmd frequency).
  4. Oscillation Z:   ±2 cm at 2 Hz — stress-test stability under rapid reversals ("unstable user").

Usage:
    python trajectory_tracking.py [config_name]

config_name defaults to "rim_controller" and resolves to
configs/controllers/osc_pd/<config_name>.yaml. Pass "default" to compare baseline gains.

Results saved to results/trajectory_tracking_<config>_<timestamp>.npz.
To load in a notebook:
    data = np.load("...", allow_pickle=True)
    circle = data["circle"].item()
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np
from arm_client.robot import Pose
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).parent))
from _utils import collect, sample_state, save, setup_robot

# ---------------------------------------------------------------------------
# Trajectory parameters
# ---------------------------------------------------------------------------

CIRCLE_RADIUS_M = 0.05       # 5 cm
CIRCLE_OMEGA_RAD_S = 1.0     # angular velocity → period ≈ 6.28 s
CIRCLE_DURATION_S = 8.0      # ~1.3 full circles (steady-state portion skips the ramp)
CIRCLE_RAMP_S = 1.5          # linear radius ramp-in to avoid a step at t=0

STEP_Z_M = 0.05              # 5 cm upward step
STEP_RECORD_S = 3.0          # record after step command
STEP_SETTLE_S = 2.0          # settle at home before / after step

SINE_AMP_M = 0.03            # ±3 cm
SINE_FREQ_HZ = 0.5           # 0.5 Hz
SINE_DURATION_S = 10.0       # 5 full cycles

OSCL_AMP_M = 0.02            # ±2 cm  ("unstable user")
OSCL_FREQ_HZ = 2.0           # 2 Hz
OSCL_DURATION_S = 5.0        # 10 full cycles

RATE_HZ = 200.0


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _follow_and_record(
    robot,
    ref_ori: Rotation,
    cmd_positions: np.ndarray,  # (N, 3)
    cmd_orientation: Rotation,
    rate_hz: float,
) -> dict:
    """Command each position at rate_hz while recording state. Returns stacked-array dict."""
    dt = 1.0 / rate_hz
    samples: list[dict] = []
    next_tick = time.perf_counter()
    for pos in cmd_positions:
        robot.set_target(pose=Pose(position=pos, orientation=cmd_orientation))
        samples.append(sample_state(robot, ref_ori))
        next_tick += dt
        time.sleep(max(0.0, next_tick - time.perf_counter()))
    return {k: np.array([s[k] for s in samples]) for k in samples[0]}


def _step_metrics(z: np.ndarray, home_z: float, target_z: float, rate_hz: float) -> dict:
    """Compute rise time (10→90%), overshoot %, settling time (2% band), and SS error."""
    total_step = target_z - home_z
    if abs(total_step) < 1e-6:
        return {}
    norm = (z - home_z) / total_step

    # Rise time
    idx10 = int(np.argmax(norm >= 0.10))
    idx90 = int(np.argmax(norm >= 0.90))
    rise_time_s = max(0.0, (idx90 - idx10) / rate_hz)

    # Overshoot
    peak = norm.max() if total_step > 0 else -norm.min()
    overshoot_pct = max(0.0, (peak - 1.0) * 100.0)

    # Settling time: last sample outside ±2 % band
    in_band = np.abs(norm - 1.0) <= 0.02
    settled_idx = len(in_band)
    for i in range(len(in_band) - 1, -1, -1):
        if not in_band[i]:
            settled_idx = i + 1
            break
    settling_time_s = settled_idx / rate_hz

    ss_z = z[-int(rate_hz * 0.5):].mean()
    return {
        "rise_time_s": float(rise_time_s),
        "overshoot_pct": float(overshoot_pct),
        "settling_time_s": float(settling_time_s),
        "ss_error_m": float(abs(ss_z - target_z)),
    }


def _sine_metrics(z_resp: np.ndarray, z_cmd: np.ndarray, freq_hz: float, rate_hz: float) -> dict:
    """Amplitude ratio and phase lag at cmd frequency using DFT."""
    n = len(z_resp)
    freqs = np.fft.rfftfreq(n, d=1.0 / rate_hz)
    idx = int(np.argmin(np.abs(freqs - freq_hz)))

    cmd_fft = np.fft.rfft(z_cmd - z_cmd.mean())
    resp_fft = np.fft.rfft(z_resp - z_resp.mean())

    transfer = resp_fft[idx] / (cmd_fft[idx] + 1e-12)
    return {
        "amp_ratio": float(abs(transfer)),
        "phase_lag_deg": float(-np.angle(transfer, deg=True)),
        "rmse_m": float(np.sqrt(np.mean((z_resp - z_cmd) ** 2))),
    }


# ---------------------------------------------------------------------------
# Trajectory tests
# ---------------------------------------------------------------------------

def run_circle(robot, home_pose, ref_ori: Rotation) -> dict:
    print(f"\n{'=' * 55}")
    print(f"[1] Circle x-y  R={CIRCLE_RADIUS_M * 100:.0f} cm  ω={CIRCLE_OMEGA_RAD_S:.1f} rad/s")

    n = int(CIRCLE_DURATION_S * RATE_HZ)
    t = np.linspace(0.0, CIRCLE_DURATION_S, n, endpoint=False)
    ramp = np.clip(t / CIRCLE_RAMP_S, 0.0, 1.0)
    r_t = ramp * CIRCLE_RADIUS_M

    cx, cy, cz = home_pose.position
    x_cmd = cx + r_t * np.cos(CIRCLE_OMEGA_RAD_S * t)
    y_cmd = cy + r_t * np.sin(CIRCLE_OMEGA_RAD_S * t)
    z_cmd = np.full(n, cz)
    cmd_positions = np.stack([x_cmd, y_cmd, z_cmd], axis=1)

    robot.set_target(pose=home_pose)
    time.sleep(2.0)

    data = _follow_and_record(robot, ref_ori, cmd_positions, ref_ori, RATE_HZ)
    data["cmd_position"] = cmd_positions

    steady = t >= CIRCLE_RAMP_S
    path_err = np.sqrt(
        (data["position"][:, 0] - x_cmd) ** 2 + (data["position"][:, 1] - y_cmd) ** 2
    )
    rmse_xy = float(np.sqrt(np.mean(path_err[steady] ** 2)))
    max_err = float(path_err[steady].max())
    ori_rms_deg = float(np.degrees(np.sqrt(np.mean(data["ori_err_rad"][steady] ** 2))))

    print(f"  Path RMSE (steady-state):  {rmse_xy * 1e3:.2f} mm")
    print(f"  Max path error:            {max_err * 1e3:.2f} mm")
    print(f"  Orientation RMS:           {ori_rms_deg:.3f} deg")
    return data


def run_step_z(robot, home_pose, ref_ori: Rotation) -> dict:
    print(f"\n{'=' * 55}")
    print(f"[2] Step Z  +{STEP_Z_M * 100:.0f} cm")

    robot.set_target(pose=home_pose)
    time.sleep(STEP_SETTLE_S)

    target_pos = home_pose.position.copy()
    target_pos[2] += STEP_Z_M
    robot.set_target(pose=Pose(position=target_pos, orientation=ref_ori))
    data = collect(robot, ref_ori, STEP_RECORD_S, RATE_HZ)
    data["target_position"] = target_pos

    m = _step_metrics(data["position"][:, 2], home_pose.position[2], target_pos[2], RATE_HZ)
    if m:
        print(f"  Rise time:    {m['rise_time_s'] * 1e3:.0f} ms")
        print(f"  Overshoot:    {m['overshoot_pct']:.1f} %")
        print(f"  Settling:     {m['settling_time_s'] * 1e3:.0f} ms  (2 % band)")
        print(f"  SS error:     {m['ss_error_m'] * 1e3:.2f} mm")
        data.update({f"metric_{k}": np.array(v) for k, v in m.items()})

    robot.set_target(pose=home_pose)
    time.sleep(STEP_SETTLE_S)
    return data


def run_sine_z(robot, home_pose, ref_ori: Rotation) -> dict:
    print(f"\n{'=' * 55}")
    print(f"[3] Sine Z  A=±{SINE_AMP_M * 100:.0f} cm  f={SINE_FREQ_HZ} Hz")

    n = int(SINE_DURATION_S * RATE_HZ)
    t = np.linspace(0.0, SINE_DURATION_S, n, endpoint=False)
    cx, cy, cz = home_pose.position
    z_cmd = cz + SINE_AMP_M * np.sin(2.0 * np.pi * SINE_FREQ_HZ * t)
    cmd_positions = np.stack([np.full(n, cx), np.full(n, cy), z_cmd], axis=1)

    robot.set_target(pose=home_pose)
    time.sleep(2.0)

    data = _follow_and_record(robot, ref_ori, cmd_positions, ref_ori, RATE_HZ)
    data["cmd_position"] = cmd_positions

    m = _sine_metrics(data["position"][:, 2], z_cmd, SINE_FREQ_HZ, RATE_HZ)
    print(f"  Amplitude ratio:  {m['amp_ratio']:.3f}  (1.0 = perfect tracking)")
    print(f"  Phase lag:        {m['phase_lag_deg']:.1f} deg")
    print(f"  RMSE:             {m['rmse_m'] * 1e3:.2f} mm")
    data.update({f"metric_{k}": np.array(v) for k, v in m.items()})

    robot.set_target(pose=home_pose)
    time.sleep(2.0)
    return data


def run_oscillation_z(robot, home_pose, ref_ori: Rotation) -> dict:
    print(f"\n{'=' * 55}")
    print(f"[4] Oscillation Z  A=±{OSCL_AMP_M * 100:.0f} cm  f={OSCL_FREQ_HZ} Hz  (stability check)")

    n = int(OSCL_DURATION_S * RATE_HZ)
    t = np.linspace(0.0, OSCL_DURATION_S, n, endpoint=False)
    cx, cy, cz = home_pose.position
    z_cmd = cz + OSCL_AMP_M * np.sin(2.0 * np.pi * OSCL_FREQ_HZ * t)
    cmd_positions = np.stack([np.full(n, cx), np.full(n, cy), z_cmd], axis=1)

    robot.set_target(pose=home_pose)
    time.sleep(2.0)

    data = _follow_and_record(robot, ref_ori, cmd_positions, ref_ori, RATE_HZ)
    data["cmd_position"] = cmd_positions

    z = data["position"][:, 2]
    half = len(z) // 2
    amp_first = float(np.abs(z[:half] - cz).max())
    amp_second = float(np.abs(z[half:] - cz).max())
    growth_ratio = amp_second / (amp_first + 1e-9)
    stable = bool(growth_ratio < 1.2)

    print(f"  Amplitude first half:   {amp_first * 1e3:.2f} mm")
    print(f"  Amplitude second half:  {amp_second * 1e3:.2f} mm")
    print(f"  Growth ratio:           {growth_ratio:.3f}  {'✓ Stable' if stable else '⚠  UNSTABLE'}")
    data["metric_amp_growth_ratio"] = np.array(growth_ratio)
    data["metric_stable"] = np.array(stable)

    robot.set_target(pose=home_pose)
    time.sleep(2.0)
    return data


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    config_name = sys.argv[1] if len(sys.argv) > 1 else "rim_controller"
    config_path = Path(__file__).parents[3] / "configs" / "controllers" / "osc_pd" / f"{config_name}.yaml"
    if not config_path.exists():
        print(f"Config not found: {config_path}")
        sys.exit(1)

    robot, home_pose = setup_robot(config_path)
    ref_ori = home_pose.orientation

    save(
        {
            "home_position": home_pose.position.copy(),
            "config": np.bytes_(config_name),
            "circle": run_circle(robot, home_pose, ref_ori),
            "step_z": run_step_z(robot, home_pose, ref_ori),
            "sine_z": run_sine_z(robot, home_pose, ref_ori),
            "osc_z": run_oscillation_z(robot, home_pose, ref_ori),
        },
        f"trajectory_tracking_{config_name}",
    )

    robot.set_target(pose=home_pose)
    time.sleep(1.0)
    robot.shutdown()


if __name__ == "__main__":
    main()
