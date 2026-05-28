"""
Feedforward wrench injection benchmark.

Three phases, all with the position target fixed at home:

  Phase 1 — Baseline (no feedforward):
    Robot holds home pose. Logs position and wrench to establish noise floor.

  Phase 2 — Injection (constant +z force):
    A constant feedforward force is injected. At equilibrium the EE should
    shift by δz = F_ff / K_z (≈ 5 mm for F=5 N, K=1000 N/m).
    Validates sign convention, magnitude, and orientation stability.

  Phase 3 — Timeout:
    Publishing is stopped (robot._target_wrench = None). The controller
    must zero ff_force_ within feedforward.timeout_ticks (default 100 ms).
    Validates the stale-feedforward safety mechanism.

Run with rim_controller.yaml (feedforward.enabled = true).
Results saved to results/feedforward_injection_<timestamp>.npz.
"""

from __future__ import annotations

import sys
import time

import numpy as np

sys.path.insert(0, str(__import__("pathlib").Path(__file__).parent))
from _utils import OSC_RIM_CONFIG, collect, print_summary, save, setup_robot

FF_FORCE_N = -20.0  # feedforward force magnitude along z (N)
BASELINE_S = 2.0
INJECT_S = 5.0
TIMEOUT_S = 4.0  # long enough to see full recovery after ff zeroes (~100ms timeout + settling)
RATE_HZ = 200.0


def main() -> None:
    robot, home_pose = setup_robot(OSC_RIM_CONFIG)
    ref_ori = home_pose.orientation
    robot.set_target(pose=home_pose)
    time.sleep(1.0)

    # --- Phase 1: Baseline ---
    print("\n[Phase 1] Baseline — no feedforward")
    robot.set_target_wrench(force=[0.0, 0.0, 0.0])
    baseline = collect(robot, ref_ori, BASELINE_S, RATE_HZ)
    baseline_z = baseline["position"][:, 2].mean()
    print_summary("Baseline", baseline, home_pose.position)

    # --- Phase 2: Injection ---
    print(f"\n[Phase 2] Injecting +z feedforward: {FF_FORCE_N} N")
    robot.set_target_wrench(force=[0.0, 0.0, FF_FORCE_N])
    inject = collect(robot, ref_ori, INJECT_S, RATE_HZ)

    ss_z = inject["position"][-int(RATE_HZ * 0.5) :, 2].mean()
    delta_z = ss_z - baseline_z
    k_estimated = FF_FORCE_N / delta_z if abs(delta_z) > 1e-4 else float("inf")
    print(f"  Steady-state δz: {delta_z * 1e3:.2f} mm  (expected ≈ {FF_FORCE_N / 1000 * 1e3:.1f} mm for K=1000)")
    print(f"  Estimated K_z from δz: {k_estimated:.0f} N/m")
    print_summary("Injection", inject, home_pose.position)

    # --- Phase 3: Timeout ---
    print(f"\n[Phase 3] Stopping feedforward — robot should recover within timeout_ticks")
    # Set _target_wrench to None so robot.py stops publishing entirely.
    # This mimics a process crash and lets the controller's stale counter trigger.
    robot._target_wrench = None  # noqa: SLF001  (intentional — tests the safety timeout)
    timeout_data = collect(robot, ref_ori, TIMEOUT_S, RATE_HZ)

    # Expect position to converge back toward baseline_z
    recovery_z = timeout_data["position"][-int(RATE_HZ * 0.1) :, 2].mean()
    print(f"  Position after timeout: z={recovery_z * 1e3:.1f} mm  (baseline: {baseline_z * 1e3:.1f} mm)")
    residual = abs(recovery_z - baseline_z)
    print(f"  Residual offset: {residual * 1e3:.2f} mm  {'✓ OK' if residual < 0.003 else '⚠ large'}")

    save(
        {
            "baseline": baseline,
            "inject": inject,
            "timeout": timeout_data,
            "ff_force_n": np.array(FF_FORCE_N),
            "home_position": home_pose.position.copy(),
            "delta_z_m": np.array(delta_z),
            "k_estimated": np.array(k_estimated),
        },
        "feedforward_injection",
    )

    # Restore zero wrench and shut down cleanly
    robot.set_target_wrench(force=[0.0, 0.0, 0.0])
    robot.set_target(pose=home_pose)
    time.sleep(1.0)
    robot.shutdown()


if __name__ == "__main__":
    main()
