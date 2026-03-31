#!/usr/bin/env python3
"""
Test script for wrist rotation functionality.

Executes a single twist with fixed parameters to validate joint-space control.
"""

import time
import numpy as np
from arm_client.robot import Robot


def main():
    # Configuration
    TARGET_ANGLE_RAD = np.deg2rad(125) # (rad)
    SPEED_RAD_S = 0.20 # (rad/s)

    print("=" * 60)
    print("WRIST ROTATION TEST - Joint-Space Control")
    print("=" * 60)
    print(f"Target Angle: {np.degrees(TARGET_ANGLE_RAD):.3g} deg.")
    print(f"Speed: {np.degrees(SPEED_RAD_S):.3g} deg./s")
    print("=" * 60)

    # Setup robot
    print("\nInitializing robot...")
    robot = Robot(namespace="fr3")
    robot.wait_until_ready()
    print("Robot ready")

    # Switch controller
    print("Switching to joint_trajectory_controller...")
    robot.controller_switcher_client.switch_controller("joint_trajectory_controller")
    print("Controller switched")

    # Initialize joint 7 to 45 degrees
    print("\nInitializing joint 7 to 45 degrees...")
    q_init = robot.q.copy()
    q_init[6] = np.radians(45.0)  # Set joint 7 to 45 degrees
    robot.joint_trajectory_controller_client.send_joint_config(
        joint_names=robot.config.joint_names,
        joint_config=q_init.tolist(),
        time_to_goal=3.0,
        blocking=True
    )
    print("✓ Joint 7 initialized to 45°")

    # Get starting configuration
    start_q = robot.q.copy()
    start_joint7 = start_q[6]
    print(f"\nStarting joint configuration:")
    print(f"  Joint 7 (wrist): {start_joint7:.3g} rad ({np.degrees(start_joint7):.3g} deg.)")

    # Calculate trajectory duration for one direction
    duration_per_direction = abs(TARGET_ANGLE_RAD / SPEED_RAD_S)

    print(f"\nTrajectory Parameters:")
    print(f"  Duration per direction: {duration_per_direction:.3g} s")
    print(f"  Total duration (forward + back): {2 * duration_per_direction:.3g} s")

    # Get joint names from robot config
    joint_names = robot.config.joint_names

    # ===== FORWARD ROTATION =====
    print("\n" + "=" * 60)
    print("FORWARD ROTATION")
    print("=" * 60)
    target_joint7_forward = start_joint7 + TARGET_ANGLE_RAD
    q_target_forward = start_q.copy()
    q_target_forward[6] = target_joint7_forward

    print(f"  Rotating from {np.degrees(start_joint7):.3g}° to {np.degrees(target_joint7_forward):.3g}°")
    print(f"  Duration: {duration_per_direction:.3g} s\n")

    t0 = time.perf_counter()
    robot.joint_trajectory_controller_client.send_joint_config(
        joint_names=joint_names,
        joint_config=q_target_forward.tolist(),
        time_to_goal=duration_per_direction,
        blocking=False  # Non-blocking so we can record data
    )

    # Record joint angles during forward rotation
    recorded_times_fwd = []
    recorded_angles_fwd = []
    poll_rate = 20.0  # Hz
    poll_period = 1.0 / poll_rate
    max_wait = duration_per_direction + 1.0

    print("  Recording joint angles during forward rotation:")
    while (time.perf_counter() - t0) < max_wait:
        elapsed = time.perf_counter() - t0
        current_q = robot.q.copy()
        current_j7 = current_q[6]

        recorded_times_fwd.append(elapsed)
        recorded_angles_fwd.append(np.degrees(current_j7))

        print(f"    t={elapsed:.3f}s: Joint 7 = {np.degrees(current_j7):.3g} deg.")

        time.sleep(poll_period)

    elapsed_fwd = time.perf_counter() - t0
    print(f"\n  Forward rotation completed in {elapsed_fwd:.3g} s")

    # ===== REVERSE ROTATION =====
    print("\n" + "=" * 60)
    print("REVERSE ROTATION")
    print("=" * 60)
    q_target_reverse = start_q.copy()
    q_target_reverse[6] = start_joint7

    print(f"  Rotating from {np.degrees(target_joint7_forward):.3g}° back to {np.degrees(start_joint7):.3g}°")
    print(f"  Duration: {duration_per_direction:.3g} s\n")

    t0_reverse = time.perf_counter()
    robot.joint_trajectory_controller_client.send_joint_config(
        joint_names=joint_names,
        joint_config=q_target_reverse.tolist(),
        time_to_goal=duration_per_direction,
        blocking=False  # Non-blocking so we can record data
    )

    # Record joint angles during reverse rotation
    recorded_times_rev = []
    recorded_angles_rev = []

    print("  Recording joint angles during reverse rotation:")
    while (time.perf_counter() - t0_reverse) < max_wait:
        elapsed = time.perf_counter() - t0_reverse
        current_q = robot.q.copy()
        current_j7 = current_q[6]

        recorded_times_rev.append(elapsed)
        recorded_angles_rev.append(np.degrees(current_j7))

        print(f"    t={elapsed:.3f}s: Joint 7 = {np.degrees(current_j7):.3g} deg.")

        time.sleep(poll_period)

    elapsed_rev = time.perf_counter() - t0_reverse
    print(f"\n  Reverse rotation completed in {elapsed_rev:.3g} s")

    # Hold final position
    print("\nHolding final position...")
    time.sleep(1.0)

    # Verify final position
    final_q = robot.q.copy()
    final_joint7 = final_q[6]

    print("\n" + "=" * 60)
    print("TEST RESULTS")
    print("=" * 60)
    print(f"Starting Joint 7:    {np.degrees(start_joint7):.2f} deg.")
    print(f"Target Joint 7:      {np.degrees(target_joint7_forward):.2f} deg.")
    print(f"Final Joint 7:       {np.degrees(final_joint7):.2f} deg.")
    print(f"Forward rotation:    {recorded_angles_fwd[-1] - recorded_angles_fwd[0]:.2f} deg.")
    print(f"Reverse rotation:    {recorded_angles_rev[0] - recorded_angles_rev[-1]:.2f} deg.")
    print(f"Return error:        {abs(final_joint7 - start_joint7):.6f} rad ({np.degrees(abs(final_joint7 - start_joint7)):.2f} deg.)")

    if abs(final_joint7 - start_joint7) < 0.05:  # ~3 degrees tolerance
        print("\n✓ TEST PASSED - Rotation and return completed successfully!")
    else:
        print("\n✗ TEST FAILED - Did not return to starting position")

    print("=" * 60)

    robot.shutdown()


if __name__ == "__main__":
    main()
