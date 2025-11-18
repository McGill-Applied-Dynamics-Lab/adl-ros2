#!/usr/bin/env python3
"""Test the new trajectory interface for fr3_pose_controller.

This script demonstrates:
1. Single-pose mode: Go to a target pose
2. Trajectory mode: Follow a sinusoidal motion in Z axis
"""

import numpy as np
import time
import matplotlib.pyplot as plt

from arm_client.robot import Robot, Pose, Twist
from arm_client import CONFIG_DIR

# Initialize robot
robot = Robot(namespace="fr3")
robot.wait_until_ready()

# robot.home()

print("=" * 60)
print("Testing FR3 Pose Controller Trajectory Interface")
print("=" * 60)

# --- 1. Switch to fr3_pose_controller
print("\n[1/5] Switching to fr3_pose_controller...")
robot.controller_switcher_client.switch_controller("fr3_pose_controller")
time.sleep(1.0)

# # --- 2. Single pose mode - Go to start position
# print("\n[2/5] Single-pose mode")
# print("Moving to start position: [0.4, 0.0, 0.4]")
# start_position = np.array([0.4, 0.0, 0.4])
# current_orientation = robot.end_effector_pose.orientation

# robot.set_target(position=start_position)
# print("  Waiting for robot to reach target...")
# time.sleep(3.0)  # Give time for trajectory to complete

# current_pos = robot.end_effector_pose.position
# print(f"  Current position: [{current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f}]")
# print(f"  Target position:  [{start_position[0]:.3f}, {start_position[1]:.3f}, {start_position[2]:.3f}]")
# error = np.linalg.norm(current_pos - start_position)
# print(f"  Position error: {error * 1000:.1f} mm")

# if error < 0.01:  # 10mm tolerance
#     print("  ✓ Single-pose test PASSED")
# else:
#     print("  ✗ Single-pose test FAILED - position error too large")

# --- 3. Trajectory mode - Sinusoidal motion in Z
print("\n[3/5] Trajectory mode - Sinusoidal plunge")
print("Generating sinusoidal trajectory (2 seconds, 50 waypoints)")

# Generate sinusoidal trajectory starting from negative amplitude (smooth start)
waypoints = []
time_from_start = []
amplitude = 0.2  # 3cm amplitude
frequency = 0.1  # 0.25 Hz (half cycle in 2s)
duration = 10.0  # 2 seconds
n_points = 20  # SPARSE waypoints - let controller smooth between them
phase_offset = np.pi / 2  # Start at 90 degrees to begin at zero crossing
PLOT_TRAJ = False

start_position = robot.end_effector_pose.position.copy()
start_orientation = robot.end_effector_pose.orientation

center_position = start_position.copy()
center_orientation = start_orientation

center_position[2] -= amplitude

# Generate waypoints with sufficient density for smooth motion
# Linear interpolation between waypoints requires enough points to capture the sinusoid
for i in range(n_points):
    t = i * duration / (n_points - 1)

    # Sinusoid starting from current position (zero offset at t=0)
    z_offset = amplitude * np.sin(2 * np.pi * frequency * t + phase_offset)
    z_dot = 2 * np.pi * frequency * amplitude * np.cos(2 * np.pi * frequency * t + phase_offset)

    position = center_position.copy()
    position[2] += z_offset  # Sinusoidal motion in Z

    pose = Pose(position, center_orientation)
    twist = Twist(np.array([0.0, 0.0, z_dot]), np.array([0.0, 0.0, 0.0]))
    waypoints.append((pose, twist))
    time_from_start.append(t)

max_velocity = 2 * np.pi * frequency * amplitude  # derivative of sine

print(f"  Start position: [{start_position[0]:.3f}, {start_position[1]:.3f}, {start_position[2]:.3f}]")
print(f"  Waypoint count: {len(waypoints)}")
print(f"  Duration: {duration}s")
print(f"  Frequency: {frequency} Hz")
print(f"  Amplitude: {amplitude * 1000:.1f} mm")
print(f"  Max theoretical velocity: {max_velocity * 1000:.1f} mm/s")

# **Plot the trajectory**
if PLOT_TRAJ:
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

    # Extract Z positions and velocities from waypoints
    z_positions = [wp[0].position[2] for wp in waypoints]
    z_velocities = [wp[1].linear[2] for wp in waypoints]

    # Plot Z position
    ax1.plot(time_from_start, z_positions, "b-o", linewidth=2, markersize=4, label="Z Position")
    ax1.set_xlabel("Time (s)", fontsize=12)
    ax1.set_ylabel("Z Position (m)", fontsize=12)
    ax1.set_title("Trajectory Z Position vs Time", fontsize=14, fontweight="bold")
    ax1.grid(True, alpha=0.3)
    ax1.legend()

    # Plot Z velocity
    ax2.plot(time_from_start, z_velocities, "r-o", linewidth=2, markersize=4, label="Z Velocity")
    ax2.set_xlabel("Time (s)", fontsize=12)
    ax2.set_ylabel("Z Velocity (m/s)", fontsize=12)
    ax2.set_title("Trajectory Z Velocity vs Time", fontsize=14, fontweight="bold")
    ax2.grid(True, alpha=0.3)
    ax2.legend()

    plt.tight_layout()
    plt.show(block=False)
    plt.pause(0.1)
    input("Press Enter to continue...")


# Execute trajectory
print("  Sending trajectory to controller...")
robot.execute_trajectory(waypoints, time_from_start)

# TODO: Get the points while executing to plot commanded vs actual

print("  Trajectory sent! Waiting for execution to complete...")
robot.wait_for_trajectory_completion(duration, timeout_margin=0.5)

current_pos = robot.end_effector_pose.position
expected_final_pos = waypoints[-1].position
error = np.linalg.norm(current_pos - expected_final_pos)

print(f"  Current position: [{current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f}]")
print(f"  Final waypoint:   [{expected_final_pos[0]:.3f}, {expected_final_pos[1]:.3f}, {expected_final_pos[2]:.3f}]")
print(f"  Position error: {error * 1000:.1f} mm")

if error < 0.01:  # 10mm tolerance
    print("  ✓ Trajectory test PASSED")
else:
    print("  ✗ Trajectory test FAILED - position error too large")

# # --- 4. Return to start using single-pose mode
# print("\n[4/5] Return to start position (single-pose mode)")
# print(f"Moving back to [{start_position[0]:.3f}, {start_position[1]:.3f}, {start_position[2]:.3f}]")
# robot.set_target(position=start_position)
# time.sleep(2.0)

# current_pos = robot.end_effector_pose.position
# error = np.linalg.norm(current_pos - start_position)
# print(f"  Position error: {error * 1000:.1f} mm")

# if error < 0.01:
#     print("  ✓ Return test PASSED")
# else:
#     print("  ✗ Return test FAILED")

# # --- 5. Go home
# print("\n[5/5] Returning to home position...")
# robot.home()
# print("  Done!")

# print("\n" + "=" * 60)
# print("All tests complete!")
# print("=" * 60)

# Shutdown
robot.shutdown()
