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
# ----------------------------------------------------------------------------------------------------------------------
# --- 1. Switch to fr3_pose_controller
# ----------------------------------------------------------------------------------------------------------------------
print("\n1 --- Switching to fr3_pose_controller...")
robot.controller_switcher_client.switch_controller("fr3_pose_controller")
robot.fr3_pose_controller_parameters_client.load_param_config(
    file_path=CONFIG_DIR / "controllers" / "fr3_pose" / "default.yaml"
)
time.sleep(1.0)


# ----------------------------------------------------------------------------------------------------------------------
# --- 2. Single pose mode - Go to start position
# ----------------------------------------------------------------------------------------------------------------------
print("\n2 --- Single-pose mode")
print("Moving to start position: [0.4, 0.0, 0.5]")
start_position = np.array([0.4, 0.0, 0.5])
current_orientation = robot.end_effector_pose.orientation

robot.set_target(position=start_position)
print("  Waiting for robot to reach target...")
time.sleep(3.0)  # Give time for trajectory to complete

current_pos = robot.end_effector_pose.position
print(f"  Current position: [{current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f}]")
print(f"  Target position:  [{start_position[0]:.3f}, {start_position[1]:.3f}, {start_position[2]:.3f}]")
error = np.linalg.norm(current_pos - start_position)
print(f"  Position error: {error * 1000:.1f} mm")

if error < 0.01:  # 10mm tolerance
    print("  ✓ Single-pose test PASSED")
else:
    print("  ✗ Single-pose test FAILED - position error too large")


# ----------------------------------------------------------------------------------------------------------------------
# --- 3. Trajectory mode - Sinusoidal motion in Z
# ----------------------------------------------------------------------------------------------------------------------
print("\n3 --- Trajectory mode - Sinusoidal plunge")
print("Generating sinusoidal trajectory (2 seconds, 50 waypoints)")

#! Generate sinusoidal trajectory starting from negative amplitude (smooth start)
waypoints = []
time_from_start = []
amplitude = 0.1  # 3cm amplitude
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


#! Execute trajectory
print("  Sending trajectory to controller...")
robot.execute_trajectory(waypoints, time_from_start)

ee_poses = []
ts = []
ee_forces = []

print("  Trajectory sent! Waiting for execution to complete...")
start_time = time.time()
while robot.wait_for_trajectory_completion(duration, timeout_margin=0.5):
    ee_force = robot.end_effector_wrench["force"]
    ee_pose = robot.end_effector_pose
    time_stamp = time.time() - start_time

    ee_poses.append(ee_pose)
    ts.append(time_stamp)
    ee_forces.append(ee_force)


current_pos = robot.end_effector_pose.position
expected_final_pos = waypoints[-1][0].position
error = np.linalg.norm(current_pos - expected_final_pos)

print(f"  Current position: [{current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f}]")
print(f"  Final waypoint:   [{expected_final_pos[0]:.3f}, {expected_final_pos[1]:.3f}, {expected_final_pos[2]:.3f}]")
print(f"  Position error: {error * 1000:.1f} mm")

if error < 0.01:  # 10mm tolerance
    print("  ✓ Trajectory test PASSED")
else:
    print("  ✗ Trajectory test FAILED - position error too large")

# ----------------------------------------------------------------------------------------------------------------------
# --- 4. Plot actual vs commanded trajectory
# ----------------------------------------------------------------------------------------------------------------------
print("\n4 --- Plotting trajectory tracking performance...")
z_actual = [pose.position[2] for pose in ee_poses]
z_commanded = [wp[0].position[2] for wp in waypoints]
t_commanded = time_from_start

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

# Plot Z position
ax1.plot(ts, z_actual, "b-", linewidth=2, label="Actual")
ax1.plot(t_commanded, z_commanded, "o--", color="red", linewidth=2, label="Commanded")
ax1.set_xlabel("Time (s)", fontsize=12)
ax1.set_ylabel("Z Position (m)", fontsize=12)
ax1.set_title("Trajectory Tracking: Z Position", fontsize=14, fontweight="bold")
ax1.grid(True, alpha=0.3)
ax1.legend()

# Plot tracking error
if len(ts) > 0:
    # Interpolate commanded trajectory to match actual samples
    z_commanded_interp = np.interp(ts, t_commanded, z_commanded)
    tracking_error = np.array(z_actual) - z_commanded_interp

    ax2.plot(ts, tracking_error * 1000, "r-", linewidth=2)  # Convert to mm
    ax2.set_xlabel("Time (s)", fontsize=12)
    ax2.set_ylabel("Tracking Error (mm)", fontsize=12)
    ax2.set_title("Z Position Tracking Error", fontsize=14, fontweight="bold")
    ax2.grid(True, alpha=0.3)
    ax2.axhline(y=0, color="k", linestyle="--", alpha=0.3)

    # Print statistics
    print(f"  Mean tracking error: {np.mean(np.abs(tracking_error)) * 1000:.2f} mm")
    print(f"  Max tracking error: {np.max(np.abs(tracking_error)) * 1000:.2f} mm")
    print(f"  RMS tracking error: {np.sqrt(np.mean(tracking_error**2)) * 1000:.2f} mm")


# Plot end-effector forces
fig2, ax3 = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
ee_forces = np.array(ee_forces)
ax3[0].plot(ts, ee_forces[:, 0], "b-", linewidth=2)
ax3[0].set_ylabel("Force X (N)", fontsize=12)
ax3[0].grid(True, alpha=0.3)

ax3[1].plot(ts, ee_forces[:, 1], "b-", linewidth=2)
ax3[1].set_ylabel("Force Y (N)", fontsize=12)
ax3[1].grid(True, alpha=0.3)

ax3[2].plot(ts, ee_forces[:, 2], "b-", linewidth=2)
ax3[2].set_ylabel("Force Z (N)", fontsize=12)
ax3[2].set_xlabel("Time (s)", fontsize=12)
ax3[2].grid(True, alpha=0.3)

ax3[0].set_title("End-Effector Forces During Trajectory", fontsize=14, fontweight="bold")

plt.tight_layout()
plt.show(block=False)
plt.pause(0.1)
input("Press Enter to exit...")

# Shutdown
robot.shutdown()
