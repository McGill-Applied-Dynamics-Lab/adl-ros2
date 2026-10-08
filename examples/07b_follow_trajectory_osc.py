#!/usr/bin/env python3
"""Follow a Cartesian trajectory with osc_controller (example 07 for a controller without a trajectory input).

osc_controller only takes target_pose / target_twist, so the trajectory is interpolated on the
client and streamed at 500 Hz by `Robot.stream_cartesian_traj` (blocking). This script:
1. Switches to osc_controller and loads its default gains
2. Moves to a start position (a one-waypoint streamed trajectory)
3. Follows the same sinusoidal plunge in z as example 07
4. Plots the measured z against the streamed target, and the external force
"""

import matplotlib.pyplot as plt
import numpy as np
from arm_client.robot import Pose, Robot, Twist

from arm_client import CONFIG_DIR

STREAM_HZ = 500.0

robot = Robot(namespace="fr3")
robot.wait_until_ready()

print("=" * 60)
print("Trajectory following with osc_controller (streamed targets)")
print("=" * 60)

# ----------------------------------------------------------------------------------------------------------------------
# --- 1. Switch to osc_controller
# ----------------------------------------------------------------------------------------------------------------------
print("\n1 --- Switching to osc_controller...")
robot.controller_switcher_client.switch_controller("osc_controller")
robot.osc_controller_parameters_client.load_param_config(file_path=CONFIG_DIR / "controllers" / "osc" / "default.yaml")

# ----------------------------------------------------------------------------------------------------------------------
# --- 2. Go to the start position
# ----------------------------------------------------------------------------------------------------------------------
print("\n2 --- Moving to start position: [0.4, 0.0, 0.5]")
start_position = np.array([0.4, 0.0, 0.5])
orientation = robot.end_effector_pose.orientation
# A first waypoint after t = 0 starts the trajectory from the current pose
robot.stream_cartesian_traj([Pose(start_position, orientation)], [3.0], rate_hz=STREAM_HZ)

current_pos = robot.end_effector_pose.position
error = np.linalg.norm(current_pos - start_position)
print(f"  Current position: {np.round(current_pos, 3)}  (error {error * 1000:.1f} mm)")

# ----------------------------------------------------------------------------------------------------------------------
# --- 3. Sinusoidal plunge in z
# ----------------------------------------------------------------------------------------------------------------------
print("\n3 --- Sinusoidal plunge")

#! Same trajectory as example 07: down by 2 x amplitude and back up, starting and ending at rest
amplitude = 0.1  # [m] half the plunge depth
frequency = 0.1  # [Hz] one full down-and-up cycle in 1 / frequency = 10 s
duration = 10.0  # [s]
n_points = 20  # sparse waypoints: stream_cartesian_traj fits a cubic spline through them
phase_offset = np.pi / 2  # Start at the top of the sine: the current position, zero velocity

start_position = robot.end_effector_pose.position.copy()
center_position = start_position.copy()
center_position[2] -= amplitude

waypoints = []
time_from_start = []
for i in range(n_points):
    t = i * duration / (n_points - 1)
    position = center_position.copy()
    position[2] += amplitude * np.sin(2 * np.pi * frequency * t + phase_offset)
    waypoints.append(Pose(position, orientation))  # velocities come from the spline
    time_from_start.append(t)

print(f"  {n_points} waypoints over {duration:g} s, amplitude {amplitude * 1000:.0f} mm, streamed at {STREAM_HZ:g} Hz")
print(f"  Max theoretical velocity: {2 * np.pi * frequency * amplitude * 1000:.1f} mm/s")

#! Stream it, logging the target and the measured state on every tick
ts, z_target, z_actual, forces = [], [], [], []


def log_tick(t: float, target: Pose, twist: Twist) -> None:
    ts.append(t)
    z_target.append(target.position[2])
    z_actual.append(robot.end_effector_pose.position[2])
    forces.append(robot.end_effector_external_wrench["force"].copy())


stats = robot.stream_cartesian_traj(waypoints, time_from_start, rate_hz=STREAM_HZ, on_tick=log_tick)
print(f"  Streamed {stats['ticks']} targets, largest gap {stats['max_gap_s'] * 1000:.1f} ms")

current_pos = robot.end_effector_pose.position
error = np.linalg.norm(current_pos - waypoints[-1].position)
print(f"  Final position error: {error * 1000:.1f} mm")

# ----------------------------------------------------------------------------------------------------------------------
# --- 4. Plot tracking and forces
# ----------------------------------------------------------------------------------------------------------------------
print("\n4 --- Plotting trajectory tracking performance...")
ts, z_target, z_actual, forces = map(np.array, (ts, z_target, z_actual, forces))
tracking_error = z_actual - z_target
print(f"  Mean tracking error: {np.mean(np.abs(tracking_error)) * 1000:.2f} mm")
print(f"  Max tracking error: {np.max(np.abs(tracking_error)) * 1000:.2f} mm")
print(f"  RMS tracking error: {np.sqrt(np.mean(tracking_error**2)) * 1000:.2f} mm")

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
ax1.plot(ts, z_actual, "b-", linewidth=2, label="Actual")
ax1.plot(ts, z_target, "--", color="red", linewidth=2, label="Streamed target")
ax1.plot(time_from_start, [w.position[2] for w in waypoints], "o", color="red", label="Waypoints")
ax1.set_ylabel("Z Position (m)", fontsize=12)
ax1.set_title("Trajectory Tracking: Z Position (osc_controller)", fontsize=14, fontweight="bold")
ax1.grid(True, alpha=0.3)
ax1.legend()

ax2.plot(ts, tracking_error * 1000, "r-", linewidth=2)
ax2.axhline(y=0, color="k", linestyle="--", alpha=0.3)
ax2.set_xlabel("Time (s)", fontsize=12)
ax2.set_ylabel("Tracking Error (mm)", fontsize=12)
ax2.set_title("Z Position Tracking Error", fontsize=14, fontweight="bold")
ax2.grid(True, alpha=0.3)

fig2, ax3 = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
for k, name in enumerate("XYZ"):
    ax3[k].plot(ts, forces[:, k], "b-", linewidth=2)
    ax3[k].set_ylabel(f"Force {name} (N)", fontsize=12)
    ax3[k].grid(True, alpha=0.3)
ax3[2].set_xlabel("Time (s)", fontsize=12)
ax3[0].set_title("End-Effector Forces During Trajectory", fontsize=14, fontweight="bold")

plt.tight_layout()
plt.show(block=False)
plt.pause(0.1)
input("Press Enter to exit...")

robot.shutdown()
