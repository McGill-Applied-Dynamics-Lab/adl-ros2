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
from scipy.spatial.transform import Rotation

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


#! Stream it, logging the target and the measured pose on every tick
ts, target_pos, target_quat, actual_pos, actual_quat, forces = [], [], [], [], [], []


def log_tick(t: float, target: Pose, twist: Twist) -> None:
    actual = robot.end_effector_pose
    ts.append(t)
    target_pos.append(target.position.copy())
    target_quat.append(target.orientation.as_quat())
    actual_pos.append(actual.position.copy())
    actual_quat.append(actual.orientation.as_quat())
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
ts, target_pos, actual_pos, forces = map(np.array, (ts, target_pos, actual_pos, forces))
target_rot, actual_rot = Rotation.from_quat(target_quat), Rotation.from_quat(actual_quat)

# Roll-pitch-yaw: rotations about the base x, y, z axes (extrinsic xyz). The tool points down, so
# roll sits near +-180 deg: unwrap the target in time, and express the measurement as target +
# wrapped difference so both stay on the same branch.
target_rpy = np.unwrap(target_rot.as_euler("xyz"), axis=0)
actual_rpy = target_rpy + (actual_rot.as_euler("xyz") - target_rot.as_euler("xyz") + np.pi) % (2 * np.pi) - np.pi
pos_error = (actual_pos - target_pos) * 1e3  # mm
rpy_error = np.degrees(actual_rpy - target_rpy)  # deg
# Orientation error as one angle (independent of the Euler convention)
angle_error = np.degrees((actual_rot * target_rot.inv()).magnitude())

print("  Position error [mm]      " + "  ".join(f"{a}: RMS {np.sqrt(np.mean(e**2)):5.2f} max {np.max(np.abs(e)):5.2f}" for a, e in zip("xyz", pos_error.T)))
print("  Orientation error [deg]  " + "  ".join(f"{a}: RMS {np.sqrt(np.mean(e**2)):5.2f} max {np.max(np.abs(e)):5.2f}" for a, e in zip(("roll", "pitch", "yaw"), rpy_error.T)))
print(f"  Orientation error angle: RMS {np.sqrt(np.mean(angle_error**2)):.2f} deg, max {np.max(angle_error):.2f} deg")


def plot_tracking(title, labels, unit, actual, target, error, error_unit, waypoint_values=None):
    fig, axs = plt.subplots(3, 2, figsize=(14, 9), sharex=True)
    for k, label in enumerate(labels):
        axs[k, 0].plot(ts, actual[:, k], "b-", linewidth=2, label="Actual")
        axs[k, 0].plot(ts, target[:, k], "--", color="red", linewidth=2, label="Target")
        if waypoint_values is not None:
            axs[k, 0].plot(time_from_start, waypoint_values[:, k], "o", color="red", markersize=4, label="Waypoints")
        axs[k, 0].set_ylabel(f"{label} ({unit})", fontsize=12)
        axs[k, 1].plot(ts, error[:, k], "r-", linewidth=1.5)
        axs[k, 1].axhline(y=0, color="k", linestyle="--", alpha=0.3)
        axs[k, 1].set_ylabel(f"{label} error ({error_unit})", fontsize=12)
        for ax in axs[k]:
            ax.grid(True, alpha=0.3)
    axs[0, 0].legend()
    axs[0, 0].set_title(f"{title}: actual vs target", fontsize=13, fontweight="bold")
    axs[0, 1].set_title(f"{title}: error (actual - target)", fontsize=13, fontweight="bold")
    for ax in axs[-1]:
        ax.set_xlabel("Time (s)", fontsize=12)
    fig.tight_layout()
    return fig


waypoint_pos = np.array([w.position for w in waypoints])
waypoint_rpy = np.degrees(np.unwrap(Rotation.from_quat([w.orientation.as_quat() for w in waypoints]).as_euler("xyz"), axis=0))
plot_tracking("Position (osc_controller)", ("x", "y", "z"), "m", actual_pos, target_pos, pos_error, "mm", waypoint_pos)
plot_tracking(
    "Orientation, roll-pitch-yaw (osc_controller)",
    ("roll", "pitch", "yaw"),
    "deg",
    np.degrees(actual_rpy),
    np.degrees(target_rpy),
    rpy_error,
    "deg",
    waypoint_rpy,
)

fig3, ax3 = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
for k, name in enumerate("XYZ"):
    ax3[k].plot(ts, forces[:, k], "b-", linewidth=2)
    ax3[k].set_ylabel(f"Force {name} (N)", fontsize=12)
    ax3[k].grid(True, alpha=0.3)
ax3[2].set_xlabel("Time (s)", fontsize=12)
ax3[0].set_title("End-Effector Forces During Trajectory", fontsize=14, fontweight="bold")
fig3.tight_layout()

plt.show(block=False)
plt.pause(0.1)
input("Press Enter to exit...")

robot.shutdown()
