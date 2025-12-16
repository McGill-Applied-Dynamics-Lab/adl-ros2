#!/usr/bin/env python3
"""
Minimal validation script for testing probing functionality.
Tests: (1) plunge at multiple locations, (2) plot saved data.

Based on the working structure of 07_follow_trajectory.py
"""

import time
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from pathlib import Path
import pickle

from arm_client.robot import Robot, Pose, Twist
from arm_client import CONFIG_DIR


def plunge(
    robot: Robot,
    start_xyz: np.ndarray,
    depth: float,
    plunge_time: float = 1.0,
    traj_freq: float = 100.0,
    fixed_ori: R | None = None,
):
    """
    Quarter-sine plunge from start_xyz to final depth.

    Args:
        robot: Robot client (already ready & in Cartesian control).
        start_xyz: np.array([x, y, z_surface]) start point of plunge.
        depth: positive = move down along -Z (final z = z_surface - depth).
        plunge_time: seconds to complete the plunge.
        traj_freq: Frequency of trajectory points per second.
        fixed_ori: Fixed orientation during plunge.

    Returns:
        ts: timestamps
        target_poses: target poses during plunge
        ee_poses: actual end-effector poses
        ee_forces: measured forces
    """
    if fixed_ori is None:
        fixed_ori = robot.end_effector_pose.orientation

    # Compute plunge trajectory
    z0 = float(start_xyz[2])
    zf = z0 - float(depth)  # positive depth goes down
    N = max(1, int(plunge_time * traj_freq))

    waypoints = []
    time_from_start = []

    # Generate quarter-sine trajectory (k = 0..N)
    for k in range(N + 1):
        s = k / N  # 0..1
        z = z0 + (zf - z0) * np.sin(0.5 * np.pi * s)
        t = k * plunge_time / N

        target_position = np.array([start_xyz[0], start_xyz[1], z], dtype=float)
        target_pose = Pose(target_position, fixed_ori)
        twist = Twist(np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0]))

        waypoints.append((target_pose, twist))
        time_from_start.append(t)

    # Execute trajectory
    robot.execute_trajectory(waypoints, time_from_start)

    ee_poses = []
    ts = []
    ee_forces = []

    start_time = time.time()
    while robot.wait_for_trajectory_completion(plunge_time, timeout_margin=0.5):
        ee_force = robot.end_effector_wrench["force"]
        ee_pose = robot.end_effector_pose
        time_stamp = time.time() - start_time

        ee_poses.append(ee_pose.copy())
        ts.append(time_stamp)
        ee_forces.append(ee_force.copy())

    # Store waypoints as target poses for comparison
    target_poses = [wp[0] for wp in waypoints]

    return ts, target_poses, ee_poses, ee_forces


def plot_results(data_file: Path):
    """
    Plot saved probing data.

    Args:
        data_file: Path to pickle file containing experiment data
    """
    # Load data
    with open(data_file, "rb") as f:
        exp_dict = pickle.load(f)

    print(f"\nLoaded data from: {data_file}")
    print(f"Number of probes: {len(exp_dict['ee_forces'])}")

    # Create figure with subplots
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle("Probing Validation Results", fontsize=16, fontweight='bold')

    # Plot 1: Z position over time for all probes
    ax = axes[0, 0]
    for i in range(len(exp_dict['ts'])):
        ts = exp_dict['ts'][i]
        ee_positions = exp_dict['ee_positions'][i]
        # Extract z positions from array of position arrays
        z_positions = [pos[2] for pos in ee_positions]
        ax.plot(ts, z_positions, label=f"Probe {i+1}", alpha=0.7, linewidth=2)
    ax.set_xlabel("Time (s)", fontsize=12)
    ax.set_ylabel("Z Position (m)", fontsize=12)
    ax.set_title("Z Position During Plunges", fontsize=13, fontweight='bold')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Plot 2: Forces over time
    ax = axes[0, 1]
    for i in range(len(exp_dict['ts'])):
        ts = exp_dict['ts'][i]
        forces = exp_dict['ee_forces'][i]
        # Calculate force magnitude from force vectors
        force_magnitude = [np.linalg.norm(f) for f in forces]
        ax.plot(ts, force_magnitude, label=f"Probe {i+1}", alpha=0.7, linewidth=2)
    ax.set_xlabel("Time (s)", fontsize=12)
    ax.set_ylabel("Force Magnitude (N)", fontsize=12)
    ax.set_title("Force Magnitude During Plunges", fontsize=13, fontweight='bold')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Plot 3: Force components for first probe
    ax = axes[1, 0]
    if len(exp_dict['ee_forces']) > 0:
        ts = exp_dict['ts'][0]
        forces = exp_dict['ee_forces'][0]
        # Convert to numpy array for easy indexing
        forces_array = np.array(forces)
        ax.plot(ts, forces_array[:, 0], label="Fx", alpha=0.8, linewidth=2)
        ax.plot(ts, forces_array[:, 1], label="Fy", alpha=0.8, linewidth=2)
        ax.plot(ts, forces_array[:, 2], label="Fz", alpha=0.8, linewidth=2)
        ax.set_xlabel("Time (s)", fontsize=12)
        ax.set_ylabel("Force (N)", fontsize=12)
        ax.set_title("Force Components (First Probe)", fontsize=13, fontweight='bold')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.axhline(y=0, color='k', linestyle='--', alpha=0.3)

    # Plot 4: Probe locations (top view)
    ax = axes[1, 1]
    probe_locations = exp_dict["probe_locations"]
    scatter = ax.scatter(probe_locations[:, 0], probe_locations[:, 1],
               s=150, c=range(len(probe_locations)), cmap='viridis',
               edgecolors='black', linewidth=2, zorder=3)
    for i, loc in enumerate(probe_locations):
        ax.annotate(f"{i+1}", (loc[0], loc[1]),
                   ha='center', va='center', color='white', fontweight='bold', fontsize=10)
    ax.scatter([0], [0], s=200, c='red', marker='x', linewidth=3, label='Home', zorder=4)
    ax.set_xlabel("X offset (m)", fontsize=12)
    ax.set_ylabel("Y offset (m)", fontsize=12)
    ax.set_title("Probe Locations (Relative to Home)", fontsize=13, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    ax.legend()

    plt.tight_layout()

    # Save figure
    plot_file = data_file.parent / f"{data_file.stem}_plot.png"
    plt.savefig(plot_file, dpi=150, bbox_inches='tight')
    print(f"Plot saved to: {plot_file}")

    plt.show(block=False)
    plt.pause(0.1)


# ---------------------------
# Configuration
# ---------------------------

# Define home position (modify as needed for your setup)
HOME_X = 0.6  # (m)
HOME_Y = -0.10  # (m)
HOME_Z = 0.30  # (m)

# Probing parameters
PLUNGE_DEPTH = 0.020  # 20mm plunge depth (m)
PLUNGE_TIME = 1.0     # 1 second plunge duration (s)
TRAJ_FREQ = 10.0      # Trajectory frequency (Hz) - reduced for stability
SETTLE_TIME = 3.0     # Time to wait after moving to position (s)

# Test locations: small radius around home (relative offsets in meters)
test_offsets = [
    [0.0, 0.0],      # center (home)
    [0.02, 0.0],     # +X
    [0.0, 0.02],     # +Y
    [-0.02, 0.0],    # -X
    [0.0, -0.02],    # -Y
]

# Base orientation (modify as needed)
base_ori = R.from_euler("xyz", [-180, 0, 0], degrees=True)

# ---------------------------
# Setup Robot
# ---------------------------
print("\n Initializing robot...")
robot = Robot(namespace="fr3")
robot.wait_until_ready()

# Switch to Cartesian pose controller
robot.controller_switcher_client.switch_controller("fr3_pose_controller")
robot.fr3_pose_controller_parameters_client.load_param_config(
    file_path=CONFIG_DIR / "controllers" / "fr3_pose" / "default.yaml"
)
print("   Robot ready!")

# ---------------------------
# Move to Home
# ---------------------------
print(f"\n2. Moving to home position: [{HOME_X:.3f}, {HOME_Y:.3f}, {HOME_Z:.3f}]")
home_position = np.array([HOME_X, HOME_Y, HOME_Z])
home_pose = Pose(position=home_position, orientation=base_ori)

robot.set_target(pose=home_pose)
print(f"   Waiting {SETTLE_TIME}s for robot to reach target...")
time.sleep(SETTLE_TIME)

current_pos = robot.end_effector_pose.position
print(f"   Current position: [{current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f}]")
error = np.linalg.norm(current_pos - home_position)
print(f"   Position error: {error * 1000:.1f} mm")

# ---------------------------
# Execute Probing Sequence
# ---------------------------
print(f"\n3. Executing probing sequence ({len(test_offsets)} locations)")
input("   Press Enter to start probing validation sequence...")

# Storage for results
ts_list = []
target_poses_list = []
ee_poses_list = []
ee_forces_list = []
probe_locations = []

# Iterate over test locations
for i, offset in enumerate(test_offsets):
    x = HOME_X + offset[0]
    y = HOME_Y + offset[1]
    z = HOME_Z

    print(f"\n=== Probe {i + 1}/{len(test_offsets)} ===")
    print(f"  Offset from home: [{offset[0]:+.3f}, {offset[1]:+.3f}] m")

    # Move to probe location (XY position at surface Z)
    surface_xyz = np.array([x, y, z], dtype=float)
    surface_pose = Pose(position=surface_xyz, orientation=base_ori)

    print(f"  Moving to surface position...")
    robot.set_target(pose=surface_pose)
    time.sleep(SETTLE_TIME)

    current_pos = robot.end_effector_pose.position
    print(f"  Current position: [{current_pos[0]:.4f}, {current_pos[1]:.4f}, {current_pos[2]:.4f}]")

    # Execute plunge
    print(f"  Executing plunge...")
    ts, target_poses, ee_poses, ee_forces = plunge(
        robot=robot,
        start_xyz=surface_xyz,
        depth=PLUNGE_DEPTH,
        plunge_time=PLUNGE_TIME,
        traj_freq=TRAJ_FREQ,
        fixed_ori=base_ori,
    )

    # Store results
    ts_list.append(ts)
    target_poses_list.append(target_poses)
    ee_poses_list.append(ee_poses)
    ee_forces_list.append(ee_forces)
    probe_locations.append(offset)

    if len(ee_forces) > 0:
        max_force = np.max(np.linalg.norm(ee_forces, axis=1))
        print(f"  Max force: {max_force:.3f} N")
    else:
        print(f"  Warning: No data recorded!")

    # Retract to home
    print(f"  Retracting to home...")
    robot.set_target(pose=home_pose)
    time.sleep(SETTLE_TIME)

# ---------------------------
# Return Home & Shutdown
# ---------------------------
print("\n4. Returning to home position...")
robot.set_target(pose=home_pose)
time.sleep(2.0)

robot.shutdown()
print("   Robot shutdown complete.")

# ---------------------------
# Save Results
# ---------------------------
print("\n5. Saving results...")

# Convert to numpy arrays
ts_array = np.array(ts_list, dtype=object)
target_positions_array = np.array(
    [[pose.position for pose in trial] for trial in target_poses_list], dtype=object
)
target_orientations_array = np.array(
    [[pose.orientation.as_quat() for pose in trial] for trial in target_poses_list], dtype=object
)
ee_positions_array = np.array(
    [[pose.position for pose in trial] for trial in ee_poses_list], dtype=object
)
ee_orientations_array = np.array(
    [[pose.orientation.as_quat() for pose in trial] for trial in ee_poses_list], dtype=object
)
ee_forces_array = np.array(ee_forces_list, dtype=object)
probe_locations_array = np.array(probe_locations)

# Create experiment dictionary
exp_dict = {
    "ts": ts_array,
    "target_positions": target_positions_array,
    "target_orientations": target_orientations_array,
    "ee_positions": ee_positions_array,
    "ee_orientations": ee_orientations_array,
    "ee_forces": ee_forces_array,
    "probe_locations": probe_locations_array,
    "config": {
        "home_position": home_position,
        "plunge_depth": PLUNGE_DEPTH,
        "plunge_time": PLUNGE_TIME,
        "traj_freq": TRAJ_FREQ,
    }
}

# Save to pickle file
PROJECT_ROOT = Path(__file__).resolve().parent
results_dir = PROJECT_ROOT / "results"
results_dir.mkdir(parents=True, exist_ok=True)

timestamp = time.strftime("%Y%m%d_%H%M%S")
data_file = results_dir / f"validation_{timestamp}.pkl"

with open(data_file, "wb") as f:
    pickle.dump(exp_dict, f)
print(f"   Results saved to: {data_file}")

# ---------------------------
# Plot Results
# ---------------------------
print("\n6. Generating plots...")
plot_results(data_file)

print("\n" + "=" * 60)
print("Validation Complete!")
print("=" * 60)

input("Press Enter to exit...")

