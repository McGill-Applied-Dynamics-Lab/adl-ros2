# %%
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as R

import time

from arm_client.robot import Robot
from arm_client import CONFIG_DIR

robot = Robot(namespace="fr3")
robot.wait_until_ready()

# %%
print(robot.end_effector_pose)
print(robot.joint_values)
print(robot.end_effector_wrench)  # added to print initial wrench

# # %%
# print("Going to home position...")
# robot.home()
# homing_pose = robot.end_effector_pose.copy()


# %%
# Parameters for the trajectory
start_position = np.array([0.40, 0.0, 0.30])
traj_freq = 50.0
sin_freq_x = 0.10  # rot / s
# sin_freq_z = 0.125  # rot / s
amplitude = 0.025  # [m]
max_time = 10.0

# %%
# robot.controller_switcher_client.switch_controller("cartesian_impedance_controller")
# robot.cartesian_controller_parameters_client.load_param_config(
#     # file_path="config/control/gravity_compensation.yaml"
#     # file_path="config/control/default_operational_space_controller.yaml"
#     # file_path="config/control/clipped_cartesian_impedance.yaml"
#     file_path=CONFIG_DIR / "controllers" / "default_cartesian_impedance.yaml"
# )

robot.controller_switcher_client.switch_controller("osc_pd_controller")
robot.osc_pd_controller_parameters_client.load_param_config(
    file_path=CONFIG_DIR / "controllers" / "osc_pd" / "probe_controller.yaml"
)


# %%
# The move_to function will publish a pose to /target_pose while interpolation linearly
print("Moving to start position...")
robot.move_to(position=start_position, speed=0.15)

# %%
# The set_target will directly publish the pose to /target_pose
ee_forces = []  # added to record forces
ee_poses = []
target_poses = []
ts = []

time.sleep(1.0)  # Wait a moment to ensure everything is settled
print("Starting trajectory...")
target_pose = robot.end_effector_pose.copy()
rate = robot.node.create_rate(traj_freq)

omega = 2.0 * np.pi * sin_freq_x

# Timing variables
start_time = time.perf_counter()
dt = 1.0 / traj_freq
count = 0
t = 0.0
slow_loops_count = 0

start_ori = target_pose.orientation

while t < max_time:
    loop_start_time = time.perf_counter()

    # Use deterministic time based on count, not wall clock time
    t = loop_start_time - start_time

    x = start_position[0]
    y = start_position[1]
    z = start_position[2] + amplitude * np.sin(omega * t)
    target_pose.position = np.array([x, y, z])

    # update orientation
    """
    yaw_amp_deg = 25.0
    yaw_omega = 2.0 * np.pi * 0.2  # 0.2 Hz yaw wiggle
    yaw_t = np.deg2rad(yaw_amp_deg) * np.sin(yaw_omega * t)
    Rz = R.from_euler("y", yaw_t)
    target_pose.orientation = Rz * start_ori
    """
    target_pose.orientation = R.from_euler("xyz", [-180, 0, 0], degrees=True)  # roll, pitch, yaw

    robot.set_target(pose=target_pose)

    data_start = time.perf_counter()
    ee_poses.append(robot.end_effector_pose.copy())
    ee_forces.append(robot.end_effector_wrench["force"].copy())
    target_poses.append(target_pose.copy())
    ts.append(t)

    # Print detailed timing every 1 second
    if count % int(traj_freq / 10) == 0:
        elapsed_time = time.perf_counter() - start_time
        expected_time = count * dt
        total_loop_time = time.perf_counter() - loop_start_time
        print(
            f"Count: {count}, Expected: {expected_time:.3f}s, Actual: {elapsed_time:.3f}s, Error: {elapsed_time - expected_time:.3f}s"
        )
        # Print forces
        print(f"  Current 'z' Force: {ee_forces[-1]}")

    count += 1

    # Calculate precise sleep time to maintain exact frequency
    loop_execution_time = time.perf_counter() - loop_start_time
    sleep_time = max(0, dt - loop_execution_time)

    # Performance monitoring: if we're consistently too slow, warn and adapt
    if loop_execution_time > dt:
        slow_loops_count += 1
        if slow_loops_count <= 5:  # Show first 5 slow loops in detail
            print(f"⚠️  Slow loop {slow_loops_count}: {loop_execution_time * 1000:.2f}ms vs target {dt * 1000:.2f}ms")

    if sleep_time > 0:
        time.sleep(sleep_time)

stop_time = time.perf_counter()
print("Trajectory finished.")
print(f"Trajectory time: {stop_time - start_time:.4f} seconds")
print(f"Total points collected: {len(ts)}")

# Plot the trajectory results
if len(ts) > 0:
    ts_array = np.array(ts)
    target_x = [pose.position[2] for pose in target_poses]
    actual_x = [pose.position[2] for pose in ee_poses]
    actual_forces_z = [force[2] for force in ee_forces]

    plt.figure(figsize=(12, 8))

    # Plot X position vs time
    plt.subplot(2, 2, 1)
    plt.plot(ts_array, target_x, "b-", label="Target X", linewidth=2)
    plt.plot(ts_array, actual_x, "r--", label="Actual X", linewidth=1)
    plt.xlabel("Time (s)")
    plt.ylabel("X Position (m)")
    plt.title("X Position Trajectory")
    plt.legend()
    plt.grid(True)

    # Plot tracking error
    plt.subplot(2, 2, 2)
    error_x = np.array(actual_x) - np.array(target_x)
    plt.plot(ts_array, error_x * 1000, "g-", linewidth=1)  # Convert to mm
    plt.xlabel("Time (s)")
    plt.ylabel("Tracking Error (mm)")
    plt.title("X Position Tracking Error")
    plt.grid(True)

    # Plot timing analysis
    """
    plt.subplot(2, 2, 3)
    expected_times = np.linspace(0, max_time, len(ts_array))
    timing_error = ts_array - expected_times
    plt.plot(expected_times, timing_error * 1000, "purple", linewidth=1)  # Convert to ms
    plt.xlabel("Expected Time (s)")
    plt.ylabel("Timing Error (ms)")
    plt.title("Trajectory Timing Consistency")
    plt.grid(True)
    """
    # Plot Z Force
    plt.subplot(2, 2, 3)
    plt.plot(ts_array, actual_forces_z, "m-", linewidth=1)
    plt.xlabel("Time (s)")
    plt.ylabel("Z Force (N)")
    plt.title("End-Effector Z Force")
    plt.grid(True)

    # Plot frequency analysis
    plt.subplot(2, 2, 4)
    if len(ts_array) > 1:
        dt_actual = np.diff(ts_array)
        freq_actual = 1.0 / dt_actual
        plt.plot(ts_array[1:], freq_actual, "orange", linewidth=1)
        plt.axhline(y=traj_freq, color="red", linestyle="--", label=f"Target: {traj_freq} Hz")
        plt.xlabel("Time (s)")
        plt.ylabel("Actual Frequency (Hz)")
        plt.title("Trajectory Execution Frequency")
        plt.legend()
        plt.grid(True)

    plt.tight_layout()
    plt.show()

    # Print statistics
    print(f"\nTrajectory Statistics:")
    print(f"  Mean tracking error: {np.mean(np.abs(error_x)) * 1000:.2f} mm")
    print(f"  Max tracking error: {np.max(np.abs(error_x)) * 1000:.2f} mm")
    if len(ts_array) > 1:
        print(f"  Mean execution frequency: {np.mean(freq_actual):.2f} Hz")
        print(f"  Frequency std dev: {np.std(freq_actual):.2f} Hz")

robot.shutdown()
