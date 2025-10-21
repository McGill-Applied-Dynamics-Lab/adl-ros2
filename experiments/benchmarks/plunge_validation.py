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
print(robot.q)

# # %%
# print("Going to home position...")
# robot.home()
# homing_pose = robot.end_effector_pose.copy()


# %%
# Parameters for the trajectory
start_position = np.array([0.55, 0.0, 0.35])
traj_freq = 20.0  # Publish rate of the trajectory
plunge_time = 1.0
depth = 0.05  # [m]
n_cycles = 100
max_time = n_cycles * plunge_time * 4

z0 = start_position[2]
zf = z0 - depth  # Move down by 'depth' meters
sin_period = 1 / (plunge_time * 4)
omega = 2.0 * np.pi / (4 * plunge_time)  # radians per second

# %%
# robot.controller_switcher_client.switch_controller("cartesian_impedance_controller")
# robot.cartesian_controller_parameters_client.load_param_config(
#     # file_path="config/control/gravity_compensation.yaml"
#     # file_path="config/control/default_operational_space_controller.yaml"
#     # file_path="config/control/clipped_cartesian_impedance.yaml"
#     file_path=CONFIG_DIR / "controllers" / "default_cartesian_impedance.yaml"
# )

# robot.controller_switcher_client.switch_controller("haptic_controller")
# robot.haptic_controller_parameters_client.load_param_config(
#     file_path=CONFIG_DIR / "controllers" / "probe_controller.yaml"
# )

# robot.controller_switcher_client.switch_controller("haptic_controller")
# robot.haptic_controller_parameters_client.load_param_config(
#     file_path=CONFIG_DIR / "controllers" / "haptic_controller.yaml"
# )

robot.controller_switcher_client.switch_controller("joint_space_controller")
robot.joint_space_controller_parameters_client.load_param_config(
    file_path=CONFIG_DIR / "controllers" / "joint_space_controller.yaml"
)

# %%
# The move_to function will publish a pose to /target_pose while interpolation linearly
print("Moving to start position...")
robot.move_to(position=start_position, speed=0.15)

robot.set_target(position=start_position)
time.sleep(1.0)  # Wait a moment to ensure everything is settled

# %%
# The set_target will directly publish the pose to /target_pose
ee_poses = []
target_poses = []
q_list = []
tau_list = []
ts = []

target_rot = R.from_euler("xyz", [-180, 0, 0], degrees=True)  # base orientation ([roll, pitch, yaw], degrees)

time.sleep(1.0)  # Wait a moment to ensure everything is settled
print("Starting trajectory...")
target_pose = robot.end_effector_pose.copy()
# rate = robot.node.create_rate(traj_freq)

x = start_position[0]
y = start_position[1]
z = z0 - 0.0
target_pose.position = np.array([x, y, z])
target_pose.orientation = target_rot

robot.set_target(pose=target_pose)


# Timing variables
start_time = time.perf_counter()
dt = 1.0 / traj_freq
count = 0
t = 0.0
slow_loops_count = 0

while t < max_time:
    loop_start_time = time.perf_counter()

    # Use deterministic time based on count, not wall clock time
    t = loop_start_time - start_time

    x = start_position[0]
    y = start_position[1]
    z = z0 - depth * np.sin(omega * t)

    target_pose.position = np.array([x, y, z])
    target_pose.orientation = target_rot

    robot.set_target(pose=target_pose)

    ee_poses.append(robot.end_effector_pose.copy())
    target_poses.append(target_pose.copy())
    ts.append(t)
    q_list.append(robot.q.copy())
    tau_list.append(robot.tau.copy())

    # # Print detailed timing every 2 second
    # if count % int(traj_freq * 2) == 0:
    #     elapsed_time = time.perf_counter() - start_time
    #     expected_time = count * dt
    #     total_loop_time = time.perf_counter() - loop_start_time
    #     print(
    #         f"Count: {count}, Expected: {expected_time:.3f}s, Actual: {elapsed_time:.3f}s, Error: {elapsed_time - expected_time:.3f}s"
    #     )

    count += 1

    # Calculate precise sleep time to maintain exact frequency
    loop_execution_time = time.perf_counter() - loop_start_time
    sleep_time = max(0, dt - loop_execution_time)

    if sleep_time > 0:
        time.sleep(sleep_time)

    # else:
    #     print("Warning: Loop is running behind schedule!")

stop_time = time.perf_counter()
print("Trajectory finished.")
print(f"Trajectory time: {stop_time - start_time:.4f} seconds")
print(f"Total points collected: {len(ts)}")

# Plot the trajectory results
if len(ts) > 0:
    # Extract data for plotting
    ts_array = np.array(ts)
    target_x = [pose.position[0] for pose in target_poses]
    actual_x = [pose.position[0] for pose in ee_poses]

    target_y = [pose.position[1] for pose in target_poses]
    actual_y = [pose.position[1] for pose in ee_poses]

    target_z = [pose.position[2] for pose in target_poses]
    actual_z = [pose.position[2] for pose in ee_poses]

    # rpy angles
    target_r = [pose.orientation.as_euler("xyz", degrees=True)[0] for pose in target_poses]
    actual_r = [pose.orientation.as_euler("xyz", degrees=True)[0] for pose in ee_poses]

    target_p = [pose.orientation.as_euler("xyz", degrees=True)[1] for pose in target_poses]
    actual_p = [pose.orientation.as_euler("xyz", degrees=True)[1] for pose in ee_poses]

    target_yaw = [pose.orientation.as_euler("xyz", degrees=True)[2] for pose in target_poses]
    actual_yaw = [pose.orientation.as_euler("xyz", degrees=True)[2] for pose in ee_poses]

    # Figure 1 - Position (3x1 subplots)
    fig1, axes1 = plt.subplots(3, 1, figsize=(10, 12))

    # X Position
    axes1[0].plot(ts_array, target_x, "b-", label="Target X", linewidth=2)
    axes1[0].plot(ts_array, actual_x, "r--", label="Actual X", linewidth=1)
    axes1[0].set_xlabel("Time (s)")
    axes1[0].set_ylabel("X Position (m)")
    axes1[0].set_title("X Position Trajectory")
    axes1[0].legend()
    axes1[0].grid(True)

    # Y Position
    axes1[1].plot(ts_array, target_y, "b-", label="Target Y", linewidth=2)
    axes1[1].plot(ts_array, actual_y, "r--", label="Actual Y", linewidth=1)
    axes1[1].set_xlabel("Time (s)")
    axes1[1].set_ylabel("Y Position (m)")
    axes1[1].set_title("Y Position Trajectory")
    axes1[1].legend()
    axes1[1].grid(True)

    # Z Position
    axes1[2].plot(ts_array, target_z, "b-", label="Target Z", linewidth=2)
    axes1[2].plot(ts_array, actual_z, "r--", label="Actual Z", linewidth=1)
    axes1[2].set_xlabel("Time (s)")
    axes1[2].set_ylabel("Z Position (m)")
    axes1[2].set_title("Z Position Trajectory")
    axes1[2].legend()
    axes1[2].grid(True)

    plt.tight_layout()

    # Figure 2 - Orientation (3x1 subplots)
    fig2, axes2 = plt.subplots(3, 1, figsize=(10, 12))

    # Roll
    axes2[0].plot(ts_array, target_r, "b-", label="Target Roll", linewidth=2)
    axes2[0].plot(ts_array, actual_r, "r--", label="Actual Roll", linewidth=1)
    axes2[0].set_xlabel("Time (s)")
    axes2[0].set_ylabel("Roll (degrees)")
    axes2[0].set_title("Roll Trajectory")
    axes2[0].legend()
    axes2[0].grid(True)

    # Pitch
    axes2[1].plot(ts_array, target_p, "b-", label="Target Pitch", linewidth=2)
    axes2[1].plot(ts_array, actual_p, "r--", label="Actual Pitch", linewidth=1)
    axes2[1].set_xlabel("Time (s)")
    axes2[1].set_ylabel("Pitch (degrees)")
    axes2[1].set_title("Pitch Trajectory")
    axes2[1].legend()
    axes2[1].grid(True)

    # Yaw
    axes2[2].plot(ts_array, target_yaw, "b-", label="Target Yaw", linewidth=2)
    axes2[2].plot(ts_array, actual_yaw, "r--", label="Actual Yaw", linewidth=1)
    axes2[2].set_xlabel("Time (s)")
    axes2[2].set_ylabel("Yaw (degrees)")
    axes2[2].set_title("Yaw Trajectory")
    axes2[2].legend()
    axes2[2].grid(True)

    plt.tight_layout()

    # Figure 3 - Joint Positions
    fig3, axes3 = plt.subplots(7, 1, figsize=(10, 12))
    for i in range(7):
        joint_i = [q[i] * 180 / np.pi for q in q_list]
        axes3[i].plot(ts_array, joint_i, "g-", label=f"Joint {i + 1}", linewidth=1)
        axes3[i].set_xlabel("Time (s)")
        axes3[i].set_ylabel("Position (deg)")
        axes3[i].set_title(f"Joint {i + 1} Position")
        axes3[i].legend()
        axes3[i].grid(True)

    plt.tight_layout()

    # --- Figure 4 - Joint Torques
    fig4, axes4 = plt.subplots(7, 1, figsize=(10, 12))
    for i in range(7):
        tau_i = [tau[i] for tau in tau_list]
        axes4[i].plot(ts_array, tau_i, "m-", label=f"Joint {i + 1}", linewidth=1)
        axes4[i].set_xlabel("Time (s)")
        axes4[i].set_ylabel("Torque (Nm)")
        axes4[i].set_title(f"Joint {i + 1} Torque")
        axes4[i].legend()
        axes4[i].grid(True)

    plt.tight_layout()

    # Print statistics
    print(f"\nTrajectory Statistics:")

    # Calculate tracking errors for all axes
    error_x = np.array(actual_x) - np.array(target_x)
    error_y = np.array(actual_y) - np.array(target_y)
    error_z = np.array(actual_z) - np.array(target_z)

    # print(f"  Mean X tracking error: {np.mean(np.abs(error_x)) * 1000:.2f} mm")
    # print(f"  Max X tracking error: {np.max(np.abs(error_x)) * 1000:.2f} mm")
    # print(f"  Mean Y tracking error: {np.mean(np.abs(error_y)) * 1000:.2f} mm")
    # print(f"  Max Y tracking error: {np.max(np.abs(error_y)) * 1000:.2f} mm")
    print(f"  Mean Z tracking error: {np.mean(np.abs(error_z)) * 1000:.2f} mm")
    # print(f"  Max Z tracking error: {np.max(np.abs(error_z)) * 1000:.2f} mm")

    # Calculate orientation tracking errors
    error_roll = np.array(actual_r) - np.array(target_r)
    error_pitch = np.array(actual_p) - np.array(target_p)
    error_yaw = np.array(actual_yaw) - np.array(target_yaw)

    # print(f"  Mean Roll tracking error: {np.mean(np.abs(error_roll)):.3f} degrees")
    # print(f"  Max Roll tracking error: {np.max(np.abs(error_roll)):.3f} degrees")
    print(f"  Mean Pitch tracking error: {np.mean(np.abs(error_pitch)):.3f} degrees")
    # print(f"  Max Pitch tracking error: {np.max(np.abs(error_pitch)):.3f} degrees")
    # print(f"  Mean Yaw tracking error: {np.mean(np.abs(error_yaw)):.3f} degrees")
    # print(f"  Max Yaw tracking error: {np.max(np.abs(error_yaw)):.3f} degrees")

    if len(ts_array) > 1:
        dt_actual = np.diff(ts_array)
        freq_actual = 1.0 / dt_actual
        # print(f"  Mean execution frequency: {np.mean(freq_actual):.2f} Hz")
        # print(f"  Frequency std dev: {np.std(freq_actual):.2f} Hz")

    # Show both figures simultaneously
    plt.show()

robot.shutdown()
