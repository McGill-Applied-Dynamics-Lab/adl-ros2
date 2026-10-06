"""Try to follow a "figure eight" target on the xy plane, elongated along y and centered on the start pose."""

# %%
import matplotlib.pyplot as plt
import numpy as np
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
# Parameters for the figure eight (2:1 Lissajous; the slow axis is the long one)
amplitude_y = 0.2  # [m] half-length of the eight along y
amplitude_x = 0.08  # [m] half-width of each lobe along x
ctrl_freq = 500.0
sin_freq_y = 0.125  # rot / s
sin_freq_x = 0.25  # rot / s
max_time = 8.0

# %%
# robot.controller_switcher_client.switch_controller("cartesian_impedance_controller")
# robot.cartesian_controller_parameters_client.load_param_config(
#     # file_path="config/control/gravity_compensation.yaml"
#     # file_path="config/control/default_operational_space_controller.yaml"
#     # file_path="config/control/clipped_cartesian_impedance.yaml"
#     file_path=CONFIG_DIR / "controllers" / "default_cartesian_impedance.yaml"
# )

# robot.controller_switcher_client.switch_controller("osc_pd_controller")
# robot.osc_pd_controller_parameters_client.load_param_config(
#     file_path=CONFIG_DIR / "controllers" / "osc_pd" / "default.yaml"
# )

robot.controller_switcher_client.switch_controller("osc_controller")
robot.osc_controller_parameters_client.load_param_config(file_path=CONFIG_DIR / "controllers" / "osc" / "default.yaml")


# %%
# Center the figure eight on the current end-effector position (the trajectory starts at the center)
center = robot.end_effector_pose.position.copy()
print(f"Figure eight center: {center}")

# %%
# The set_target will directly publish the pose to /target_pose
ee_poses = []
target_poses = []
ts = []

print("Starting to draw a figure eight...")
t = 0.0
target_pose = robot.end_effector_pose.copy()
rate = robot.node.create_rate(ctrl_freq)

while t < max_time:
    x = amplitude_x * np.sin(2 * np.pi * sin_freq_x * t) + center[0]
    y = amplitude_y * np.sin(2 * np.pi * sin_freq_y * t) + center[1]
    z = center[2]
    target_pose.position = np.array([x, y, z])

    robot.set_target(pose=target_pose)

    rate.sleep()

    ee_poses.append(robot.end_effector_pose.copy())
    target_poses.append(robot._target_pose.copy())
    ts.append(t)

    t += 1.0 / ctrl_freq

while t < max_time + 1.0:
    # Just wait a bit for the end effector to settle

    rate.sleep()

    ee_poses.append(robot.end_effector_pose.copy())
    target_poses.append(robot._target_pose.copy())
    ts.append(t)

    t += 1.0 / ctrl_freq


print("Done drawing a figure eight!")


# %%
x_t = [target_pose_sample.position[0] for target_pose_sample in target_poses]
y_t = [target_pose_sample.position[1] for target_pose_sample in target_poses]

# %%
# === Normal params ===
x_ee = [ee_pose.position[0] for ee_pose in ee_poses]
y_ee = [ee_pose.position[1] for ee_pose in ee_poses]

# %%
fig, ax = plt.subplots(1, 2, figsize=(10, 5))
ax[0].plot(y_ee, x_ee, label="current")
ax[0].plot(y_t, x_t, label="target", linestyle="--")
ax[0].set_xlabel("$y$")
ax[0].set_ylabel("$x$")
ax[0].set_aspect("equal")
# ax[0].legend()
ax[1].plot(ts, y_ee, label="current")
ax[1].plot(ts, y_t, label="target", linestyle="--")
ax[1].set_xlabel("$t$")
ax[1].legend()

for a in ax:
    a.grid()

fig.tight_layout()

plt.show()

# %%

print("Going back home.")
robot.home()

# %%
robot.shutdown()
