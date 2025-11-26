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
print(robot.end_effector_wrench)  # added to print initial wrench

robot.controller_switcher_client.switch_controller("osc_pd_controller")
robot.osc_pd_controller_parameters_client.load_param_config(
    file_path=CONFIG_DIR / "controllers" / "osc_pd" / "probe_controller.yaml"
)


print("Moving to start position...")
start_position = np.array([0.615, -0.0714, 0.331 + 0.05])  # 0.331
start_position = np.array([0.550, -0.0714, 0.35])  # button location

# inc = np.array([-0.05,-0.05,0])
# start_position = start_position + inc
# robot.move_to(position=start_position, speed=0.05)
# time.sleep(1.0)


"""
target_pose = robot.end_effector_pose.copy()
target_pose.orientation = R.from_euler("xyz", [-180, 0, 0], degrees=True)  # roll, pitch, yaw
robot.set_target(pose=target_pose)
time.sleep(1.0)
"""

"""
target_pose = robot.end_effector_pose.copy()
for step in np.linspace(-180, -270, 60):
    target_pose.orientation = R.from_euler("xyz", [step, 0, 0], degrees=True)  # roll, pitch, yaw
    # robot.set_target(pose=target_pose)
    time.sleep(0.500)
"""

target_pose = robot.end_effector_pose.copy()
target_pose.orientation = R.from_euler("xyz", [-270, 0, 0], degrees=True)  # roll, pitch, yaw
robot.move_to(pose=target_pose, time_to_move=10)
time.sleep(1.0)

robot.shutdown()
