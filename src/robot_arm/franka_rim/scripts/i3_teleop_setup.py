"""Setup robot for teleop"""

import time
import numpy as np
from scipy.spatial.transform import Rotation as R

# %%
from arm_client import CONFIG_DIR
from arm_client.robot import Robot, Pose

# robot = Robot()
robot = Robot(namespace="fr3")
robot.wait_until_ready(timeout=2.0)


# # --- Home the robot ---
# if not robot.is_homed():
#     robot.home(time_to_home=3.0)

# --- To Switch Controllers ---
controller_name = "osc_pd_controller"  # osc_pd_controller, gravity_compensation, ....
robot.controller_switcher_client.switch_controller(controller_name)
robot.haptic_controller_parameters_client.load_param_config(
    file_path=CONFIG_DIR / "controllers" / "osc_pd" / "default.yaml"
)
# robot.controller_switcher_client.switch_controller("joint_space_controller")
# robot.joint_space_controller_parameters_client.load_param_config(
#     file_path=CONFIG_DIR / "controllers" / "joint_space" / "default.yaml"
# )

# --- Move To ---
target_rot = R.from_euler("xyz", [-180, 0, 0], degrees=True)  # base orientation ([roll, pitch, yaw], degrees)
target_position = np.array([0.35, 0.0, 0.1])  # target position

target_pose = Pose(position=target_position, orientation=target_rot)

robot.move_to(pose=target_pose)

# --- Cleanup
robot.shutdown()
print("Done")
try:
    robot.shutdown()
except Exception as e:
    print(f"Error shutting down robot: {e}")
