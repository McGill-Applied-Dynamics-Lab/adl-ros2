"""Try to follow a "figure eight" target on the yz plane."""

import time

# %%
from arm_client import CONFIG_DIR
from arm_client.robot import Robot


# robot = Robot()
robot = Robot(namespace="fr3")
robot.wait_until_ready(timeout=2.0)


# --- Home the robot ---
if not robot.is_homed():
    robot.home(time_to_home=3.0)

# --- To Switch Controllers ---
controller_name = "haptic_controller"  # osc_pd_controller, gravity_compensation, ....
robot.controller_switcher_client.switch_controller(controller_name)

robot.shutdown()
print("Done")
try:
    robot.shutdown()
except Exception as e:
    print(f"Error shutting down robot: {e}")
