"""Try to follow a "figure eight" target on the yz plane."""

import time

# %%
from arm_client import CONFIG_DIR
from arm_client.robot import Robot


# robot = Robot()
robot = Robot(namespace="fr3")
robot.wait_until_ready(timeout=2.0)

# --- To Switch Controllers ---
controller_name = "gravity_compensation"  # osc_pd_controller, gravity_compensation, ....
robot.controller_switcher_client.switch_controller(controller_name)

print("Done")
robot.shutdown()
