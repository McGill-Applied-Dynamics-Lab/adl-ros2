"""Setup of the DelayRIM experiments."""

import time

# %%
from arm_client import CONFIG_DIR
from arm_client.robot import Robot


# robot = Robot()
robot = Robot(namespace="fr3")
robot.wait_until_ready(timeout=2.0)


# --- Home the robot ---
# if not robot.is_homed():
#     robot.home(time_to_home=3.0)

# --- To Switch Controllers ---
controller_name = "osc_pd_controller"  # osc_pd_controller, gravity_compensation, ....
robot.controller_switcher_client.switch_controller(controller_name)
robot.osc_pd_controller_parameters_client.load_param_config(
    file_path=CONFIG_DIR / "controllers" / "osc_pd" / "default.yaml"
)


robot.shutdown()
print("Done")
try:
    robot.shutdown()
except Exception as e:
    print(f"Error shutting down robot: {e}")
