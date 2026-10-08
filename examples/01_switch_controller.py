"""Try to follow a "figure eight" target on the yz plane."""

# %%
from arm_client.robot import Robot

from arm_client import CONFIG_DIR

# robot = Robot()
robot = Robot(namespace="fr3")
robot.wait_until_ready(timeout=2.0)

# --- To Switch Controllers ---
# controller_name = "gravity_compensation"  # osc_pd_controller, gravity_compensation, ....
# robot.controller_switcher_client.switch_controller(controller_name)


controller_name = "osc_controller"  # osc_controller, gravity_compensation, ....
robot.controller_switcher_client.switch_controller(controller_name)

robot.osc_controller_parameters_client.load_param_config(file_path=CONFIG_DIR / "controllers" / "osc" / "default.yaml")


print("Done")
robot.shutdown()

# %%
