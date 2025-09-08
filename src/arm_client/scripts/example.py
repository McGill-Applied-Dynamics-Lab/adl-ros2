"""Try to follow a "figure eight" target on the yz plane."""

import time

# %%
from arm_client import CONFIG_DIR
from arm_client.robot import Robot


# robot = Robot()
robot = Robot(namespace="fr3")
robot.wait_until_ready()

# --- Parameter Client ---
# #! Print current parameters
# ctrl_params: list = robot.cartesian_controller_parameters_client.list_parameters()
# params_values = robot.cartesian_controller_parameters_client.get_parameters(ctrl_params)

# print("Current controller parameters:")
# for param, value in zip(ctrl_params, params_values):
#     if param == "robot_description":
#         continue
#     print(f" - {param}: {value}")

# #! You can set parameters manually
# new_values = [
#     ("task.k_pos_x", 500.0),
#     ("task.d_pos_x", 10.0),
# ]
# robot.cartesian_controller_parameters_client.set_parameters(new_values)

#! Or load them from a yaml file
robot.cartesian_controller_parameters_client.load_param_config(
    file_path=CONFIG_DIR / "control/default_cartesian_impedance.yaml"
)


# --- To Switch Controllers ---
controller_name = "gravity_compensation"  # osc_pd_controller, gravity_compensation, ....
robot.controller_switcher_client.switch_controller(controller_name)

print("Done")
robot.shutdown()
