"""Home the robot"""

import time

from arm_client.robot import Robot
from arm_client import CONFIG_DIR


# robot = Robot()
robot = Robot(namespace="fr3")
robot.wait_until_ready(timeout=2.0)

# -- Switch to a cartesian controller
# cartesian_impedance_controller, osc_pd_controller

# # OSC PD CONTROLLER
# robot.controller_switcher_client.switch_controller("osc_pd_controller")
# # robot.cartesian_controller_parameters_client.load_param_config(
# #     # file_path="config/control/gravity_compensation.yaml"
# #     # file_path="config/control/default_operational_space_controller.yaml"
# #     # file_path="config/control/clipped_cartesian_impedance.yaml"
# #     file_path=CONFIG_DIR / "control" / "default_cartesian_impedance.yaml"
# # )
# robot.osc_pd_controller_parameters_client.load_param_config(
#     file_path=CONFIG_DIR / "controllers" / "osc_pd" / "default.yaml"
# )

#
robot.controller_switcher_client.switch_controller("fr3_pose_controller")
robot.fr3_pose_controller_parameters_client.load_param_config(
    file_path=CONFIG_DIR / "controllers" / "osc_pd" / "default.yaml"
)

# -- Move
position = [0.4, 0.0, 0.4]  # x, y, z in meters
robot.move_to(position=position, speed=0.05)

print("Done")
robot.shutdown()
