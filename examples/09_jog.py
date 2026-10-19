"""Jog the end effector with fr3_pose_controller: hold-to-move, then a pose target."""

import time

from arm_client.robot import Robot

from arm_client import CONFIG_DIR

robot = Robot(namespace="fr3")
robot.wait_until_ready(timeout=2.0)

robot.controller_switcher_client.switch_controller("fr3_pose_controller")
robot.fr3_pose_controller_parameters_client.load_param_config(
    file_path=CONFIG_DIR / "controllers" / "fr3_pose" / "default.yaml"
)
start = robot.end_effector_pose

# -- Jog: up at 2 cm/s for 2 s, then brake to a stop and hold there
print("Starting linear jog")
robot.jog(linear=[0.1, 0.0, -0.02])
time.sleep(2.0)
robot.stop_jog()
time.sleep(0.5)
print(f"Jogged {(robot.end_effector_pose.position - start.position) * 1e3} mm")

# -- Rotate about the base z axis at 0.2 rad/s for 1 s
print("Starting angular jog")
robot.jog(angular=[0.2, 0.3, -0.2])
time.sleep(5.0)
robot.stop_jog()
time.sleep(0.5)

# -- A pose target takes over: back to the start
robot.move_to(pose=start)
print("Back at the start pose")

robot.shutdown()
