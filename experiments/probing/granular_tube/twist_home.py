"""Home the robot for twist experiments."""

from arm_client.robot import Robot
from scipy.spatial.transform import Rotation as R
import time
import numpy as np

robot = Robot(namespace="fr3")
robot.wait_until_ready()

robot.controller_switcher_client.switch_controller("fr3_pose_controller")
start_pose = robot.end_effector_pose.copy()
start_position = np.array([0.493, 0.0455, 0.406])
start_orientation = R.from_euler("xyz", [-180, 0, 0], degrees=True)
start_pose.position = start_position
start_pose.orientation = start_orientation
robot.set_target(pose=start_pose)
time.sleep(5.0)

robot.shutdown()
