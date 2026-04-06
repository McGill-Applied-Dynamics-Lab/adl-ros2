"""Home the robot, then set end-effector orientation to (180, 0, 0) deg."""

import time

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from arm_client import CONFIG_DIR
from arm_client.robot import Pose, Robot, Twist


robot = Robot(namespace="fr3")
robot.wait_until_ready(timeout=2.0)

print("Homing robot...")
robot.home()
robot.reset_targets()

print("Switching to fr3_pose_controller...")
robot.controller_switcher_client.switch_controller("fr3_pose_controller")
robot.fr3_pose_controller_parameters_client.load_param_config(
    file_path=CONFIG_DIR / "controllers" / "fr3_pose" / "default.yaml"
)
time.sleep(0.1)

current_pose = robot.end_effector_pose
target_orientation = Rotation.from_euler("xyz", [180.0, 0.0, 0.0], degrees=True)
target_pose = Pose(position=current_pose.position, orientation=target_orientation)

print("Moving to target orientation (roll, pitch, yaw) = (180, 0, 0) deg...")
duration = 2.0
n_points = 21
waypoints = []
time_from_start = []

slerp = Slerp(
    [0.0, duration],
    Rotation.from_quat(
        [current_pose.orientation.as_quat(), target_pose.orientation.as_quat()]
    ),
)

for t in np.linspace(0.0, duration, n_points):
    waypoint_pose = Pose(
        position=current_pose.position.copy(),
        orientation=slerp([t])[0],
    )
    waypoint_twist = Twist(np.zeros(3), np.zeros(3))
    waypoints.append((waypoint_pose, waypoint_twist))
    time_from_start.append(float(t))

robot.execute_trajectory(waypoints, time_from_start)
while robot.wait_for_trajectory_completion(duration, timeout_margin=1.0):
    time.sleep(0.01)


print("Done")
robot.shutdown()
