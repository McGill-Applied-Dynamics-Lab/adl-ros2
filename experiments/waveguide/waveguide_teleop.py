"""Waveguide Teleoperation

Read the Waveguide outputs and process them to robot commands.

The commands are:
- slide in x
- slide in y
- up/down in z
- close/open gripper

"""

from arm_client.robot import Robot

# Initialization
robot = Robot(namespace="fr3")
robot.wait_until_ready()

print(robot.end_effector_pose)
print(robot.q)

print("Going to home position...")
robot.home()
homing_pose = robot.end_effector_pose.copy()

# --- Control Mappings ---

# --- Main Loop ---


# --- Cleanup
robot.shutdown()
