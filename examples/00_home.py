"""Home the robot"""

from arm_client.robot import Robot


# robot = Robot()
robot = Robot(namespace="fr3")
robot.wait_until_ready(timeout=2.0)

robot.home()

print("Done")
robot.shutdown()

