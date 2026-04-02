"""Open the FR3 gripper fully."""

from arm_client.robot import Robot
from arm_client.gripper.franka_hand import FrankaHand


robot = Robot(namespace="fr3")
robot.wait_until_ready(timeout=2.0)

gripper = FrankaHand(node=robot.node, namespace="fr3")
gripper.wait_until_ready(timeout=5.0)

print("Opening gripper to max width...")
gripper.open(speed=0.1, block=True)

print("Done")
robot.shutdown()
