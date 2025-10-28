"""Example on how to control the Franka gripper"""

import time

from arm_client.gripper.franka_hand import Gripper, GripperConfig

# Create gripper configuration for Franka gripper
gripper_config = GripperConfig(
    max_width=0.08,  # 8 cm maximum width
    min_width=0.0,  # Fully closed
    default_speed=0.1,  # 10 cm/s default speed
    default_force=50.0,  # 50 N default force
)

# Initialize gripper with fr3 namespace
gripper = Gripper(namespace="fr3", gripper_config=gripper_config)

# Wait for gripper to be ready
print("Waiting for gripper to be ready...")
gripper.wait_until_ready()

print(f"Gripper ready! Current width: {gripper.value:.4f} m")
print(f"Joint states: {gripper.joint_states}")
print(f"Max width: {gripper.max_width} m")
print(f"Min width: {gripper.min_width} m")

# Demonstrate gripper functionality
print("\nOpening gripper...")
success = gripper.open()
print(f"Open action {'succeeded' if success else 'failed'}")

if success:
    time.sleep(2)
    print(f"Gripper width after opening: {gripper.value:.4f} m")

print("\nSetting target width to 0.04 m...")
success = gripper.set_target(0.04)
print(f"Set target action {'succeeded' if success else 'failed'}")

if success:
    time.sleep(2)
    print(f"Gripper width after set target: {gripper.value:.4f} m")

print("\nClosing gripper...")
success = gripper.close()
print(f"Close action {'succeeded' if success else 'failed'}")

if success:
    time.sleep(2)
    print(f"Gripper width after closing: {gripper.value:.4f} m")

print("\nResetting gripper...")
success = gripper.reset()
print(f"Reset action {'succeeded' if success else 'failed'}")

print("\nGripper demonstration complete!")
