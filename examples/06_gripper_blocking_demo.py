"""Example demonstrating blocking and non-blocking gripper operations"""

import time

from arm_client.gripper.franka_hand import Gripper, GripperConfig

# Create gripper configuration
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

print("\n=== BLOCKING OPERATIONS (block=True) ===")

print("Opening gripper (blocking)...")
success = gripper.open(block=True)  # This will wait until complete
print(f"Open action {'succeeded' if success else 'failed'}")

time.sleep(1)
print(f"Current width: {gripper.value:.4f} m")

print("Closing gripper (blocking)...")
success = gripper.close(block=True)  # This will wait until complete
print(f"Close action {'succeeded' if success else 'failed'}")

time.sleep(1)
print(f"Current width: {gripper.value:.4f} m")

print("\n=== NON-BLOCKING OPERATIONS (block=False) ===")

print("Opening gripper (non-blocking)...")
future = gripper.open(block=False)  # Returns immediately
print("Open action started, continuing with other work...")

# Do other work while gripper is moving
for i in range(5):
    print(f"Doing other work... {i + 1}/5")
    time.sleep(0.5)
    if gripper.is_action_done(future):
        print("Gripper finished opening!")
        break

# Wait for completion and get result
try:
    success = gripper.wait_for_action(future, timeout=5.0)
    print(f"Open action {'succeeded' if success else 'failed'}")
    print(f"Current width: {gripper.value:.4f} m")
except TimeoutError:
    print("Open action timed out!")

print("\nSetting target to 0.04m (non-blocking)...")
target_future = gripper.set_target(0.04, block=False)

# Check status periodically
while not gripper.is_action_done(target_future):
    print("Target action still running...")
    time.sleep(0.5)

# Get the result
success = gripper.get_action_result(target_future)
print(f"Set target action {'succeeded' if success else 'failed'}")
print(f"Current width: {gripper.value:.4f} m")

print("\n=== MULTIPLE NON-BLOCKING ACTIONS ===")

print("Starting multiple actions in sequence...")

# Start opening
open_future = gripper.open(block=False)
print("1. Open action started")

# Wait for it to complete
gripper.wait_for_action(open_future)
print("2. Open action completed")

# Start closing immediately
close_future = gripper.close(block=False)
print("3. Close action started")

# Start reset after a delay (will queue up)
time.sleep(2)
reset_future = gripper.reset(block=False)
print("4. Reset action started")

# Wait for all to complete
gripper.wait_for_action(close_future)
print("5. Close action completed")

gripper.wait_for_action(reset_future)
print("6. Reset action completed")

print("\nDemo complete!")
print(f"Final gripper width: {gripper.value:.4f} m")

print("\n=== USAGE SUMMARY ===")
print("Blocking mode (block=True, default):")
print("  - gripper.open()  # Waits until complete")
print("  - Returns bool indicating success")
print("")
print("Non-blocking mode (block=False):")
print("  - future = gripper.open(block=False)  # Returns immediately")
print("  - gripper.is_action_done(future)  # Check if done")
print("  - gripper.wait_for_action(future)  # Wait for completion")
print("  - gripper.get_action_result(future)  # Get result when done")
