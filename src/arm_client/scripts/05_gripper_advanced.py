"""Example on how to control the Franka gripper with YAML configuration"""

import time
from pathlib import Path

from arm_client.gripper.franka_hand import Gripper, GripperConfig

# Option 1: Use default configuration
print("=== Example 1: Using default configuration ===")
gripper_default = Gripper(namespace="fr3")

print(f"Default max width: {gripper_default.max_width} m")
print(f"Default min width: {gripper_default.min_width} m")

# Option 2: Use custom configuration
print("\n=== Example 2: Using custom configuration ===")
custom_config = GripperConfig(
    max_width=0.085,  # Slightly larger max width
    min_width=0.001,  # Small minimum width
    default_speed=0.05,  # Slower default speed
    default_force=40.0,  # Lower default force
)

gripper_custom = Gripper(namespace="fr3", gripper_config=custom_config)

print(f"Custom max width: {gripper_custom.max_width} m")
print(f"Custom min width: {gripper_custom.min_width} m")

# Option 3: Load configuration from YAML file
print("\n=== Example 3: Using YAML configuration ===")
try:
    # Load configuration from the example YAML file
    config_path = Path(__file__).parent.parent.parent / "configs" / "gripper" / "franka_gripper_example.yaml"
    yaml_config = GripperConfig.from_yaml(config_path)
    gripper_yaml = Gripper(namespace="fr3", gripper_config=yaml_config)

    print(f"YAML max width: {gripper_yaml.max_width} m")
    print(f"YAML min width: {gripper_yaml.min_width} m")
    print(f"YAML default speed: {yaml_config.default_speed} m/s")
    print(f"YAML default force: {yaml_config.default_force} N")
except FileNotFoundError:
    print("YAML configuration file not found. Using default configuration.")
    gripper_yaml = gripper_default

# Demonstrate functionality with one of the grippers
print("\n=== Demonstrating gripper functionality ===")
gripper = gripper_custom  # Use the custom configured gripper

try:
    # Wait for gripper to be ready
    print("Waiting for gripper to be ready...")
    gripper.wait_until_ready(timeout=5.0)

    print(f"Gripper ready! Current width: {gripper.value:.4f} m")
    print(f"Joint states: {gripper.joint_states}")

    # Demonstrate various operations
    print("\nOpening gripper...")
    success = gripper.open(speed=0.05)  # Custom speed
    print(f"Open action {'succeeded' if success else 'failed'}")

    if success:
        time.sleep(2)
        print(f"Gripper width after opening: {gripper.value:.4f} m")

    print("\nSetting target width to 0.04 m...")
    success = gripper.set_target(0.04, speed=0.03)  # Custom speed
    print(f"Set target action {'succeeded' if success else 'failed'}")

    if success:
        time.sleep(2)
        print(f"Gripper width after set target: {gripper.value:.4f} m")

    print("\nClosing gripper...")
    success = gripper.close(force=30.0, speed=0.02)  # Custom force and speed
    print(f"Close action {'succeeded' if success else 'failed'}")

    if success:
        time.sleep(2)
        print(f"Gripper width after closing: {gripper.value:.4f} m")

    print("\nResetting gripper...")
    success = gripper.reset()
    print(f"Reset action {'succeeded' if success else 'failed'}")

except TimeoutError:
    print("Timeout waiting for gripper to be ready.")
    print("Make sure the Franka gripper server is running and publishing joint states.")
except Exception as e:
    print(f"Error during gripper operation: {e}")

print("\nGripper demonstration complete!")
print("\nTip: You can customize the gripper behavior by:")
print("1. Modifying the GripperConfig parameters")
print("2. Creating your own YAML configuration file")
print("3. Passing custom speed/force parameters to individual methods")
