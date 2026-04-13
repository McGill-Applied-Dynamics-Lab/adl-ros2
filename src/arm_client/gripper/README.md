# Franka Gripper Client

This implementation provides a Python client for controlling the Franka gripper using ROS2 actions.

## Features

The Franka gripper client provides the following functionality:

### Properties
- `joint_states`: Returns the current position of both gripper finger joints as a numpy array
- `value`: Returns the current gripper width in meters
- `max_width`: Returns the maximum gripper width (configured)
- `min_width`: Returns the minimum gripper width (configured)
- `is_ready()`: Returns True if the gripper is ready to operate

### Methods
- `close(force=None, speed=None, block=True)`: Close the gripper using the grasp action
- `open(speed=None, block=True)`: Open the gripper to maximum width using the move action
- `reset(block=True)`: Reset/home the gripper using the homing action
- `set_target(target_width, speed=None, block=True)`: Set a specific target width using the move action
- `wait_until_ready(timeout=10.0)`: Wait until the gripper is ready
- `is_open(threshold=0.01)`: Check if the gripper is open

### Non-blocking Operation Helpers
- `wait_for_action(future, timeout=None)`: Wait for a non-blocking action to complete
- `is_action_done(future)`: Check if a non-blocking action is complete
- `get_action_result(future)`: Get the result of a completed non-blocking action

## Usage

### Basic Usage

```python
from arm_client.gripper.franka_hand import Gripper, GripperConfig

# Create gripper with default configuration
gripper = Gripper(namespace="fr3")

# Wait for gripper to be ready
gripper.wait_until_ready()

# Basic operations
gripper.open()
gripper.set_target(0.04)  # 4 cm width
gripper.close()
gripper.reset()
```

### Custom Configuration

```python
# Create custom configuration
config = GripperConfig(
    max_width=0.085,      # 8.5 cm maximum
    min_width=0.001,      # 1 mm minimum
    default_speed=0.05,   # 5 cm/s default speed
    default_force=40.0,   # 40 N default force
)

gripper = Gripper(namespace="fr3", gripper_config=config)
```

### YAML Configuration

```python
from pathlib import Path

# Load configuration from YAML file
config_path = Path("configs/gripper/my_gripper_config.yaml")
config = GripperConfig.from_yaml(config_path)
gripper = Gripper(namespace="fr3", gripper_config=config)
```

### Custom Parameters

```python
# Use custom speed and force for individual operations
gripper.open(speed=0.02)  # Slow opening
gripper.close(force=30.0, speed=0.01)  # Gentle closing
gripper.set_target(0.03, speed=0.05)  # Custom target with speed
```

## Configuration

The `GripperConfig` class accepts the following parameters:

- `max_width` (float): Maximum gripper width in meters (default: 0.08)
- `min_width` (float): Minimum gripper width in meters (default: 0.0)
- `default_speed` (float): Default movement speed in m/s (default: 0.1)
- `default_force` (float): Default grasping force in N (default: 50.0)
- `grasp_epsilon_inner` (float): Inner epsilon for grasp action (default: 0.005)
- `grasp_epsilon_outer` (float): Outer epsilon for grasp action (default: 0.005)
- `action_timeout` (float): Timeout for action calls in seconds (default: 10.0)
- `max_joint_delay` (float): Maximum delay for joint states before stale warning (default: 1.0)

Topic and action names can also be configured but usually don't need to be changed.

## ROS2 Interface

The client uses the following ROS2 interfaces:

### Actions
- `franka_gripper/grasp` (franka_msgs/action/Grasp) - For closing with force control
- `franka_gripper/move` (franka_msgs/action/Move) - For opening and position control
- `franka_gripper/homing` (franka_msgs/action/Homing) - For resetting the gripper

### Topics
- `franka_gripper/joint_states` (sensor_msgs/msg/JointState) - For gripper state feedback

## Error Handling

The client includes comprehensive error handling:

- Action server availability checking
- Timeout handling for action calls
- Input validation for target widths
- Stale joint state detection
- Proper error messages and logging

## Examples

See the example scripts:
- `scripts/04_gripper.py` - Basic usage example
- `scripts/05_gripper_advanced.py` - Advanced usage with configurations
- `configs/gripper/franka_gripper_example.yaml` - Example configuration file