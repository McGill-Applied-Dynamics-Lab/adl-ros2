# Teleop 
Remote robot control and teleoperation system with multiple interface options.

## Sub-Packages
### `teleop`
Main teleoperation package with core functionality.

- **Purpose**: Primary teleoperation interface and control logic
- **Language**: Python
- **Key Features**: Multi-input support, safety monitoring, command filtering

#### Launch Files


#### Nodes

- i3_teleop_virtual_coupling; To control the robot through a virtual spring between with the I3

*To be tested/updated*

- Gamepad:
- i3_teleop_position
- i3_teleop
- joy_teleop

##### i3_teleop_virtual_coupling



### Devices
#### **`inverse3_ros2`**
ROS2 wrapper for the inverse3 device by Haply. Communicates with the device through the web socket.

Publishes the current *pose* and *twist* of the Inverse 3. The pose is relative to the workspace centre. 

**Parameters**
*TODO: Change to link to .yaml param file when it's implemented*

- *i3_pose_topic_name*: Name of the pose topic (Default: '/i3/pose')
- *i3_twist_topic_name*: Name of the twist topic (Default: '/i3/twist')
- *i3_wrench_topic_name*: Name of the desired wrench topic (Default: '/i3/wrench')
- *workspace_centre*: Absolute coordinate of the workspace centre. 
- *frequency*: Publish frequency (Default: 1000.0)

**Topics**

- */i3/pose* [PoseStamped]
- */i3/twist* [TwistStamped]

**Subscriptions**

- */i3/wrench* [WrenchStamped]

#### `network_sim`
5G and fixed network simulation.

- **Purpose**: Network latency simulation and communication testing
- **Language**: Python  
- **Key Features**: Latency injection

#### `teleop_interfaces`
Message and service definitions for teleoperation.

- **Purpose**: Custom ROS2 interfaces for teleop commands
- **Language**: ROS2 Interface Definition Language
- **Key Features**: Command messages, status feedback, configuration services


## Supported Input Methods

### Keyboard Control
TODO
Direct keyboard input for quick testing and development.

```bash
# Launch keyboard teleoperation
ros2 run TODO

# Controls:
# WASD: Translation
# QE: Rotation around Z
# RF: Up/Down motion
# Space: Emergency stop
```

### Joystick/Gamepad
Xbox, PlayStation, and generic USB controllers.


```bash
# Launch with joystick support
ros2 launch TODO
```



## Configuration

### Device Configuration

Configure input devices in `config/devices.yaml`:

```yaml
keyboard:
  enabled: true
  key_mappings:
    'w': [0.0, 0.1, 0.0]  # Forward
    's': [0.0, -0.1, 0.0] # Backward
    'a': [-0.1, 0.0, 0.0] # Left
    'd': [0.1, 0.0, 0.0]  # Right
    
joystick:
  enabled: true
  device: "/dev/input/js0"
  deadzone: 0.1
  max_velocity: 0.5
  button_map:
    emergency_stop: 9
    mode_switch: 8
```


## Quick Start

### Basic Keyboard Control

```bash
# Terminal 1: Launch robot system
ros2 launch robot_arm_bringup gazebo.launch.py

# Terminal 2: Start teleoperation
ros2 run teleop teleop_keyboard
```

### Joystick Setup

```bash
# Check joystick device
ls /dev/input/js*

# Test joystick input  
jstest /dev/input/js0

# Launch with joystick
ros2 launch teleop joystick.launch.py device:=/dev/input/js0
```

### Network Teleoperation

```bash
# Server side (robot)
ros2 launch teleop network_server.launch.py

# Client side (operator)
ros2 run teleop network_client --server=ROBOT_IP:8080
```


## Network Simulation

The `network_sim` package provides tools for testing teleoperation under various network conditions:

TODO

## API Reference

