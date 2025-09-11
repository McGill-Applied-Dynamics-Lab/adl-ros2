# Teleop 
Remote robot control and teleoperation system with multiple interface options.

## Packages
### `teleop`

Main package. Teleoperation nodes (to process device inputs) and launch files

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

## Quick Start

### Inverse3
TODO
```bash
```

### Gamepad
TODO

### Network Simulation
You can add delays between messages using... 

Add this to your launch file
```python
...
```
