# Teleop
ROS 2 packages for the teleoperation of a robotic arm

Packages:
- **teleop**: Main package - Transform inputs from external devices to robot command
- **inverse3_ros2**: Interface w/ inverse3.
- **teleop_interfaces**: Interfaces (msgs) for the other packages


## Nodes
**inverse3_node**
- Node to communicate w/ the I3
- In: Desired contact forces
- Out: i3 position in its ws

The goal here is to not do any computation in regards of the robot. 
To see: Is this adding an undesired latency?
The idea was to have two components that could be used separately

**i3_teleop**
Takes the position of i3, contact forces of the robot and computes:
- Desired ee position/vel
- Desired i3 forces

This is where scaling is applied! And where we compute if we are in pos or vel control.

## Launch Files
Teleop with Inverse 3:
```
ros2 launch teleop i3_teleop.launch.py
```