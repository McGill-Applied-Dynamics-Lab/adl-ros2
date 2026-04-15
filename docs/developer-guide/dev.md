# Dev Guide

## Data Analysis
### With Foxglove
Site: https://app.foxglove.dev/mc-gill/dashboard

```download
sudo apt install ros-$ROS_DISTRO-foxglove-bridge
```

To run:
```
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

## Teleop
```
ros2 run inverse3_ros2 inverse3_node
```

## Delay Rim
```
ros2 launch franka_rim franka_rim.launch.py delay:=5g compensation:=zoh
```