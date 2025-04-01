# adg-ros2
ROS2 packages for the Applied Dynamics Group at McGill.

Developped for ROS2 Humble with Ubuntu 22.04. 

## Packages
* **[robot_arm](robot_arm/README.md)**: To send/process commands to the Franka Research 3
* **[teleop](teleop/README.md)**: To teleoperate a robotic arm with the inverse 3 or a joystick


## Installing
- ROS dependencies
```
rosdep install --from-paths src --ignore-src --rosdistro humble -y --skip-keys "libfranka pinocchio"
```