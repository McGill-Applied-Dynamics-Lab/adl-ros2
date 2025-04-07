# adg-ros2
ROS2 packages for the Applied Dynamics Group at McGill.

Developped for ROS2 Humble with Ubuntu 22.04. 

## Packages
* **[robot_arm](robot_arm/README.md)**: To send/process commands to the Franka Research 3
* **[teleop](teleop/README.md)**: To teleoperate a robotic arm with the inverse 3 or a joystick
* **[robot_tasks](robot_tasks/README.md)**: To execute task w/ the robot arm

## Installing
- ROS dependencies
```
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -y --skip-keys "libfranka pinocchio"
```

## Testing
### Franka-PC
1. Run tests
```
colcon test --event-handlers console_direct+ --packages-select robot_arm_testing
```

2. Process results w/ xunit-viewer
```
xunit-viewer -r build/robot_arm_testing/test_results
```

Note: Step 1 & 2 can be run using the *ROS2 Test - robot_arm_testing* task in VsCode

3. Copy results to host (*run on host*)
```
scp csirois@franka-pc:~/workspaces/franka_ros2_ws/index.html ~/workspaces/franka_ros2_ws
```

4. Open `index.html` in browser

### On Host