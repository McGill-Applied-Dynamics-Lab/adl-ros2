# adg-ros2
ROS2 packages for the Applied Dynamics Group at McGill.

Developped for ROS2 Humble with Ubuntu 22.04. 

## Packages
* **[robot_arm](robot_arm/README.md)**: To send/process commands to the Franka Research 3
* **[teleop](teleop/README.md)**: To teleoperate a robotic arm with the inverse 3 or a joystick
* **[robot_tasks](robot_tasks/README.md)**: To execute task w/ the robot arm

## Installing
1. Build Docker

2. ROS dependencies
```
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -y --skip-keys "libfranka pinocchio"
```

3. Python deps
```
# numpy 1.25 needed for pinocchio. 2.0 makes it fail
pip install numpy==1.25.2 
pip install torch torchvision torchaudio 
pip install skrl
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

**Note**
The node can be launced in the docker container