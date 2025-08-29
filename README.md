# adg-ros2
ROS2 packages for the Applied Dynamics Group at McGill. 

Main use:
- Teleoperation using the Inverse3
- DelayRIM
- Deployment of policies

## Features

## For Contributing
See **[docs/README.md](docs/README.md)** on how to build and update the docs. 



Developed for ROS2 Humble with Ubuntu 22.04. 

## Packages
* **[robot_arm](robot_arm/README.md)**: To send/process commands to the Franka Research 3
* **[teleop](teleop/README.md)**: To teleoperate a robotic arm with the inverse 3 or a joystick
* **[robot_tasks](robot_tasks/README.md)**: To execute task w/ the robot arm

## Usage

Connecting to the docker container in a terminal
```
docker exec -it franka_ros2 /bin/bash
```


## Installing
1. Create a ros2 workspace
2. In `src`, clone `adg_ros2`, `franka_ros2`, `isaac_ros2_messages`(?)
TODO: commands
*Note* name of folder needs to be `adg_ros2`, with an underscore and not an hyphen

3. Create symlink to .devcontainer
```
ln -s src/adg_ros2/.devcontainer .devcontainer
```

4. Build Docker
Vs code steps to build


5. ROS dependencies
```
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -y --skip-keys "libfranka pinocchio"
```

6. Check installation success
```
python3 scripts/verify_installation.py
```

7. Build ros2 packages
*from root directory*
```
colcon build --symlink-install --cmake-args '-DCMAKE_BUILD_TYPE=Debug'
```

8. Source ros2
```
source install/setup.bash
```

9. Launch example
TODO: Teleop example w/ rviz only

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
The node can be launched in the docker container

## Teleop
```
ros2 run inverse3_ros2 inverse3_node
```

## Delay Rim
```
ros2 launch franka_rim franka_rim.launch.py delay:=5g compensation:=zoh
```