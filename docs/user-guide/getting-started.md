# Getting Started

This guide will help you get up and running with the ADG ROS2 packages quickly.

## Prerequisites

Before you begin, ensure you have the following installed:

- **Ubuntu 22.04 LTS**: The supported operating system
- **ROS2 Humble**: The Robot Operating System 2 distribution
- **Python 3.10+**: For Python-based nodes and utilities
- **Git**: For cloning repositories
- **Colcon**: ROS2 build tool

## System Setup

### 1. Install ROS2 Humble

If you haven't already installed ROS2 Humble, follow the [official installation guide](https://docs.ros.org/en/humble/Installation.html).

```bash
# Source ROS2 environment
source /opt/ros/humble/setup.bash
```

### 2. Create Workspace

```bash
# Create a new ROS2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone the repository
git clone https://github.com/McGill-Applied-Dynamics-Group/adg-ros2.git
```

### 3. Install Dependencies

```bash
# Navigate to workspace root
cd ~/ros2_ws

# Install ROS dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Install Python dependencies
pip install -r src/adg_ros2/requirements.txt
```

### 4. Build the Workspace

```bash
# Build all packages
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

## Basic Usage

### Robot Arm Control

To start the robot arm control system:

```bash
# Launch the robot arm bringup
ros2 launch robot_arm_bringup robot_arm.launch.py

# In another terminal, test joint control
ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "..."
```

### Teleoperation

To start teleoperation:

```bash
# Launch teleoperation node
ros2 launch teleop teleop.launch.py

# Use keyboard controls or joystick
ros2 run teleop teleop_keyboard
```

### Robot Tasks

To run robot tasks with RL:

```bash
# Launch task environment
ros2 launch robot_tasks lift_task.launch.py

# Run trained agent
ros2 run robot_tasks run_agent.py
```

## Verification

To verify your installation is working correctly:

```bash
# Check that all nodes are discoverable
ros2 pkg list | grep -E "(robot_arm|robot_tasks|teleop|adg_ros2_utils)"

# Run basic tests
colcon test --packages-select adg_ros2_utils
colcon test-result
```

## Next Steps

- Explore the [API Reference](../reference/) for detailed documentation
- Check out the [Tutorials](../tutorials/) for step-by-step examples
- Review the [Developer Guide](../developer-guide/contributing.md) if you want to contribute

## Troubleshooting

### Common Issues

**Q: Build fails with missing dependencies**

A: Make sure you've run `rosdep install` and installed Python requirements:

```bash
rosdep install --from-paths src --ignore-src -r -y
pip install -r src/adg_ros2/requirements.txt
```

**Q: Nodes not found after building**

A: Ensure you've sourced the workspace:

```bash
source install/setup.bash
```

**Q: Permission denied when accessing robot**

A: Make sure your user is in the appropriate groups and has permissions for device access.

For more help, check our [GitHub Issues](https://github.com/McGill-Applied-Dynamics-Group/adg-ros2/issues) or start a [Discussion](https://github.com/McGill-Applied-Dynamics-Group/adg-ros2/discussions).
