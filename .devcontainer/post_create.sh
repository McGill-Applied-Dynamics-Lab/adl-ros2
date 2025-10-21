#!/bin/bash
set -e

echo "=== Post-create setup started ==="

# Update package lists
echo "Updating apt package lists..."
sudo apt-get update

# Initialize rosdep if not already done
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    echo "Initializing rosdep..."
    sudo rosdep init
fi

# Create COLCON_IGNORE files
echo "Creating COLCON_IGNORE files..."
touch /home/ros/ros2_ws/src/adl-ros2/src/_legacy/COLCON_IGNORE

# Update rosdep
echo "Updating rosdep..."
rosdep update

# Navigate to workspace
cd /home/ros/ros2_ws

# Install dependencies for current packages
echo "Installing package dependencies..."
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y

# Build the workspace
echo "Building ROS2 workspace..."
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install --cmake-args '-DCMAKE_BUILD_TYPE=Debug'

# Setup shell environment
echo "Setting up shell environment..."
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/src/adl-ros2/scripts/ros_aliases.sh" >> ~/.bashrc

# Source the workspace for current session
source install/setup.bash

# Test the installation
echo "Testing ROS2 installation..."
python3 src/adl-ros2/scripts/test_install.py

echo "=== Post-create setup completed ==="
