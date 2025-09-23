#!/bin/bash

# Exit immediately if any command fails
set -e

# Check if WS_FOLDER is set
if [ -z "$WS_FOLDER" ]; then
    echo "Error: WS_FOLDER environment variable is not set."
    exit 1
fi

# Source ROS2 environment
source /opt/ros/$ROS_DISTRO/setup.bash

#! ros2 dependencies
# Update rosdep and install dependencies if src directory has files
if [ -d "/home/ros/ros2_ws/src" ] && [ "$(ls -A /home/ros/ros2_ws/src)" ]; then
    cd /home/ros/ros2_ws
    
    # Update apt only if needed (check if cache is older than 1 day)
    if [ ! -f /var/lib/apt/periodic/update-success-stamp ] || [ $(find /var/lib/apt/periodic/update-success-stamp -mtime +1) ]; then
        echo "Updating apt package lists..."
        apt update
    fi
    
    # Initialize rosdep if not done
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        echo "Initializing rosdep..."
        rosdep init
    fi
    
    echo "Updating rosdep..."
    rosdep update
    
    echo "Installing ROS dependencies..."
    rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y --skip-keys "libfranka pinocchio" || true
    
    # Source workspace if it exists
    if [ -f "install/setup.bash" ]; then
        source install/setup.bash
    fi
fi

# Execute the command passed to docker run
exec "$@"