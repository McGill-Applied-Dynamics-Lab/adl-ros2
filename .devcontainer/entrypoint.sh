#!/bin/bash

# Exit immediately if any command fails
set -e

#  Check if WS_FOLDER is set
if [ -z "$WS_FOLDER" ]; then
    echo "Error: WS_FOLDER environment variable is not set."
    exit 1
fi

#! ROSDEP
# Initialize rosdep (if not already initialized)
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init || echo "rosdep already initialized"
fi

# Update rosdep
rosdep update

# Install ROS dependencies
if [ -d "$WS_FOLDER/src" ]; then
    sudo apt-get update
    rosdep install --from-paths "$WS_FOLDER/src" --ignore-src -y --skip-keys libfranka
else
    echo "Error: /workspace/src not found. Make sure your workspace is mounted correctly."
    bash
    exit 1
fi

# Execute the command passed to the container
exec "$@"

# Pause the terminal to see the output
echo "!!entrypoint.sh done!! Press any key to exit..."
read -n 1