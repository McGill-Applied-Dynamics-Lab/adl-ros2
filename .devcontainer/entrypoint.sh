#!/bin/bash

# Exit immediately if any command fails
set -e

# Check if WS_FOLDER is set
if [ -z "$WS_FOLDER" ]; then
    echo "Error: WS_FOLDER environment variable is not set."
    exit 1
fi

#! ros2 dependencies
# # Update rosdep and install dependencies if src directory has files
# if [ -d "/home/ros/ros2_ws/src" ] && [ "$(ls -A /home/ros/ros2_ws/src)" ]; then
#     cd /home/ros/ros2_ws
#     rosdep update
#     rosdep install --from-paths src --ignore-src --rosdistro humble -y --skip-keys "libfranka pinocchio" || true
# fi


# Execute the command passed to docker run
exec "$@"

# # Pause the terminal to see the output
# echo "!!entrypoint.sh done!! Press any key to exit..."
# read -n 1