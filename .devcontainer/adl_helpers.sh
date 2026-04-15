#!/bin/bash
# ADL ROS2 Dev Container Helper Functions
# Source this file from your ~/.bashrc or ~/.zshrc

# Run a command inside the adl_ros2 container with the workspace sourced
adl_exec() {
    docker exec -it adl_ros2 bash -c "cd /home/ros/ros2_ws/src/adl-ros2 && source /home/ros/ros2_ws/install/setup.bash && $*"
}

# Open an interactive shell in the adl_ros2 container
adl_shell() {
    docker exec -it adl_ros2 bash -c "cd /home/ros/ros2_ws/src/adl-ros2 && source /home/ros/ros2_ws/install/setup.bash && bash --login"
}

# Rebuild the workspace inside the container
adl_build() {
    adl_exec cd /home/ros/ros2_ws "&&" colcon build --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=Debug"
}

# Run the homing script
adl_home() {
    adl_exec python src/arm_client/scripts/00_home.py
}

echo "✓ ADL helpers loaded. Available commands:"
echo "  adl_shell      - Open interactive shell in the container"
echo "  adl_exec CMD   - Run a command in the container"
echo "  adl_build      - Rebuild the ROS2 workspace"
echo "  adl_home       - Run the homing script"
