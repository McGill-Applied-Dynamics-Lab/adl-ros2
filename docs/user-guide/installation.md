# Installation

Detailed installation instructions for the ADG ROS2 packages.

## System Requirements

### Operating System
- **Ubuntu 22.04 LTS** (Recommended)
- **Ubuntu 20.04 LTS** (Limited support)

### Software Dependencies
- **ROS2 Humble** (Primary support)
- **Python 3.10+**
- **CMake 3.16+**
- **GCC 9.0+ or Clang 10.0+**

### Hardware Requirements
- **Minimum RAM**: 8GB (16GB recommended)
- **Storage**: 10GB free space
- **CPU**: x86_64 architecture
- **GPU**: Optional but recommended for Isaac Sim

## Installation Methods

### Method 1: Source Installation (Recommended)

This method gives you access to the latest features and allows for custom modifications.

#### Step 1: Install ROS2 Humble

```bash
# Update package list
sudo apt update && sudo apt upgrade -y

# Install ROS2 Humble Desktop
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

sudo apt update
sudo apt install ros-humble-desktop python3-argcomplete ros-dev-tools
```

#### Step 2: Install Build Tools

```bash
# Install colcon and other build tools
sudo apt install python3-colcon-common-extensions python3-rosdep2

# Initialize rosdep
sudo rosdep init
rosdep update
```

#### Step 3: Create Workspace and Clone

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone the main repository
git clone https://github.com/McGill-Applied-Dynamics-Group/adg-ros2.git

# Clone dependencies (if any external dependencies are needed)
vcs import < adg_ros2/dependencies.repos  # If this file exists
```

#### Step 4: Install Dependencies

```bash
cd ~/ros2_ws

# Install ROS dependencies
rosdep install --from-paths src --ignore-src -r -y

# Install Python dependencies
cd src/adg_ros2
pip install -r requirements.txt
```

#### Step 5: Build the Workspace

```bash
cd ~/ros2_ws

# Build all packages
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/ros2_ws/install/setup.bash
```

### Method 2: Docker Installation

For a containerized setup, especially useful for development and testing.

#### Prerequisites
```bash
# Install Docker
sudo apt update
sudo apt install docker.io docker-compose
sudo usermod -aG docker $USER
# Log out and back in for group changes to take effect
```

#### Using Docker

```bash
# Clone repository
git clone https://github.com/McGill-Applied-Dynamics-Group/adg-ros2.git
cd adg_ros2

# Build Docker container
docker-compose build

# Run container with GUI support
docker-compose up -d
docker exec -it adg_ros2_container bash
```

## Hardware-Specific Setup

### Franka FR3 Robot

If you're using a physical Franka FR3 robot:

```bash
# Install Franka Control Interface
sudo apt install ros-humble-franka-*

# Set up real-time kernel (recommended)
# Follow instructions at: https://frankaemika.github.io/docs/installation_linux.html
```

### Isaac Sim Integration

For NVIDIA Isaac Sim support:

```bash
# Install NVIDIA Container Toolkit
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt update
sudo apt install -y nvidia-container-toolkit
```

## Configuration

### Environment Setup

Add these lines to your `~/.bashrc`:

```bash
# ROS2 Environment
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Optional: Set default RMW implementation
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Optional: Configure DDS settings
export FASTRTPS_DEFAULT_PROFILES_FILE=~/ros2_ws/src/adg_ros2/config/fastdds_profile.xml
```

### Network Configuration

For multi-machine setups:

```bash
# Set ROS_DOMAIN_ID (0-101, default is 0)
export ROS_DOMAIN_ID=42

# For specific network interface
export ROS_LOCALHOST_ONLY=0
```

## Verification

### Test Installation

```bash
# Check ROS2 installation
ros2 doctor

# List available packages
ros2 pkg list | grep -E "(robot_arm|robot_tasks|teleop|adg_ros2_utils)"

# Test build
cd ~/ros2_ws
colcon build --packages-select adg_ros2_utils

# Run tests
colcon test --packages-select adg_ros2_utils
colcon test-result --verbose
```

### Launch Demo

```bash
# Test basic functionality
ros2 launch robot_arm_bringup demo.launch.py
```

## Troubleshooting

### Common Installation Issues

**Issue**: `colcon build` fails with CMake errors
```bash
# Solution: Update CMake
sudo apt install cmake
```

**Issue**: Python import errors
```bash
# Solution: Check Python path and install missing packages
pip install --upgrade pip
pip install -r requirements.txt
```

**Issue**: Permission denied for device access
```bash
# Solution: Add user to appropriate groups
sudo usermod -a -G dialout $USER
sudo usermod -a -G plugdev $USER
# Log out and back in
```

**Issue**: ROS2 nodes not visible across machines
```bash
# Solution: Check firewall and network settings
sudo ufw allow from 192.168.1.0/24  # Adjust subnet as needed
```

### Getting Help

- **Documentation**: Check the [API Reference](../reference/)
- **Issues**: Report bugs on [GitHub Issues](https://github.com/McGill-Applied-Dynamics-Group/adg-ros2/issues)
- **Community**: Join our [GitHub Discussions](https://github.com/McGill-Applied-Dynamics-Group/adg-ros2/discussions)

## Next Steps

After successful installation:

1. Follow the [Getting Started Guide](getting-started.md)
2. Try the [Quick Start Tutorial](quick-start.md)
3. Explore the [Package Documentation](../packages/index.md)
