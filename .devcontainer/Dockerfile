ARG ROS_DISTRO=humble

FROM osrf/ros:${ROS_DISTRO}-desktop AS base

ENV ROS_DISTRO=${ROS_DISTRO}

ARG DEBIAN_FRONTEND=noninteractive
ARG WS_FOLDER=/home/ros/ros2_ws

#! Setup User
# Create a non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]


# Delete existing user if it exists
RUN if getent passwd ${USER_UID}; then \
    userdel -r $(getent passwd ${USER_UID} | cut -d: -f1); \
    fi

# Delete existing group if it exists
RUN if getent group ${USER_GID}; then \
    groupdel $(getent group ${USER_GID} | cut -d: -f1); \
    fi

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

# Set up sudo
RUN apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*



#! Packages
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    python3-pip \
    vim \
    evtest \
    jstest-gtk \
    python3-serial \
    ### >>> purge Ubuntu’s old SymPy — torch needs ≥ 1.13
    && apt-get purge -y python3-sympy \
    && apt-get autoremove -y \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    vim \
    build-essential \
    cmake \
    wget \
    git \
    unzip \
    pip \
    python3-venv \
    python3-ament-package \
    python3-flake8 \
    python3-rosdep \
    python3-setuptools \
    python3-vcstool \
    python3-colcon-common-extensions \
    cmake \
    curl \
    dpkg \
    && curl --proto '=https' --tlsv1.2 -sSf https://just.systems/install.sh | bash -s -- --to /usr/local/bin \
    && rm -rf /var/lib/apt/lists/*

# ROS Packages
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    libeigen3-dev \
    # libignition-gazebo6-dev \
    libpoco-dev \
    # ros-${ROS_DISTRO}-ackermann-msgs \
    ros-${ROS_DISTRO}-ament-cmake \
    ros-${ROS_DISTRO}-ament-cmake-clang-format \
    ros-${ROS_DISTRO}-ament-cmake-clang-tidy \
    # ros-${ROS_DISTRO}-gazebo-ros \
    # ros-${ROS_DISTRO}-generate-parameter-library \
    ros-${ROS_DISTRO}-control-toolbox \
    ros-${ROS_DISTRO}-controller-interface \
    ros-${ROS_DISTRO}-control-msgs \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-hardware-interface \
    # ros-${ROS_DISTRO}-hardware-interface-testing \
    # ros-${ROS_DISTRO}-joint-state-broadcaster \
    # ros-${ROS_DISTRO}-joint-state-publisher-gui \
    # ros-${ROS_DISTRO}-launch-testing \
    # ros-${ROS_DISTRO}-moveit \
    # ros-${ROS_DISTRO}-realtime-tools \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-control-test-assets \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-sensor-msgs \
    # ros-${ROS_DISTRO}-xacro \
    # ros-${ROS_DISTRO}-ros-gz \
    # ros-${ROS_DISTRO}-topic-based-ros2-control \
    ros-${ROS_DISTRO}-pinocchio \
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-foxglove-bridge \
    && rm -rf /var/lib/apt/lists/*

# Symlink python3 to python
RUN ln -s /usr/bin/python3 /usr/bin/python

USER $USERNAME

RUN mkdir -p /home/ros/ros2_ws

#! Python deps
# Upgrade pip
RUN python3 -m pip install --upgrade pip

# Install Python dependencies from requirements.txt
COPY ./requirements.txt /tmp/requirements.txt
RUN pip install -r /tmp/requirements.txt
# RUN python3 -m pip install --upgrade numpy

# Pytorch
# RUN pip3 install --no-deps --break-system-packages torch torchvision torchaudio 

# #! Setup jtest-gtk
USER $USERNAME
RUN mkdir -p /home/ros/.config/jstest-gtk

#! Happly
USER root
# Needed to read the device
RUN usermod -a -G dialout $USERNAME

#! Environment variables
ENV PATH=/opt/openrobots/bin:$PATH \
    SHELL=/bin/bash \
    XDG_RUNTIME_DIR=/run/user/"${USER_UID}"  \
    WS_FOLDER=${WS_FOLDER}


FROM base AS franka
USER $USERNAME
WORKDIR /home/ros/ros2_ws

# === FRANKA ROS2 ===
RUN git clone https://github.com/frankarobotics/franka_ros2.git src/franka_ros2 \
    && cd src/franka_ros2 \
    && git checkout v2.0.2 \
    && cd /home/ros/ros2_ws \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    && sudo apt-get update \
    && vcs import src < src/franka_ros2/franka.repos --recursive --skip-existing \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y \
    && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && touch src/franka_ros2/COLCON_IGNORE src/libfranka/COLCON_IGNORE src/franka_description/COLCON_IGNORE