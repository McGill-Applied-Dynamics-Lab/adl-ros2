FROM osrf/ros:humble-desktop-full

ARG DEBIAN_FRONTEND=noninteractive

ARG USER_UID=1000
ARG USER_GID=1000
ARG USERNAME=user

ARG WS_FOLDER=/ros2_ws

#! Add User
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && mkdir -p -m 0700 /run/$USERNAME/"${USER_UID}" \
    && mkdir -p -m 0700 /run/$USERNAME/"${USER_UID}"/gdm \
    && chown $USERNAME:$USERNAME /run/$USERNAME/"${USER_UID}" \
    # && chown $USERNAME:$USERNAME /$WS_FOLDER \
    && chown $USERNAME:$USERNAME /run/$USERNAME/"${USER_UID}"/gdm \
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

#! My packages
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    python3-pip \
	vim \
    evtest \ 
    jstest-gtk \ 
    python3-serial \
	&& rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=

# Setup jtest-gtk
USER $USERNAME
RUN mkdir -p /home/user/.config/jstest-gtk
USER root

#! ROS Packages
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    libeigen3-dev \
    # libignition-gazebo6-dev \
    libpoco-dev \
    # ros-humble-ackermann-msgs \
    ros-humble-ament-cmake \
    ros-humble-ament-cmake-clang-format \
    ros-humble-ament-cmake-clang-tidy \
    # ros-humble-gazebo-ros \
    # ros-humble-generate-parameter-library \
    ros-humble-control-toolbox \
    ros-humble-controller-interface \
    ros-humble-control-msgs \
    ros-humble-controller-manager \
    ros-humble-geometry-msgs \
    ros-humble-hardware-interface \
    # ros-humble-hardware-interface-testing \
    # ros-humble-joint-state-broadcaster \
    # ros-humble-joint-state-publisher-gui \
    # ros-humble-launch-testing \
    # ros-humble-moveit \
    # ros-humble-realtime-tools \
    ros-humble-ros2-control \
    ros-humble-ros2-control-test-assets \
    ros-humble-ros2-controllers \
    ros-humble-sensor-msgs \
    # ros-humble-xacro \
    # ros-humble-ros-gz \
    # ros-humble-topic-based-ros2-control \
    ros-humble-pinocchio \
    ros-humble-joy \
    && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=

#! Pinocchio
RUN mkdir -p /etc/apt/keyrings && \
    curl -fsSL http://robotpkg.openrobots.org/packages/debian/robotpkg.asc | tee /etc/apt/keyrings/robotpkg.asc && \
    echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" \
    | tee /etc/apt/sources.list.d/robotpkg.list && \
    apt-get update && \
    apt-get install -y robotpkg-pinocchio && \
    apt-get clean -y && \
    rm -rf /var/lib/apt/lists/*

#! libfranka
RUN mkdir /home/user/source_code
RUN cd /home/user/source_code && git clone https://github.com/frankaemika/libfranka.git \
    && cd libfranka \
    && git checkout 0.15.0 \
    && git submodule init \
    && git submodule update \
    && mkdir build && cd build \
    && cmake -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/opt/openrobots/lib/cmake .. \
    && make franka -j$(nproc) \
    && cpack -G DEB \workspaces \
    && sudo dpkg -i libfranka*.deb

RUN addgroup realtime
RUN usermod -a -G realtime $USERNAME

#! Happly
# Needed to read the device
RUN usermod -a -G dialout $USERNAME

# #! Dependencies
# # Initialize rosdep
# RUN rosdep init || echo "rosdep already initialized" && \
#     rosdep update

# # USER $USERNAMEs
# # Install dependencies using rosdep
# RUN rosdep install --from-paths /${WS_FOLDER}/src --ignore-src -y --skip-keys libfranka
# USER root

# #! Python
# # RUN pip install -e python/robotic_arm_controller 
# COPY entrypoint.sh /entrypoint.sh
# RUN chmod +x /entrypoint.sh
# ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
# Copy the requirements file
COPY requirements.txt /tmp/requirements.txt

# Install Python dependencies
RUN pip install -r /tmp/requirements.txt

#! Environment variables
ENV PATH=/opt/openrobots/bin:$PATH \
    SHELL=/bin/bash \
    XDG_RUNTIME_DIR=/run/user/"${USER_UID}"  \
    WS_FOLDER=${WS_FOLDER}
    # PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH \
    # LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH \
    # PYTHONPATH=/opt/openrobots/lib/python3.10/site-packages:$PYTHONPATH \
    # CMAKE_PREFIX_PATH=/opt/openrobots/lib/cmake:$CMAKE_PREFIX_PATH

#! Entrypoint script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
# ENTRYPOINT ["/entrypoint.sh"]

USER $USERNAME
# RUN echo "export GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH}:/ros2_ws/src/" >> ~/.bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source ${WS_FOLDER}/install/setup.bash" >> ~/.bashrc

# WORKDIR ${WS_FOLDER}

# set the default user to the newly created user
CMD ["/bin/bash"]
