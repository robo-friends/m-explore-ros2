# FROM nvidia/cuda:11.7.1-devel-ubuntu22.04
FROM ubuntu:jammy
# FROM nvidia/cuda:11.1.1-cudnn8-devel-ubuntu20.04
# FROM ubuntu:focal

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update && apt-get install --no-install-recommends -y \
    apt-utils \
    bzip2 \
    lbzip2 \ 
    tar \
    wget \
    libzbar0 \
    unzip \
    build-essential \
    zlib1g-dev \
    libcurl4-gnutls-dev \
    locales \
    curl  \
    gnupg2  \
    lsb-release \
    ca-certificates \
    && apt autoremove -y && apt clean -y \
    && rm -rf /var/lib/apt/lists/*

# https://index.ros.org/doc/ros2/Installation/Crystal/Linux-Install-Debians/
ENV ROS_DISTRO=humble

ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
    && wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add - \ 
    && apt update && apt install --no-install-recommends -y \ 
    ros-${ROS_DISTRO}-ros-base \
    ros-${ROS_DISTRO}-rclcpp-cascade-lifecycle \
    ros-${ROS_DISTRO}-geographic-msgs \
    ros-${ROS_DISTRO}-camera-info-manager \
    ros-${ROS_DISTRO}-launch-testing-ament-cmake \
    ros-${ROS_DISTRO}-diagnostic-updater \ 
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-gazebo-ros \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-gazebo-msgs \
    ros-${ROS_DISTRO}-gazebo-plugins \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-message-filters \
    ros-${ROS_DISTRO}-image-transport \ 
    ros-${ROS_DISTRO}-rqt* \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-behaviortree-cpp-v3 \
    ros-${ROS_DISTRO}-angles \
    ros-${ROS_DISTRO}-ompl \ 
    ros-${ROS_DISTRO}-turtlebot3* \
    ros-${ROS_DISTRO}-image-geometry \
    && apt autoremove && apt clean -y \
    && rm -rf /var/lib/apt/lists/* 

# Install ROS2 gazebo dependencies
RUN apt update && apt-get install --no-install-recommends -y \
    libglvnd0 \
    libglx0 \
    libegl1 \
    libxext6 \
    libx11-6 \
    libblkid-dev \
    e2fslibs-dev \
    libboost-all-dev \
    libaudit-dev \
    git \
    nano \
    # ------------------------------
    && apt autoremove && apt clean -y \ 
    && rm -rf /var/lib/apt/lists/*

RUN apt update && apt install --no-install-recommends -y \
    python3-dev \
    python3-pip \
    python3-colcon-common-extensions \
    && pip3 install rosdep \ 
    && rosdep init \
    && rosdep update \
    && apt autoremove && apt clean -y \
    && rm -rf /var/lib/apt/lists/* 

# Or your actual UID, GID on Linux if not the default 1000
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create a non-root user to use if preferred - see https://aka.ms/vscode-remote/containers/non-root-user.
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    # Add sudo support for non-root user
    && apt-get update && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && apt-get autoremove && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/* 

RUN \
    mkdir -p /home/${USERNAME}/.ignition/fuel/ \
    && echo "servers:\n  -\n    name: osrf\n    url: https://api.ignitionrobotics.org" >> /home/${USERNAME}/.ignition/fuel/config.yaml \
    && chown ${USERNAME} /home/${USERNAME}/.ignition \
    && GAZEBO_SOURCE="source /usr/share/gazebo/setup.sh" \
    && echo $GAZEBO_SOURCE >> "/home/${USERNAME}/.bashrc" \
    && chown ${USERNAME} /home/${USERNAME}/.ignition

# ROS2 source setup
RUN ROS_SOURCE="source /opt/ros/${ROS_DISTRO}/setup.sh" \
    && echo $ROS_SOURCE >> "/home/${USERNAME}/.bashrc"

WORKDIR /workspace/ros_ws/src
# Give permission to non-root user to access the workspace
RUN chown -R ${USERNAME} /workspace/ros_ws
RUN git clone https://github.com/charlielito/slam_gmapping.git --branch feature/namespace_launch

ENV TURTLEBOT3_MODEL=waffle
ENV GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/${ROS_DISTRO}/share/turtlebot3_gazebo/models

# Switch back to dialog for any ad-hoc use of apt-get
ENV DEBIAN_FRONTEND=
CMD ["/bin/bash"]