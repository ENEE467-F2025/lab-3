########################################
##  Base Image for Turtle Simulation  ##
########################################

FROM osrf/ros:jazzy-desktop
ENV ROS_DISTRO=jazzy

SHELL ["/bin/bash", "-c"]

WORKDIR ${HOME}

ENV DEBIAN_FRONTEND=noninteractive

# Basic setup
RUN apt-get update && apt-get upgrade -y && apt-get install -y --no-install-recommends --allow-unauthenticated \
    autoconf \
    automake \
    bash-completion \
    build-essential \
    ca-certificates \
    cmake \
    curl \
    g++ \
    git \
    iproute2 \
    iputils-ping \
    libxext-dev \
    libx11-dev \
    make \
    mc \
    mesa-utils \
    nano \
    vim \
    tree \
    pkg-config \
    software-properties-common \
    sudo \
    tmux \
    tzdata \
    xclip \
    x11proto-gl-dev \
    ros-jazzy-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Set datetime and timezone correctly
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo '$TZ' | tee -a /etc/timezone

ENV DEBIAN_FRONTEND=dialog

# install ros packages
RUN apt-get update && apt-get install -y \
    ros-jazzy-turtlesim

# Setup user env
RUN groupadd -g 1111 -o robot
RUN useradd -ms "/bin/bash -i" --uid 1111 -g 1111 --groups sudo,video robot \
    && echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER robot
WORKDIR /home/robot

ENV USER=robot \
    LANG=en_US.UTF-8 \
    HOME=/home/robot \
    XDG_RUNTIME_DIR=/run/user/1111 \
    TZ=America/New_York

# ros workspace
RUN source /opt/ros/jazzy/setup.bash \
    && mkdir -p ~/ros2_ws/src

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc \
    && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc \
    && echo "alias clc=clear" >> ~/.bashrc

ENTRYPOINT ["/bin/bash", "-i"]