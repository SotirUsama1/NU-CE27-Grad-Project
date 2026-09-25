FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# packages.ros.org currently presents a mismatched TLS certificate on GitHub-hosted
# runners. This ROS-documented HTTPS mirror can be overridden at build time.
ARG ROS_APT_REPOSITORY=https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu

RUN apt-get -o Acquire::Retries=5 update \
 && apt-get -o Acquire::Retries=5 install -y --no-install-recommends \
    locales \
    ca-certificates \
    curl \
 && locale-gen en_US.UTF-8 \
 && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] ${ROS_APT_REPOSITORY} jammy main" \
      > /etc/apt/sources.list.d/ros2.list \
 && apt-get -o Acquire::Retries=5 update \
 && apt-get -o Acquire::Retries=5 install -y --no-install-recommends \
    ros-humble-ros-base \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-rqt \
    ros-humble-turtlesim \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-msgs \
    ros-humble-turtlebot3-gazebo \
    ros-humble-turtlebot3-description \
    ros-humble-turtlebot3-teleop \
    ros-humble-slam-toolbox \
    ros-humble-cartographer-ros \
 && apt-get purge -y curl \
 && apt-get autoremove -y \
 && rm -rf /var/lib/apt/lists/*

CMD ["gazebo"]
