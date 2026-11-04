FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_LOCALHOST_ONLY=0
ENV ROS_DOMAIN_ID=17
ENV SDL_AUDIODRIVER=dummy
ENV ALSA_PCM_CARD=default


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
 && apt-get purge -y curl \
 && apt-get autoremove -y \
 && rm -rf /var/lib/apt/lists/*

# Cached ROS foundation layer. It remains reusable when application packages change.
RUN apt-get -o Acquire::Retries=5 update \
 && apt-get -o Acquire::Retries=5 install -y --no-install-recommends \
    ros-humble-ros-base \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-rmw-cyclonedds-cpp \
 && rm -rf /var/lib/apt/lists/*

# Pass a unique value to refresh only this package layer on each CI build.
ARG DYNAMIC_PACKAGES_CACHEBUST=0
RUN echo "Refreshing ROS application packages (${DYNAMIC_PACKAGES_CACHEBUST})" \
 && apt-get -o Acquire::Retries=5 update \
 && apt-get -o Acquire::Retries=5 install -y --no-install-recommends \
    ros-humble-rqt \
    ros-humble-demo-nodes-cpp \
    ros-humble-turtlesim \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /main

CMD ["bash"]