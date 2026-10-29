FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV SDL_AUDIODRIVER=dummy
ENV ALSA_PCM_CARD=default

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_LOCALHOST_ONLY=0
ENV ROS_DOMAIN_ID=17

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
 && echo "deb [arch=$(dpkg --print-architecture) " \
      "signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] " \
      "${ROS_APT_REPOSITORY} jammy main" \
      > /etc/apt/sources.list.d/ros2.list

RUN apt-get -o Acquire::Retries=5 update \
 && apt-get -o Acquire::Retries=5 install -y --no-install-recommends \
    ros-humble-ros-base \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-rmw-cyclonedds-cpp \
 && apt-get purge -y curl \
 && apt-get autoremove -y \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/*

RUN echo "source /usr/share/gazebo/setup.sh" >> /root/.bashrc && \
    echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

CMD ["bash"]