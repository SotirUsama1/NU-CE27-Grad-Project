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
    ros-humble-rviz2 \
    ros-humble-xacro \
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
    ros-humble-gazebo-ros2-control \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-ur \
    ros-humble-teleop-twist-keyboard \
    ros-humble-teleop-twist-joy \
    ros-humble-twist-mux \
    ros-humble-robot-localization \
    ros-humble-interactive-marker-twist-server \
    ros-humble-imu-filter-madgwick \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pil \
    python3-yaml \
    python3-numpy \
    python3-pip \
    git \
    build-essential \
 && apt-get autoremove -y \
 && rm -rf /var/lib/apt/lists/*

# Build Jackal, Husky and the UR Gazebo Classic simulation from source
WORKDIR /opt/ros2_robots/src
RUN git clone --depth 1 -b foxy-devel   https://github.com/jackal/jackal.git \
 && git clone --depth 1 -b foxy-devel   https://github.com/jackal/jackal_simulator.git \
 && git clone --depth 1 -b humble-devel https://github.com/husky/husky.git \
 && git clone --depth 1 -b humble       https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation.git \
  && grep -rl "spawner.py" jackal jackal_simulator --include=*.py | xargs -r sed -i 's/spawner\.py/spawner/g' \
 && find jackal/jackal_description/urdf -type f \( -name '*.xacro' -o -name '*.urdf' -o -name '*.gazebo' \) -exec perl -0pi -e 's/<!--.*?-->//gs' {} + \
 && sed -i "s#'/usr/share/gazebo-11/models/:'#':/usr/share/gazebo-11/models/:'#" jackal_simulator/jackal_gazebo/launch/gazebo.launch.py husky/husky_gazebo/launch/gazebo.launch.py \
 && cd /opt/ros2_robots \
 && apt-get -o Acquire::Retries=5 update \
 && rosdep init && rosdep update \
 && . /opt/ros/humble/setup.sh \
 && rosdep install --from-paths src --ignore-src -r -y \
 && colcon build \
 && rm -rf build log /var/lib/apt/lists/*

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
 && echo "source /opt/ros2_robots/install/setup.bash" >> /root/.bashrc

WORKDIR /main

CMD ["bash"]
