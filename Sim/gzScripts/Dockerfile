FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# 1. Set up locale (Strictly required for ROS 2)
RUN apt-get update && apt-get install -y locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# 2. Install prerequisites and X11 graphics libraries
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libxext6 \
    && rm -rf /var/lib/apt/lists/*

# 3. Add the ROS 2 apt repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 4. Install ROS 2 Humble, Gazebo 11, and the ROS-Gazebo Bridge
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-humble-gazebo-ros-pkgs \
    && rm -rf /var/lib/apt/lists/*

# 5. Automatically source ROS 2 and Gazebo for interactive bash sessions
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /usr/share/gazebo/setup.sh" >> /root/.bashrc

CMD ["gazebo"]
