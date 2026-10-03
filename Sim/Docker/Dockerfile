FROM sotirusama/gzbase:latest

RUN apt-get -o Acquire::Retries=5 install -y --no-install-recommends \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-cartographer-ros

RUN apt-get autoremove -y \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /main

CMD ["bash"]