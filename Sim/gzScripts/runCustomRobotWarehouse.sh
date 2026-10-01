#!/usr/bin/env bash
# Custom_Robot_Warehouse: Gazebo started with ROS 2 so the robot plugins can load.
# Build the plugins once first: ./Sim/plugins/build_plugins.sh
# Demo from another terminal: ./Sim/gzScripts/runCustomRobotDemo.sh
# Talk to the robots from another terminal:
#   docker exec -it warehouse_sim bash    (then: source /opt/ros/humble/setup.bash)
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
WORLD="${1:-worlds/Custom_Robot_Warehouse/Custom_Robot_Warehouse.world}"

xhost +local:docker >/dev/null 2>&1

echo "[INFO] Launching ${WORLD} with ROS 2 (container name: warehouse_sim)"

docker run -it --rm \
    --name warehouse_sim \
    --gpus all \
    --env="DISPLAY=${DISPLAY}" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="NVIDIA_VISIBLE_DEVICES=all" \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --env="__NV_PRIME_RENDER_OFFLOAD=1" \
    --env="__GLX_VENDOR_LIBRARY_NAME=nvidia" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="${HOME}/.Xauthority:/root/.Xauthority:rw" \
    --volume="${SCRIPT_DIR}/../models:/workspace/models:rw" \
    --volume="${SCRIPT_DIR}/../worlds:/workspace/worlds:rw" \
    sotirusama/gzclassic:latest \
    bash -c 'source /opt/ros/humble/setup.bash && \
             source /usr/share/gazebo/setup.sh && \
             export GAZEBO_MODEL_PATH="$GAZEBO_MODEL_PATH:/workspace/models" && \
             export GAZEBO_RESOURCE_PATH="$GAZEBO_RESOURCE_PATH:/workspace" && \
             export GAZEBO_PLUGIN_PATH="$GAZEBO_PLUGIN_PATH:/opt/ros/humble/lib" && \
             exec gazebo --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so "$1"' -- "${WORLD}"
