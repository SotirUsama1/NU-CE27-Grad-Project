#!/usr/bin/env bash
# MiR 250 + UR10e mobile manipulator in Our_Structured_Warehouse (Gazebo Classic + ROS 2 Humble).
# Uses the separate test image (the main sotirusama/gzclassic image is unchanged). Build it once:
#   docker build -f Sim/gzScripts/Dockerfile.mir_ur -t gzclassic-mir-ur:latest Sim/gzScripts
# Usage: ./Sim/gzScripts/runMirUrWarehouse.sh [--rebuild] [launch args, e.g. ur_type:=ur16e x:=0.06]
# Talk to the robot from another terminal:
#   docker exec -it mir_ur_sim bash    (the workspace is sourced automatically)
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
SIM_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
IMAGE=gzclassic-mir-ur:latest
PACKAGES=(mir_ur_warehouse mir_description ur_description realsense2_description
          realsense_gazebo_plugin robotiq_description ira_laser_tools)

if [ "$1" = "--rebuild" ]; then
    rm -rf "${SIM_DIR}/ros2_ws/build" "${SIM_DIR}/ros2_ws/install" "${SIM_DIR}/ros2_ws/log"
    shift
fi

if [ ! -f "${SIM_DIR}/ros2_ws/install/setup.bash" ]; then
    echo "[INFO] Building the ROS 2 workspace (first run only)..."
    docker run --rm --user "$(id -u):$(id -g)" --env HOME=/tmp \
        --volume="${SIM_DIR}/ros2_ws:/workspace/ros2_ws:rw" \
        "${IMAGE}" \
        bash -c 'source /opt/ros/humble/setup.bash && cd /workspace/ros2_ws && \
                 colcon build --symlink-install --packages-up-to '"${PACKAGES[*]}"
fi

xhost +local:docker >/dev/null 2>&1

if nvidia-smi >/dev/null 2>&1; then
    GPU_ARGS=(--gpus all
              --env="NVIDIA_VISIBLE_DEVICES=all"
              --env="NVIDIA_DRIVER_CAPABILITIES=all"
              --env="__NV_PRIME_RENDER_OFFLOAD=1"
              --env="__GLX_VENDOR_LIBRARY_NAME=nvidia")
    echo "[INFO] Rendering on the NVIDIA GPU"
elif [ -d /dev/dri ]; then
    GPU_ARGS=(--device=/dev/dri)
    echo "[WARNING] NVIDIA driver not available, rendering on the integrated GPU"
else
    GPU_ARGS=()
    echo "[WARNING] No GPU available, using software rendering (slow)"
fi

if [ -n "$(docker ps -aq --filter name=^/mir_ur_sim$)" ]; then
    echo "[WARNING] A mir_ur_sim container already exists, replacing it"
    docker rm -f mir_ur_sim >/dev/null
fi

echo "[INFO] Launching MiR + UR in the warehouse (container name: mir_ur_sim)"

docker run -it --rm \
    --name mir_ur_sim \
    "${GPU_ARGS[@]}" \
    --env="DISPLAY=${DISPLAY}" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="PYTHONDONTWRITEBYTECODE=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="${HOME}/.Xauthority:/root/.Xauthority:rw" \
    --volume="${SIM_DIR}/models:/workspace/models:rw" \
    --volume="${SIM_DIR}/worlds:/workspace/worlds:rw" \
    --volume="${SIM_DIR}/ros2_ws:/workspace/ros2_ws:rw" \
    "${IMAGE}" \
    bash -c 'source /opt/ros/humble/setup.bash && \
             source /usr/share/gazebo/setup.sh && \
             source /workspace/ros2_ws/install/setup.bash && \
             export GAZEBO_MODEL_PATH="$GAZEBO_MODEL_PATH:/workspace/models" && \
             export GAZEBO_RESOURCE_PATH="$GAZEBO_RESOURCE_PATH:/workspace" && \
             exec ros2 launch mir_ur_warehouse mir_ur_warehouse.launch.py "$@"' -- "$@"
