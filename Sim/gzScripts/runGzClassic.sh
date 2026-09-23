#!/usr/bin/env bash
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"

# 1. Ensure local directories exist on the host
mkdir -p "${SCRIPT_DIR}/../models"
mkdir -p "${SCRIPT_DIR}/../worlds"

# 2. Authorize the container to use the host X server
xhost +local:docker >/dev/null 2>&1

# 3. Handle default argument
if [ $# -eq 0 ]; then
    echo "[INFO] No simulation arguments provided. Defaulting to an empty world."
    SET_COMMAND="worlds/empty.world"
else
    SET_COMMAND="$@"
fi

echo "[INFO] Launching Gazebo Classic Docker Container (native Ubuntu + NVIDIA GPU passthrough)..."
echo "[INFO] Passing arguments to gazebo: $SET_COMMAND"

# 4. Execute the Docker command
docker run -it --rm \
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
    bash -c 'source /usr/share/gazebo/setup.sh && \
             export GAZEBO_MODEL_PATH="$GAZEBO_MODEL_PATH:/workspace/models" && \
            export GAZEBO_RESOURCE_PATH="$GAZEBO_RESOURCE_PATH:/workspace" && \
             exec gazebo --verbose "$@"' -- ${SET_COMMAND}
