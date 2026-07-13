#!/usr/bin/env bash
# Exit immediately if a command exits with a non-zero status
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"

# 1. Ensure local directories exist on the host to prevent Docker from creating them as root
mkdir -p "${SCRIPT_DIR}/../models"
mkdir -p "${SCRIPT_DIR}/../worlds"

# 2. WSLg note: unlike a bare X11/VcXsrv setup, WSLg's X11/Wayland sockets under
#    /mnt/wslg and /tmp/.X11-unix are already authorized for containers on this host —
#    there is no xhost step and no .Xauthority mount needed here. If you ever move this
#    script to a real Linux host with a classic X server, you'll need to reintroduce
#    xhost +local:docker and an .Xauthority mount there.

# 3. Sanity check we're actually inside WSL before doing GPU/WSLg-specific setup
if ! grep -qi microsoft /proc/version 2>/dev/null; then
    echo "[WARNING] This does not look like a WSL2 environment. The GPU/WSLg flags below"
    echo "          are WSL-specific and will likely fail or no-op elsewhere."
fi

# 4. Handle default argument if none are supplied to the script
if [ $# -eq 0 ]; then
    echo "[INFO] No simulation arguments provided. Defaulting to an empty world."
    SET_COMMAND="worlds/empty.world"
else
    # Forward all arguments passed to this script directly to the inner command
    SET_COMMAND="$@"
fi

echo "[INFO] Launching Gazebo Classic Docker Container (WSLg + NVIDIA GPU passthrough)..."
echo "[INFO] Passing arguments to gazebo: $SET_COMMAND"

# 5. Execute the Docker command
docker run -it --rm \
    --gpus all \
    --device=/dev/dxg \
    --env="DISPLAY=${DISPLAY}" \
    --env="WAYLAND_DISPLAY=${WAYLAND_DISPLAY}" \
    --env="XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR}" \
    --env="PULSE_SERVER=${PULSE_SERVER}" \
    --env="LD_LIBRARY_PATH=/usr/lib/wsl/lib" \
    --env="MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/mnt/wslg:/mnt/wslg" \
    --volume="/usr/lib/wsl:/usr/lib/wsl" \
    --volume="${SCRIPT_DIR}/../models:/workspace/my_models:ro" \
    --volume="${SCRIPT_DIR}/../worlds:/workspace/my_worlds:ro" \
    sotirusama/gzclassic:latest \
    bash -c 'source /usr/share/gazebo/setup.sh && \
             export GAZEBO_MODEL_PATH="$GAZEBO_MODEL_PATH:/workspace/my_models" && \
             export GAZEBO_RESOURCE_PATH="$GAZEBO_RESOURCE_PATH:/workspace/my_worlds" && \
             exec gazebo --verbose "$@"' -- ${SET_COMMAND}
