#!/usr/bin/env bash
# Heterogeneous warehouse world (branch: hetero-warehouse-world)
#   Our_Structured_Warehouse + 2x MiR/UR with suction grippers
#   Same world Husky and Jackal already use on sotirusama/gzclassic:devvv
#
# Build the image once:
#   docker build -f Sim/gzScripts/Dockerfile.hetero -t gzclassic-hetero:latest Sim/gzScripts
#
# Usage:
#   ./Sim/gzScripts/runHeteroWarehouseWorld.sh [--rebuild] [launch args...]
#
# Then in other terminals (same container):
#   docker exec -it hetero_warehouse_sim bash
#   ros2 launch warehouse_husky husky.launch.py start_gazebo:=false
#   ros2 launch warehouse_husky jackal_spawn.launch.py
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
SIM_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_DIR="$(cd "${SIM_DIR}/.." && pwd)"
IMAGE=gzclassic-hetero:latest
WORLD=/workspace/worlds/Our_Structured_Warehouse/Our_Structured_Warehouse.world
MIR_PACKAGES=(mir_ur_warehouse mir_description ur_description realsense2_description
              realsense_gazebo_plugin robotiq_description ira_laser_tools)

NOFILE_LIMIT="${NOFILE_LIMIT:-65536}"
if ulimit -n "${NOFILE_LIMIT}" 2>/dev/null; then
    echo "[INFO] Host nofile limit: $(ulimit -n)"
else
    echo "[WARNING] Could not raise host nofile to ${NOFILE_LIMIT} (current: $(ulimit -n))"
fi

if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
    echo "[INFO] Building ${IMAGE} (first run)..."
    docker build -f "${SCRIPT_DIR}/Dockerfile.hetero" -t "${IMAGE}" "${SCRIPT_DIR}"
fi

if [ "$1" = "--rebuild" ]; then
    rm -rf "${SIM_DIR}/ros2_ws/build" "${SIM_DIR}/ros2_ws/install" "${SIM_DIR}/ros2_ws/log"
    rm -rf "${SIM_DIR}/ros_ws/build" "${SIM_DIR}/ros_ws/install" "${SIM_DIR}/ros_ws/log"
    shift
fi

NEED_MIR=0
if [ ! -f "${SIM_DIR}/ros2_ws/install/setup.bash" ] || \
   [ ! -d "${SIM_DIR}/ros2_ws/install/mir_ur_warehouse" ]; then
    NEED_MIR=1
fi
if [ "${NEED_MIR}" = "1" ]; then
    echo "[INFO] Building MiR workspace (Sim/ros2_ws)..."
    docker run --rm --user "$(id -u):$(id -g)" --env HOME=/tmp \
        --volume="${SIM_DIR}/ros2_ws:/workspace/ros2_ws:rw" \
        "${IMAGE}" \
        bash -c 'source /opt/ros/humble/setup.bash && cd /workspace/ros2_ws && \
                 colcon build --symlink-install --packages-up-to '"${MIR_PACKAGES[*]}"
fi

echo "[INFO] Building warehouse_husky (Sim/ros_ws)..."
docker run --rm --user "$(id -u):$(id -g)" --env HOME=/tmp \
    --volume="${SIM_DIR}/ros_ws:/main/ros_ws:rw" \
    "${IMAGE}" \
    bash -c 'source /opt/ros/humble/setup.bash && \
             source /opt/ros2_robots/install/setup.bash && \
             cd /main/ros_ws && colcon build --symlink-install --packages-select warehouse_husky'

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

for name in hetero_warehouse_sim mir_ur_sim; do
    if [ -n "$(docker ps -aq --filter name=^/${name}$)" ]; then
        echo "[WARNING] Replacing existing container: ${name}"
        docker rm -f "${name}" >/dev/null
    fi
done

echo "[INFO] Launching hetero warehouse (2x MiR suction + shared Husky/Jackal world)"
echo "[INFO] Container: hetero_warehouse_sim"

docker run -it --rm \
    --name hetero_warehouse_sim \
    --ulimit "nofile=${NOFILE_LIMIT}:${NOFILE_LIMIT}" \
    --net=host \
    "${GPU_ARGS[@]}" \
    --env="DISPLAY=${DISPLAY}" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="PYTHONDONTWRITEBYTECODE=1" \
    --env="NOFILE_LIMIT=${NOFILE_LIMIT}" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="${HOME}/.Xauthority:/root/.Xauthority:rw" \
    --volume="${SIM_DIR}/models:/workspace/models:rw" \
    --volume="${SIM_DIR}/worlds:/workspace/worlds:rw" \
    --volume="${SIM_DIR}/ros2_ws:/workspace/ros2_ws:rw" \
    --volume="${SIM_DIR}/ros_ws:/main/ros_ws:rw" \
    --volume="${REPO_DIR}/maps:/main/maps:rw" \
    "${IMAGE}" \
    bash -c 'ulimit -n "${NOFILE_LIMIT:-65536}" 2>/dev/null || true
             source /opt/ros/humble/setup.bash
             source /usr/share/gazebo/setup.sh
             source /opt/ros2_robots/install/setup.bash
             source /main/ros_ws/install/setup.bash
             source /workspace/ros2_ws/install/setup.bash
             export GAZEBO_MODEL_PATH="$GAZEBO_MODEL_PATH:/workspace/models"
             export GAZEBO_RESOURCE_PATH="$GAZEBO_RESOURCE_PATH:/workspace"
             export GAZEBO_MODEL_DATABASE_URI=
             exec ros2 launch mir_ur_warehouse hetero_warehouse.launch.py \
               world:="'"${WORLD}"'" "$@"' -- "$@"
