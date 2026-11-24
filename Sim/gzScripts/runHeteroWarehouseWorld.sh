#!/usr/bin/env bash
# Heterogeneous warehouse world (branch: hetero-warehouse-world)
#   Our_Structured_Warehouse + 2x MiR/UR + Husky/Jackal with UR+suction
#
# Build the image once:
#   docker build -f Sim/gzScripts/Dockerfile.hetero -t gzclassic-hetero:latest Sim/gzScripts
#
# Usage:
#   ./Sim/gzScripts/runHeteroWarehouseWorld.sh [--rebuild] [launch args...]
# Spawns: Gazebo + 2x MiR (suction) + Husky (UR10e+large) + Jackal (UR5e+small).
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
SIM_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_DIR="$(cd "${SIM_DIR}/.." && pwd)"
IMAGE=gzclassic-hetero:latest
WORLD=/workspace/worlds/Our_Structured_Warehouse/Our_Structured_Warehouse.world
# Open-floor poses — clear of zone-A racks (A1–A4 near x≈6,y≈0) and MiR docks.
HUSKY_X="${HUSKY_X:--1.5}"
HUSKY_Y="${HUSKY_Y:-2.0}"
HUSKY_YAW="${HUSKY_YAW:-0.0}"
JACKAL_X="${JACKAL_X:-0.0}"
JACKAL_Y="${JACKAL_Y:-8.0}"
JACKAL_YAW="${JACKAL_YAW:-0.0}"
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

# Stale forklift_robot develop-install (from stashed WIP) poisons PYTHONPATH/ament
# even though the source package is not on this branch.
if [ -d "${SIM_DIR}/ros2_ws/install/forklift_robot" ] || \
   [ -d "${SIM_DIR}/ros2_ws/build/forklift_robot" ]; then
    echo "[INFO] Removing stale forklift_robot install/build (not on this branch)"
    rm -rf "${SIM_DIR}/ros2_ws/install/forklift_robot" \
           "${SIM_DIR}/ros2_ws/build/forklift_robot"
fi

# Always rebuild mir_ur_warehouse so new arm xacros / controller YAMLs are installed.
echo "[INFO] Building MiR workspace (Sim/ros2_ws)..."
docker run --rm --user "$(id -u):$(id -g)" --env HOME=/tmp \
    --volume="${SIM_DIR}/ros2_ws:/workspace/ros2_ws:rw" \
    "${IMAGE}" \
    bash -c 'source /opt/ros/humble/setup.bash && cd /workspace/ros2_ws && \
             colcon build --symlink-install --packages-up-to '"${MIR_PACKAGES[*]}"

echo "[INFO] Building warehouse_husky (Sim/ros_ws)..."
docker run --rm --user "$(id -u):$(id -g)" --env HOME=/tmp \
    --volume="${SIM_DIR}/ros_ws:/main/ros_ws:rw" \
    --volume="${SIM_DIR}/ros2_ws:/workspace/ros2_ws:rw" \
    "${IMAGE}" \
    bash -c 'source /opt/ros/humble/setup.bash && \
             source /opt/ros2_robots/install/setup.bash && \
             source /workspace/ros2_ws/install/setup.bash && \
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

# Host-network Gazebo masters left from a crashed run make the next gzserver
# die with "Header is empty" / vector::_M_default_append. Match runMirUrWarehouse
# (no --net=host) and clear leftovers on the host.
if pgrep -a gzserver >/dev/null 2>&1 || pgrep -a gzclient >/dev/null 2>&1; then
    echo "[WARNING] Killing leftover host Gazebo processes"
    pkill -9 gzserver 2>/dev/null || true
    pkill -9 gzclient 2>/dev/null || true
    sleep 1
fi

echo "[INFO] Launching hetero warehouse (2x MiR + Husky + Jackal)"
echo "[INFO] Container: hetero_warehouse_sim"

# -it when a TTY is attached; plain -i otherwise (CI / agent / nohup).
DOCKER_TTY=(-i)
if [ -t 0 ] && [ -t 1 ]; then
    DOCKER_TTY=(-it)
fi

docker run "${DOCKER_TTY[@]}" --rm \
    --name hetero_warehouse_sim \
    --ulimit "nofile=${NOFILE_LIMIT}:${NOFILE_LIMIT}" \
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
             # model://husky_description|jackal_description needs the *share*
             # parents on GAZEBO_MODEL_PATH before gzserver starts, or meshes
             # resolve empty and the robots spawn invisible.
             HUSKY_SHARE="$(ros2 pkg prefix husky_description)/share"
             JACKAL_SHARE="$(ros2 pkg prefix jackal_description)/share"
             UR_SHARE="$(ros2 pkg prefix ur_description)/share"
             export GAZEBO_MODEL_PATH="$GAZEBO_MODEL_PATH:${HUSKY_SHARE}:${JACKAL_SHARE}:${UR_SHARE}:/workspace/models"
             export GAZEBO_RESOURCE_PATH="$GAZEBO_RESOURCE_PATH:/workspace"
             export GAZEBO_MODEL_DATABASE_URI=

             ros2 launch mir_ur_warehouse hetero_warehouse.launch.py \
               world:="'"${WORLD}"'" "$@" &
             MIR_PID=$!

             # Wait until both MiR arms are up (second robot spawns after the first).
             echo "[INFO] Waiting for MiR controllers before spawning Husky/Jackal..."
             for i in $(seq 1 180); do
               if ros2 control list_controllers --controller-manager /mir_light/controller_manager 2>/dev/null \
                    | grep -q joint_trajectory_controller; then
                 break
               fi
               sleep 1
             done
             sleep 5

             echo "[INFO] Spawning Husky at ('"${HUSKY_X}"', '"${HUSKY_Y}"')"
             ros2 launch warehouse_husky husky.launch.py start_gazebo:=false \
               x:="'"${HUSKY_X}"'" y:="'"${HUSKY_Y}"'" yaw:="'"${HUSKY_YAW}"'" &
             HUSKY_PID=$!

             # Let Husky claim /controller_manager before Jackal appears.
             sleep 8

             echo "[INFO] Spawning Jackal at ('"${JACKAL_X}"', '"${JACKAL_Y}"')"
             ros2 launch warehouse_husky jackal_spawn.launch.py \
               x:="'"${JACKAL_X}"'" y:="'"${JACKAL_Y}"'" yaw:="'"${JACKAL_YAW}"'" &
             JACKAL_PID=$!

             wait $MIR_PID' -- "$@"
