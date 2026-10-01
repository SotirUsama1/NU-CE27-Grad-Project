#!/usr/bin/env bash
# Builds the Gazebo plugins inside the simulation image and installs them where the
# models expect them (/workspace/models/<model>/plugins inside the container).
set -e
SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." &> /dev/null && pwd)"

docker run --rm \
    --user "$(id -u):$(id -g)" \
    --volume="${SIM_DIR}/plugins:/src:ro" \
    --volume="${SIM_DIR}/models:/workspace/models:rw" \
    sotirusama/gzclassic:latest \
    bash -c 'set -e
             source /opt/ros/humble/setup.bash
             build() {  # build <plugin dir> <library> <model>...
                 mkdir -p /tmp/build/$1 && cd /tmp/build/$1
                 cmake /src/$1 -DCMAKE_BUILD_TYPE=Release > /dev/null
                 make -j"$(nproc)"
                 for model in "${@:3}"; do
                     mkdir -p /workspace/models/$model/plugins
                     cp $2 /workspace/models/$model/plugins/
                 done
             }
             build conveyor_belt libconveyor_belt.so dock_conveyor
             build Custom_Robot libcustom_robot.so picker_robot transport_robot'
echo "[INFO] Installed the plugins into Sim/models/{dock_conveyor,picker_robot,transport_robot}/plugins"
