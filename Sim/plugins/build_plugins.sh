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
             mkdir -p /tmp/build/conveyor_belt /workspace/models/dock_conveyor/plugins
             cd /tmp/build/conveyor_belt
             cmake /src/conveyor_belt -DCMAKE_BUILD_TYPE=Release > /dev/null
             make -j"$(nproc)"
             cp libconveyor_belt.so /workspace/models/dock_conveyor/plugins/'
echo "[INFO] Installed Sim/models/dock_conveyor/plugins/libconveyor_belt.so"
