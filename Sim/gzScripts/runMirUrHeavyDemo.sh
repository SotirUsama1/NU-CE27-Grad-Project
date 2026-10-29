#!/usr/bin/env bash
# Two MiR 250 + UR10e robots. The six-cup robot carries the 2 kg (top shelf) and the 4 kg
# onto the conveyor. The twelve-cup robot carries the 8 kg onto the conveyor.
# Restart the simulation first (./Sim/gzScripts/runMirUrWarehouse.sh), then run this.
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"

docker exec -i "${CONTAINER:-mir_ur_sim}" bash -c 'source /opt/ros/humble/setup.bash && \
                                                   source /usr/share/gazebo/setup.sh && \
                                                   python3 -u -' < "${SCRIPT_DIR}/../tools/demo_mir_ur_heavy_boxes.py"
