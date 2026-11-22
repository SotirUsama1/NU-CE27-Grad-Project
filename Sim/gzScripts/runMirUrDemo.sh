#!/usr/bin/env bash
# MiR 250 + UR10e demo: shelf -> suction pick -> LOAD station -> conveyor -> truck.
# Start the simulation first (./Sim/gzScripts/runMirUrWarehouse.sh), then run this in another terminal.
# The demo resets the robot and the box it uses, so it can be run again.
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"

docker exec -i "${CONTAINER:-mir_ur_sim}" bash -c 'source /opt/ros/humble/setup.bash && \
                                                   source /usr/share/gazebo/setup.sh && \
                                                   python3 -u -' < "${SCRIPT_DIR}/../tools/demo_mir_ur_pick_to_conveyor.py"
