#!/usr/bin/env bash
# Shelf -> picker -> transport robot -> conveyor -> truck demo.
# Start the simulation first (./Sim/gzScripts/runCustomRobotWarehouse.sh), then run this in another terminal.
# The demo resets the robots and the box it uses, so it can be run again.
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"

docker exec -i warehouse_sim bash -c 'source /opt/ros/humble/setup.bash && \
                                      source /usr/share/gazebo/setup.sh && \
                                      python3 -u -' < "${SCRIPT_DIR}/../tools/demo_pick_to_conveyor.py"
