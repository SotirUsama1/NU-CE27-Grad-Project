#!/usr/bin/env bash
# Four-robot conveyor demo (sequential, one robot at a time):
#   MiR light 2 kg, Jackal 2 kg (UR5e+small), Husky 8 kg, MiR heavy 8 kg.
# Start the hetero sim first, then run this.
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
CONTAINER="${CONTAINER:-hetero_warehouse_sim}"

if ! docker ps --format '{{.Names}}' | grep -qx "${CONTAINER}"; then
  echo "[ERROR] ${CONTAINER} is not running. Start ./Sim/gzScripts/runHeteroWarehouseWorld.sh first."
  exit 1
fi

echo "[INFO] Waiting for all four arm controllers..."
docker exec "${CONTAINER}" bash -lc '
  source /opt/ros/humble/setup.bash
  source /opt/ros2_robots/install/setup.bash
  source /main/ros_ws/install/setup.bash
  source /workspace/ros2_ws/install/setup.bash
  for i in $(seq 1 120); do
    ok=1
    ros2 control list_controllers --controller-manager /mir_light/controller_manager 2>/dev/null \
      | grep -q "joint_trajectory_controller.*active" || ok=0
    ros2 control list_controllers --controller-manager /mir_ur/controller_manager 2>/dev/null \
      | grep -q "joint_trajectory_controller.*active" || ok=0
    ros2 control list_controllers --controller-manager /controller_manager 2>/dev/null \
      | grep -q "joint_trajectory_controller.*active" || ok=0
    ros2 control list_controllers --controller-manager /jackal/controller_manager 2>/dev/null \
      | grep -q "joint_trajectory_controller.*active" || ok=0
    if [ "$ok" = 1 ]; then
      echo "[INFO] All arm JTCs active"
      exit 0
    fi
    sleep 2
  done
  echo "[ERROR] Timed out waiting for arm controllers"
  exit 1
'

echo "[INFO] Starting conveyor demo"
docker exec -i "${CONTAINER}" bash -c 'source /opt/ros/humble/setup.bash && \
                                                   source /usr/share/gazebo/setup.sh && \
                                                   source /opt/ros2_robots/install/setup.bash && \
                                                   source /main/ros_ws/install/setup.bash && \
                                                   source /workspace/ros2_ws/install/setup.bash && \
                                                   python3 -u -' < "${SCRIPT_DIR}/../tools/demo_hetero_box_delivery.py"
