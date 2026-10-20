#!/usr/bin/env bash
set -e

PROJECT_DIR="/home/ubuntu/Documents/Grad/NU-CE27-Grad-Project"
BENCH_DIR="${PROJECT_DIR}/Sim/ros_ws/log/nav2_benchmark"
RESULTS_DIR="${BENCH_DIR}/stress_results"
mkdir -p "${RESULTS_DIR}"

DOCKER_IMAGE="sotirusama/gzclassic:devvv"

# Function to run benchmark for a specific controller
run_benchmark() {
    local CONTROLLER="$1"
    local ROBOT="${2:-husky}"
    local DURATION="${3:-45}"
    local LAPS="${4:-4}"
    local DOMAIN="${5:-10}"

    echo ""
    echo "========================================================================"
    echo " [STARTING] Phase: ${CONTROLLER^^} on ${ROBOT^^} (Laps: ${LAPS}, Domain: ${DOMAIN})"
    echo "========================================================================"

    # Ensure no lingering containers or gzserver
    docker rm -f "nav2_bench_${CONTROLLER}" 2>/dev/null || true
    pkill -9 -f gzserver 2>/dev/null || true

    docker run --name "nav2_bench_${CONTROLLER}" --rm --net=host --privileged --gpus all \
        --env="ROS_DOMAIN_ID=${DOMAIN}" \
        --env="ROS_LOCALHOST_ONLY=0" \
        --env="FASTRTPS_DEFAULT_PROFILES_FILE=/main/ros_ws/install/warehouse_husky/share/warehouse_husky/config/fastdds_udp.xml" \
        --volume="${PROJECT_DIR}/Sim/models:/main/models:rw" \
        --volume="${PROJECT_DIR}/Sim/worlds:/main/worlds:rw" \
        --volume="${PROJECT_DIR}/Sim/ros_ws:/main/ros_ws:rw" \
        --volume="${PROJECT_DIR}/maps:/main/maps:rw" \
        --volume="${PROJECT_DIR}/scripts:/main/scripts:rw" \
        "${DOCKER_IMAGE}" \
        bash -c "export GUI=false && export RVIZ=false && export ROS_DOMAIN_ID=${DOMAIN} && /main/ros_ws/log/nav2_benchmark/run_stress_suite.sh ${CONTROLLER} ${ROBOT} ${DURATION} ${LAPS}"

    echo "=== Completed Phase: ${CONTROLLER^^} ==="
    sleep 10
}

# 1. Run MPPI
run_benchmark "mppi" "husky" 45 4 11

# 2. Run RPP
run_benchmark "rpp" "husky" 50 4 22

# 3. Print Final Benchmark Report
echo ""
echo "========================================================================"
echo " FINAL NAV2 LOCAL PLANNER COMPARISON REPORT (MPPI vs RPP)"
echo "========================================================================"
python3 "${PROJECT_DIR}/scripts/generate_benchmark_report.py"


