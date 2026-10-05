#!/usr/bin/env bash
# generate_2d_map.sh — Automates ROS2 Nav2 map saving for Structured_Warehouse
#
# Usage:
#   Interactive mode:      bash scripts/generate_2d_map.sh
#   Non-interactive mode:  bash scripts/generate_2d_map.sh --non-interactive

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
MAPS_DIR="$PROJECT_ROOT/maps"
MAP_NAME="our_structured_warehouse_map"
NON_INTERACTIVE=false

for arg in "$@"; do
    case "$arg" in
        --non-interactive|--ci)
            NON_INTERACTIVE=true
            shift
            ;;
    esac
done

mkdir -p "$MAPS_DIR"

echo "============================================"
echo "  2D Occupancy Grid Map Generator"
echo "  Target: $MAP_NAME"
echo "============================================"

if [ "$NON_INTERACTIVE" = false ]; then
    echo ""
    echo "Required services running in separate terminals (inside Docker):"
    echo "  Terminal 1 (Gazebo Simulation):"
    echo "    ros2 launch warehouse_husky husky.launch.py world_path:=Our_Structured_Warehouse/Our_Structured_Warehouse.world"
    echo ""
    echo "  Terminal 2 (SLAM Toolbox):"
    echo "    ros2 launch slam_toolbox online_async_launch.py params_file:=/main/ros_ws/src/warehouse_husky/config/mapper_params_online_async.yaml use_sim_time:=true"
    echo ""
    echo "  Terminal 3 (Robot Teleoperation):"
    echo "    ros2 run teleop_twist_keyboard teleop_twist_keyboard"
    echo "============================================"
    read -rp "Press [ENTER] when SLAM exploration is complete..."
else
    echo "Running in non-interactive mode. Proceeding to save map..."
fi

echo ""
echo "Saving occupancy grid to $MAPS_DIR/$MAP_NAME ..."
ros2 run nav2_map_server map_saver_cli -f "$MAPS_DIR/$MAP_NAME"

# Deterministic verification of generated files
PGM_FILE="$MAPS_DIR/${MAP_NAME}.pgm"
YAML_FILE="$MAPS_DIR/${MAP_NAME}.yaml"

if [[ -f "$PGM_FILE" && -s "$PGM_FILE" && -f "$YAML_FILE" && -s "$YAML_FILE" ]]; then
    echo ""
    echo "✅ Map successfully generated and verified!"
    echo "   Image:    $PGM_FILE ($(stat -c%s "$PGM_FILE") bytes)"
    echo "   Metadata: $YAML_FILE"
    exit 0
else
    echo ""
    echo "❌ ERROR: Map saving failed or output files are empty!"
    echo "   Expected PGM:  $PGM_FILE"
    echo "   Expected YAML: $YAML_FILE"
    exit 1
fi
