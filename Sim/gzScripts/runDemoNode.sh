#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 || ( "$1" != listener && "$1" != talker ) ]]; then
  echo "Usage: $0 listener|talker" >&2
  exit 2
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
CONFIG="${CYCLONEDDS_CONFIG:-$SCRIPT_DIR/cyclonedds.local.xml}"
IMAGE="${ROS_IMAGE:-sotirusama/gzclassic:cyclonedds}"
DOMAIN="${ROS_DOMAIN_ID:-17}"

[[ -r "$CONFIG" ]] || {
  echo "CycloneDDS config is not readable: $CONFIG" >&2
  exit 1
}

if [[ "$1" == listener ]]; then
  NODE=listener
else
  NODE=talker
fi

docker run --rm -it \
  --network host \
  --mount "type=bind,src=$(realpath "$CONFIG"),dst=/etc/cyclonedds/cyclonedds.xml,readonly" \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e ROS_LOCALHOST_ONLY=0 \
  -e ROS_DOMAIN_ID="$DOMAIN" \
  -e CYCLONEDDS_URI=file:///etc/cyclonedds/cyclonedds.xml \
  "$IMAGE" \
  bash -lc "source /opt/ros/humble/setup.bash && ros2 run demo_nodes_cpp $NODE"