#!/usr/bin/env bash
# Start Botforge rosbridge + nav2_mission_planner companion services.
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/env.sh"

# Mission Planner clients on the LAN need non-localhost DDS + open :9090
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

exec ros2 launch nav2_mission_planner nav2_mission_planner.launch.py \
  "${@}"
