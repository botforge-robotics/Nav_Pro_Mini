#!/usr/bin/env bash
# Start Botforge rosbridge/rosapi + launch_manager.
#
# Relocated from nav2_mission_planner: the app talks to rosbridge/rosapi
# directly and independently of navpromini_sdk (see
# navpromini-sdk-is-optional-not-gateway in project memory), so those stay;
# launch_manager moved into navpromini_launch_manager, camera streaming and
# tf2_buffer_server were dropped (unused).
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/env.sh"

# Mission Planner clients on the LAN need non-localhost DDS + open :9090
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

exec ros2 launch navpromini_launch_manager bringup.launch.py \
  "${@}"
