#!/usr/bin/env bash
# Start the NavProMini SDK HTTP/WebSocket API server.
#
# Mirrors start_mission_planner.sh: env.sh does the ROS and workspace sourcing
# for every navpro unit, so this script only has to say what is different about
# this one. Hardcoding the setup.bash paths here would silently skip the
# uros_ws overlay and the NAVPRO_WS override that env.sh handles.
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/env.sh"

# SDK clients are on the LAN, so DDS must not be localhost-only — the server
# talks to nodes in other processes and answers requests from other machines.
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

exec ros2 launch navpromini_sdk sdk.launch.py \
  "${@}"
