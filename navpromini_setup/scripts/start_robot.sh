#!/usr/bin/env bash
# Always-on hardware bringup (lidar, odom, micro-ROS) — separate from Wi‑Fi setup.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f /opt/navpro/scripts/env.sh ]]; then
  # shellcheck disable=SC1091
  source /opt/navpro/scripts/env.sh
else
  # shellcheck disable=SC1091
  source "${SCRIPT_DIR}/env.sh"
fi

exec ros2 launch navpromini_setup robot_bringup.launch.py \
  start_slam:=false \
  start_nav:=false
