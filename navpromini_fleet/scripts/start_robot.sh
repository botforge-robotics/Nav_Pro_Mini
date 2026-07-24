#!/usr/bin/env bash
# Always-on hardware bringup (no SLAM / no Nav2) — Phase A.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Prefer installed share path if present
if [[ -f /opt/navpro/scripts/env.sh ]]; then
  # shellcheck disable=SC1091
  source /opt/navpro/scripts/env.sh
else
  # shellcheck disable=SC1091
  source "${SCRIPT_DIR}/env.sh"
fi

exec ros2 launch navpromini_fleet robot_with_mux.launch.py \
  start_slam:=false \
  start_nav:=false
