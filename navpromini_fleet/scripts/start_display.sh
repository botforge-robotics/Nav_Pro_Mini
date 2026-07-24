#!/usr/bin/env bash
# OLED / LED status via micro-ROS topics.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f /opt/navpro/scripts/env.sh ]]; then
  # shellcheck disable=SC1091
  source /opt/navpro/scripts/env.sh
else
  # shellcheck disable=SC1091
  source "${SCRIPT_DIR}/env.sh"
fi

STATE="${NAVPRO_DISPLAY_STATE:-}"
AP_SSID="${NAVPRO_AP_SSID:-}"
AP_PASSWORD="${NAVPRO_AP_PASSWORD:-navprosetup}"
NAME="${NAVPRO_ROBOT_NAME:-}"

CFG="${NAVPRO_FLEET_YAML:-/etc/navpro/fleet.yaml}"
if [[ -z "${STATE}" ]]; then
  if [[ ! -f "${CFG}" ]]; then
    STATE=setup
    if [[ -f /run/navpro/display_state ]]; then
      STATE="$(head -n1 /run/navpro/display_state | tr -d '\r')"
      AP_SSID="$(sed -n '2p' /run/navpro/display_state | tr -d '\r')"
      AP_PASSWORD="$(sed -n '3p' /run/navpro/display_state | tr -d '\r')"
      AP_PASSWORD="${AP_PASSWORD:-navprosetup}"
    fi
  else
    STATE=need_map
    NAME="$(python3 - <<PY
import yaml
from pathlib import Path
d=yaml.safe_load(Path("${CFG}").read_text()) or {}
print(d.get("name") or "")
PY
)"
  fi
fi

export NAVPRO_DISPLAY_STATE="${STATE}"
export NAVPRO_AP_SSID="${AP_SSID}"
export NAVPRO_AP_PASSWORD="${AP_PASSWORD:-navprosetup}"
export NAVPRO_ROBOT_NAME="${NAME}"

exec ros2 launch navpromini_fleet display.launch.py \
  state:="${STATE}" \
  robot_name:="${NAME}" \
  ap_ssid:="${AP_SSID}"
