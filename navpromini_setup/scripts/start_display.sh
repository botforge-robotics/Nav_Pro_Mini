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

CFG="${NAVPRO_ROBOT_YAML:-/etc/navpro/robot.yaml}"
ALT_CFG="${NAVPRO_ALT_ROBOT_YAML:-/etc/navpro/fleet.yaml}"
HINT=/run/navpro/display_state

if [[ -f "${CFG}" ]] || [[ -f "${ALT_CFG}" ]]; then
  USE_CFG="${CFG}"
  [[ -f "${CFG}" ]] || USE_CFG="${ALT_CFG}"
  STATE="${STATE:-ready}"
  if [[ "${STATE}" == "setup" ]]; then
    STATE=ready
  fi
  NAME="$(python3 - <<PY
import yaml
from pathlib import Path
d=yaml.safe_load(Path("${USE_CFG}").read_text()) or {}
robot=d.get("robot") if isinstance(d.get("robot"), dict) else {}
print(d.get("name") or robot.get("name") or "")
PY
)"
elif [[ -f "${HINT}" ]]; then
  HINT_STATE="$(head -n1 "${HINT}" | tr -d '\r')"
  HINT_L2="$(sed -n '2p' "${HINT}" | tr -d '\r')"
  HINT_L3="$(sed -n '3p' "${HINT}" | tr -d '\r')"
  STATE="${STATE:-${HINT_STATE:-setup}}"
  case "${STATE}" in
    setup)
      AP_SSID="${HINT_L2:-${AP_SSID}}"
      AP_PASSWORD="${HINT_L3:-${AP_PASSWORD:-navprosetup}}"
      ;;
    *)
      NAME="${HINT_L2:-${NAME}}"
      ;;
  esac
else
  STATE="${STATE:-setup}"
fi

export NAVPRO_DISPLAY_STATE="${STATE}"
export NAVPRO_AP_SSID="${AP_SSID}"
export NAVPRO_AP_PASSWORD="${AP_PASSWORD:-navprosetup}"
export NAVPRO_ROBOT_NAME="${NAME}"

LAUNCH_ARGS=( "state:=${STATE}" )
[[ -n "${NAME}" ]] && LAUNCH_ARGS+=( "robot_name:=${NAME}" )
[[ -n "${AP_SSID}" ]] && LAUNCH_ARGS+=( "ap_ssid:=${AP_SSID}" )
[[ -n "${AP_PASSWORD}" ]] && LAUNCH_ARGS+=( "ap_password:=${AP_PASSWORD}" )

exec ros2 launch navpromini_setup display.launch.py "${LAUNCH_ARGS[@]}"
