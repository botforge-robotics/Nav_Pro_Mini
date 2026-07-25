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
CONN_AP="navpro-setup-ap"

wifi_site_online() {
  # Site Wi‑Fi connected (not the setup AP) with an address.
  local line device dtype state conn
  while IFS= read -r line; do
    IFS=':' read -r device dtype state conn <<<"${line}"
    [[ "${dtype}" == "wifi" ]] || continue
    [[ "${state}" == "connected" ]] || continue
    [[ -n "${conn}" && "${conn}" != "${CONN_AP}" ]] || continue
    hostname -I 2>/dev/null | grep -q '[0-9]' && return 0
  done < <(nmcli -t -f DEVICE,TYPE,STATE,CONNECTION device status 2>/dev/null || true)
  return 1
}

setup_ap_up() {
  nmcli -t -f NAME,STATE connection show --active 2>/dev/null \
    | grep -q "^${CONN_AP}:activated$"
}

# --- decide initial OLED state from reality, not stale hints ---
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
elif setup_ap_up; then
  # Real hotspot is on — show setup credentials from hint if present.
  STATE="${STATE:-setup}"
  if [[ -f "${HINT}" ]]; then
    HINT_STATE="$(head -n1 "${HINT}" | tr -d '\r')"
    HINT_L2="$(sed -n '2p' "${HINT}" | tr -d '\r')"
    HINT_L3="$(sed -n '3p' "${HINT}" | tr -d '\r')"
    STATE="${HINT_STATE:-setup}"
    AP_SSID="${HINT_L2:-${AP_SSID}}"
    AP_PASSWORD="${HINT_L3:-${AP_PASSWORD:-navprosetup}}"
  fi
elif wifi_site_online; then
  # Already on Wi‑Fi — never show fake setup mode.
  STATE="${STATE:-ready}"
  if [[ "${STATE}" == "setup" ]]; then
    STATE=ready
  fi
  if [[ -f "${HINT}" ]]; then
    HINT_STATE="$(head -n1 "${HINT}" | tr -d '\r')"
    HINT_L2="$(sed -n '2p' "${HINT}" | tr -d '\r')"
    if [[ "${HINT_STATE}" != "setup" ]]; then
      STATE="${HINT_STATE:-ready}"
      NAME="${HINT_L2:-${NAME}}"
    fi
  fi
  # Clear stale setup hint so the node does not flip back.
  mkdir -p /run/navpro
  printf 'ready\n%s\n\n' "${NAME}" > /run/navpro/display_state
elif [[ -f "${HINT}" ]]; then
  HINT_STATE="$(head -n1 "${HINT}" | tr -d '\r')"
  HINT_L2="$(sed -n '2p' "${HINT}" | tr -d '\r')"
  HINT_L3="$(sed -n '3p' "${HINT}" | tr -d '\r')"
  # Stale "setup" hint with no AP → treat as boot/ready, not setup.
  if [[ "${HINT_STATE}" == "setup" ]]; then
    STATE="${STATE:-boot}"
  else
    STATE="${STATE:-${HINT_STATE:-boot}}"
    NAME="${HINT_L2:-${NAME}}"
    AP_SSID="${HINT_L2:-${AP_SSID}}"
    AP_PASSWORD="${HINT_L3:-${AP_PASSWORD:-navprosetup}}"
  fi
else
  STATE="${STATE:-boot}"
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
