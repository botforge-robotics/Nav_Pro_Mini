#!/usr/bin/env bash
# Fleet agent: heartbeat + zenoh CLIENT (requires /etc/navpro/fleet.yaml).
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f /opt/navpro/scripts/env.sh ]]; then
  # shellcheck disable=SC1091
  source /opt/navpro/scripts/env.sh
else
  # shellcheck disable=SC1091
  source "${SCRIPT_DIR}/env.sh"
fi

CFG="${NAVPRO_FLEET_YAML:-/etc/navpro/fleet.yaml}"
if [[ ! -f "${CFG}" ]]; then
  echo "No ${CFG} — waiting for provisioning" >&2
  exit 0
fi

export NAVPRO_MAPS_DIR="${NAVPRO_MAPS_DIR:-/var/lib/navpro/maps}"
mkdir -p "${NAVPRO_MAPS_DIR}"

# Parse YAML (supports flat Phase-A keys and older nested robot:/server_url forms).
get_yaml() {
  local key="$1"
  python3 - <<PY
import yaml
from pathlib import Path
d = yaml.safe_load(Path("${CFG}").read_text()) or {}
robot = d.get("robot") if isinstance(d.get("robot"), dict) else {}
key = "${key}"
if key == "name":
    print(d.get("name") or robot.get("name") or "")
elif key == "server_ip":
    ip = d.get("server_ip") or ""
    if not ip:
        url = str(d.get("server_url") or "")
        # http://host:port → host
        if "://" in url:
            url = url.split("://", 1)[1]
        ip = url.split("/")[0].split(":")[0]
    print(ip)
elif key == "serial":
    print(d.get("serial") or robot.get("serial") or "")
else:
    print(d.get(key) or "")
PY
}

NAME="$(get_yaml name)"
SERVER_IP="$(get_yaml server_ip)"
DOMAIN="$(get_yaml ros_domain_id)"
DOMAIN="${DOMAIN:-0}"
export ROS_DOMAIN_ID="${DOMAIN}"

TEMPLATE=""
for cand in \
  "${SCRIPT_DIR}/../config/zenoh-client.json5.template" \
  /opt/navpro/zenoh/zenoh-client.json5.template \
  "${WS}/src/navpromini_fleet/config/zenoh-client.json5.template" \
  "$(ros2 pkg prefix navpromini_fleet 2>/dev/null)/share/navpromini_fleet/config/zenoh-client.json5.template"
do
  if [[ -n "${cand}" && -f "${cand}" ]]; then
    TEMPLATE="${cand}"
    break
  fi
done

ZENOH_CFG=/etc/navpro/zenoh/client.json5
mkdir -p /etc/navpro/zenoh
if [[ -n "${TEMPLATE}" && -n "${NAME}" && -n "${SERVER_IP}" ]]; then
  sed -e "s/__SERVER_IP__/${SERVER_IP}/g" \
      -e "s/__ROBOT_NAME__/${NAME}/g" \
      -e "s/__ROS_DOMAIN_ID__/${DOMAIN}/g" \
      "${TEMPLATE}" > "${ZENOH_CFG}"
fi

ZENOH_BIN="${ZENOH_BRIDGE_BIN:-/opt/navpro/zenoh/zenoh-bridge-ros2dds}"
if [[ -x "${ZENOH_BIN}" && -f "${ZENOH_CFG}" ]]; then
  "${ZENOH_BIN}" -c "${ZENOH_CFG}" &
  ZENOH_PID=$!
  trap 'kill ${ZENOH_PID} 2>/dev/null || true' EXIT
else
  echo "WARN: zenoh bridge not started (missing binary or config)" >&2
fi

exec ros2 launch navpromini_fleet fleet.launch.py config_path:="${CFG}"
