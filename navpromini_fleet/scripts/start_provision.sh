#!/usr/bin/env bash
# Hotspot captive portal when unprovisioned.
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
if [[ -f "${CFG}" ]]; then
  echo "${CFG} present — provision not needed"
  exit 0
fi

export NAVPRO_DISPLAY_STATE=setup
exec ros2 run navpromini_fleet provision_portal
