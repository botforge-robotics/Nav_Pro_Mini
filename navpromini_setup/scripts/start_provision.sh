#!/usr/bin/env bash
# Wi‑Fi setup hotspot — only when no saved/nearby Wi‑Fi (logic in portal).
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f /opt/navpro/scripts/env.sh ]]; then
  # shellcheck disable=SC1091
  source /opt/navpro/scripts/env.sh
else
  # shellcheck disable=SC1091
  source "${SCRIPT_DIR}/env.sh"
fi

export NAVPRO_DISPLAY_STATE=setup
exec ros2 run navpromini_setup provision_portal
