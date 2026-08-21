#!/usr/bin/env bash
# Install and enable navpro-sdk.service. Run with sudo, on the robot.
#
#   sudo bash ~/NavProMini_ws/src/navpromini_sdk/systemd/install_sdk_service.sh
#
# Idempotent: safe to re-run after a rebuild to refresh the unit and script.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

install -m 0755 "${HERE}/start_sdk.sh"       /opt/navpro/scripts/start_sdk.sh
install -m 0644 "${HERE}/navpro-sdk.service" /etc/systemd/system/navpro-sdk.service

systemctl daemon-reload
systemctl enable --now navpro-sdk.service

echo
systemctl --no-pager --lines=0 status navpro-sdk.service || true
echo
echo "Check it answers:  curl -s http://127.0.0.1:8090/api/v1/system/info"
