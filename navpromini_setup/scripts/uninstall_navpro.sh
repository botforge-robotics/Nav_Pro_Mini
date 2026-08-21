#!/usr/bin/env bash
# Reset NavPro boot services and config (keeps Wi‑Fi profiles and udev).
# Usage: sudo bash uninstall_navpro.sh
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run as root: sudo bash $0" >&2
  exit 1
fi

echo "==> Stopping NavPro services"
for u in navpro-fleet.service navpro-provision.service navpro-robot.service navpro-display.service; do
  systemctl disable --now "${u}" 2>/dev/null || true
  rm -f "/etc/systemd/system/${u}"
  rm -f "/etc/systemd/system/multi-user.target.wants/${u}"
done
systemctl daemon-reload
systemctl reset-failed 2>/dev/null || true

echo "==> Removing /opt/navpro and robot config"
rm -rf /opt/navpro
rm -f /etc/navpro/robot.yaml /etc/navpro/fleet.yaml /etc/navpro/fleet.yaml.tmp
rm -rf /etc/navpro/zenoh
rm -f /run/navpro/display_state
rmdir /run/navpro 2>/dev/null || true

nmcli connection delete navpro-setup-ap 2>/dev/null || true

echo "==> Removed. USB rules and saved Wi‑Fi profiles were kept."
echo "    Reinstall: sudo bash ~/NavProMini_ws/src/navpromini_setup/scripts/install_navpro.sh"
