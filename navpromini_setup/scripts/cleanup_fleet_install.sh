#!/usr/bin/env bash
# Remove RMF / fleet install leftovers from the robot (systemd, zenoh, fleet.yaml).
# Keeps NetworkManager Wi‑Fi profiles and udev rules (still useful for nav2 setup).
# Run on the Pi: sudo bash cleanup_fleet_install.sh
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run as root: sudo bash $0" >&2
  exit 1
fi

echo "==> Stopping / disabling fleet-era units"

UNITS=(
  navpro-fleet.service
  navpro-provision.service
  navpro-robot.service
  navpro-display.service
)

for u in "${UNITS[@]}"; do
  systemctl disable --now "${u}" 2>/dev/null || true
  rm -f "/etc/systemd/system/${u}"
  rm -f "/etc/systemd/system/multi-user.target.wants/${u}"
done

systemctl daemon-reload
systemctl reset-failed 2>/dev/null || true

echo "==> Removing fleet config + zenoh + old /opt/navpro scripts"
rm -f /etc/navpro/fleet.yaml /etc/navpro/fleet.yaml.tmp
rm -rf /etc/navpro/zenoh
rm -rf /opt/navpro/zenoh
rm -rf /opt/navpro/scripts
# Drop whole /opt/navpro if empty-ish
rmdir /opt/navpro 2>/dev/null || rm -rf /opt/navpro

echo "==> Clearing runtime display hint"
rm -f /run/navpro/display_state
rmdir /run/navpro 2>/dev/null || true

echo "==> Optional NM cleanup (setup AP + site connection names from fleet portal)"
nmcli connection delete navpro-setup-ap 2>/dev/null || true
# Keep navpro-site-wifi if you still want that Wi‑Fi; uncomment to remove:
# nmcli connection delete navpro-site-wifi 2>/dev/null || true

echo "==> Done. Fleet services removed."
echo ""
echo "Still present (kept on purpose):"
echo "  /etc/udev/rules.d/99-navpro.rules   # lidar/BMS — reuse with navpromini_setup"
echo "  /etc/navpro/                        # dir; robot.yaml comes from new setup"
echo "  /var/lib/navpro/maps                # local maps if any"
echo "  NM Wi‑Fi profiles                   # so robot can still join known SSIDs"
echo ""
echo "Next (nav2 standalone):"
echo "  colcon build --packages-select navpromini_setup"
echo "  sudo bash ~/NavProMini_ws/src/navpromini_setup/systemd/install_robot_services.sh"
