#!/usr/bin/env bash
# Install systemd units + scripts under /opt/navpro (standalone robot — no fleet/zenoh).
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run as root: sudo bash $0" >&2
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
USER_NAME="${SUDO_USER:-${NAVPRO_USER:-pi}}"
USER_HOME="$(getent passwd "${USER_NAME}" | cut -d: -f6)"
WS="${NAVPRO_WS:-${USER_HOME}/NavProMini_ws}"

echo "==> install_robot_services user=${USER_NAME} ws=${WS}"

install -d -m 0755 /opt/navpro/scripts /etc/navpro /var/lib/navpro/maps

for f in env.sh start_robot.sh start_provision.sh start_display.sh setup_robot_pi.sh; do
  if [[ -f "${PKG_ROOT}/scripts/${f}" ]]; then
    install -m 0755 "${PKG_ROOT}/scripts/${f}" "/opt/navpro/scripts/${f}"
  fi
done

if [[ -f "${PKG_ROOT}/udev/99-navpro.rules" ]]; then
  install -m 0644 "${PKG_ROOT}/udev/99-navpro.rules" /etc/udev/rules.d/99-navpro.rules
  udevadm control --reload-rules || true
fi

# Disable legacy fleet unit if present
systemctl disable --now navpro-fleet.service 2>/dev/null || true

for unit in navpro-robot.service navpro-provision.service navpro-display.service; do
  src="${SCRIPT_DIR}/${unit}"
  dst="/etc/systemd/system/${unit}"
  sed -e "s|REPLACE_USER|${USER_NAME}|g" -e "s|REPLACE_WS|${WS}|g" "${src}" > "${dst}"
  chmod 0644 "${dst}"
done

systemctl daemon-reload
systemctl enable navpro-robot.service navpro-provision.service navpro-display.service

echo "==> Enabled: navpro-robot  navpro-provision  navpro-display"
echo "    (separate services; no fleet / zenoh)"
echo "    OLED/LED only via navpro-display (status_display_node) — hint file + topic"
echo "    Start: systemctl start navpro-display navpro-robot navpro-provision"
echo "    Hotspot only if no saved Wi‑Fi is nearby — portal asks Wi‑Fi + robot name only"
