#!/usr/bin/env bash
# NavPro Mini — one-shot robot install (run once on the Pi).
#
# Does:
#   1) OS helpers (NetworkManager, UART, dialout)
#   2) Shell env (ROS, micro-ROS, NavPro workspace, ROS_DOMAIN_ID)
#   3) USB udev rules (/dev/rplidar, /dev/battery_bms)
#   4) Boot services: Wi‑Fi setup hotspot, OLED/LED, hardware ROS
#
# Usage:
#   cd ~/NavProMini_ws && colcon build --packages-select navpromini_setup
#   sudo bash src/navpromini_setup/scripts/install_navpro.sh
#
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

echo "==> NavPro install (user=${USER_NAME}  workspace=${WS})"

# --- packages ---
apt-get update -y
apt-get install -y network-manager python3-yaml curl || true

# --- UART for ESP32 micro-ROS ---
systemctl disable --now serial-getty@ttyAMA0.service 2>/dev/null || true
if [[ -f /boot/firmware/config.txt ]]; then
  CFG=/boot/firmware/config.txt
elif [[ -f /boot/config.txt ]]; then
  CFG=/boot/config.txt
else
  CFG=""
fi
if [[ -n "${CFG}" ]]; then
  grep -q '^enable_uart=1' "${CFG}" || echo 'enable_uart=1' >> "${CFG}"
  grep -q 'dtparam=uart0' "${CFG}" || echo 'dtparam=uart0=on' >> "${CFG}" || true
fi
usermod -aG dialout "${USER_NAME}" || true

# --- dirs ---
install -d -m 0755 /opt/navpro/scripts /etc/navpro /var/lib/navpro/maps

# --- USB udev ---
if [[ -f "${PKG_ROOT}/udev/99-navpro.rules" ]]; then
  install -m 0644 "${PKG_ROOT}/udev/99-navpro.rules" /etc/udev/rules.d/99-navpro.rules
  udevadm control --reload-rules || true
  udevadm trigger || true
  echo "==> USB rules installed (/dev/rplidar, /dev/battery_bms)"
fi

# --- shell environment (ROS + micro-ROS + workspace + domain) ---
BASHRC="${USER_HOME}/.bashrc"
MARKER="# NAVPROMINI_PI_ENV"
if ! grep -q "${MARKER}" "${BASHRC}" 2>/dev/null; then
  cat >> "${BASHRC}" <<EOF

${MARKER}
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-0}
source /opt/ros/jazzy/setup.bash 2>/dev/null || true
if [[ -f ${WS}/install/setup.bash ]]; then source ${WS}/install/setup.bash; fi
if [[ -f ${USER_HOME}/uros_ws/install/setup.bash ]]; then source ${USER_HOME}/uros_ws/install/setup.bash; fi
EOF
  chown "${USER_NAME}:${USER_NAME}" "${BASHRC}"
  echo "==> Shell environment added to ${BASHRC}"
else
  echo "==> Shell environment already present"
fi

nmcli device set wlan0 managed yes 2>/dev/null || true

# --- helper scripts for systemd ---
for f in env.sh start_robot.sh start_provision.sh start_display.sh; do
  if [[ -f "${PKG_ROOT}/scripts/${f}" ]]; then
    install -m 0755 "${PKG_ROOT}/scripts/${f}" "/opt/navpro/scripts/${f}"
  fi
done
# Keep this installer available under /opt/navpro for re-runs
install -m 0755 "${PKG_ROOT}/scripts/install_navpro.sh" /opt/navpro/scripts/install_navpro.sh

# --- remove obsolete units quietly (if any) ---
for obsolete in navpro-fleet.service; do
  systemctl disable --now "${obsolete}" 2>/dev/null || true
  rm -f "/etc/systemd/system/${obsolete}"
  rm -f "/etc/systemd/system/multi-user.target.wants/${obsolete}"
done
rm -rf /opt/navpro/zenoh /etc/navpro/zenoh 2>/dev/null || true
# Prefer robot.yaml; drop old identity file if both would confuse Wi‑Fi setup
if [[ -f /etc/navpro/fleet.yaml ]] && [[ ! -f /etc/navpro/robot.yaml ]]; then
  # Best-effort migrate name / wifi_ssid only
  python3 - <<'PY' 2>/dev/null || true
import yaml
from pathlib import Path
raw = yaml.safe_load(Path("/etc/navpro/fleet.yaml").read_text()) or {}
robot = raw.get("robot") if isinstance(raw.get("robot"), dict) else {}
out = {
    "name": raw.get("name") or robot.get("name") or "",
    "serial": raw.get("serial") or robot.get("serial") or "",
    "wifi_ssid": raw.get("wifi_ssid") or "",
}
out = {k: v for k, v in out.items() if v}
if out.get("name") or out.get("wifi_ssid"):
    Path("/etc/navpro/robot.yaml").write_text(yaml.safe_dump(out, sort_keys=False))
PY
fi
rm -f /etc/navpro/fleet.yaml /etc/navpro/fleet.yaml.tmp 2>/dev/null || true

# --- systemd services ---
UNIT_DIR="${PKG_ROOT}/systemd"
for unit in navpro-robot.service navpro-provision.service navpro-display.service; do
  src="${UNIT_DIR}/${unit}"
  dst="/etc/systemd/system/${unit}"
  sed -e "s|REPLACE_USER|${USER_NAME}|g" -e "s|REPLACE_WS|${WS}|g" "${src}" > "${dst}"
  chmod 0644 "${dst}"
done

systemctl daemon-reload
systemctl enable navpro-display.service navpro-robot.service navpro-provision.service
systemctl restart navpro-display.service navpro-robot.service navpro-provision.service || \
  systemctl start navpro-display.service navpro-robot.service navpro-provision.service

echo ""
echo "==> Install complete"
echo "    Services:"
echo "      navpro-display   — OLED / LED status"
echo "      navpro-robot     — lidar, odom, micro-ROS hardware"
echo "      navpro-provision — Wi‑Fi setup hotspot (when needed)"
echo ""
echo "    If no known Wi‑Fi is nearby, connect your phone to:"
echo "      SSID  NavPro-Setup-<last6 of MAC>"
echo "      Pass  navprosetup"
echo "      Open  http://10.42.0.1/  → set Wi‑Fi + robot name"
echo ""
echo "    Check:  systemctl status navpro-display navpro-robot navpro-provision"
echo "    Reboot recommended once after first install."
