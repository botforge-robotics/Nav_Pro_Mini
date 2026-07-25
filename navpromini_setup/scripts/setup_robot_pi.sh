#!/usr/bin/env bash
# One-time Pi OS setup: NetworkManager, UART, udev, dirs (no RMF / zenoh).
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run as root: sudo bash $0" >&2
  exit 1
fi

USER_NAME="${SUDO_USER:-${USER:-pi}}"
USER_HOME="$(getent passwd "${USER_NAME}" | cut -d: -f6)"
WS="${NAVPRO_WS:-${USER_HOME}/NavProMini_ws}"
PKG_SRC="${WS}/src/navpromini_setup"

echo "==> NavPro setup_robot_pi (user=${USER_NAME} ws=${WS})"

apt-get update -y
apt-get install -y network-manager python3-yaml curl || true

# UART for ESP32 micro-ROS on ttyAMA0
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

install -d -m 0755 /etc/navpro /var/lib/navpro/maps
if [[ -f "${PKG_SRC}/udev/99-navpro.rules" ]]; then
  install -m 0644 "${PKG_SRC}/udev/99-navpro.rules" /etc/udev/rules.d/99-navpro.rules
  udevadm control --reload-rules || true
  udevadm trigger || true
fi

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
fi

nmcli device set wlan0 managed yes 2>/dev/null || true

echo "==> setup_robot_pi done. Reboot recommended."
echo "    Next: colcon build --packages-select navpromini_setup"
echo "    Then: sudo bash ${PKG_SRC}/systemd/install_robot_services.sh"
