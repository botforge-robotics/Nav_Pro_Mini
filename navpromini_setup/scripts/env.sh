#!/usr/bin/env bash
umask 000
# Source ROS + workspaces for systemd units.
# Note: do not enable nounset here — ROS setup.bash references unset vars.
set -eo pipefail

USER_NAME="${NAVPRO_USER:-${SUDO_USER:-$(id -un)}}"
USER_HOME="$(getent passwd "${USER_NAME}" | cut -d: -f6 || echo /home/${USER_NAME})"
WS="${NAVPRO_WS:-${USER_HOME}/NavProMini_ws}"

# Default 0 for Mission Planner / rosbridge over Wi-Fi. Set to 1 only for isolated local-only use.
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
if [[ -f "${USER_HOME}/uros_ws/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${USER_HOME}/uros_ws/install/setup.bash"
fi

# Power-loss recovery: if install/setup.bash was damaged during an interrupted update,
# automatically restore from the atomic snapshot install.prev to ensure boot reliability.
if [[ ! -f "${WS}/install/setup.bash" ]] && [[ -d "${WS}/install.prev" ]]; then
  echo "WARN: ${WS}/install corrupted or incomplete. Restoring from ${WS}/install.prev..." >&2
  rm -rf "${WS}/install"
  cp -al "${WS}/install.prev" "${WS}/install" 2>/dev/null || cp -r "${WS}/install.prev" "${WS}/install"
fi

# shellcheck disable=SC1091
source "${WS}/install/setup.bash"
set -u
