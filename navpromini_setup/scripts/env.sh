#!/usr/bin/env bash
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
# shellcheck disable=SC1091
source "${WS}/install/setup.bash"
set -u
