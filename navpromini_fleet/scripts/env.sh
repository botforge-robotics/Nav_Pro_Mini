#!/usr/bin/env bash
# Source ROS + workspaces for systemd units.
# Note: do not enable nounset here — ROS setup.bash references unset vars
# (e.g. AMENT_TRACE_SETUP_FILES) and will abort under `set -u`.
set -eo pipefail

USER_NAME="${NAVPRO_USER:-${SUDO_USER:-$(id -un)}}"
USER_HOME="$(getent passwd "${USER_NAME}" | cut -d: -f6 || echo /home/${USER_NAME})"
WS="${NAVPRO_WS:-${USER_HOME}/NavProMini_ws}"

export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# Parent start_*.sh may use `set -u`; relax around ament setup scripts.
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
