#!/usr/bin/env bash
# NavPro Mini — Detached companion update script.
# Safely pulls and builds latest robot packages with atomic rollback and power-loss recovery.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STATUS_FILE="/var/lib/navpro/update_status.json"
LOG_FILE="/var/log/navpro/update.log"
mkdir -p /var/lib/navpro /var/log/navpro

# Source env if available
if [[ -f "${SCRIPT_DIR}/env.sh" ]]; then
  # shellcheck disable=SC1091
  source "${SCRIPT_DIR}/env.sh"
fi

USER_NAME="${NAVPRO_USER:-navpromini}"
USER_HOME="$(getent passwd "${USER_NAME}" | cut -d: -f6 || echo /home/${USER_NAME})"
WS="${NAVPRO_WS:-${USER_HOME}/NavProMini_ws}"
SRC_DIR="${WS}/src"
INSTALL_DIR="${WS}/install"
BACKUP_DIR="${WS}/install.prev"

# Redirect stdout & stderr to log file while also keeping console output
exec >> >(tee -a "${LOG_FILE}") 2>&1

write_status() {
  local phase="$1"
  local progress="$2"
  local msg="$3"
  local commit="${4:-}"
  local error="${5:-}"
  python3 -c "
import json, time
status = {
  'phase': '$phase',
  'progress': $progress,
  'message': '''$msg''',
  'commit': '$commit',
  'error': '''$error''',
  'timestamp': time.time()
}
with open('$STATUS_FILE', 'w') as f:
  json.dump(status, f, indent=2)
" 2>/dev/null || true
}

echo ""
echo "=================================================="
echo "=== NavPro Companion Update started at $(date) ==="
echo "=================================================="

# Check branch
BRANCH="$(git -C "${SRC_DIR}" rev-parse --abbrev-ref HEAD 2>/dev/null || echo 'nav2')"
PREV_COMMIT="$(git -C "${SRC_DIR}" rev-parse HEAD 2>/dev/null || echo '')"

write_status "in_progress" 10 "Checking pre-flight preconditions..." "${PREV_COMMIT}"

# Pre-flight disk space check: require >= 1.2 GB
FREE_KB=$(df -k / | awk 'NR==2 {print $4}')
if [[ "${FREE_KB}" -lt 1200000 ]]; then
  echo "Error: Insufficient disk space on / (free: ${FREE_KB} KB, require >= 1200000 KB)"
  write_status "failed" 0 "Insufficient disk space" "${PREV_COMMIT}" "Require at least 1.2 GB free on root"
  exit 1
fi

# Snapshot current install directory if it exists
if [[ -d "${INSTALL_DIR}" ]]; then
  echo "==> Creating atomic backup snapshot: ${INSTALL_DIR} -> ${BACKUP_DIR}"
  write_status "in_progress" 20 "Creating backup snapshot of working build..." "${PREV_COMMIT}"
  rm -rf "${BACKUP_DIR}"
  cp -al "${INSTALL_DIR}" "${BACKUP_DIR}" 2>/dev/null || cp -r "${INSTALL_DIR}" "${BACKUP_DIR}"
fi

# Rollback trap
rollback() {
  local exit_code=$?
  if [[ $exit_code -ne 0 ]]; then
    echo "==> ERROR detected during update! Initiating rollback..."
    write_status "failed" 0 "Update failed during execution. Rolling back to previous state..." "${PREV_COMMIT}" "Build or pull failure"
    if [[ -n "${PREV_COMMIT}" ]]; then
      git -C "${SRC_DIR}" reset --hard "${PREV_COMMIT}" || true
    fi
    if [[ -d "${BACKUP_DIR}" ]]; then
      rm -rf "${INSTALL_DIR}"
      cp -al "${BACKUP_DIR}" "${INSTALL_DIR}" 2>/dev/null || cp -r "${BACKUP_DIR}" "${INSTALL_DIR}"
      echo "==> Restored install from ${BACKUP_DIR}"
    fi
    # Restart services on known-good build
    systemctl restart navpro-robot.service navpro-display.service navpro-mission-planner.service navpro-sdk.service navpro-mcp.service || true
    write_status "failed" 0 "Rollback completed. Restored last stable software." "${PREV_COMMIT}" "Update aborted and rolled back."
  fi
}
trap rollback EXIT

# Phase: Pulling
echo "==> Fetching and updating source on branch ${BRANCH}..."
write_status "pulling" 30 "Pulling latest code from origin/${BRANCH}..." "${PREV_COMMIT}"
if [[ "$(id -u)" -eq 0 && -n "${ROBOT_USER}" && "${ROBOT_USER}" != "root" ]]; then
  sudo -u "${ROBOT_USER}" git -c safe.directory=* -C "${SRC_DIR}" fetch origin "${BRANCH}"
else
  git -c safe.directory=* -C "${SRC_DIR}" fetch origin "${BRANCH}"
fi
git -c safe.directory=* -C "${SRC_DIR}" reset --hard "origin/${BRANCH}"
NEW_COMMIT="$(git -c safe.directory=* -C "${SRC_DIR}" rev-parse HEAD)"

# Also update navpromini_sdk repo (including MCP server) if present at USER_HOME/navpromini_sdk
if [[ -d "${USER_HOME}/navpromini_sdk/.git" ]]; then
  echo "==> Updating ${USER_HOME}/navpromini_sdk..."
  SDK_BRANCH=$(git -c safe.directory=* -C "${USER_HOME}/navpromini_sdk" rev-parse --abbrev-ref HEAD 2>/dev/null || echo "main")
  git -c safe.directory=* -C "${USER_HOME}/navpromini_sdk" fetch origin "${SDK_BRANCH}" || true
  git -c safe.directory=* -C "${USER_HOME}/navpromini_sdk" pull --ff-only origin "${SDK_BRANCH}" || git -c safe.directory=* -C "${USER_HOME}/navpromini_sdk" pull origin "${SDK_BRANCH}" || true
  if [[ -d "/opt/navpro/mcp_venv" ]]; then
    /opt/navpro/mcp_venv/bin/pip install --upgrade pip || true
    /opt/navpro/mcp_venv/bin/pip install -e "${USER_HOME}/navpromini_sdk/clients/python" || true
    /opt/navpro/mcp_venv/bin/pip install -e "${USER_HOME}/navpromini_sdk/clients/mcp" || true
  fi
fi

# Phase: Building
echo "==> Building workspace: ${WS}..."
write_status "building" 50 "Compiling packages with colcon..." "${NEW_COMMIT}"

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
if [[ -f "${USER_HOME}/uros_ws/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${USER_HOME}/uros_ws/install/setup.bash"
fi

cd "${WS}"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Phase: Re-running installer scripts (to register any new services/udev)
if [[ -f "${SRC_DIR}/navpromini_setup/scripts/install_navpro.sh" ]]; then
  echo "==> Updating installed service units and udev rules..."
  write_status "restarting" 85 "Applying system service and udev updates..." "${NEW_COMMIT}"
  bash "${SRC_DIR}/navpromini_setup/scripts/install_navpro.sh" || true
fi

# Phase: Restarting
echo "==> Restarting NavPro services..."
write_status "restarting" 90 "Restarting robot services..." "${NEW_COMMIT}"
systemctl restart navpro-robot.service navpro-display.service navpro-mission-planner.service navpro-sdk.service navpro-mcp.service

# Remove EXIT trap since success
trap - EXIT

echo "==> Update successfully completed!"
write_status "success" 100 "Update completed successfully. Running on commit ${NEW_COMMIT:0:8}." "${NEW_COMMIT}"

echo "=================================================="
echo "=== Update finished at $(date) ==="
echo "=================================================="
