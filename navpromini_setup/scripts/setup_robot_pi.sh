#!/usr/bin/env bash
# Thin wrapper — use install_navpro.sh (single installer).
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "${SCRIPT_DIR}/install_navpro.sh" "$@"
