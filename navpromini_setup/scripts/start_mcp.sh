#!/usr/bin/env bash
# Start the NavPro Mini Model Context Protocol (MCP) Server.
# Runs as an SSE daemon on :8091 for LAN AI agents.
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "${SCRIPT_DIR}/env.sh" ]; then
  source "${SCRIPT_DIR}/env.sh"
fi

export NAVPRO_ROBOT_HOST="${NAVPRO_ROBOT_HOST:-127.0.0.1}"
export NAVPRO_ROBOT_PORT="${NAVPRO_ROBOT_PORT:-8090}"
export MCP_SSE_HOST="${MCP_SSE_HOST:-0.0.0.0}"
export MCP_SSE_PORT="${MCP_SSE_PORT:-8091}"

exec /opt/navpro/mcp_venv/bin/navpromini-mcp   --host "${NAVPRO_ROBOT_HOST}"   --port "${NAVPRO_ROBOT_PORT}"   --transport sse   --sse-host "${MCP_SSE_HOST}"   --sse-port "${MCP_SSE_PORT}"   "${@}"
