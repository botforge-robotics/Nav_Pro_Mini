#!/usr/bin/env bash
# Copy RMF site artifacts from navpromini_rmf_sim into navpro-fleet-server/site/
#
# IMPORTANT: dereference symlinks (-L). Colcon --symlink-install leaves
# site/*.building.yaml as links into the host workspace; Docker cannot follow
# those paths, so building_map_server never starts and the GUI map is empty.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# Prefer package share src tree when installed; fall back to this package dir.
SRC_SITE="${SRC_SITE:-}"
if [[ -z "${SRC_SITE}" ]]; then
  if command -v ros2 >/dev/null 2>&1; then
    PKG_SHARE="$(ros2 pkg prefix navpromini_rmf_sim 2>/dev/null || true)/share/navpromini_rmf_sim/site"
    if [[ -d "${PKG_SHARE}" ]]; then
      # Resolve through install→build→src if needed
      SRC_SITE="$(readlink -f "${PKG_SHARE}" 2>/dev/null || echo "${PKG_SHARE}")"
    fi
  fi
fi
if [[ -z "${SRC_SITE}" || ! -d "${SRC_SITE}" ]]; then
  SRC_SITE="${ROOT}/site"
fi

FLEET_SITE="${FLEET_SITE:-/mnt/68185C18185BE39A/Botforge/Projects/navpro-fleet-server/site}"

if [[ ! -d "${FLEET_SITE}" ]]; then
  echo "ERROR: fleet site dir not found: ${FLEET_SITE}" >&2
  echo "Set FLEET_SITE to your navpro-fleet-server/site path." >&2
  exit 1
fi

# Always sync from the real source tree (not install symlink forest).
if [[ -d /home/chaitu/NavProMini_ws/src/navpromini_rmf_sim/site ]]; then
  SRC_SITE="/home/chaitu/NavProMini_ws/src/navpromini_rmf_sim/site"
fi

echo "Syncing ${SRC_SITE} -> ${FLEET_SITE} (dereferencing symlinks)"
# -L: copy symlink targets as real files so Docker :ro mounts can read them
rsync -aL --delete \
  --exclude 'registry-snapshot.json' \
  --exclude 'generated/' \
  "${SRC_SITE}/" "${FLEET_SITE}/"

# Sanity: building must be a regular file readable as data
if [[ ! -f "${FLEET_SITE}/site.building.yaml" ]]; then
  echo "ERROR: site.building.yaml missing after sync" >&2
  exit 1
fi
if [[ -L "${FLEET_SITE}/site.building.yaml" ]]; then
  echo "ERROR: site.building.yaml is still a symlink; Docker cannot use it" >&2
  exit 1
fi

echo "OK: site.building.yaml $(wc -c < "${FLEET_SITE}/site.building.yaml") bytes"
echo "OK: nav_graphs: $(ls "${FLEET_SITE}/nav_graphs" 2>/dev/null | tr '\n' ' ')"
echo "OK: fleet_config: $(ls "${FLEET_SITE}/fleet_config" 2>/dev/null | tr '\n' ' ')"
echo
echo "Restart rmf-core so it picks up the map:"
echo "  cd $(dirname "${FLEET_SITE}") && docker compose restart rmf-core"
