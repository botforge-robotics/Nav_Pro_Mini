#!/usr/bin/env bash
# Download zenoh-bridge-ros2dds for this arch into /opt/navpro/zenoh
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run as root: sudo bash $0" >&2
  exit 1
fi

ARCH="$(uname -m)"
case "${ARCH}" in
  aarch64|arm64) ZARCH="aarch64-unknown-linux-gnu" ;;
  x86_64) ZARCH="x86_64-unknown-linux-gnu" ;;
  *) echo "Unsupported arch: ${ARCH}" >&2; exit 1 ;;
esac

VER="${ZENOH_BRIDGE_VERSION:-1.1.1}"
NAME="zenoh-bridge-ros2dds-${VER}-${ZARCH}"
URL="https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases/download/${VER}/${NAME}.zip"
DEST=/opt/navpro/zenoh
TMP="$(mktemp -d)"
trap 'rm -rf "${TMP}"' EXIT

echo "==> Installing zenoh-bridge-ros2dds ${VER} (${ZARCH})"
mkdir -p "${DEST}"
curl -fsSL -o "${TMP}/zenoh.zip" "${URL}"
unzip -o "${TMP}/zenoh.zip" -d "${TMP}/out"
BIN="$(find "${TMP}/out" -type f -name 'zenoh-bridge-ros2dds' | head -n1)"
if [[ -z "${BIN}" ]]; then
  echo "Binary not found in archive" >&2
  exit 1
fi
install -m 0755 "${BIN}" "${DEST}/zenoh-bridge-ros2dds"
echo "==> Installed ${DEST}/zenoh-bridge-ros2dds"
"${DEST}/zenoh-bridge-ros2dds" --help >/dev/null 2>&1 || true
