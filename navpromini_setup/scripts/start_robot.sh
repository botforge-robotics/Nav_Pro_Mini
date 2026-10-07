#!/usr/bin/env bash
# Always-on hardware bringup (lidar, odom, micro-ROS) — separate from Wi‑Fi setup.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f /opt/navpro/scripts/env.sh ]]; then
  # shellcheck disable=SC1091
  source /opt/navpro/scripts/env.sh
else
  # shellcheck disable=SC1091
  source "${SCRIPT_DIR}/env.sh"
fi

# Clean serial buffers and reset DTR/RTS on RPLidar to prevent handshake hang on restart
if [[ -e /dev/rplidar ]]; then
  python3 -c "import serial, time
try:
    s = serial.Serial('/dev/rplidar', 115200, timeout=0.1)
    s.dtr = False
    s.rts = False
    time.sleep(0.05)
    s.reset_input_buffer()
    s.reset_output_buffer()
    s.close()
except Exception:
    pass" 2>/dev/null || true
fi

exec ros2 launch navpromini_setup robot_bringup.launch.py \
  start_slam:=false \
  start_nav:=false
