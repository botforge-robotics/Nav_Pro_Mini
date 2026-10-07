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

# Ensure RPLidar hardware is stopped and reset to IDLE state.
# If previous driver died while lidar was actively scanning, the hardware continuously dumps
# scan packets and ignores all initialization commands unless a STOP (0xA5 0x25) is sent.
if [[ -e /dev/rplidar ]]; then
  python3 -c "import serial, time
try:
    s = serial.Serial('/dev/rplidar', 115200, timeout=0.2)
    s.write(b'\xa5\x25')
    time.sleep(0.08)
    s.reset_input_buffer()
    s.reset_output_buffer()
    s.write(b'\xa5\x40')
    time.sleep(0.1)
    s.reset_input_buffer()
    s.reset_output_buffer()
    s.close()
except Exception:
    pass" 2>/dev/null || true
fi

exec ros2 launch navpromini_setup robot_bringup.launch.py \
  start_slam:=false \
  start_nav:=false
