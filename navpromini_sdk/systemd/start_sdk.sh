#!/bin/bash
# Launcher for navpro-sdk.service. Mirrors start_mission_planner.sh.
set -e
source /opt/ros/jazzy/setup.bash
source /home/navpromini/NavProMini_ws/install/setup.bash
exec ros2 launch navpromini_sdk sdk.launch.py "$@"
