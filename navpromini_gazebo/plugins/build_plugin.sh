#!/usr/bin/env bash
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

export PKG_CONFIG_PATH=$(find /opt/ros/jazzy/opt -name pkgconfig 2>/dev/null | tr '\n' ':'):$PKG_CONFIG_PATH

/usr/bin/rcc -name LidarTopicBridge LidarTopicBridge.qrc -o qrc_LidarTopicBridge.cpp
moc LidarTopicBridge.cpp -o LidarTopicBridge.moc
g++ -shared -fPIC -O2 -std=c++17 \
    $(pkg-config --cflags gz-sim8 gz-gui8) \
    LidarTopicBridge.cpp qrc_LidarTopicBridge.cpp \
    -o libLidarTopicBridge.so \
    $(pkg-config --libs gz-sim8 gz-gui8)

rm -f LidarTopicBridge.moc qrc_LidarTopicBridge.cpp
echo "[build_plugin.sh] Successfully built libLidarTopicBridge.so"
