#!/usr/bin/env python3
"""Publish host system health (CPU temperature) for the UI status bar.

The Daly BMS already gives us battery temperatures via battery_node
(`battery/temperatures`, and BatteryState.temperature), but nothing was
reporting the Pi's own thermal state. That matters on this robot: measured
82.6degC on an idle-ish system, which is close enough to the Pi's ~85degC
throttle point that a UI readout is worth having.

Publishes:
  /system/cpu_temperature (std_msgs/Float32) — degrees Celsius.

Reads the standard Linux thermal sysfs interface. Zone selection prefers a
zone whose `type` looks like a CPU/SoC sensor (on this Pi:
thermal_zone0 type=cpu-thermal), falling back to zone 0, so this keeps
working if zone ordering changes between kernels.
"""

from __future__ import annotations

import glob
import os
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

_CPU_ZONE_HINTS = ('cpu', 'soc', 'x86_pkg_temp', 'package')


def _find_cpu_thermal_zone() -> Optional[str]:
    zones = sorted(glob.glob('/sys/class/thermal/thermal_zone*'))
    if not zones:
        return None
    for zone in zones:
        try:
            with open(os.path.join(zone, 'type'), 'r') as f:
                ztype = f.read().strip().lower()
        except OSError:
            continue
        if any(hint in ztype for hint in _CPU_ZONE_HINTS):
            return zone
    return zones[0]


class SystemMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__('navpromini_system_monitor')

        self.declare_parameter('publish_rate_hz', 1.0)
        self.declare_parameter('thermal_zone', '')  # override auto-detection

        rate = float(self.get_parameter('publish_rate_hz').value)
        override = str(self.get_parameter('thermal_zone').value).strip()

        self._zone = override or _find_cpu_thermal_zone()
        self._temp_pub = self.create_publisher(Float32, 'system/cpu_temperature', 10)

        if self._zone is None:
            self.get_logger().warn(
                'No thermal zone found — CPU temperature will not be published'
            )
        else:
            try:
                with open(os.path.join(self._zone, 'type'), 'r') as f:
                    ztype = f.read().strip()
            except OSError:
                ztype = 'unknown'
            self.get_logger().info(
                f'system_monitor: reading CPU temperature from {self._zone} (type={ztype})'
            )

        self.create_timer(max(0.1, 1.0 / rate) if rate > 0 else 1.0, self._tick)

    def _read_cpu_temp_c(self) -> Optional[float]:
        if self._zone is None:
            return None
        try:
            with open(os.path.join(self._zone, 'temp'), 'r') as f:
                raw = f.read().strip()
        except OSError as exc:
            self.get_logger().warn(
                f'Failed to read {self._zone}/temp: {exc}', throttle_duration_sec=30.0
            )
            return None
        try:
            value = float(raw)
        except ValueError:
            return None
        # sysfs reports millidegrees on essentially all platforms, but a few
        # expose plain degrees — treat implausibly small values as already-C.
        return value / 1000.0 if abs(value) > 1000.0 else value

    def _tick(self) -> None:
        temp = self._read_cpu_temp_c()
        if temp is None:
            return
        self._temp_pub.publish(Float32(data=temp))


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = SystemMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
