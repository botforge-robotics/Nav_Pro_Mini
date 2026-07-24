#!/usr/bin/env python3
"""Periodic POST /robots/:id/heartbeat with health + nav_mode (Phase A)."""

from __future__ import annotations

import os
import time
from pathlib import Path
from typing import Any, Optional

import rclpy
import requests
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, LaserScan
from std_msgs.msg import String

from navpromini_fleet.fleet_config import (
    DEFAULT_FLEET_PATH,
    FleetConfig,
    load_fleet_config,
    save_fleet_config,
)


def _read_cpu_temp() -> Optional[float]:
    for p in (
        Path('/sys/class/thermal/thermal_zone0/temp'),
        Path('/sys/devices/virtual/thermal/thermal_zone0/temp'),
    ):
        try:
            milli = int(p.read_text().strip())
            return milli / 1000.0
        except (OSError, ValueError):
            continue
    return None


def _mem_used_frac() -> Optional[float]:
    try:
        info: dict[str, int] = {}
        for line in Path('/proc/meminfo').read_text().splitlines():
            parts = line.split()
            if len(parts) >= 2 and parts[0].endswith(':'):
                info[parts[0][:-1]] = int(parts[1])
        total = info.get('MemTotal')
        avail = info.get('MemAvailable')
        if total and avail is not None and total > 0:
            return 1.0 - (avail / total)
    except OSError:
        pass
    return None


def _loadavg() -> Optional[float]:
    try:
        return float(Path('/proc/loadavg').read_text().split()[0])
    except (OSError, ValueError, IndexError):
        return None


class HeartbeatNode(Node):
    def __init__(self) -> None:
        super().__init__('navpro_heartbeat')
        self.declare_parameter('config_path', str(DEFAULT_FLEET_PATH))
        self.declare_parameter('period_s', 2.0)
        self.declare_parameter('lidar_stale_s', 3.0)

        self._cfg: Optional[FleetConfig] = load_fleet_config(
            Path(str(self.get_parameter('config_path').value))
        )
        self._battery_soc: Optional[float] = None
        self._lidar_ok = False
        self._last_scan_t = 0.0
        self._nav_mode = (self._cfg.nav_mode if self._cfg else 'HARDWARE') or 'HARDWARE'

        self.create_subscription(BatteryState, 'battery/state', self._on_battery, 10)
        self.create_subscription(LaserScan, 'scan', self._on_scan, 10)
        self.create_subscription(String, 'navpro/nav_mode', self._on_nav_mode, 10)
        self._mode_pub = self.create_publisher(String, 'robot_mode', 10)

        period = max(float(self.get_parameter('period_s').value), 0.5)
        self.create_timer(period, self._tick)
        self.get_logger().info(
            f'heartbeat: robot_id={(self._cfg.robot_id if self._cfg else None)!r} '
            f'server={(self._cfg.server_ip if self._cfg else None)!r}'
        )

    def _on_battery(self, msg: BatteryState) -> None:
        if msg.percentage >= 0.0:
            # sensor_msgs: percentage is 0–1 when known
            self._battery_soc = float(msg.percentage) * 100.0 if msg.percentage <= 1.0 else float(msg.percentage)

    def _on_scan(self, _msg: LaserScan) -> None:
        self._last_scan_t = time.monotonic()
        self._lidar_ok = True

    def _on_nav_mode(self, msg: String) -> None:
        mode = (msg.data or '').strip().upper()
        if mode:
            self._nav_mode = mode
            if self._cfg is not None:
                self._cfg.nav_mode = mode

    def _reload_cfg(self) -> None:
        path = Path(str(self.get_parameter('config_path').value))
        cfg = load_fleet_config(path)
        if cfg is not None:
            self._cfg = cfg
            if cfg.nav_mode:
                self._nav_mode = cfg.nav_mode

    def _tick(self) -> None:
        stale = float(self.get_parameter('lidar_stale_s').value)
        if self._last_scan_t and (time.monotonic() - self._last_scan_t) > stale:
            self._lidar_ok = False

        mode_msg = String()
        mode_msg.data = self._nav_mode
        self._mode_pub.publish(mode_msg)

        if self._cfg is None or not self._cfg.robot_id:
            self._reload_cfg()
        if self._cfg is None or not self._cfg.robot_id or not self._cfg.server_ip:
            self.get_logger().warn('heartbeat skipped: missing robot_id/server_ip', throttle_duration_sec=30.0)
            return

        body: dict[str, Any] = {
            'mode': self._nav_mode,
            'lidarOk': self._lidar_ok,
            'cpu': _loadavg(),
            'mem': _mem_used_frac(),
            'temp': _read_cpu_temp(),
            'batterySoc': self._battery_soc,
            'extra': {
                'nav_mode': self._nav_mode,
                'ready_for_tasks': self._nav_mode in ('NAV_READY', 'NAV_ACTIVE'),
                'hostname': os.uname().nodename,
            },
        }
        # Drop Nones for cleaner payload
        body = {k: v for k, v in body.items() if v is not None}
        url = f'{self._cfg.api_base}/robots/{self._cfg.robot_id}/heartbeat'
        headers = {'X-Provisioning-Token': self._cfg.provisioning_token}
        try:
            resp = requests.post(url, headers=headers, json=body, timeout=5.0)
            if resp.status_code >= 400:
                self.get_logger().warn(
                    f'heartbeat HTTP {resp.status_code}: {resp.text[:200]}',
                    throttle_duration_sec=20.0,
                )
        except requests.RequestException as exc:
            self.get_logger().warn(f'heartbeat error: {exc}', throttle_duration_sec=20.0)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = HeartbeatNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Persist last nav_mode
        if node._cfg is not None:  # noqa: SLF001
            try:
                save_fleet_config(node._cfg)  # noqa: SLF001
            except OSError:
                pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
