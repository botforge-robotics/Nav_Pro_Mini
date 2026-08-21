#!/usr/bin/env python3
"""System identity and health."""

from __future__ import annotations

import os
import shutil
import socket
import time

from .base import BaseHandler

SDK_VERSION = '1.0.0'
API_VERSION = 'v1'

# How stale a stream may be before health calls it unhealthy. Generous
# multiples of each source's nominal rate, so a momentarily busy CPU does not
# read as a dead sensor.
_FRESH_LIMITS = {
    'odom': ('pose_odom', 2.0),
    'battery': ('battery', 5.0),
    'imu': ('imu', 2.0),
    'lidar': ('scan', 3.0),
    'cpu_temperature': ('cpu_temperature', 10.0),
}


def _robot_identity() -> dict:
    """Identity from navpromini_setup, falling back sanely if unavailable.

    Imported lazily and defensively: the SDK must still answer /system/info on
    a machine where navpromini_setup is not installed, because that endpoint is
    the first thing anyone calls when debugging a robot.
    """
    name = serial = None
    try:
        from navpromini_setup.robot_config import load_robot_config, read_cpu_serial
        cfg = load_robot_config()
        if cfg is not None:
            name = cfg.name
        serial = read_cpu_serial()
    except Exception:  # noqa: BLE001
        pass
    return {
        'name': name or socket.gethostname(),
        'serial': serial,
        'hostname': socket.gethostname(),
    }


class InfoHandler(BaseHandler):
    def get(self) -> None:
        ident = _robot_identity()
        self.send({
            'robot': ident,
            'sdk_version': SDK_VERSION,
            'api_version': API_VERSION,
            'ros_distro': os.environ.get('ROS_DISTRO', 'unknown'),
            'model': 'NavProMini',
            'uptime_sec': round(time.monotonic(), 1),
            'capabilities': {
                'mapping': True,
                'navigation': True,
                'docking': True,
                'docking_method': 'apriltag',
                'camera': True,
                'virtual_walls': False,
                'fixed_routes': False,
                'missions': False,
            },
        })


class HealthHandler(BaseHandler):
    """Per-subsystem freshness, plus one overall verdict.

    Reports each source separately rather than a single boolean: "the robot is
    unhealthy" is not actionable, "lidar last published 40s ago" is.
    """

    def get(self) -> None:
        sources = {}
        for label, (key, limit) in _FRESH_LIMITS.items():
            _value, age = self.bridge.get_with_age(key)
            sources[label] = {
                'ok': age is not None and age <= limit,
                'age_sec': age,
                'limit_sec': limit,
            }
        try:
            usage = shutil.disk_usage('/')
            disk = {'total_gb': round(usage.total / 1e9, 1),
                    'free_gb': round(usage.free / 1e9, 1),
                    'used_percent': round(100.0 * usage.used / usage.total, 1)}
        except OSError:
            disk = None

        self.send({
            'healthy': all(s['ok'] for s in sources.values()),
            'sources': sources,
            'cpu_temperature_c': self.bridge.get('cpu_temperature'),
            'disk': disk,
        })
