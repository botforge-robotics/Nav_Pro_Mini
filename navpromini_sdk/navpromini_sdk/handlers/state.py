#!/usr/bin/env python3
"""Live telemetry reads. Every response is a cache lookup — never blocking."""

from __future__ import annotations

from .base import BaseHandler


class PoseHandler(BaseHandler):
    """Robot pose, preferring the map frame.

    Falls back to odom when AMCL is not running (idle or mapping mode) and says
    which frame it used. An odom pose silently labelled "map" would be a
    genuinely dangerous lie for anything that stores waypoints.
    """

    def get(self) -> None:
        value, age = self.bridge.get_with_age('pose_map')
        if value is None:
            value, age = self.bridge.get_with_age('pose_odom')
        if value is None:
            self.fail(503, 'no_data', 'No pose available — is the robot started?')
            return
        self.send({'data': value, 'age_sec': age,
                   'localized': value.get('frame') == 'map'})


class VelocityHandler(BaseHandler):
    def get(self) -> None:
        self.send(self.cached('velocity', 'odometry'))


class BatteryHandler(BaseHandler):
    def get(self) -> None:
        payload = self.cached('battery', 'battery')
        extra = self.bridge.get('battery_info')
        if extra:
            payload['detail'] = extra
        self.send(payload)


class ImuHandler(BaseHandler):
    def get(self) -> None:
        self.send(self.cached('imu', 'IMU'))


class ScanHandler(BaseHandler):
    def get(self) -> None:
        self.send(self.cached('scan', 'laser scan'))


class TemperatureHandler(BaseHandler):
    def get(self) -> None:
        battery = self.bridge.get('battery') or {}
        self.send({
            'cpu_c': self.bridge.get('cpu_temperature'),
            'battery_c': battery.get('temperature'),
        })
