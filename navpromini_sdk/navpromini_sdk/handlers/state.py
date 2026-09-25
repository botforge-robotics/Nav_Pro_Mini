#!/usr/bin/env python3
"""Live telemetry reads. Every response is a cache lookup — never blocking."""

from __future__ import annotations

import os
import time

from .base import BaseHandler

_last_cpu_sample: tuple[float, float, float] | None = None
_last_cpu_util_pct: float | None = None


def _get_cpu_load_pct() -> float | None:
    """Calculates instantaneous CPU utilization (%) from /proc/stat.

    Uses consecutive sample deltas (total vs idle ticks) so it reports
    true processor utilization (0.0% - 100.0%) instead of system load average.
    """
    global _last_cpu_sample, _last_cpu_util_pct
    try:
        with open('/proc/stat', 'r') as f:
            first_line = f.readline()
        if not first_line.startswith('cpu '):
            return None
        parts = [float(x) for x in first_line.split()[1:]]
        idle = parts[3] + (parts[4] if len(parts) > 4 else 0.0)
        total = sum(parts)
        now = time.monotonic()

        if _last_cpu_sample is not None:
            prev_total, prev_idle, prev_ts = _last_cpu_sample
            d_total = total - prev_total
            d_idle = idle - prev_idle
            dt = now - prev_ts
            if d_total > 0 and dt >= 0.1:
                pct = 100.0 * (1.0 - (d_idle / d_total))
                _last_cpu_util_pct = round(min(100.0, max(0.0, pct)), 1)
                _last_cpu_sample = (total, idle, now)
        else:
            time.sleep(0.04)
            with open('/proc/stat', 'r') as f:
                line2 = f.readline()
            parts2 = [float(x) for x in line2.split()[1:]]
            idle2 = parts2[3] + (parts2[4] if len(parts2) > 4 else 0.0)
            total2 = sum(parts2)
            d_total = total2 - total
            d_idle = idle2 - idle
            if d_total > 0:
                _last_cpu_util_pct = round(
                    min(100.0, max(0.0, 100.0 * (1.0 - d_idle / d_total))), 1
                )
            _last_cpu_sample = (total2, idle2, time.monotonic())

        return _last_cpu_util_pct
    except Exception:
        return None


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
            'cpu_load_pct': _get_cpu_load_pct(),
        })



class RobotStateHandler(BaseHandler):
    """The doc's full canonical Robot State (§18) as one composed object —
    {robot, lifecycle, mode, connection, hardware, battery, map,
    localization, navigation, mission, dock, error} — for a client that
    wants the whole picture in one call, or wants to mirror the doc's model
    1:1 rather than assembling it itself from several endpoints.

    Purely compositional: every field is read from state already
    tracked/cached elsewhere in this handler set (system.py's lifecycle/
    health, mode.py's ModeState, navigation.py's/docking.py's/missions.py's
    trackers, RosBridge's cache) — no new ROS subscriptions, and this does
    not replace those individual endpoints, which stay cheaper for a client
    that only needs one field.
    """

    def get(self) -> None:
        # Lazy imports: avoids asserting a module-load order between state.py
        # and system.py/mode.py/navigation.py/docking.py/missions.py, same
        # reasoning as the lazy imports already used between mode.py/maps.py.
        from . import system
        from .docking import TRACKER as dock_tracker
        from .missions import RUNNER as mission_runner
        from .navigation import TRACKER as nav_tracker

        bridge = self.bridge
        mode_state = self.opts['mode_state']
        store = self.opts['store']

        lifecycle = system.lifecycle_snapshot()
        health = system.health_sources(bridge)
        battery = bridge.get('battery') or {}

        pose, age = bridge.get_with_age('pose_map')
        cov_x = pose.get('cov_x', 0.0) if pose else 0.0
        cov_y = pose.get('cov_y', 0.0) if pose else 0.0
        # In ROS 2 AMCL, /amcl_pose only updates when the robot moves (update_min_d / update_min_a).
        # When stationary at dock or room, age naturally grows while remaining valid and latched.
        # The robot is truly unlocalized only if no pose_map exists or if covariance is dispersed (> 0.45m^2).
        is_high_cov = (cov_x > 0.45 or cov_y > 0.45)
        localized = (pose is not None) and (not is_high_cov)
        if pose is None:
            pose, age = bridge.get_with_age('pose_odom')

        map_name = mode_state.map_name if mode_state.mode == 'navigation' else None

        self.send({
            'robot': system.robot_identity(),
            'lifecycle': lifecycle['lifecycle'],
            'mode': mode_state.mode,
            'connection': {'wifi': system.wifi_online()},
            # Doc's example keys (motor/encoder/...) don't map to distinct
            # topics this SDK actually observes — reporting the real signal
            # names (see system._FRESH_LIMITS) rather than inventing a
            # motor/encoder split with no data behind it.
            'hardware': {label: ('OK' if s['ok'] else 'FAULT')
                        for label, s in health.items()},
            'battery': {'percentage': battery.get('percentage'),
                       'charging': bool(battery.get('charging'))},
            'map': {'id': map_name, 'name': map_name} if map_name else None,
            'is_localized': localized,
            'localization': {
                'status': 'LOCALIZED' if localized else ('POOR' if is_high_cov else 'UNKNOWN'),
                'x': pose.get('x') if pose else None,
                'y': pose.get('y') if pose else None,
                'yaw': pose.get('theta') if pose else None,
                'cov_x': cov_x,
                'cov_y': cov_y,
                'age_sec': age,
            },
            # goal_id: this SDK tracks the in-flight goal by its target, not
            # a discrete id — null rather than fabricating one (see
            # ros_bridge.py's module docstring on null vs. an invented value).
            'navigation': {'status': nav_tracker.state, 'goal_id': None},
            'mission': {'status': mission_runner.state,
                       'mission_id': mission_runner.mission_id,
                       'pause_reason': mission_runner.pause_reason},
            'dock': {'configured': bridge.get('dock_pose') is not None,
                    'status': bridge.get('dock_status'),
                    'operation': dock_tracker.state},
            'system': {
                'cpu_load_pct': _get_cpu_load_pct(),
                'cpu_temperature_c': bridge.get('cpu_temperature'),
                'battery_temperature_c': battery.get('temperature'),
            },
            'error': lifecycle['detail'] if lifecycle['lifecycle'] == 'ERROR' else None,
        })

