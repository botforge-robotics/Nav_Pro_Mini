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

        pose, _age = bridge.get_with_age('pose_map')
        localized = pose is not None
        if pose is None:
            pose, _age = bridge.get_with_age('pose_odom')

        map_name = mode_state.map_name or store.current_map()

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
            'map': {'id': map_name, 'name': map_name},
            'localization': {
                'status': 'LOCALIZED' if localized else 'UNKNOWN',
                'x': pose.get('x') if pose else None,
                'y': pose.get('y') if pose else None,
                'yaw': pose.get('theta') if pose else None,
            },
            # goal_id: this SDK tracks the in-flight goal by its target, not
            # a discrete id — null rather than fabricating one (see
            # ros_bridge.py's module docstring on null vs. an invented value).
            'navigation': {'status': nav_tracker.state, 'goal_id': None},
            'mission': {'status': mission_runner.state,
                       'mission_id': mission_runner.mission_id},
            'dock': {'configured': bridge.get('dock_pose') is not None,
                    'status': bridge.get('dock_status'),
                    'operation': dock_tracker.state},
            'error': lifecycle['detail'] if lifecycle['lifecycle'] == 'ERROR' else None,
        })
