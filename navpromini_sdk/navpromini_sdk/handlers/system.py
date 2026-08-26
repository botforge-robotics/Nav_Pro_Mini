#!/usr/bin/env python3
"""System identity and health."""

from __future__ import annotations

import os
import shutil
import socket
import subprocess
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

# Lifecycle (doc §4/§5): system readiness, separate from operating `mode`
# (idle/mapping/navigation — see handlers/mode.py).
#
# navpro-sdk.service only Requires=navpro-robot.service, which itself has no
# dependency on Wi-Fi/provisioning at all (After=network-online.target only)
# — so this process is typically alive and reachable (over the setup
# hotspot's own subnet, 10.42.0.1, before site Wi-Fi even joins) throughout
# PROVISIONING and WIFI_CONNECTING too, not just after. Detected the same way
# navpromini_setup's status_display_node.py already does (nmcli), rather than
# inventing a second source of truth for "is the setup AP up" / "is site
# Wi-Fi online". Only BOOTING (before any navpro-*.service has started at
# all) is genuinely unobservable from here — a client sees connection-refused
# during that phase, which is the correct signal.
#
# Computed on a timer (tick_lifecycle, driven from server.py alongside the
# existing mode.reconcile_mode timer) rather than per-request, for the same
# reason GET /mode is a cache read: it stays cheap under load (the nmcli
# calls below are real subprocess spawns), and events fire on the actual
# transition instead of only when polled.
_LIFECYCLE_GRACE_SEC = 30.0
_started_at = time.monotonic()
_lifecycle_cache = {'lifecycle': 'HARDWARE_STARTING', 'since_sec': 0.0, 'detail': ''}
# Edge-detected separately from the READY/ERROR lifecycle classification
# above (which is grace-period-gated, deliberately tolerant of normal
# startup warm-up — see _LIFECYCLE_GRACE_SEC). hardware.error is the doc's
# per-fault signal (§20) and should fire the moment a required source goes
# stale, including well after boot — e.g. a LiDAR cable pulled hours into a
# READY session, long past any startup grace period.
_failing_sources_last: frozenset = frozenset()

_SETUP_AP_CONN = 'navpro-setup-ap'


def _setup_ap_active() -> bool:
    """True only if nmcli reports the setup hotspot connection actually up —
    mirrors status_display_node.py's _setup_ap_really_up()."""
    try:
        r = subprocess.run(['nmcli', '-t', '-f', 'NAME,STATE', 'connection', 'show', '--active'],
                           capture_output=True, text=True, timeout=5)
        for line in (r.stdout or '').splitlines():
            if line.startswith(f'{_SETUP_AP_CONN}:') and 'activated' in line.lower():
                return True
    except Exception:  # noqa: BLE001
        pass
    return False


def _wifi_site_online() -> bool:
    """True if a Wi-Fi device is connected to a real (non-setup-AP) network
    with an IP — mirrors status_display_node.py's _wifi_site_online()."""
    try:
        r = subprocess.run(['nmcli', '-t', '-f', 'DEVICE,TYPE,STATE,CONNECTION', 'device', 'status'],
                           capture_output=True, text=True, timeout=5)
        for line in (r.stdout or '').splitlines():
            parts = line.split(':')
            if len(parts) < 4:
                continue
            _dev, dtype, state, conn = parts[0], parts[1], parts[2], parts[3]
            if dtype != 'wifi' or state != 'connected' or not conn or conn == _SETUP_AP_CONN:
                continue
            ip = subprocess.run(['hostname', '-I'], capture_output=True, text=True, timeout=3)
            if ip.returncode == 0 and (ip.stdout or '').strip():
                return True
    except Exception:  # noqa: BLE001
        pass
    return False


def _has_wifi_config() -> bool:
    """Whether /etc/navpro/robot.yaml (or the older fleet.yaml) exists —
    the doc's own "Wi-Fi credentials?" branch condition (§5). Imported
    lazily/defensively, same reasoning as _robot_identity() below."""
    try:
        from navpromini_setup.robot_config import config_path_present
        return config_path_present()
    except Exception:  # noqa: BLE001
        return False


def tick_lifecycle(bridge) -> None:
    global _lifecycle_cache, _failing_sources_last

    if _setup_ap_active():
        state = 'PROVISIONING'
        detail = 'setup hotspot active — no Wi-Fi credentials configured yet'
    elif not _wifi_site_online():
        state = 'WIFI_CONNECTING'
        detail = ('joining saved Wi-Fi' if _has_wifi_config()
                  else 'no Wi-Fi credentials yet and no setup hotspot up')
    else:
        failing = []
        for label, (key, limit) in _FRESH_LIMITS.items():
            _value, age = bridge.get_with_age(key)
            if age is None or age > limit:
                failing.append(label)

        failing_now = frozenset(failing)
        if failing_now and failing_now != _failing_sources_last:
            new = failing_now - _failing_sources_last
            bridge.emit_event('hardware.error',
                              {'sources': sorted(new), 'all_failing': sorted(failing_now)})
        _failing_sources_last = failing_now

        elapsed = time.monotonic() - _started_at
        if not failing:
            state, detail = 'READY', ''
        elif elapsed < _LIFECYCLE_GRACE_SEC:
            state = 'HARDWARE_STARTING'
            detail = f'waiting on: {", ".join(failing)}'
        else:
            state = 'ERROR'
            detail = f'{", ".join(failing)} unavailable — navigation cannot start'

    if state != _lifecycle_cache['lifecycle']:
        if state in ('READY', 'ERROR'):
            bridge.emit_event('robot.ready' if state == 'READY' else 'robot.error',
                              {'lifecycle': state, 'detail': detail})
        bridge.get_logger().info(f'lifecycle: {_lifecycle_cache["lifecycle"]} -> {state}'
                                 + (f' ({detail})' if detail else ''))
    _lifecycle_cache = {'lifecycle': state,
                        'since_sec': round(time.monotonic() - _started_at, 1),
                        'detail': detail}


class LifecycleHandler(BaseHandler):
    def get(self) -> None:
        self.send(_lifecycle_cache)


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


robot_identity = _robot_identity  # public alias — reused by state.RobotStateHandler


def health_sources(bridge) -> dict:
    """Per-subsystem freshness (same computation HealthHandler.get() sends as
    'sources'), factored out so state.RobotStateHandler can compose it into
    the doc's full Robot State model (§18) without duplicating the loop."""
    sources = {}
    for label, (key, limit) in _FRESH_LIMITS.items():
        _value, age = bridge.get_with_age(key)
        sources[label] = {
            'ok': age is not None and age <= limit,
            'age_sec': age,
            'limit_sec': limit,
        }
    return sources


def lifecycle_snapshot() -> dict:
    """Current lifecycle state — same value GET /system/lifecycle returns.
    Reused by state.RobotStateHandler to avoid a second HTTP round-trip."""
    return dict(_lifecycle_cache)


def wifi_online() -> bool:
    """Public alias of the tick_lifecycle Wi-Fi check, for
    state.RobotStateHandler's `connection.wifi` field."""
    return _wifi_site_online()


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
                'missions': True,
            },
        })


class HealthHandler(BaseHandler):
    """Per-subsystem freshness, plus one overall verdict.

    Reports each source separately rather than a single boolean: "the robot is
    unhealthy" is not actionable, "lidar last published 40s ago" is.
    """

    def get(self) -> None:
        sources = health_sources(self.bridge)
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
