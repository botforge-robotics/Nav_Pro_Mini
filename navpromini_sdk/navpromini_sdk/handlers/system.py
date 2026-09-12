#!/usr/bin/env python3
"""System identity and health."""

from __future__ import annotations

import json
import os
from pathlib import Path
import shutil
import socket
import subprocess
import time

import tornado.ioloop

from .base import ApiError, BaseHandler

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
_SETUP_AP_PREFIX = 'NavPro-Setup'


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


UPDATE_STATUS_FILE = Path('/var/lib/navpro/update_status.json')
UPDATE_LOG_FILE = Path('/var/log/navpro/update.log')
DEFAULT_WS = Path('/home/navpromini/NavProMini_ws')


def _get_ws_path() -> Path:
    ws_env = os.environ.get('NAVPRO_WS')
    if ws_env:
        return Path(ws_env)
    user_home = Path.home()
    if (user_home / 'NavProMini_ws').is_dir():
        return user_home / 'NavProMini_ws'
    return DEFAULT_WS


def _get_git_info(src_dir: Path) -> dict:
    info = {
        'current_commit': 'unknown',
        'current_commit_short': 'unknown',
        'current_commit_message': '',
        'current_commit_date': '',
        'branch': 'nav2',
        'remote_branch': 'origin/nav2',
        'latest_commit': None,
        'commits_behind': 0,
        'changelog': [],
    }
    if not (src_dir / '.git').is_dir():
        return info

    try:
        r = subprocess.run(['git', '-c', 'safe.directory=*', '-C', str(src_dir), 'rev-parse', '--abbrev-ref', 'HEAD'],
                           capture_output=True, text=True, timeout=3)
        if r.returncode == 0 and r.stdout.strip():
            info['branch'] = r.stdout.strip()
            info['remote_branch'] = f"origin/{info['branch']}"

        r = subprocess.run(['git', '-c', 'safe.directory=*', '-C', str(src_dir), 'log', '-1', '--format=%H%n%h%n%s%n%ci'],
                           capture_output=True, text=True, timeout=3)
        if r.returncode == 0 and r.stdout.strip():
            lines = r.stdout.strip().splitlines()
            if len(lines) >= 4:
                info['current_commit'] = lines[0]
                info['current_commit_short'] = lines[1]
                info['current_commit_message'] = lines[2]
                info['current_commit_date'] = lines[3]

        r = subprocess.run(['git', '-c', 'safe.directory=*', '-C', str(src_dir), 'rev-list', '--count', f"HEAD..{info['remote_branch']}"],
                           capture_output=True, text=True, timeout=3)
        if r.returncode == 0 and r.stdout.strip().isdigit():
            info['commits_behind'] = int(r.stdout.strip())

        if info['commits_behind'] > 0:
            r = subprocess.run(['git', '-c', 'safe.directory=*', '-C', str(src_dir), 'log', '-n', '20', '--format=%h %s', f"HEAD..{info['remote_branch']}"],
                               capture_output=True, text=True, timeout=3)
            if r.returncode == 0 and r.stdout.strip():
                info['changelog'] = r.stdout.strip().splitlines()

            r = subprocess.run(['git', '-c', 'safe.directory=*', '-C', str(src_dir), 'rev-parse', info['remote_branch']],
                               capture_output=True, text=True, timeout=3)
            if r.returncode == 0 and r.stdout.strip():
                info['latest_commit'] = r.stdout.strip()
        else:
            info['latest_commit'] = info['current_commit']
    except Exception:  # noqa: BLE001
        pass
    return info


def _evaluate_update_safety(bridge, opts: dict) -> tuple[bool, list[str]]:
    blockers = []
    # 1. Motion check
    odom = bridge.get('pose_odom')
    if odom and isinstance(odom, dict):
        twist = odom.get('twist', {}).get('twist', {})
        vx = twist.get('linear', {}).get('x', 0.0)
        wz = twist.get('angular', {}).get('z', 0.0)
        if abs(vx) > 0.05 or abs(wz) > 0.05:
            blockers.append(f'Robot is moving (linear: {vx:.2f} m/s, angular: {wz:.2f} rad/s)')

    # 2. Mode check
    mode_state = opts.get('mode_state')
    if mode_state and getattr(mode_state, 'current_mode', None) == 'mapping':
        blockers.append('Mapping mode is active — finish or cancel mapping first')

    # 3. Mission runner check
    mission_runner = opts.get('mission_runner')
    if mission_runner and getattr(mission_runner, 'is_active', False):
        blockers.append('A mission is currently in progress')

    # 4. Battery check
    battery = bridge.get('battery')
    if battery and isinstance(battery, dict):
        percentage = battery.get('percentage')
        charging = battery.get('charging', False) or battery.get('power_supply_status') == 1
        if percentage is not None and percentage < 30 and not charging:
            blockers.append(f'Battery is low ({percentage:.0f}%), connect charger or dock before updating')

    # 5. Disk space check
    try:
        usage = shutil.disk_usage('/')
        if usage.free < 1.2e9:
            blockers.append(f'Insufficient disk space: {usage.free / 1e9:.1f} GB free (require >= 1.2 GB)')
    except OSError:
        pass

    return (len(blockers) == 0, blockers)


class UpdatesHandler(BaseHandler):
    def get(self) -> None:
        ws = _get_ws_path()
        git_info = _get_git_info(ws / 'src')
        can_update, blockers = _evaluate_update_safety(self.bridge, self.opts)

        last_status = None
        if UPDATE_STATUS_FILE.is_file():
            try:
                last_status = json.loads(UPDATE_STATUS_FILE.read_text())
            except Exception:
                pass

        self.send({
            'update_available': git_info['commits_behind'] > 0,
            'current_commit': git_info['current_commit'],
            'current_commit_short': git_info['current_commit_short'],
            'current_commit_message': git_info['current_commit_message'],
            'current_commit_date': git_info['current_commit_date'],
            'branch': git_info['branch'],
            'remote_branch': git_info['remote_branch'],
            'latest_commit': git_info['latest_commit'],
            'commits_behind': git_info['commits_behind'],
            'changelog': git_info['changelog'],
            'safety': {
                'can_update': can_update,
                'blockers': blockers,
            },
            'last_update': last_status,
        })


class UpdateCheckHandler(BaseHandler):
    async def post(self) -> None:
        ws = _get_ws_path()
        src_dir = ws / 'src'
        branch = 'nav2'
        try:
            r = subprocess.run(['git', '-c', 'safe.directory=*', '-C', str(src_dir), 'rev-parse', '--abbrev-ref', 'HEAD'],
                               capture_output=True, text=True, timeout=3)
            if r.returncode == 0 and r.stdout.strip():
                branch = r.stdout.strip()
            fetch_cmd = ['git', '-c', 'safe.directory=*', '-C', str(src_dir), 'fetch', 'origin', branch]
            try:
                import pwd
                owner = pwd.getpwuid(src_dir.stat().st_uid).pw_name
                if os.geteuid() == 0 and owner != 'root':
                    fetch_cmd = ['sudo', '-u', owner] + fetch_cmd
            except Exception:
                pass

            await tornado.ioloop.IOLoop.current().run_in_executor(
                None,
                lambda: subprocess.run(fetch_cmd, capture_output=True, text=True, timeout=30)
            )
        except Exception as exc:
            raise ApiError(500, 'fetch_failed', f'git fetch failed: {exc}')

        git_info = _get_git_info(src_dir)
        can_update, blockers = _evaluate_update_safety(self.bridge, self.opts)
        self.send({
            'checked': True,
            'update_available': git_info['commits_behind'] > 0,
            'commits_behind': git_info['commits_behind'],
            'current_commit': git_info['current_commit'],
            'latest_commit': git_info['latest_commit'],
            'changelog': git_info['changelog'],
            'safety': {
                'can_update': can_update,
                'blockers': blockers,
            },
        })


class UpdateApplyHandler(BaseHandler):
    def post(self) -> None:
        can_update, blockers = _evaluate_update_safety(self.bridge, self.opts)
        if not can_update:
            raise ApiError(409, 'safety_check_failed',
                           f"Cannot apply update: {'; '.join(blockers)}",
                           {'blockers': blockers})

        # Check if already running
        if UPDATE_STATUS_FILE.is_file():
            try:
                cur = json.loads(UPDATE_STATUS_FILE.read_text())
                if cur.get('phase') in ('in_progress', 'pulling', 'building', 'restarting'):
                    ts = cur.get('timestamp', 0)
                    if time.time() - ts < 600:  # 10 minutes
                        raise ApiError(409, 'update_already_in_progress',
                                       'An update is already in progress',
                                       {'phase': cur.get('phase'), 'progress': cur.get('progress')})
            except (ApiError, tornado.web.HTTPError):
                raise
            except Exception:
                pass

        # Script location
        script_path = Path('/opt/navpro/scripts/update_companion.sh')
        if not script_path.is_file():
            ws = _get_ws_path()
            src_script = ws / 'src' / 'navpromini_setup' / 'scripts' / 'update_companion.sh'
            if src_script.is_file():
                script_path = src_script
            else:
                raise ApiError(500, 'updater_not_found', 'update_companion.sh not found on system')

        try:
            # Popen detached session so it survives SDK process restarts
            subprocess.Popen(['/bin/bash', str(script_path)],
                             start_new_session=True,
                             stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL)
        except Exception as exc:
            raise ApiError(500, 'spawn_failed', f'Failed to launch updater: {exc}')

        self.send({
            'status': 'started',
            'message': 'Companion software update process initiated in background',
        })


class UpdateStatusHandler(BaseHandler):
    def get(self) -> None:
        status = {
            'phase': 'idle',
            'progress': 0,
            'message': 'No update in progress',
            'commit': None,
            'error': None,
            'timestamp': None,
        }
        if UPDATE_STATUS_FILE.is_file():
            try:
                status = json.loads(UPDATE_STATUS_FILE.read_text())
            except Exception:
                pass

        log_tail = []
        if UPDATE_LOG_FILE.is_file():
            try:
                lines = UPDATE_LOG_FILE.read_text().splitlines()
                log_tail = lines[-60:]
            except Exception:
                pass

        self.send({
            'status': status,
            'log_tail': log_tail,
        })


class ToggleKeyboardHandler(BaseHandler):
    """Toggle onboard virtual keyboard on the robot's local screen (:0)."""
    def post(self) -> None:
        try:
            # 1. Primary: busctl under navpromini user session bus
            res = subprocess.run([
                'sudo', '-u', 'navpromini',
                'XDG_RUNTIME_DIR=/run/user/1000',
                'DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus',
                'busctl', '--user', 'call',
                'org.onboard.Onboard',
                '/org/onboard/Onboard/Keyboard',
                'org.onboard.Onboard.Keyboard', 'ToggleVisible'
            ], capture_output=True, timeout=2)

            if res.returncode == 0:
                self.send({'status': 'ok', 'action': 'toggled_busctl'})
                return

            # 2. Check if onboard is running; if not launch it as navpromini
            p = subprocess.run(['pgrep', '-f', 'onboard'], capture_output=True, text=True)
            if p.returncode != 0:
                subprocess.Popen([
                    'sudo', '-u', 'navpromini',
                    'DISPLAY=:0',
                    'XAUTHORITY=/home/navpromini/.Xauthority',
                    '/usr/bin/onboard'
                ], start_new_session=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                self.send({'status': 'ok', 'action': 'launched'})
                return

            # 3. Fallback: signal
            subprocess.run(['pkill', '-USR1', '-f', 'onboard'], timeout=2)
            self.send({'status': 'ok', 'action': 'toggled_signal'})
        except Exception as exc:
            raise ApiError(500, 'keyboard_toggle_failed', str(exc))


class WifiStatusHandler(BaseHandler):
    """GET /api/v1/system/wifi/status - returns Wi-Fi connection info and IP."""
    def get(self) -> None:
        connected = _wifi_site_online()
        ssid = ""
        ip = ""
        interface = "wlan0"
        try:
            # 1. Try active dev wifi first to get broadcast SSID
            r_act = subprocess.run(['nmcli', '-t', '-f', 'ACTIVE,SSID', 'dev', 'wifi'],
                                   capture_output=True, text=True, timeout=5)
            for line in (r_act.stdout or '').splitlines():
                if line.startswith('yes:'):
                    ssid = line[4:].strip()
                    break

            # 2. Fallback to device status connection
            if not ssid:
                r = subprocess.run(['nmcli', '-t', '-f', 'DEVICE,TYPE,STATE,CONNECTION', 'device', 'status'],
                                   capture_output=True, text=True, timeout=5)
                for line in (r.stdout or '').splitlines():
                    parts = line.split(':')
                    if len(parts) >= 4 and parts[1] == 'wifi':
                        interface = parts[0]
                        if parts[2] == 'connected' and parts[3] != _SETUP_AP_CONN:
                            ssid = parts[3].strip()
                            break

            # Strip any internal netplan prefix if present (e.g. netplan-wlan0-MySSID -> MySSID)
            if ssid.startswith('netplan-'):
                sub_parts = ssid.split('-', 2)
                ssid = sub_parts[-1] if len(sub_parts) > 2 else ssid

            ip_cmd = subprocess.run(['hostname', '-I'], capture_output=True, text=True, timeout=3)
            if ip_cmd.returncode == 0:
                ip = (ip_cmd.stdout or '').strip().split(' ')[0]
        except Exception:
            pass

        self.send({
            'connected': connected,
            'ssid': ssid,
            'ip': ip,
            'interface': interface,
            'hotspot_active': _setup_ap_active()
        })


class WifiScanHandler(BaseHandler):
    """GET /api/v1/system/wifi/scan - scan nearby available Wi-Fi networks."""
    def get(self) -> None:
        networks = []
        seen = set()
        try:
            # Try with --rescan yes first, fallback to cached scan
            r = subprocess.run(['nmcli', '-t', '-f', 'SSID,SIGNAL,SECURITY', 'device', 'wifi', 'list', '--rescan', 'yes'],
                               capture_output=True, text=True, timeout=10)
            if r.returncode != 0:
                r = subprocess.run(['nmcli', '-t', '-f', 'SSID,SIGNAL,SECURITY', 'device', 'wifi', 'list'],
                                   capture_output=True, text=True, timeout=6)
            for line in (r.stdout or '').splitlines():
                parts = line.split(':')
                if len(parts) >= 2:
                    raw_ssid = parts[0].strip()
                    if not raw_ssid or raw_ssid.startswith(_SETUP_AP_PREFIX):
                        continue
                    # Clean netplan prefix if present
                    display_ssid = raw_ssid
                    if display_ssid.startswith('netplan-'):
                        sub_parts = display_ssid.split('-', 2)
                        display_ssid = sub_parts[-1] if len(sub_parts) > 2 else display_ssid

                    if display_ssid in seen:
                        continue
                    seen.add(display_ssid)

                    try:
                        signal = int(parts[1])
                    except (ValueError, IndexError):
                        signal = 50
                    sec = parts[2] if len(parts) > 2 else ""
                    networks.append({
                        'ssid': display_ssid,
                        'raw_ssid': raw_ssid,
                        'signal': signal,
                        'security': sec,
                        'protected': bool(sec and sec != '--')
                    })
        except Exception:
            pass
        self.send({'networks': networks, 'count': len(networks)})


class WifiConnectHandler(BaseHandler):
    """POST /api/v1/system/wifi/connect - join a Wi-Fi network."""
    def post(self) -> None:
        data = self.body(('ssid',))
        ssid = str(data['ssid']).strip()
        password = str(data.get('password') or '').strip()
        if not ssid:
            raise ApiError(400, 'invalid_ssid', 'SSID cannot be empty')

        cmd = ['nmcli', 'device', 'wifi', 'connect', ssid]
        if password:
            cmd.extend(['password', password])

        try:
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
            if r.returncode == 0:
                self.bridge.emit_event('wifi.connected', {'ssid': ssid})
                self.send({'success': True, 'ssid': ssid, 'output': r.stdout})
            else:
                raise ApiError(400, 'wifi_connect_failed', r.stderr or r.stdout or 'Failed to connect to Wi-Fi')
        except subprocess.TimeoutExpired:
            raise ApiError(504, 'wifi_timeout', 'Connection to Wi-Fi timed out')
        except Exception as exc:
            raise ApiError(500, 'wifi_error', str(exc))


# -----------------------------------------------------------------------------
# Robot UI App Self-Update & 1-Level Backup Rollback Engine
# -----------------------------------------------------------------------------
APP_STATUS_FILE = Path('/tmp/navpro_app_update_status.json')
APP_BASE_DIR = Path('/home/navpromini/navpromini_robot_ui_app')
APP_CURRENT_DIR = APP_BASE_DIR / 'current'
APP_BACKUP_DIR = APP_BASE_DIR / 'backup'
APP_TEMP_DIR = APP_BASE_DIR / 'temp'
APP_LEGACY_DIR = Path('/home/navpromini/navpromini_robot_ui')

def _read_current_app_version() -> str:
    for candidate in [APP_CURRENT_DIR / 'version.json', APP_LEGACY_DIR / 'version.json']:
        if candidate.is_file():
            try:
                data = json.loads(candidate.read_text())
                return data.get('version', '1.0.0')
            except Exception:
                pass
    return '1.0.0'

def _compare_semver(v1: str, v2: str) -> int:
    def parse(v):
        return [int(x) if x.isdigit() else 0 for x in v.lstrip('v').split('.')]
    p1, p2 = parse(v1), parse(v2)
    for i in range(max(len(p1), len(p2))):
        n1 = p1[i] if i < len(p1) else 0
        n2 = p2[i] if i < len(p2) else 0
        if n1 > n2:
            return 1
        if n1 < n2:
            return -1
    return 0

class AppUpdateCheckHandler(BaseHandler):
    """GET /api/v1/system/app/update/check - check GitHub releases for new Robot UI app release."""
    def get(self) -> None:
        cur_ver = _read_current_app_version()
        repo = 'botforge-robotics/navpromini_robot_ui'
        url = f'https://api.github.com/repos/{repo}/releases/latest'
        
        info = {
            'current_version': cur_ver,
            'latest_version': cur_ver,
            'update_available': False,
            'release_name': f'v{cur_ver}',
            'release_notes': '',
            'published_at': '',
            'download_url': None,
            'asset_name': None,
            'asset_size': 0
        }

        try:
            import urllib.request
            req = urllib.request.Request(url, headers={'User-Agent': 'NavProMini-App', 'Accept': 'application/vnd.github.v3+json'})
            with urllib.request.urlopen(req, timeout=5) as resp:
                if resp.status == 200:
                    data = json.loads(resp.read().decode('utf-8'))
                    tag = data.get('tag_name', '').lstrip('v')
                    if tag:
                        info['latest_version'] = tag
                        info['release_name'] = data.get('name') or f'v{tag}'
                        info['release_notes'] = data.get('body', '')
                        info['published_at'] = data.get('published_at', '')
                        if _compare_semver(tag, cur_ver) > 0:
                            info['update_available'] = True
                        
                        assets = data.get('assets', [])
                        target_asset = None
                        for a in assets:
                            name = a.get('name', '').lower()
                            if 'aarch64' in name and name.endswith('.appimage'):
                                target_asset = a
                                break
                            elif name.endswith('.appimage') and not target_asset:
                                target_asset = a
                            elif name.endswith('.zip') and not target_asset:
                                target_asset = a
                        if target_asset:
                            info['download_url'] = target_asset.get('browser_download_url')
                            info['asset_name'] = target_asset.get('name')
                            info['asset_size'] = target_asset.get('size', 0)
        except Exception as exc:
            info['check_error'] = str(exc)

        self.send(info)

class AppUpdateApplyHandler(BaseHandler):
    """POST /api/v1/system/app/update/apply - auto-download, rotate 1-level backup, and restart."""
    def post(self) -> None:
        data = self.body()
        download_url = data.get('download_url')
        target_version = data.get('target_version', 'latest')

        if APP_STATUS_FILE.is_file():
            try:
                st = json.loads(APP_STATUS_FILE.read_text())
                if st.get('state') in ('downloading', 'backing_up', 'applying'):
                    raise ApiError(409, 'update_in_progress', 'An app update is already in progress')
            except Exception:
                pass

        def run_update_thread():
            import urllib.request
            import threading
            try:
                APP_BASE_DIR.mkdir(parents=True, exist_ok=True)
                APP_CURRENT_DIR.mkdir(parents=True, exist_ok=True)
                APP_TEMP_DIR.mkdir(parents=True, exist_ok=True)

                def update_progress(state, progress, msg, error=None):
                    APP_STATUS_FILE.write_text(json.dumps({
                        'state': state,
                        'progress': progress,
                        'message': msg,
                        'error': error,
                        'timestamp': time.time()
                    }))

                update_progress('downloading', 15, 'Contacting release servers...')
                url = download_url
                if not url:
                    api_url = 'https://api.github.com/repos/botforge-robotics/navpromini_robot_ui/releases/latest'
                    req = urllib.request.Request(api_url, headers={'User-Agent': 'NavProMini-App'})
                    with urllib.request.urlopen(req, timeout=6) as r:
                        rel = json.loads(r.read().decode())
                        for a in rel.get('assets', []):
                            if a.get('name', '').endswith('.AppImage'):
                                url = a.get('browser_download_url')
                                break

                if not url:
                    update_progress('failed', 0, 'No downloadable AppImage asset found', error='Asset not found')
                    return

                target_file = APP_TEMP_DIR / 'NavProMiniRobotUI-aarch64.AppImage'
                update_progress('downloading', 30, 'Downloading release package...')
                
                req = urllib.request.Request(url, headers={'User-Agent': 'NavProMini-App'})
                with urllib.request.urlopen(req, timeout=60) as resp, open(target_file, 'wb') as out_f:
                    total = int(resp.headers.get('Content-Length', 0))
                    dl = 0
                    while True:
                        chunk = resp.read(65536)
                        if not chunk:
                            break
                        out_f.write(chunk)
                        dl += len(chunk)
                        if total > 0:
                            pct = 30 + int((dl / total) * 45)
                            update_progress('downloading', pct, f'Downloading {dl // 1048576}MB of {total // 1048576}MB...')

                target_file.chmod(0o755)

                # Rotate backup: delete old backup, move current to backup
                update_progress('backing_up', 80, 'Rotating previous version backup...')
                if APP_BACKUP_DIR.exists():
                    shutil.rmtree(APP_BACKUP_DIR)
                APP_BACKUP_DIR.mkdir(parents=True, exist_ok=True)

                if any(APP_CURRENT_DIR.iterdir()):
                    for item in APP_CURRENT_DIR.iterdir():
                        dest = APP_BACKUP_DIR / item.name
                        if item.is_dir():
                            shutil.copytree(item, dest)
                        else:
                            shutil.copy2(item, dest)
                elif APP_LEGACY_DIR.exists():
                    shutil.copytree(APP_LEGACY_DIR, APP_BACKUP_DIR / 'legacy_ui', dirs_exist_ok=True)

                # Install new AppImage
                update_progress('applying', 90, 'Installing new version...')
                dest_appimage = APP_CURRENT_DIR / 'NavProMiniRobotUI-aarch64.AppImage'
                shutil.move(str(target_file), str(dest_appimage))
                dest_appimage.chmod(0o755)

                ver_data = {'version': target_version, 'updated_at': time.time()}
                (APP_CURRENT_DIR / 'version.json').write_text(json.dumps(ver_data))

                # Restart
                update_progress('restarting', 98, 'Restarting NavPro Mini UI...')
                time.sleep(1)
                subprocess.run(['pkill', '-9', '-f', 'NavProMiniRobotUI'], timeout=3)
                subprocess.run(['pkill', '-9', '-f', 'epiphany'], timeout=3)
                subprocess.run(['pkill', '-9', '-f', 'navpromini_robot_ui_runner'], timeout=3)
                
                launcher = Path('/home/navpromini/navpromini_robot_ui/start_robot_screen.sh')
                if launcher.is_file():
                    subprocess.Popen(['sudo', '-u', 'navpromini', 'bash', str(launcher)],
                                     start_new_session=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

                update_progress('completed', 100, 'Update completed successfully!')
            except Exception as e:
                update_progress('failed', 0, f'Update failed: {e}', error=str(e))

        import threading
        t = threading.Thread(target=run_update_thread, daemon=True)
        t.start()
        self.send({'status': 'initiated', 'message': 'App update initiated in background'})

class AppUpdateStatusHandler(BaseHandler):
    """GET /api/v1/system/app/update/status - retrieve download and installation status."""
    def get(self) -> None:
        if APP_STATUS_FILE.is_file():
            try:
                data = json.loads(APP_STATUS_FILE.read_text())
                self.send(data)
                return
            except Exception:
                pass
        self.send({'state': 'idle', 'progress': 0, 'message': 'No update in progress'})

class AppUpdateRollbackHandler(BaseHandler):
    """POST /api/v1/system/app/update/rollback - restore previous version from backup."""
    def post(self) -> None:
        if not APP_BACKUP_DIR.exists() or not any(APP_BACKUP_DIR.iterdir()):
            raise ApiError(404, 'no_backup', 'No previous version backup available to rollback')
        
        try:
            shutil.rmtree(APP_CURRENT_DIR, ignore_errors=True)
            APP_CURRENT_DIR.mkdir(parents=True, exist_ok=True)
            for item in APP_BACKUP_DIR.iterdir():
                dest = APP_CURRENT_DIR / item.name
                if item.is_dir():
                    shutil.copytree(item, dest)
                else:
                    shutil.copy2(item, dest)
            
            subprocess.run(['pkill', '-9', '-f', 'NavProMiniRobotUI'], timeout=3)
            launcher = Path('/home/navpromini/navpromini_robot_ui/start_robot_screen.sh')
            if launcher.is_file():
                subprocess.Popen(['sudo', '-u', 'navpromini', 'bash', str(launcher)],
                                 start_new_session=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.send({'status': 'rolled_back', 'message': 'Restored previous backup version and restarted'})
        except Exception as e:
            raise ApiError(500, 'rollback_failed', str(e))



