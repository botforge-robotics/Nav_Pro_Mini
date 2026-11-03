#!/usr/bin/env python3
"""Robot-side motion parameters and safety clamps.

Single source of truth:
The SQLite database on the robot (~/.navpromini_sdk.db) persists these settings
in table `kv` under key `motion_settings`.

Both `navigation_launch.launch.py` (via RewrittenYaml) and `dock_manager_node.py`
load dynamic parameters directly from the DB on startup.
`navpromini_sdk` exposes GET/PUT REST endpoints so the Mission Planner GUI
can query, tune, and save parameters within strictly enforced safety clamps.
"""

from __future__ import annotations

import json
import os
import sqlite3
from pathlib import Path
from typing import Any, Dict, Optional

# Finalized default values established through testing & calibration
DEFAULT_MOTION_SETTINGS: Dict[str, float] = {
    'max_linear_speed': 0.35,     # m/s (Autonomous cruising speed ceiling)
    'min_linear_speed': 0.05,     # m/s (Creep speed near goal tolerance)
    'max_angular_speed': 0.30,    # rad/s (~17.2 deg/s, calm heading alignment)
    'min_angular_speed': 0.10,    # rad/s (~5.7 deg/s)
    'linear_accel': 0.050,        # m/s^2 (Auto-computed: max_linear_speed / 7.0, gentle anti-overshoot ramp)
    'linear_decel': -0.117,       # m/s^2 (Auto-computed: -max_linear_speed / 3.0, smooth stopping)
    'angular_accel': 0.50,        # rad/s^2 (Auto-computed: max_angular_speed * 1.67, responsive turns)
    'undock_speed': 0.13,         # m/s (Doubled undock roll-off speed)
    'dock_speed': 0.13,           # m/s (Doubled dock visual servo approach speed)
}

def compute_derived_accelerations(
    max_linear_speed: float,
    max_angular_speed: Optional[float] = None,
) -> Dict[str, float]:
    """Auto-configure linear and angular acceleration/deceleration from max speeds.

    Formulas:
      - linear_accel = max_linear_speed / 7.0
        Gentle acceleration ramp (0.029 m/s² at 0.2 m/s, 0.050 m/s² at 0.35 m/s).
        Prevents surging forward on small turns and overshooting road network lines.
      - linear_decel = -max_linear_speed / 3.0
        Smooth, firm stopping brake rate without pitching or wheel skid.
      - angular_accel = max_angular_speed * 1.67
        Responsive rotational acceleration for crisp heading alignment.
    """
    v_max = float(max_linear_speed)
    acc_lin = round(v_max / 7.0 + 1e-9, 3)
    dec_lin = -round(v_max / 3.0 + 1e-9, 3)

    out = {
        'linear_accel': acc_lin,
        'linear_decel': dec_lin,
    }
    if max_angular_speed is not None:
        w_max = float(max_angular_speed)
        out['angular_accel'] = round(w_max * 1.67, 2)
    return out

# Safety clamps: absolute min/max limits that cannot be violated
SAFETY_LIMITS: Dict[str, Dict[str, Any]] = {
    'max_linear_speed': {
        'min': 0.10,
        'max': 0.40,
        'step': 0.01,
        'unit': 'm/s',
        'label': 'Max Linear Speed',
        'desc': 'Maximum autonomous cruising speed in open hallways',
    },
    'min_linear_speed': {
        'min': 0.02,
        'max': 0.12,
        'step': 0.01,
        'unit': 'm/s',
        'label': 'Min Linear Speed',
        'desc': 'Approach creep speed when arriving at goal points',
    },
    'max_angular_speed': {
        'min': 0.10,
        'max': 0.50,
        'step': 0.01,
        'unit': 'rad/s',
        'label': 'Max Angular Speed',
        'desc': 'Heading rotation speed during turns and reorientation',
    },
    'min_angular_speed': {
        'min': 0.05,
        'max': 0.20,
        'step': 0.01,
        'unit': 'rad/s',
        'label': 'Min Angular Speed',
        'desc': 'Minimum turning velocity threshold',
    },
    'linear_accel': {
        'min': 0.02,
        'max': 0.20,
        'step': 0.001,
        'unit': 'm/s²',
        'label': 'Linear Acceleration',
        'desc': 'Auto-computed: max_linear_speed / 7.0 (gentle ramp, zero overshoot on turns)',
        'auto_formula': 'v_max / 7',
    },
    'linear_decel': {
        'min': -0.40,
        'max': -0.04,
        'step': 0.001,
        'unit': 'm/s²',
        'label': 'Linear Deceleration',
        'desc': 'Auto-computed: -max_linear_speed / 3.0 (controlled progressive stopping rate)',
        'auto_formula': '-v_max / 3',
    },
    'angular_accel': {
        'min': 0.10,
        'max': 1.00,
        'step': 0.01,
        'unit': 'rad/s²',
        'label': 'Angular Acceleration',
        'desc': 'Auto-computed: max_angular_speed * 1.67 (responsive rotational ramp for crisp heading alignment)',
        'auto_formula': 'w_max * 1.67',
    },
    'undock_speed': {
        'min': 0.05,
        'max': 0.20,
        'step': 0.01,
        'unit': 'm/s',
        'label': 'Undocking Speed',
        'desc': 'Straight drive-off speed leaving the charging dock slot',
    },
    'dock_speed': {
        'min': 0.05,
        'max': 0.20,
        'step': 0.01,
        'unit': 'm/s',
        'label': 'Dock Approach Speed',
        'desc': 'Maximum visual servo speed approaching the dock (tapers down near dock)',
    },
}


def get_db_path() -> Path:
    """Resolve the SQLite database path on the robot."""
    env_path = os.environ.get('NAVPRO_SDK_DATA')
    if env_path and os.path.exists(env_path):
        return Path(env_path)

    # Check root database if running as root service
    for candidate in [
        Path('/root/.navpromini_sdk.db'),
        Path(os.path.expanduser('~/.navpromini_sdk.db')),
        Path('/home/navpromini/.navpromini_sdk.db'),
    ]:
        if candidate.is_file():
            return candidate

    return Path(os.path.expanduser('~/.navpromini_sdk.db'))


def clamp_setting(key: str, val: Any) -> float:
    """Clamp a setting to its defined safety limits."""
    try:
        fval = float(val)
    except (TypeError, ValueError):
        fval = DEFAULT_MOTION_SETTINGS.get(key, 0.0)

    limits = SAFETY_LIMITS.get(key)
    if not limits:
        return fval

    min_v = limits['min']
    max_v = limits['max']
    return max(min_v, min(max_v, fval))


def get_motion_settings() -> Dict[str, float]:
    """Retrieve motion settings from the robot SQLite database, merged with defaults."""
    settings = dict(DEFAULT_MOTION_SETTINGS)
    db_path = get_db_path()

    if db_path.exists():
        try:
            conn = sqlite3.connect(str(db_path), timeout=5.0)
            with conn:
                conn.execute(
                    'CREATE TABLE IF NOT EXISTS kv (key TEXT PRIMARY KEY, value TEXT NOT NULL)'
                )
                row = conn.execute(
                    'SELECT value FROM kv WHERE key = ?', ('motion_settings',)
                ).fetchone()
                if row and row[0]:
                    loaded = json.loads(row[0])
                    if isinstance(loaded, dict):
                        for k, v in loaded.items():
                            if k in DEFAULT_MOTION_SETTINGS:
                                settings[k] = clamp_setting(k, v)
            conn.close()
        except Exception as exc:
            print(f"[motion_config] Warning reading from DB ({db_path}): {exc}")

    # Synchronize derived acceleration/deceleration with formulas
    max_lin = settings.get('max_linear_speed', DEFAULT_MOTION_SETTINGS['max_linear_speed'])
    max_ang = settings.get('max_angular_speed', DEFAULT_MOTION_SETTINGS['max_angular_speed'])
    derived = compute_derived_accelerations(max_lin, max_ang)
    settings['linear_accel'] = clamp_setting('linear_accel', derived['linear_accel'])
    settings['linear_decel'] = clamp_setting('linear_decel', derived['linear_decel'])
    settings['angular_accel'] = clamp_setting('angular_accel', derived['angular_accel'])

    return settings


def save_motion_settings(new_settings: Dict[str, Any]) -> Dict[str, float]:
    """Clamp and persist motion settings to the robot SQLite database."""
    current = get_motion_settings()
    for k, v in new_settings.items():
        if k in DEFAULT_MOTION_SETTINGS:
            current[k] = clamp_setting(k, v)

    # Auto-calculate formula-derived dynamic parameters from speeds
    max_lin = current.get('max_linear_speed', DEFAULT_MOTION_SETTINGS['max_linear_speed'])
    max_ang = current.get('max_angular_speed', DEFAULT_MOTION_SETTINGS['max_angular_speed'])
    derived = compute_derived_accelerations(max_lin, max_ang)
    current['linear_accel'] = clamp_setting('linear_accel', derived['linear_accel'])
    current['linear_decel'] = clamp_setting('linear_decel', derived['linear_decel'])
    current['angular_accel'] = clamp_setting('angular_accel', derived['angular_accel'])

    db_path = get_db_path()
    try:
        db_path.parent.mkdir(parents=True, exist_ok=True)
        conn = sqlite3.connect(str(db_path), timeout=5.0)
        with conn:
            conn.execute('PRAGMA journal_mode=WAL')
            conn.execute('PRAGMA synchronous=FULL')
            conn.execute(
                'CREATE TABLE IF NOT EXISTS kv (key TEXT PRIMARY KEY, value TEXT NOT NULL)'
            )
            raw = json.dumps(current, indent=2)
            conn.execute(
                'INSERT OR REPLACE INTO kv (key, value) VALUES (?, ?)',
                ('motion_settings', raw),
            )
        conn.close()
    except Exception as exc:
        print(f"[motion_config] Warning saving to DB ({db_path}): {exc}")
        raise

    return current


def get_default_motion_settings() -> Dict[str, float]:
    """Retrieve default motion settings (reads custom defaults from DB if stored, else baseline)."""
    defaults = dict(DEFAULT_MOTION_SETTINGS)
    db_path = get_db_path()
    if db_path.exists():
        try:
            conn = sqlite3.connect(str(db_path), timeout=5.0)
            with conn:
                conn.execute(
                    'CREATE TABLE IF NOT EXISTS kv (key TEXT PRIMARY KEY, value TEXT NOT NULL)'
                )
                row = conn.execute(
                    'SELECT value FROM kv WHERE key = ?', ('motion_defaults',)
                ).fetchone()
                if row and row[0]:
                    loaded = json.loads(row[0])
                    if isinstance(loaded, dict):
                        for k, v in loaded.items():
                            if k in DEFAULT_MOTION_SETTINGS:
                                defaults[k] = clamp_setting(k, v)
            conn.close()
        except Exception as exc:
            print(f"[motion_config] Warning reading defaults from DB ({db_path}): {exc}")

    # Synchronize formula-derived parameters
    max_lin = defaults.get('max_linear_speed', DEFAULT_MOTION_SETTINGS['max_linear_speed'])
    max_ang = defaults.get('max_angular_speed', DEFAULT_MOTION_SETTINGS['max_angular_speed'])
    derived = compute_derived_accelerations(max_lin, max_ang)
    defaults['linear_accel'] = clamp_setting('linear_accel', derived['linear_accel'])
    defaults['linear_decel'] = clamp_setting('linear_decel', derived['linear_decel'])
    defaults['angular_accel'] = clamp_setting('angular_accel', derived['angular_accel'])
    return defaults


def set_current_as_default_motion_settings() -> Dict[str, float]:
    """Persist the present active motion settings as the new default motion settings in SQLite."""
    current = get_motion_settings()
    db_path = get_db_path()
    try:
        db_path.parent.mkdir(parents=True, exist_ok=True)
        conn = sqlite3.connect(str(db_path), timeout=5.0)
        with conn:
            conn.execute('PRAGMA journal_mode=WAL')
            conn.execute('PRAGMA synchronous=FULL')
            conn.execute(
                'CREATE TABLE IF NOT EXISTS kv (key TEXT PRIMARY KEY, value TEXT NOT NULL)'
            )
            raw = json.dumps(current, indent=2)
            conn.execute(
                'INSERT OR REPLACE INTO kv (key, value) VALUES (?, ?)',
                ('motion_defaults', raw),
            )
        conn.close()
    except Exception as exc:
        print(f"[motion_config] Warning saving defaults to DB ({db_path}): {exc}")
        raise

    return current


def restore_default_motion_settings() -> Dict[str, float]:
    """Revert the robot SQLite database to the default motion settings."""
    defaults = get_default_motion_settings()
    return save_motion_settings(defaults)


def get_nav2_param_rewrites(settings: Optional[Dict[str, float]] = None) -> Dict[str, Any]:
    """Generate Nav2 RewrittenYaml parameters from dynamic motion settings."""
    cfg = settings or get_motion_settings()

    max_lin = float(cfg.get('max_linear_speed', 0.35))
    min_lin = float(cfg.get('min_linear_speed', 0.05))
    max_ang = float(cfg.get('max_angular_speed', 0.30))
    min_ang = float(cfg.get('min_angular_speed', 0.10))
    acc_lin = float(cfg.get('linear_accel', 0.050))
    dec_lin = float(cfg.get('linear_decel', -0.117))
    acc_ang = float(cfg.get('angular_accel', 0.50))
    dec_ang = -1.0 * abs(acc_ang) * 1.4

    return {
        'controller_server.ros__parameters.FollowPath.desired_linear_vel': max_lin,
        'controller_server.ros__parameters.FollowPath.min_approach_linear_velocity': min_lin,
        'controller_server.ros__parameters.FollowPath.rotate_to_heading_angular_vel': max_ang,
        'controller_server.ros__parameters.FollowPath.max_angular_accel': acc_ang,
        'controller_server.ros__parameters.FollowPath.rotate_to_heading_min_angle': 0.40,
        'controller_server.ros__parameters.FollowPath.lookahead_dist': 0.30,
        'controller_server.ros__parameters.FollowPath.min_lookahead_dist': 0.25,
        'controller_server.ros__parameters.FollowPath.max_lookahead_dist': 0.60,
        'controller_server.ros__parameters.FollowPath.lookahead_time': 1.5,
        'controller_server.ros__parameters.FollowPath.regulated_linear_scaling_min_radius': 0.60,
        'controller_server.ros__parameters.FollowPath.regulated_linear_scaling_min_speed': 0.08,
        'controller_server.ros__parameters.FollowPathDWB.max_vel_x': max_lin,
        'controller_server.ros__parameters.FollowPathDWB.max_speed_xy': max_lin,
        'controller_server.ros__parameters.FollowPathDWB.max_vel_theta': max_ang,
        'controller_server.ros__parameters.FollowPathDWB.max_speed_theta': max_ang,
        'controller_server.ros__parameters.FollowPathDWB.acc_lim_x': acc_lin,
        'controller_server.ros__parameters.FollowPathDWB.acc_lim_theta': acc_ang,
        'controller_server.ros__parameters.FollowPathDWB.decel_lim_x': dec_lin,
        'route_server.ros__parameters.radius_to_achieve_node': 0.35,
        'route_server.ros__parameters.boundary_radius_to_achieve_node': 0.25,
        'velocity_smoother.ros__parameters.max_velocity': [max_lin, 0.0, max_ang],
        'velocity_smoother.ros__parameters.min_velocity': [-0.08, 0.0, -max_ang],
        'velocity_smoother.ros__parameters.max_accel': [acc_lin, 0.0, acc_ang],
        'velocity_smoother.ros__parameters.max_decel': [dec_lin, 0.0, dec_ang],
        'behavior_server.ros__parameters.max_rotational_vel': max_ang,
        'behavior_server.ros__parameters.min_rotational_vel': min_ang,
        'behavior_server.ros__parameters.rotational_acc_lim': acc_ang,
    }
