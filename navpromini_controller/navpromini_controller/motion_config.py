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
    'linear_accel': 0.09,         # m/s^2 (Halved breakaway acceleration, zero slip)
    'linear_decel': -0.18,        # m/s^2 (Smooth controlled deceleration)
    'angular_accel': 0.50,        # rad/s^2 (Soft progressive angular acceleration)
    'undock_speed': 0.13,         # m/s (Doubled undock roll-off speed)
    'dock_speed': 0.13,           # m/s (Doubled dock visual servo approach speed)
}

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
        'min': 0.04,
        'max': 0.20,
        'step': 0.01,
        'unit': 'm/s²',
        'label': 'Linear Acceleration',
        'desc': 'Ramp-up rate from standstill (prevents wheel slip on smooth floors)',
    },
    'linear_decel': {
        'min': -0.35,
        'max': -0.08,
        'step': 0.01,
        'unit': 'm/s²',
        'label': 'Linear Deceleration',
        'desc': 'Stopping brake rate when approaching obstacles or goal',
    },
    'angular_accel': {
        'min': 0.20,
        'max': 1.00,
        'step': 0.05,
        'unit': 'rad/s²',
        'label': 'Angular Acceleration',
        'desc': 'Rotational ramp rate entering and exiting turns',
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

    if not db_path.exists():
        return settings

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

    return settings


def save_motion_settings(new_settings: Dict[str, Any]) -> Dict[str, float]:
    """Clamp and persist motion settings to the robot SQLite database."""
    current = get_motion_settings()
    for k, v in new_settings.items():
        if k in DEFAULT_MOTION_SETTINGS:
            current[k] = clamp_setting(k, v)

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


def restore_default_motion_settings() -> Dict[str, float]:
    """Revert the robot SQLite database to the finalized default motion settings."""
    return save_motion_settings(DEFAULT_MOTION_SETTINGS)


def get_nav2_param_rewrites(settings: Optional[Dict[str, float]] = None) -> Dict[str, Any]:
    """Generate Nav2 RewrittenYaml parameters from dynamic motion settings."""
    cfg = settings or get_motion_settings()

    max_lin = float(cfg.get('max_linear_speed', 0.35))
    min_lin = float(cfg.get('min_linear_speed', 0.05))
    max_ang = float(cfg.get('max_angular_speed', 0.30))
    min_ang = float(cfg.get('min_angular_speed', 0.10))
    acc_lin = float(cfg.get('linear_accel', 0.09))
    dec_lin = float(cfg.get('linear_decel', -0.18))
    acc_ang = float(cfg.get('angular_accel', 0.50))
    dec_ang = -1.0 * abs(acc_ang) * 1.4

    return {
        'controller_server.ros__parameters.FollowPath.desired_linear_vel': max_lin,
        'controller_server.ros__parameters.FollowPath.min_approach_linear_velocity': min_lin,
        'controller_server.ros__parameters.FollowPath.rotate_to_heading_angular_vel': max_ang,
        'controller_server.ros__parameters.FollowPath.max_angular_accel': acc_ang,
        'controller_server.ros__parameters.FollowPathDWB.max_vel_x': max_lin,
        'controller_server.ros__parameters.FollowPathDWB.max_speed_xy': max_lin,
        'controller_server.ros__parameters.FollowPathDWB.max_vel_theta': max_ang,
        'controller_server.ros__parameters.FollowPathDWB.max_speed_theta': max_ang,
        'controller_server.ros__parameters.FollowPathDWB.acc_lim_x': acc_lin,
        'controller_server.ros__parameters.FollowPathDWB.acc_lim_theta': acc_ang,
        'controller_server.ros__parameters.FollowPathDWB.decel_lim_x': dec_lin,
        'velocity_smoother.ros__parameters.max_velocity': [max_lin, 0.0, max_ang],
        'velocity_smoother.ros__parameters.min_velocity': [-0.08, 0.0, -max_ang],
        'velocity_smoother.ros__parameters.max_accel': [acc_lin, 0.0, acc_ang],
        'velocity_smoother.ros__parameters.max_decel': [dec_lin, 0.0, dec_ang],
        'behavior_server.ros__parameters.max_rotational_vel': max_ang,
        'behavior_server.ros__parameters.min_rotational_vel': min_ang,
        'behavior_server.ros__parameters.rotational_acc_lim': acc_ang,
    }
