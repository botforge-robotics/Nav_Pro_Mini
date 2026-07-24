#!/usr/bin/env python3
"""Register this robot with the fleet platform API (POST /robots/register)."""

from __future__ import annotations

import argparse
import json
import sys
from typing import Any, Optional

import requests

from navpromini_fleet.fleet_config import (
    FleetConfig,
    DEFAULT_FLEET_PATH,
    load_fleet_config,
    primary_mac,
    read_cpu_serial,
    save_fleet_config,
)


def register(cfg: FleetConfig, timeout: float = 15.0) -> dict[str, Any]:
    if not cfg.server_ip or not cfg.provisioning_token:
        raise RuntimeError('server_ip and provisioning_token required')
    if not cfg.serial:
        cfg.serial = read_cpu_serial()
    url = f'{cfg.api_base}/robots/register'
    body = {
        'serial': cfg.serial,
        'name': cfg.name or cfg.serial,
        'mac': primary_mac(),
        'wifiSsid': cfg.wifi_ssid or None,
        'piVersion': 'jazzy',
        'capabilities': ['lidar', 'odom', 'microros', 'mapping', 'nav2'],
    }
    body = {k: v for k, v in body.items() if v is not None}
    headers = {
        'Content-Type': 'application/json',
        'X-Provisioning-Token': cfg.provisioning_token,
    }
    resp = requests.post(url, headers=headers, json=body, timeout=timeout)
    if resp.status_code not in (200, 201):
        raise RuntimeError(f'register failed HTTP {resp.status_code}: {resp.text[:400]}')
    data = resp.json()
    robot_id = str(data.get('id') or '')
    if not robot_id:
        raise RuntimeError(f'register response missing id: {data}')
    cfg.robot_id = robot_id
    if data.get('name'):
        cfg.name = str(data['name'])
    return data


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description='Register NavPro robot with fleet server')
    parser.add_argument('--config', default=str(DEFAULT_FLEET_PATH))
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args(argv)

    cfg = load_fleet_config()
    if cfg is None:
        print(f'No fleet config at {args.config}', file=sys.stderr)
        return 1
    if args.dry_run:
        print(json.dumps(cfg.to_dict(), indent=2))
        return 0
    try:
        data = register(cfg)
    except Exception as exc:  # noqa: BLE001
        print(f'register error: {exc}', file=sys.stderr)
        return 2
    save_fleet_config(cfg)
    print(json.dumps({'ok': True, 'robot_id': cfg.robot_id, 'name': cfg.name, 'raw': data}, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
