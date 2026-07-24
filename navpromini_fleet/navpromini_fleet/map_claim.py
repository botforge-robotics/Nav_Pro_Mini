#!/usr/bin/env python3
"""Download occupancy for a level and stage under NAVPRO_MAPS_DIR as active.yaml."""

from __future__ import annotations

import argparse
import base64
import sys
from pathlib import Path
from typing import Optional

import requests
import yaml

from navpromini_fleet.fleet_config import (
    DEFAULT_FLEET_PATH,
    DEFAULT_MAPS_DIR,
    load_fleet_config,
)


def claim_map(
    level_id: str,
    maps_dir: Path,
    api_base: str,
    token: str,
    map_name: str = 'active',
    timeout: float = 60.0,
) -> Path:
    url = f'{api_base.rstrip("/")}/levels/{level_id}/occupancy'
    headers = {'X-Provisioning-Token': token}
    resp = requests.get(url, headers=headers, timeout=timeout)
    if resp.status_code >= 400:
        raise RuntimeError(f'claim HTTP {resp.status_code}: {resp.text[:400]}')
    data = resp.json()
    maps_dir.mkdir(parents=True, exist_ok=True)
    pgm = maps_dir / f'{map_name}.pgm'
    yml = maps_dir / f'{map_name}.yaml'
    pgm.write_bytes(base64.b64decode(data['pgmBase64']))
    yaml_text = str(data.get('yamlText') or '')
    # Ensure image: points at local pgm basename
    try:
        parsed = yaml.safe_load(yaml_text) or {}
        if isinstance(parsed, dict):
            parsed['image'] = f'{map_name}.pgm'
            yaml_text = yaml.safe_dump(parsed, default_flow_style=False)
    except Exception:  # noqa: BLE001
        pass
    yml.write_text(yaml_text, encoding='utf-8')
    return yml


def main(argv: Optional[list[str]] = None) -> int:
    p = argparse.ArgumentParser(description='Claim level occupancy onto local maps dir')
    p.add_argument('--level-id', required=True)
    p.add_argument('--maps-dir', default=str(DEFAULT_MAPS_DIR))
    p.add_argument('--map-name', default='active')
    p.add_argument('--config', default=str(DEFAULT_FLEET_PATH))
    args = p.parse_args(argv)
    cfg = load_fleet_config(Path(args.config))
    if cfg is None:
        print('no fleet config', file=sys.stderr)
        return 1
    try:
        path = claim_map(
            args.level_id,
            Path(args.maps_dir),
            cfg.api_base,
            cfg.provisioning_token,
            args.map_name,
        )
    except Exception as exc:  # noqa: BLE001
        print(f'claim failed: {exc}', file=sys.stderr)
        return 2
    print(path)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
