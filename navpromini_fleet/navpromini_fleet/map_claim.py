#!/usr/bin/env python3
"""Download occupancy for a level and stage under NAVPRO_MAPS_DIR as active.yaml."""

from __future__ import annotations

import argparse
import base64
import json
import sys
from pathlib import Path
from typing import Any, Optional

import requests
import yaml

from navpromini_fleet.fleet_config import (
    DEFAULT_FLEET_PATH,
    DEFAULT_MAPS_DIR,
    load_fleet_config,
)

DEFAULT_MAP_NAME = 'active'
REVISION_FILE = '.map_revision'


def read_local_revision(maps_dir: Path) -> dict[str, str]:
    path = maps_dir / REVISION_FILE
    if not path.is_file():
        return {}
    try:
        data = json.loads(path.read_text(encoding='utf-8'))
        return data if isinstance(data, dict) else {}
    except (OSError, json.JSONDecodeError):
        return {}


def write_local_revision(
    maps_dir: Path,
    *,
    level_id: str,
    map_revision: str,
    map_name: str = DEFAULT_MAP_NAME,
) -> None:
    maps_dir.mkdir(parents=True, exist_ok=True)
    payload = {
        'levelId': level_id,
        'mapRevision': map_revision,
        'mapName': map_name,
    }
    (maps_dir / REVISION_FILE).write_text(
        json.dumps(payload, indent=2) + '\n', encoding='utf-8'
    )


def claim_map(
    level_id: str,
    maps_dir: Path,
    api_base: str,
    token: str,
    map_name: str = DEFAULT_MAP_NAME,
    timeout: float = 60.0,
) -> dict[str, Any]:
    """Download fleet GUI occupancy → maps_dir/{map_name}.{pgm,yaml}.

    Returns dict with path, mapRevision, levelId, changed.
    """
    url = f'{api_base.rstrip("/")}/levels/{level_id}/occupancy'
    headers = {'X-Provisioning-Token': token}
    resp = requests.get(url, headers=headers, timeout=timeout)
    if resp.status_code >= 400:
        raise RuntimeError(f'claim HTTP {resp.status_code}: {resp.text[:400]}')
    data = resp.json()
    maps_dir.mkdir(parents=True, exist_ok=True)
    name = map_name or str(data.get('mapName') or DEFAULT_MAP_NAME)
    pgm = maps_dir / f'{name}.pgm'
    yml = maps_dir / f'{name}.yaml'
    pgm.write_bytes(base64.b64decode(data['pgmBase64']))
    yaml_text = str(data.get('yamlText') or '')
    # Ensure image: points at local pgm basename
    try:
        parsed = yaml.safe_load(yaml_text) or {}
        if isinstance(parsed, dict):
            parsed['image'] = f'{name}.pgm'
            yaml_text = yaml.safe_dump(parsed, default_flow_style=False)
    except Exception:  # noqa: BLE001
        pass
    yml.write_text(yaml_text, encoding='utf-8')
    revision = str(data.get('mapRevision') or '')
    prev = read_local_revision(maps_dir)
    changed = (
        revision == ''
        or prev.get('mapRevision') != revision
        or prev.get('levelId') != level_id
    )
    if revision:
        write_local_revision(
            maps_dir, level_id=level_id, map_revision=revision, map_name=name
        )
    return {
        'path': yml,
        'mapName': name,
        'levelId': level_id,
        'mapRevision': revision,
        'changed': changed,
    }


def main(argv: Optional[list[str]] = None) -> int:
    p = argparse.ArgumentParser(description='Claim level occupancy onto local maps dir')
    p.add_argument('--level-id', required=True)
    p.add_argument('--maps-dir', default=str(DEFAULT_MAPS_DIR))
    p.add_argument('--map-name', default=DEFAULT_MAP_NAME)
    p.add_argument('--config', default=str(DEFAULT_FLEET_PATH))
    args = p.parse_args(argv)
    cfg = load_fleet_config(Path(args.config))
    if cfg is None:
        print('no fleet config', file=sys.stderr)
        return 1
    try:
        result = claim_map(
            args.level_id,
            Path(args.maps_dir),
            cfg.api_base,
            cfg.provisioning_token,
            args.map_name,
        )
    except Exception as exc:  # noqa: BLE001
        print(f'claim failed: {exc}', file=sys.stderr)
        return 2
    print(result['path'])
    if result.get('mapRevision'):
        print(f"revision={result['mapRevision']} changed={result['changed']}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
