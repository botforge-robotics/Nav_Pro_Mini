#!/usr/bin/env python3
"""Upload a saved occupancy map to the fleet server.

POST /api/v1/levels/:levelId/occupancy
  { pgmBase64, yamlText }
"""

from __future__ import annotations

import argparse
import base64
import sys
from pathlib import Path
from typing import Optional

import requests

from navpromini_fleet.fleet_config import DEFAULT_FLEET_PATH, load_fleet_config


def upload_occupancy(
    level_id: str,
    pgm_path: Path,
    yaml_path: Path,
    server_base: str,
    token: str,
    timeout: float = 60.0,
) -> dict:
    pgm_b64 = base64.b64encode(pgm_path.read_bytes()).decode('ascii')
    yaml_text = yaml_path.read_text(encoding='utf-8')
    url = f'{server_base.rstrip("/")}/levels/{level_id}/occupancy'
    headers = {
        'Content-Type': 'application/json',
        'X-Provisioning-Token': token,
    }
    body = {'pgmBase64': pgm_b64, 'yamlText': yaml_text}
    resp = requests.post(url, headers=headers, json=body, timeout=timeout)
    if resp.status_code >= 400:
        raise RuntimeError(f'upload HTTP {resp.status_code}: {resp.text[:500]}')
    return resp.json() if resp.content else {'ok': True}


def main(argv: Optional[list[str]] = None) -> int:
    p = argparse.ArgumentParser(description='Upload occupancy map to fleet server')
    p.add_argument('--level-id', required=True)
    p.add_argument('--pgm', required=True, type=Path)
    p.add_argument('--yaml', required=True, type=Path)
    p.add_argument('--config', default=str(DEFAULT_FLEET_PATH))
    args = p.parse_args(argv)

    cfg = load_fleet_config(Path(args.config))
    if cfg is None:
        print(f'No fleet config at {args.config}', file=sys.stderr)
        return 1
    if not args.pgm.is_file() or not args.yaml.is_file():
        print('pgm/yaml missing', file=sys.stderr)
        return 1
    try:
        data = upload_occupancy(
            args.level_id, args.pgm, args.yaml, cfg.api_base, cfg.provisioning_token
        )
    except Exception as exc:  # noqa: BLE001
        print(f'upload failed: {exc}', file=sys.stderr)
        return 2
    print(data)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
