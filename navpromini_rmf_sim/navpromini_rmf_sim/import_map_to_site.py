#!/usr/bin/env python3
"""Import a Nav2 map (pgm + yaml) into RMF site artifacts for traffic-editor."""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

import yaml

try:
    from PIL import Image
except ImportError as exc:  # pragma: no cover
    raise SystemExit('python3-pil is required: sudo apt install python3-pil') from exc


def _load_map_yaml(map_yaml: Path) -> dict:
    with map_yaml.open(encoding='utf-8') as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, dict):
        raise ValueError(f'invalid map yaml: {map_yaml}')
    return data


def _resolve_pgm(map_yaml: Path, meta: dict) -> Path:
    image_name = meta.get('image')
    if not image_name:
        raise ValueError(f'map yaml missing image key: {map_yaml}')
    pgm = map_yaml.parent / image_name
    if not pgm.is_file():
        raise FileNotFoundError(f'pgm not found: {pgm}')
    return pgm


def _write_floorplan_png(pgm: Path, png_out: Path) -> None:
    img = Image.open(pgm).convert('L')
    png_out.parent.mkdir(parents=True, exist_ok=True)
    img.save(png_out)


def _write_nav2_map(pgm: Path, meta: dict, out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    shutil.copy2(pgm, out_dir / 'map.pgm')
    nav_meta = dict(meta)
    nav_meta['image'] = 'map.pgm'
    with (out_dir / 'map.yaml').open('w', encoding='utf-8') as handle:
        yaml.safe_dump(nav_meta, handle, sort_keys=False)


def _write_building_yaml(
    building_name: str,
    floorplan_rel: str,
    out_path: Path,
) -> None:
    """Minimal building stub (meter coords). Refine in traffic-editor."""
    doc = {
        'name': building_name,
        'levels': {
            'L1': {
                'elevation': 0.0,
                'drawing': {'filename': floorplan_rel},
                'vertices': [
                    [-2.0, -8.0, 0.0, 'robot1_home',
                     {'is_charger': [4, True], 'is_parking_spot': [4, True]}],
                    [-2.0, -6.0, 0.0, 'robot2_home',
                     {'is_charger': [4, True], 'is_parking_spot': [4, True]}],
                    [0.0, -7.0, 0.0, 'mid', {}],
                ],
                'lanes': [
                    [0, 2, {
                        'bidirectional': [4, True],
                        'graph_idx': [2, 0],
                        'demo_mock_floor_name': [1, ''],
                        'demo_mock_lift_name': [1, ''],
                    }],
                    [2, 0, {
                        'bidirectional': [4, True],
                        'graph_idx': [2, 0],
                        'demo_mock_floor_name': [1, ''],
                        'demo_mock_lift_name': [1, ''],
                    }],
                    [1, 2, {
                        'bidirectional': [4, True],
                        'graph_idx': [2, 0],
                        'demo_mock_floor_name': [1, ''],
                        'demo_mock_lift_name': [1, ''],
                    }],
                    [2, 1, {
                        'bidirectional': [4, True],
                        'graph_idx': [2, 0],
                        'demo_mock_floor_name': [1, ''],
                        'demo_mock_lift_name': [1, ''],
                    }],
                ],
            },
        },
        'lifts': {},
    }
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open('w', encoding='utf-8') as handle:
        yaml.safe_dump(doc, handle, sort_keys=False, default_flow_style=None)


def _write_fleet_config(site_dir: Path, fleet_name: str, robots: list[str]) -> None:
    cfg = {
        'rmf_fleet': {
            'name': fleet_name,
            'limits': {'linear': [1.0, 1.5], 'angular': [1.2, 4.0]},
            'profile': {'footprint': 0.3, 'vicinity': 0.5},
            'battery_system': {
                'voltage': 24.0,
                'capacity': 20.0,
                'charging_current': 5.0,
            },
            'recharge_threshold': 0.2,
            'recharge_soc': 1.0,
            'task_capabilities': {'loop': True, 'delivery': False, 'clean': False},
            'transforms': {},
            'robots': {
                name: {'navigation_stack': 2, 'initial_map': 'L1', 'charger': f'{name}_home'}
                for name in robots
            },
        },
        'plugins': {'dock': {'actions': ['dock', 'undock']}},
    }
    out = site_dir / 'fleet_config' / f'{fleet_name}.yaml'
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open('w', encoding='utf-8') as handle:
        yaml.safe_dump(cfg, handle, sort_keys=False)


def _generate_nav_graph(building_yaml: Path, nav_graph_dir: Path) -> None:
    nav_graph_dir.mkdir(parents=True, exist_ok=True)
    cmd = [
        'ros2', 'run', 'rmf_building_map_tools', 'building_map_generator', 'nav',
        str(building_yaml), str(nav_graph_dir),
    ]
    print('running:', ' '.join(cmd))
    subprocess.run(cmd, check=True)


def import_map(
    map_yaml: Path,
    site_dir: Path,
    building_name: str,
    fleet_name: str,
    robots: list[str],
    generate_nav_graph: bool,
) -> None:
    map_yaml = map_yaml.resolve()
    site_dir = site_dir.resolve()
    meta = _load_map_yaml(map_yaml)
    pgm = _resolve_pgm(map_yaml, meta)

    resolution = float(meta['resolution'])
    with Image.open(pgm) as img:
        width_px, height_px = img.size
    width_m = width_px * resolution
    height_m = height_px * resolution

    map_out = site_dir / 'maps' / 'L1'
    floorplan_rel = 'maps/L1/floorplan.png'
    _write_nav2_map(pgm, meta, map_out)
    _write_floorplan_png(pgm, site_dir / floorplan_rel)

    building_yaml = site_dir / f'{building_name}.building.yaml'
    _write_building_yaml(building_name, floorplan_rel, building_yaml)
    shutil.copy2(building_yaml, site_dir / 'site.building.yaml')
    shutil.copy2(building_yaml, site_dir / 'building.yaml')

    _write_fleet_config(site_dir, fleet_name, robots)

    if generate_nav_graph:
        _generate_nav_graph(building_yaml, site_dir / 'nav_graphs')

    print(f'site written to {site_dir}')
    print(f'  building: {building_yaml.name}')
    print(f'  nav2 map: {map_out / "map.yaml"}')
    print(f'  floorplan: {floorplan_rel}')
    print('Edit lanes/chargers in traffic-editor, then re-run with --nav-graph')


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--map-yaml',
        type=Path,
        default=Path.home() / 'NavProMini_ws/src/navpromini_mapping/maps/cafe.yaml',
        help='Nav2 map yaml (pgm referenced inside)',
    )
    parser.add_argument(
        '--site-dir',
        type=Path,
        default=None,
        help='Output site directory (default: package share site/)',
    )
    parser.add_argument('--building-name', default='cafe')
    parser.add_argument('--fleet-name', default='navpromini')
    parser.add_argument(
        '--robots',
        nargs='+',
        default=['robot1', 'robot2'],
        help='Robot names for fleet_config stub',
    )
    parser.add_argument(
        '--nav-graph',
        action='store_true',
        help='Run building_map_generator nav (requires rmf_building_map_tools)',
    )
    args = parser.parse_args(argv)

    site_dir = args.site_dir
    if site_dir is None:
        pkg_root = Path(__file__).resolve().parents[1]
        site_dir = pkg_root / 'site'

    import_map(
        map_yaml=args.map_yaml,
        site_dir=site_dir,
        building_name=args.building_name,
        fleet_name=args.fleet_name,
        robots=args.robots,
        generate_nav_graph=args.nav_graph,
    )
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
