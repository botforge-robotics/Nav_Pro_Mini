#!/usr/bin/env python3
"""Generate RMF site assets from site.building.yaml.

Produces:
  - site/nav_graphs/0.yaml
  - site/generated/cafe.world + models/
  - site/fleet_config/navpromini.yaml  (robots from spawn_robot_name)
  - site/spawn_poses.yaml              (Gazebo spawn xyz/yaw)
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from pathlib import Path

import yaml


class NoAliasDumper(yaml.SafeDumper):
    def ignore_aliases(self, data):
        return True


def _site_dir_default() -> Path:
    return Path(__file__).resolve().parents[1] / 'site'


def _load_building(path: Path) -> dict:
    with path.open(encoding='utf-8') as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, dict):
        raise ValueError(f'invalid building yaml: {path}')
    return data


def _vertex_props(vertex: list) -> dict:
    if len(vertex) > 4 and isinstance(vertex[4], dict):
        return vertex[4]
    return {}


def _param_str(props: dict, key: str) -> str | None:
    raw = props.get(key)
    if raw is None:
        return None
    if isinstance(raw, list) and len(raw) >= 2:
        return str(raw[1])
    if isinstance(raw, str):
        return raw
    return None


def _estimate_scale_m_per_px(level: dict) -> float:
    """Prefer measurement distance / pixel length; else Nav2 default 0.05."""
    verts = level.get('vertices') or []
    for meas in level.get('measurements') or []:
        if not isinstance(meas, list) or len(meas) < 3:
            continue
        i, j = int(meas[0]), int(meas[1])
        props = meas[2] if isinstance(meas[2], dict) else {}
        dist = props.get('distance')
        if isinstance(dist, list) and len(dist) >= 2:
            meters = float(dist[1])
        elif isinstance(dist, (int, float)):
            meters = float(dist)
        else:
            continue
        if i >= len(verts) or j >= len(verts):
            continue
        dx = float(verts[i][0]) - float(verts[j][0])
        dy = float(verts[i][1]) - float(verts[j][1])
        pix = (dx * dx + dy * dy) ** 0.5
        if pix > 1e-6:
            return meters / pix
    return 0.05


def extract_spawns(building: dict) -> list[dict]:
    """Spawn poses in RMF/Gazebo frame (Y flipped, meters)."""
    levels = building.get('levels') or {}
    spawns: list[dict] = []
    for level_name, level in levels.items():
        scale = _estimate_scale_m_per_px(level)
        elevation = float(level.get('elevation') or 0.0)
        for vertex in level.get('vertices') or []:
            props = _vertex_props(vertex)
            robot = _param_str(props, 'spawn_robot_name')
            if not robot:
                continue
            px, py = float(vertex[0]), float(vertex[1])
            # Match rmf_building_map_tools reference_image: y is negated.
            x_m = px * scale
            y_m = -py * scale
            charger = vertex[3] if len(vertex) > 3 and vertex[3] else f'{robot}_home'
            # z high enough to clear ground_plane + wheel radius (~3 cm)
            spawns.append({
                'name': robot,
                'level': level_name,
                'x': round(x_m, 4),
                'y': round(y_m, 4),
                'z': round(0.12 + elevation, 4),
                'yaw': 0.0,
                'charger': charger,
                'parking': charger,
            })
    spawns.sort(key=lambda item: item['name'])
    return spawns


def write_fleet_config(path: Path, fleet_name: str, spawns: list[dict]) -> None:
    robots = {}
    for spawn in spawns:
        robots[spawn['name']] = {
            'charger': spawn['charger'],
            'responsive_wait': True,
            'initial_map': spawn['level'],
        }
    if not robots:
        robots = {
            'robot1': {'charger': 'parking2', 'responsive_wait': True, 'initial_map': 'L1'},
            'robot2': {'charger': 'parking1', 'responsive_wait': True, 'initial_map': 'L1'},
        }
    # Schema must match rmf EasyFullControl / demos (mechanical_system required).
    cfg = {
        'rmf_fleet': {
            'name': fleet_name,
            'limits': {'linear': [1.0, 1.5], 'angular': [1.2, 4.0]},
            'profile': {'footprint': 0.3, 'vicinity': 0.5},
            'reversible': True,
            'battery_system': {
                'voltage': 24.0,
                'capacity': 200.0,
                'charging_current': 20.0,
            },
            'mechanical_system': {
                'mass': 10.0,
                'moment_of_inertia': 5.0,
                'friction_coefficient': 0.22,
            },
            'ambient_system': {'power': 5.0},
            'tool_system': {'power': 0.0},
            'recharge_threshold': 0.05,
            'recharge_soc': 1.0,
            'publish_fleet_state': 10.0,
            'robot_state_update_frequency': 10.0,
            'account_for_battery_drain': True,
            'task_capabilities': {'loop': True, 'delivery': True, 'clean': False},
            'finishing_request': 'nothing',
            'robots': robots,
        },
        'fleet_manager': {
            'ip': '127.0.0.1',
            'port': 22011,
            'user': 'navpromini',
            'password': 'navpromini',
        },
        'reference_coordinates': {
            'L1': {
                'rmf': [[0.0, 0.0], [10.0, 0.0], [0.0, 10.0]],
                'robot': [[0.0, 0.0], [10.0, 0.0], [0.0, 10.0]],
            },
        },
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', encoding='utf-8') as handle:
        yaml.dump(cfg, handle, Dumper=NoAliasDumper, sort_keys=False)


def ensure_parking_chargers(nav_graph_path: Path) -> None:
    """EasyFullControl requires is_charger waypoints on the nav graph."""
    if not nav_graph_path.is_file():
        return
    with nav_graph_path.open('r', encoding='utf-8') as handle:
        data = yaml.safe_load(handle) or {}
    changed = False
    for level in (data.get('levels') or {}).values():
        for vertex in level.get('vertices') or []:
            if not isinstance(vertex, list) or len(vertex) < 3:
                continue
            props = vertex[2]
            if not isinstance(props, dict):
                continue
            if props.get('is_parking_spot') and not props.get('is_charger'):
                props['is_charger'] = True
                changed = True
    if changed:
        with nav_graph_path.open('w', encoding='utf-8') as handle:
            yaml.dump(data, handle, Dumper=NoAliasDumper, sort_keys=False)
        print(f'marked parking spots as chargers in {nav_graph_path}')


def ensure_lift_cabins(nav_graph_path: Path) -> None:
    """Every lift waypoint must also set lift_cabin (needed for L1↔L2 boarding)."""
    if not nav_graph_path.is_file():
        return
    with nav_graph_path.open('r', encoding='utf-8') as handle:
        data = yaml.safe_load(handle) or {}
    changed = False
    for level in (data.get('levels') or {}).values():
        for vertex in level.get('vertices') or []:
            if not isinstance(vertex, list) or len(vertex) < 3:
                continue
            props = vertex[2]
            if not isinstance(props, dict):
                continue
            lift = props.get('lift')
            if isinstance(lift, str) and lift and props.get('lift_cabin') != lift:
                props['lift_cabin'] = lift
                changed = True
    if changed:
        with nav_graph_path.open('w', encoding='utf-8') as handle:
            yaml.dump(data, handle, Dumper=NoAliasDumper, sort_keys=False)
        print(f'ensured lift_cabin on lift waypoints in {nav_graph_path}')


def write_spawn_poses(path: Path, spawns: list[dict]) -> None:
    doc = {
        'robots': {
            s['name']: {
                'x': s['x'],
                'y': s['y'],
                'z': s['z'],
                'yaw': s['yaw'],
                'level': s['level'],
                'charger': s['charger'],
            }
            for s in spawns
        }
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', encoding='utf-8') as handle:
        yaml.dump(doc, handle, Dumper=NoAliasDumper, sort_keys=False)


def _run(cmd: list[str], cwd: Path) -> None:
    print('running:', ' '.join(cmd))
    subprocess.run(cmd, check=True, cwd=str(cwd))


def _lift_shaft_aabb(text: str) -> tuple[float, float, float, float] | None:
    """Return (xmin, xmax, ymin, ymax) cutout for Lift1 cabin (+ margin)."""
    # Nested CabinDoor has its own </model>; take the pose after the Lift plugin.
    m = re.search(
        r'<model name="Lift1">[\s\S]*?'
        r'<component name="Lift">[\s\S]*?</plugin>\s*'
        r'<pose>([^<]+)</pose>\s*</model>',
        text,
    )
    if not m:
        return None
    pose = m.group(1).split()
    try:
        x, y = float(pose[0]), float(pose[1])
    except (ValueError, IndexError):
        return None
    # Cabin is 1.5 x 1.5. Negative pad → floor extends slightly *under* the
    # cabin rim (edge-to-edge flush, no dark doorway gap for small wheels).
    half = 0.75
    pad = -0.01
    return (x - half - pad, x + half + pad, y - half - pad, y + half + pad)


def _floor_slab_sdf(
    name: str,
    z_center: float,
    ambient: str,
    diffuse: str,
    cutout: tuple[float, float, float, float] | None,
) -> str:
    """Opaque floor with optional rectangular lift-shaft cutout.

    A solid slab through the shaft creates a lip vs the cabin floor (can't
    enter) and embeds the robot in floor_L2 when the cabin arrives upstairs.
    """
    # Building footprint (matches prior single-slab extents).
    xmin, xmax = -0.3, 9.7
    ymin, ymax = -22.75, -0.25
    thick = 0.05

    def link(link_name: str, cx: float, cy: float, sx: float, sy: float) -> str:
        if sx <= 1e-3 or sy <= 1e-3:
            return ''
        return f"""
      <link name="{link_name}">
        <pose>{cx:.4f} {cy:.4f} {z_center:.4f} 0 0 0</pose>
        <collision name="collision">
          <geometry><box><size>{sx:.4f} {sy:.4f} {thick}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{sx:.4f} {sy:.4f} {thick}</size></box></geometry>
          <material>
            <ambient>{ambient}</ambient>
            <diffuse>{diffuse}</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>
        </visual>
      </link>"""

    if cutout is None:
        cx, cy = 0.5 * (xmin + xmax), 0.5 * (ymin + ymax)
        body = link('slab', cx, cy, xmax - xmin, ymax - ymin)
    else:
        x0, x1, y0, y1 = cutout
        x0, x1 = max(xmin, x0), min(xmax, x1)
        y0, y1 = max(ymin, y0), min(ymax, y1)
        parts = [
            # West of shaft
            link('west', 0.5 * (xmin + x0), 0.5 * (ymin + ymax), x0 - xmin, ymax - ymin),
            # East of shaft
            link('east', 0.5 * (x1 + xmax), 0.5 * (ymin + ymax), xmax - x1, ymax - ymin),
            # South strip between (door approach)
            link('south', 0.5 * (x0 + x1), 0.5 * (ymin + y0), x1 - x0, y0 - ymin),
            # North strip between
            link('north', 0.5 * (x0 + x1), 0.5 * (y1 + ymax), x1 - x0, ymax - y1),
        ]
        body = ''.join(p for p in parts if p)

    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>{body}
    </model>"""


def _opaque_floors_and_ground(text: str) -> tuple[str, str]:
    """Insert / replace floor_L* with lift-shaft cutouts + ground plane."""
    cutout = _lift_shaft_aabb(text)
    note = (
        f'opaque floors L1/L2 cutout={tuple(round(v, 2) for v in cutout)}'
        if cutout
        else 'opaque floors L1/L2 (no lift cutout)'
    )
    floors = (
        '\n    <!-- Opaque floors + ground (generate_site_assets); '
        'lift shaft cut out so cabin floor is flush. -->'
        + _floor_slab_sdf(
            'floor_L1',
            -0.025,
            '0.55 0.55 0.52 1',
            '0.62 0.62 0.58 1',
            cutout,
        )
        + _floor_slab_sdf(
            'floor_L2',
            2.975,
            '0.50 0.52 0.55 1',
            '0.58 0.60 0.64 1',
            cutout,
        )
        + """
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>80 80</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>80 80</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.35 0.40 0.32 1</ambient>
            <diffuse>0.40 0.45 0.35 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>
        </visual>
      </link>
    </model>
"""
    )
    # Remove prior generate_site_assets floors / ground (idempotent regenerates).
    text = re.sub(
        r'\s*<!-- (?:Opaque floors|Added by generate_site_assets).*?-->\s*',
        '\n',
        text,
        flags=re.S,
    )
    for model in ('floor_L1', 'floor_L2', 'ground_plane'):
        text = re.sub(
            rf'\s*<model name="{model}">[\s\S]*?</model>\s*',
            '\n',
            text,
            count=1,
        )
    # building_map_generator emits "</world>" with no leading indent.
    if '</world>' not in text:
        raise RuntimeError('cafe.world missing </world>; cannot insert floors')
    text = text.replace('</world>', floors + '\n</world>', 1)
    if '<model name="floor_L1">' not in text:
        raise RuntimeError('failed to insert opaque floor models into world')
    return text, note


def _sanitize_gazebo_world(world_path: Path) -> None:
    """Post-process generated world for Harmonic sim usability.

    Keep ``liblift.so`` + per-model ``register_component`` Lift — that is the
    real Gazebo lift *node* (publishes ``/lift_states``). Do not strip it.
    """
    text = world_path.read_text(encoding='utf-8')
    lines = text.splitlines(keepends=True)
    out: list[str] = []
    for line in lines:
        if '<camera_pose>' in line:
            line = '        <camera_pose>3 -11 10 0 0.85 0</camera_pose>\n'
        out.append(line)
    text = ''.join(out)

    # gpu_lidar / cameras require Sensors system — building_map_generator does
    # not emit it. Without this, /robotN/scan has subscribers but no publisher,
    # so AMCL + costmap dynamic obstacles never see lidar.
    if 'gz-sim-sensors-system' not in text and 'gz::sim::systems::Sensors' not in text:
        sensors_plugin = (
            '    <plugin filename="gz-sim-sensors-system" '
            'name="gz::sim::systems::Sensors">\n'
            '      <render_engine>ogre2</render_engine>\n'
            '    </plugin>\n'
        )
        anchor = (
            '    <plugin filename="gz-sim-scene-broadcaster-system" '
            'name="gz::sim::systems::SceneBroadcaster">\n'
            '    </plugin>\n'
        )
        if anchor in text:
            text = text.replace(anchor, anchor + sensors_plugin, 1)
        else:
            text = text.replace(
                '  <world name="sim_world">\n',
                '  <world name="sim_world">\n' + sensors_plugin,
                1,
            )

    text, floor_note = _opaque_floors_and_ground(text)

    # Register opaque slabs with toggle_floors so unchecking L2 hides floor_L2.
    text = _wire_floors_into_toggle(text)

    # Real lift stack (book): world liblift.so + model register_component Lift.
    # Doors keep normal collide bitmasks; the plugin opens/closes them.
    has_lift_plugin = (
        'filename="liblift.so"' in text or "filename='liblift.so'" in text
    )
    has_lift_component = 'component name="Lift"' in text or "component name='Lift'" in text

    world_path.write_text(text, encoding='utf-8')
    notes = [floor_note]
    if 'gz-sim-sensors-system' not in text:
        notes.append('MISSING sensors-system (regenerate failed?)')
    else:
        notes.append('sensors-system')
    if has_lift_plugin and has_lift_component:
        notes.append('Gazebo liblift + Lift component (real lift node)')
    elif has_lift_plugin and not has_lift_component:
        notes.append('WARNING: liblift present but no Lift component '
                     '(set lifts.*.plugins: true)')
    else:
        notes.append('WARNING: liblift missing from world')
    print(f'sanitized {world_path.name}: {", ".join(notes)}')


def _wire_floors_into_toggle(text: str) -> str:
    """Add floor_L1 / floor_L2 to the Gazebo GUI toggle_floors plugin."""
    if 'toggle_floors' not in text:
        return text
    if '<model name="floor_L1" />' not in text:
        text = re.sub(
            r'(<floor name="L1"[^>]*>)',
            r'\1\n          <model name="floor_L1" />',
            text,
            count=1,
        )
    if '<model name="floor_L2" />' not in text:
        text = re.sub(
            r'(<floor name="L2"[^>]*>)',
            r'\1\n          <model name="floor_L2" />',
            text,
            count=1,
        )
    return text


def _make_walls_opaque(models_dir: Path) -> None:
    """Force opaque wall visuals (no glass/transparency)."""
    for sdf_path in models_dir.glob('*/model.sdf'):
        text = sdf_path.read_text(encoding='utf-8')
        original = text
        text = re.sub(
            r'<transparency>\s*[\d.]+\s*</transparency>',
            '<transparency>0.0</transparency>',
            text,
        )
        text = re.sub(
            r'<diffuse>\s*([\d.]+)\s+([\d.]+)\s+([\d.]+)(?:\s+[\d.]+)?\s*</diffuse>',
            r'<diffuse>\1 \2 \3 1</diffuse>',
            text,
        )
        if text != original:
            sdf_path.write_text(text, encoding='utf-8')
            print(f'opaque walls: {sdf_path.parent.name}')


def generate(site_dir: Path, building_name: str, fleet_name: str) -> None:
    site_dir = site_dir.resolve()
    building = site_dir / 'site.building.yaml'
    if not building.is_file():
        # Prefer any *.building.yaml
        matches = sorted(site_dir.glob('*.building.yaml'))
        if not matches:
            raise FileNotFoundError(f'no *.building.yaml under {site_dir}')
        building = matches[0]

    data = _load_building(building)
    spawns = extract_spawns(data)
    print(f'building={building.name} spawns={[s["name"] for s in spawns]}')

    nav_out = site_dir / 'nav_graphs'
    gen_dir = site_dir / 'generated'
    models_dir = gen_dir / 'models'
    nav_out.mkdir(parents=True, exist_ok=True)
    models_dir.mkdir(parents=True, exist_ok=True)

    world_out = gen_dir / f'{building_name}.world'

    # Must run from site_dir so relative drawing paths (maps/L1/...) resolve.
    _run(
        [
            'ros2', 'run', 'rmf_building_map_tools', 'building_map_generator', 'nav',
            str(building), str(nav_out),
        ],
        cwd=site_dir,
    )
    _run(
        [
            'ros2', 'run', 'rmf_building_map_tools', 'building_map_generator', 'gazebo',
            str(building), str(world_out), str(models_dir),
        ],
        cwd=site_dir,
    )
    _sanitize_gazebo_world(world_out)
    _make_walls_opaque(models_dir)
    ensure_parking_chargers(nav_out / '0.yaml')
    ensure_lift_cabins(nav_out / '0.yaml')

    write_fleet_config(site_dir / 'fleet_config' / f'{fleet_name}.yaml', fleet_name, spawns)
    write_spawn_poses(site_dir / 'spawn_poses.yaml', spawns)

    # Keep cafe.building.yaml copy for tools that look for named files.
    cafe_copy = site_dir / f'{building_name}.building.yaml'
    if building.resolve() != cafe_copy.resolve():
        cafe_copy.write_text(building.read_text(encoding='utf-8'), encoding='utf-8')

    print(f'wrote {world_out}')
    print(f'wrote {nav_out / "0.yaml"}')
    print(f'wrote fleet_config/{fleet_name}.yaml')
    print(f'wrote spawn_poses.yaml')
    for spawn in spawns:
        print(
            f'  {spawn["name"]}: '
            f'({spawn["x"]}, {spawn["y"]}, {spawn["z"]}) '
            f'level={spawn["level"]} charger={spawn["charger"]}'
        )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--site-dir', type=Path, default=None)
    parser.add_argument('--building-name', default='cafe')
    parser.add_argument('--fleet-name', default='navpromini')
    args = parser.parse_args(argv)
    site_dir = args.site_dir or _site_dir_default()
    generate(site_dir, args.building_name, args.fleet_name)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
