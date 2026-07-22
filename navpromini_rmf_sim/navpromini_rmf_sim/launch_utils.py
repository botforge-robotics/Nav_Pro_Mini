#!/usr/bin/env python3
"""Launch helpers for namespaced multi-robot RMF simulation."""

from __future__ import annotations

import tempfile
from pathlib import Path
from typing import Any, Iterable

import yaml


def parse_robot_list(raw: str) -> list[str]:
    robots = [part.strip() for part in raw.split(',') if part.strip()]
    if not robots:
        raise ValueError('robot_names must list at least one robot')
    return robots


def load_spawn_poses(spawn_yaml: Path | str | None) -> dict[str, dict]:
    if spawn_yaml is None:
        return {}
    path = Path(spawn_yaml)
    if not path.is_file():
        return {}
    with path.open(encoding='utf-8') as handle:
        data = yaml.safe_load(handle) or {}
    robots = data.get('robots') or {}
    return robots if isinstance(robots, dict) else {}


def default_spawn_pairs(
    robots: Iterable[str],
    spawn_yaml: Path | str | None = None,
) -> list[tuple[str, float, float, float, float]]:
    """Return (name, x, y, z, yaw) using spawn_poses.yaml when present."""
    presets = load_spawn_poses(spawn_yaml)
    fallback = {
        'robot1': (0.55, -12.5, 0.12, 0.0),
        'robot2': (0.55, -9.0, 0.12, 0.0),
    }
    poses: list[tuple[str, float, float, float, float]] = []
    for idx, name in enumerate(robots):
        if name in presets:
            entry = presets[name]
            poses.append((
                name,
                float(entry.get('x', 0.0)),
                float(entry.get('y', 0.0)),
                float(entry.get('z', 0.06)),
                float(entry.get('yaw', 0.0)),
            ))
        elif name in fallback:
            x, y, z, yaw = fallback[name]
            poses.append((name, x, y, z, yaw))
        else:
            x = 0.5 + (idx % 3) * 1.5
            y = -9.0 - (idx // 3) * 2.0
            poses.append((name, x, y, 0.12, 0.0))
    return poses


def _set_use_sim_time(node_params: dict[str, Any], use_sim_time: bool) -> None:
    node_params['use_sim_time'] = use_sim_time


def write_namespaced_nav2_params(
    source_yaml: Path | str,
    namespace: str,
    use_sim_time: bool = True,
    initial_pose: tuple[float, float, float] | None = None,
) -> str:
    """Write a per-robot Nav2 params file with correct frames/topics.

    RewrittenYaml alone is unreliable inside OpaqueFunction (params never load →
    DWB defaults with no critics). This does an explicit deep copy + patch.

    If ``initial_pose`` is (x, y, yaw), AMCL is configured with
    ``set_initial_pose: true`` so Nav2 can accept goals without a manual
    RViz "2D Pose Estimate". Frame must be global ``map`` (not ``robotN/map``).
    """
    with Path(source_yaml).open(encoding='utf-8') as handle:
        data = yaml.safe_load(handle)

    prefix = f'{namespace}/'
    scan = f'/{namespace}/scan'
    odom = f'/{namespace}/odom'
    base = f'{prefix}base_link'
    odom_frame = f'{prefix}odom'

    amcl = data['amcl']['ros__parameters']
    _set_use_sim_time(amcl, use_sim_time)
    amcl['base_frame_id'] = base
    amcl['odom_frame_id'] = odom_frame
    amcl['global_frame_id'] = 'map'
    amcl['scan_topic'] = 'scan'
    # Broadcast map→odom so Nav2 works without external ff_tf. Frame ids are
    # namespaced (robotN/odom) so two AMCL nodes do not fight on the same edge.
    amcl['tf_broadcast'] = True
    if initial_pose is not None:
        x, y, yaw = initial_pose
        amcl['set_initial_pose'] = True
        amcl['initial_pose'] = {
            'x': float(x),
            'y': float(y),
            'z': 0.0,
            'yaw': float(yaw),
        }

    bt = data['bt_navigator']['ros__parameters']
    _set_use_sim_time(bt, use_sim_time)
    bt['global_frame'] = 'map'
    bt['robot_base_frame'] = base
    bt['odom_topic'] = odom

    ctrl = data['controller_server']['ros__parameters']
    _set_use_sim_time(ctrl, use_sim_time)

    local = data['local_costmap']['local_costmap']['ros__parameters']
    _set_use_sim_time(local, use_sim_time)
    local['global_frame'] = odom_frame
    local['robot_base_frame'] = base
    if 'voxel_layer' in local and 'scan' in local['voxel_layer']:
        local['voxel_layer']['scan']['topic'] = scan
    if 'obstacle_layer' in local and 'scan' in local.get('obstacle_layer', {}):
        local['obstacle_layer']['scan']['topic'] = scan

    glob = data['global_costmap']['global_costmap']['ros__parameters']
    _set_use_sim_time(glob, use_sim_time)
    glob['global_frame'] = 'map'
    glob['robot_base_frame'] = base
    # Dedicated occupancy topic — RMF BuildingMap owns /map.
    glob['map_topic'] = '/nav2_map'
    if 'static_layer' in glob:
        glob['static_layer']['map_topic'] = '/nav2_map'
    if 'obstacle_layer' in glob and 'scan' in glob['obstacle_layer']:
        glob['obstacle_layer']['scan']['topic'] = scan

    amcl['map_topic'] = '/nav2_map'

    for key in (
        'planner_server',
        'smoother_server',
        'behavior_server',
        'waypoint_follower',
        'velocity_smoother',
        'collision_monitor',
    ):
        if key in data and 'ros__parameters' in data[key]:
            _set_use_sim_time(data[key]['ros__parameters'], use_sim_time)

    cm = data.get('collision_monitor', {}).get('ros__parameters', {})
    if cm:
        cm['base_frame_id'] = base
        cm['odom_frame_id'] = odom_frame
        # Sim often has no live LaserScan (gpu_lidar not publishing). An empty
        # observation source would freeze cmd_vel ("invalid source") and RMF
        # delivery looks "accepted" while the robot never moves. Rely on the
        # static map / costmap for obstacles in sim.
        cm['observation_sources'] = []
        if 'scan' in cm and isinstance(cm['scan'], dict):
            cm['scan']['topic'] = 'scan'
            cm['scan']['enabled'] = False
        if 'FootprintApproach' in cm and isinstance(cm['FootprintApproach'], dict):
            cm['FootprintApproach']['footprint_topic'] = (
                f'/{namespace}/local_costmap/published_footprint'
            )
            cm['FootprintApproach']['enabled'] = False

    out = Path(tempfile.gettempdir()) / f'navpromini_nav2_{namespace}.yaml'
    # Nav2 multi-robot: wrap under namespace so /robotN/controller_server matches.
    wrapped = {namespace: data}
    with out.open('w', encoding='utf-8') as handle:
        yaml.safe_dump(wrapped, handle, sort_keys=False)
    return str(out)


# Back-compat alias used by older launch snippets
def namespace_frame_rewrites(namespace: str) -> dict[str, str]:
    prefix = f'{namespace}/'
    return {
        'base_frame_id': f'{prefix}base_link',
        'odom_frame_id': f'{prefix}odom',
        'global_frame_id': 'map',
        'robot_base_frame': f'{prefix}base_link',
        'odom_topic': f'/{namespace}/odom',
        'scan_topic': 'scan',
    }
