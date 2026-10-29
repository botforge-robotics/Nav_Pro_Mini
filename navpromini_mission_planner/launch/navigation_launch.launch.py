#!/usr/bin/env python3
"""Mission Planner navigation wrapper (per botforge nav2_mission_planner docs).

Starts Nav2 localization (AMCL + map_server) + navigation + costmap keepout filter servers.
The app passes only the map name, e.g. map:=office.yaml — this file builds
the full path under navpromini_mapping/maps and launches the keepout filter servers.

App launch ref: navpromini_mission_planner/navigation_launch
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


ARGUMENTS = [
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        choices=['true', 'false'],
        description='Use sim time',
    ),
    DeclareLaunchArgument(
        'nav2_params_file',
        default_value=PathJoinSubstitution([
            get_package_share_directory('navpromini_navigation'),
            'config',
            'nav2_params.yaml',
        ]),
        description='Nav2 parameters',
    ),
    DeclareLaunchArgument(
        'localization_params_file',
        default_value=PathJoinSubstitution([
            get_package_share_directory('navpromini_navigation'),
            'config',
            'nav2_params.yaml',
        ]),
        description='Localization / AMCL parameters',
    ),
    DeclareLaunchArgument(
        'autostart',
        default_value='true',
        choices=['true', 'false'],
        description='Automatically startup the nav2 stack',
    ),
    # Mission Planner app always adds: map:=<mapName>.yaml
    DeclareLaunchArgument(
        'map',
        default_value='cafe.yaml',
        description='Map yaml filename only (e.g. office.yaml) from Mission Planner',
    ),
]


def _get_dock_pose_for_map(base_map_name: str) -> dict | None:
    """Retrieve authoritative saved dock pose from SQLite shared store or dock JSON."""
    import json
    import math
    import sqlite3

    db_candidates = [
        os.path.expanduser('~/.navpromini_sdk.db'),
        '/home/navpromini/.navpromini_sdk.db',
        '/root/.navpromini_sdk.db',
    ]
    for db_path in set(db_candidates):
        if not os.path.isfile(db_path):
            continue
        try:
            con = sqlite3.connect(db_path)
            cur = con.cursor()
            cur.execute("SELECT data FROM waypoints WHERE map = ? AND name = 'Charging Dock'", (base_map_name,))
            row = cur.fetchone()
            if not row:
                cur.execute("SELECT data FROM waypoints WHERE map = ? AND json_extract(data, '$.type') = 'dock'", (base_map_name,))
                row = cur.fetchone()
            con.close()
            if row:
                d = json.loads(row[0])
                if 'x' in d and 'y' in d:
                    return {
                        'x': float(d['x']),
                        'y': float(d['y']),
                        'theta': float(d.get('theta', 0.0)),
                    }
        except Exception:
            pass

    json_candidates = [
        os.path.expanduser('~/.navpromini_dock_pose.json'),
        '/home/navpromini/.navpromini_dock_pose.json',
        '/root/.navpromini_dock_pose.json',
    ]
    existing_json = [p for p in set(json_candidates) if os.path.isfile(p)]
    existing_json.sort(key=lambda p: os.path.getmtime(p), reverse=True)
    for json_path in existing_json:
        try:
            with open(json_path, 'r') as f:
                d = json.load(f)
            if 'x' in d and 'y' in d:
                theta = d.get('theta')
                if theta is None:
                    qz = d.get('qz', 0.0)
                    qw = d.get('qw', 1.0)
                    theta = 2.0 * math.atan2(qz, qw)
                return {
                    'x': float(d['x']),
                    'y': float(d['y']),
                    'theta': float(theta),
                }
        except Exception:
            pass

    return None


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    nav2_params = LaunchConfiguration('nav2_params_file')
    localization_params = LaunchConfiguration('localization_params_file')
    map_name = LaunchConfiguration('map')

    map_name_str = map_name.perform(context)
    base_map_name = map_name_str.replace('.yaml', '')

    maps_share_dir = os.path.join(
        get_package_share_directory('navpromini_mapping'), 'maps'
    )
    map_file = os.path.join(maps_share_dir, map_name_str)

    # Ensure keepout mask YAML and PGM exist before launching filter_mask_server
    keepout_yaml_path = os.path.join(maps_share_dir, f'{base_map_name}_keepout.yaml')
    try:
        from navpromini_sdk.costmap_zones import generate_keepout_mask
        generated_yaml, _ = generate_keepout_mask(base_map_name)
        keepout_yaml_path = generated_yaml
    except Exception as exc:
        print(f"[navigation_launch] generate_keepout_mask note: {exc}")

    # Dynamically inject the map's saved dock pose as AMCL's initial_pose on boot / power-on
    dock_pose = _get_dock_pose_for_map(base_map_name)
    if dock_pose:
        amcl_rewrites = {
            'amcl.ros__parameters.initial_pose.x': str(dock_pose['x']),
            'amcl.ros__parameters.initial_pose.y': str(dock_pose['y']),
            'amcl.ros__parameters.initial_pose.yaw': str(dock_pose['theta']),
        }
        localization_params_to_use = RewrittenYaml(
            source_file=localization_params,
            param_rewrites=amcl_rewrites,
            convert_types=True,
        )
        print(f"[navigation_launch] Setting AMCL initial_pose from saved dock for {base_map_name}: ({dock_pose['x']:.3f}, {dock_pose['y']:.3f}, yaw={dock_pose['theta']:.3f})")
    else:
        localization_params_to_use = localization_params

    # Preload road network in RouteServer if available for this map
    roadnet_geojson = os.path.join(maps_share_dir, f'{base_map_name}_roadnet.geojson')
    nav2_rewrites = {}
    if os.path.isfile(roadnet_geojson):
        nav2_rewrites['route_server.ros__parameters.graph_filepath'] = roadnet_geojson
        print(f"[navigation_launch] Preloading RouteServer graph: {roadnet_geojson}")

    if nav2_rewrites:
        nav2_params_to_use = RewrittenYaml(
            source_file=nav2_params,
            param_rewrites=nav2_rewrites,
            convert_types=True,
        )
    else:
        nav2_params_to_use = nav2_params.perform(context)

    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    launch_nav2 = PathJoinSubstitution(
        [pkg_nav2_bringup, 'launch', 'navigation_launch.py'])
    launch_localization = PathJoinSubstitution(
        [pkg_nav2_bringup, 'launch', 'localization_launch.py'])
    launch_dock_nodes = PathJoinSubstitution([
        get_package_share_directory('navpromini_controller'),
        'launch',
        'docking.launch.py',
    ])

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_nav2),
        launch_arguments=[
            ('use_sim_time', use_sim_time),
            ('params_file', nav2_params_to_use),
            ('autostart', autostart),
        ],
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_localization),
        launch_arguments=[
            ('use_sim_time', use_sim_time),
            ('params_file', localization_params_to_use),
            ('map', map_file),
            ('autostart', autostart),
        ],
    )

    # dock_detector_node / dock_manager_node — see docking.launch.py for why
    # this doesn't also start docking_server itself (nav2's own
    # navigation_launch.py above already does).
    dock_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_dock_nodes),
        launch_arguments=[('use_sim_time', use_sim_time)],
    )

    # Costmap Filter servers for keepout/restricted zones
    filter_mask_params = RewrittenYaml(
        source_file=nav2_params,
        param_rewrites={'yaml_filename': keepout_yaml_path},
        convert_types=True,
    )

    costmap_filter_info_server = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        output='screen',
        emulate_tty=True,
        parameters=[nav2_params],
    )

    filter_mask_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='filter_mask_server',
        output='screen',
        emulate_tty=True,
        parameters=[filter_mask_params],
    )

    costmap_filter_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_costmap_filters',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['filter_mask_server', 'costmap_filter_info_server'],
        }],
    )

    return [
        nav2,
        localization,
        dock_nodes,
        costmap_filter_info_server,
        filter_mask_server,
        costmap_filter_lifecycle_manager,
    ]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
