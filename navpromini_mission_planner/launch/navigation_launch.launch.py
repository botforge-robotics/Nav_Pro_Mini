#!/usr/bin/env python3
"""Mission Planner navigation wrapper (per botforge nav2_mission_planner docs).

Starts Nav2 localization (AMCL + map_server) + navigation + costmap keepout filter servers.
The app passes only the map name, e.g. map:=office.yaml — this file builds
the full path under navpromini_mapping/maps and launches the keepout filter servers.

App launch ref: navpromini_mission_planner/navigation_launch
"""

import os
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
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

    # Preload dynamic motion settings from robot SQLite database (single source of truth)
    motion_settings = None
    try:
        from navpromini_controller.motion_config import get_motion_settings
        motion_settings = get_motion_settings()
        print(f"[navigation_launch] Loaded dynamic motion settings from robot DB: {motion_settings}")
    except Exception as e:
        print(f"[navigation_launch] Note: using default motion settings ({e})")

    base_nav2_file = nav2_params.perform(context)
    try:
        with open(base_nav2_file, 'r') as f:
            yaml_data = yaml.safe_load(f)

        if os.path.isfile(roadnet_geojson):
            yaml_data.setdefault('route_server', {}).setdefault('ros__parameters', {})['graph_filepath'] = roadnet_geojson
            print(f"[navigation_launch] Preloading RouteServer graph: {roadnet_geojson}")

        if motion_settings:
            max_lin = float(motion_settings.get('max_linear_speed', 0.35))
            min_lin = float(motion_settings.get('min_linear_speed', 0.05))
            max_ang = float(motion_settings.get('max_angular_speed', 0.30))
            min_ang = float(motion_settings.get('min_angular_speed', 0.10))
            acc_lin = float(motion_settings.get('linear_accel', 0.09))
            dec_lin = float(motion_settings.get('linear_decel', -0.18))
            acc_ang = float(motion_settings.get('angular_accel', 0.50))
            dec_ang = -1.0 * abs(acc_ang) * 1.4

            cs_params = yaml_data.setdefault('controller_server', {}).setdefault('ros__parameters', {})
            cs_params['controller_frequency'] = 15.0

            gc = cs_params.setdefault('general_goal_checker', {})
            gc['xy_goal_tolerance'] = 0.12
            gc['yaw_goal_tolerance'] = 0.25

            fp = cs_params.setdefault('FollowPath', {})
            fp['desired_linear_vel'] = max_lin
            fp['approach_velocity_scaling_dist'] = 0.30
            fp['min_approach_linear_velocity'] = 0.12
            fp['rotate_to_heading_angular_vel'] = max_ang
            fp['max_angular_accel'] = acc_ang
            fp['rotate_to_heading_min_angle'] = 0.85
            fp['lookahead_dist'] = 0.28
            fp['min_lookahead_dist'] = 0.20
            fp['max_lookahead_dist'] = 0.45
            fp['lookahead_time'] = 1.0
            fp['use_fixed_curvature_lookahead'] = True
            fp['curvature_lookahead_dist'] = 0.35
            fp['regulated_linear_scaling_min_radius'] = 0.70
            fp['regulated_linear_scaling_min_speed'] = 0.12
            fp['use_cost_regulated_linear_velocity_scaling'] = False

            dwb = cs_params.setdefault('FollowPathDWB', {})
            dwb['max_vel_x'] = max_lin
            dwb['max_speed_xy'] = max_lin
            dwb['max_vel_theta'] = max_ang
            dwb['max_speed_theta'] = max_ang
            dwb['acc_lim_x'] = acc_lin
            dwb['acc_lim_theta'] = acc_ang
            dwb['decel_lim_x'] = dec_lin

            vs_params = yaml_data.setdefault('velocity_smoother', {}).setdefault('ros__parameters', {})
            vs_params['max_velocity'] = [max_lin, 0.0, max_ang]
            vs_params['min_velocity'] = [-0.10, 0.0, -max_ang]
            vs_params['max_accel'] = [acc_lin, 0.0, acc_ang]
            vs_params['max_decel'] = [dec_lin, 0.0, dec_ang]

            bs_params = yaml_data.setdefault('behavior_server', {}).setdefault('ros__parameters', {})
            bs_params['max_rotational_vel'] = max_ang
            bs_params['min_rotational_vel'] = min_ang
            bs_params['rotational_acc_lim'] = acc_ang

        tmp = tempfile.NamedTemporaryFile(mode='w', prefix='nav2_dyn_', suffix='.yaml', delete=False)
        yaml.dump(yaml_data, tmp)
        tmp.close()
        nav2_params_to_use = tmp.name
        print(f"[navigation_launch] Configured dynamic Nav2 YAML: {nav2_params_to_use}")
    except Exception as exc:
        print(f"[navigation_launch] Error generating dynamic Nav2 YAML: {exc}; using base params file")
        nav2_params_to_use = base_nav2_file

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

    nav2 = GroupAction([
        SetParameter('bond_timeout', 40.0),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_nav2),
            launch_arguments=[
                ('use_sim_time', use_sim_time),
                ('params_file', nav2_params_to_use),
                ('autostart', autostart),
            ],
        ),
    ])

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
