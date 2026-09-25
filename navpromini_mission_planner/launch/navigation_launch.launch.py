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
            ('params_file', nav2_params.perform(context)),
            ('autostart', autostart),
        ],
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_localization),
        launch_arguments=[
            ('use_sim_time', use_sim_time),
            ('params_file', localization_params),
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
