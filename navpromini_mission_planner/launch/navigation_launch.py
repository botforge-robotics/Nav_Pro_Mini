#!/usr/bin/env python3
"""Mission Planner navigation wrapper (per botforge nav2_mission_planner docs).

Starts Nav2 localization (AMCL + map_server) + navigation.
The app passes only the map name, e.g. map:=office.yaml — this file builds
the full path under navpromini_mapping/maps.

App launch ref: navpromini_mission_planner/navigation_launch
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


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
        default_value='warehouse.yaml',
        description='Map yaml filename only (e.g. office.yaml) from Mission Planner',
    ),
]


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    nav2_params = LaunchConfiguration('nav2_params_file')
    localization_params = LaunchConfiguration('localization_params_file')
    map_name = LaunchConfiguration('map')

    # Full path: <navpromini_mapping share>/maps/<office.yaml>
    map_file = PathJoinSubstitution([
        get_package_share_directory('navpromini_mapping'),
        'maps',
        map_name,
    ])

    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    launch_nav2 = PathJoinSubstitution(
        [pkg_nav2_bringup, 'launch', 'navigation_launch.py'])
    launch_localization = PathJoinSubstitution(
        [pkg_nav2_bringup, 'launch', 'localization_launch.py'])

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

    return [nav2, localization]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
