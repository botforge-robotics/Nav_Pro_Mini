#!/usr/bin/env python3
"""Mission Planner navigation wrapper.

The web/Flutter app passes map:=<name>.yaml. NavProMini navigation expects
map_name:=<bare name>. This wrapper accepts both.
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
from launch.substitutions import LaunchConfiguration


def _setup(context, *args, **kwargs):
    pkg_nav = get_package_share_directory('navpromini_navigation')

    map_arg = LaunchConfiguration('map').perform(context).strip()
    map_name_arg = LaunchConfiguration('map_name').perform(context).strip()

    # Prefer explicit map= from Mission Planner app; fall back to map_name
    raw = map_arg if map_arg else map_name_arg
    if raw.endswith('.yaml'):
        raw = raw[:-5]
    if raw.endswith('.pgm'):
        raw = raw[:-4]
    map_name = raw or 'warehouse'

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav, 'launch', 'navigation.launch.py')
            ),
            launch_arguments={
                'map_name': map_name,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': LaunchConfiguration('autostart'),
                'use_rviz': LaunchConfiguration('use_rviz'),
                'params_file': LaunchConfiguration('params_file'),
            }.items(),
        )
    ]


def generate_launch_description():
    pkg_nav = get_package_share_directory('navpromini_navigation')
    default_params = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')
    if not os.path.isfile(default_params):
        # fallback common name
        for candidate in ('nav2.yaml', 'navigation.yaml'):
            p = os.path.join(pkg_nav, 'config', candidate)
            if os.path.isfile(p):
                default_params = p
                break

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Map file name from Mission Planner (e.g. office.yaml)',
        ),
        DeclareLaunchArgument(
            'map_name',
            default_value='warehouse',
            description='Bare map name for NavProMini navigation',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        OpaqueFunction(function=_setup),
    ])
