#!/usr/bin/env python3
"""AMCL localization only. Pass map_name (no path); path is built automatically."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

MAPS_DIR = os.path.join(
    os.path.expanduser('~'),
    'NavProMini_ws',
    'src',
    'navpromini_mapping',
    'maps',
)


def resolve_map_yaml(map_name: str) -> str:
    name = map_name.strip()
    if name.endswith('.yaml'):
        name = name[:-5]
    if name.endswith('.pgm'):
        name = name[:-4]
    if os.path.isabs(map_name) and map_name.endswith('.yaml'):
        return map_name

    candidates = [
        os.path.join(MAPS_DIR, f'{name}.yaml'),
        os.path.join(
            get_package_share_directory('navpromini_mapping'),
            'maps',
            f'{name}.yaml',
        ),
        os.path.join(
            get_package_share_directory('navpromini_navigation'),
            'maps',
            f'{name}.yaml',
        ),
    ]
    for path in candidates:
        if os.path.isfile(path):
            return path
    return candidates[0]


def _setup(context, *args, **kwargs):
    pkg = get_package_share_directory('navpromini_navigation')
    bringup_dir = get_package_share_directory('nav2_bringup')

    map_name = LaunchConfiguration('map_name').perform(context)
    map_yaml = resolve_map_yaml(map_name)

    return [
        LogInfo(msg=[f'Localization — map_name={map_name} → {map_yaml}']),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'localization_launch.py')
            ),
            launch_arguments={
                'map': map_yaml,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': LaunchConfiguration('params_file'),
                'autostart': LaunchConfiguration('autostart'),
            }.items(),
        ),
    ]


def generate_launch_description():
    pkg = get_package_share_directory('navpromini_navigation')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument(
            'map_name',
            default_value='navpromini_map',
            description=f'Map name only → {MAPS_DIR}/<map_name>.yaml',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg, 'config', 'nav2_params.yaml'),
        ),
        OpaqueFunction(function=_setup),
    ])
