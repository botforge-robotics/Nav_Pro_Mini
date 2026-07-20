#!/usr/bin/env python3
"""Nav2 bringup for NavProMini (localization + navigation).

Pass only the map name (no path / extension). Full path is built from
the SLAM save directory: <ws>/src/navpromini_mapping/maps/<map_name>.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Same folder used by map_saver.launch.py
MAPS_DIR = os.path.join(
    os.path.expanduser('~'),
    'NavProMini_ws',
    'src',
    'navpromini_mapping',
    'maps',
)


def resolve_map_yaml(map_name: str) -> str:
    """Build absolute map yaml path from a bare map name."""
    name = map_name.strip()
    if name.endswith('.yaml'):
        name = name[:-5]
    if name.endswith('.pgm'):
        name = name[:-4]

    # Absolute path already provided
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
    # Prefer src maps dir even if not yet present (clearer error from map_server)
    return candidates[0]


def _setup(context, *args, **kwargs):
    pkg = get_package_share_directory('navpromini_navigation')
    bringup_dir = get_package_share_directory('nav2_bringup')

    map_name = LaunchConfiguration('map_name').perform(context)
    map_yaml = resolve_map_yaml(map_name)

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    use_rviz = LaunchConfiguration('use_rviz')
    autostart = LaunchConfiguration('autostart')

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'slam': 'False',
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': 'False',
            'use_localization': 'True',
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg, 'rviz', 'navigation.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    return [
        LogInfo(msg=[f'NavProMini Nav2 — map_name={map_name} → {map_yaml}']),
        bringup,
        rviz,
    ]


def generate_launch_description():
    pkg = get_package_share_directory('navpromini_navigation')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg, 'config', 'nav2_params.yaml'),
        ),
        DeclareLaunchArgument(
            'map_name',
            default_value='navpromini_map',
            description=(
                'Map name only (no path). Resolved to '
                f'{MAPS_DIR}/<map_name>.yaml'
            ),
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch Nav2 RViz with map, costmaps, and plans '
                        '(default: true). Use use_rviz:=false to disable.',
        ),
        OpaqueFunction(function=_setup),
    ])
