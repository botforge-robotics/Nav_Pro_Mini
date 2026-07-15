#!/usr/bin/env python3
"""Save current SLAM map by name into navpromini_mapping/maps/."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration

MAPS_DIR = os.path.join(
    os.path.expanduser('~'),
    'NavProMini_ws',
    'src',
    'navpromini_mapping',
    'maps',
)


def _setup(context, *args, **kwargs):
    map_name = LaunchConfiguration('map_name').perform(context).strip()
    if map_name.endswith('.yaml'):
        map_name = map_name[:-5]
    if map_name.endswith('.pgm'):
        map_name = map_name[:-4]

    os.makedirs(MAPS_DIR, exist_ok=True)
    map_path = os.path.join(MAPS_DIR, map_name)

    return [
        LogInfo(msg=[f'Saving map as {map_path}.pgm / .yaml']),
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                '-f', map_path,
            ],
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_name',
            default_value='navpromini_map',
            description=f'Map name only → saved to {MAPS_DIR}/<map_name>.pgm/.yaml',
        ),
        OpaqueFunction(function=_setup),
    ])
