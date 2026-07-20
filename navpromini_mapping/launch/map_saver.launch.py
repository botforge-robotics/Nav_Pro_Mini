#!/usr/bin/env python3
"""Save current SLAM map by name into navpromini_mapping/maps/.

Requires slam_toolbox (or another node) to be publishing /map.
slam_toolbox uses TRANSIENT_LOCAL durability — map_saver must match.
"""

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

    map_topic = LaunchConfiguration('map_topic').perform(context).strip()
    timeout = LaunchConfiguration('save_map_timeout').perform(context).strip()

    os.makedirs(MAPS_DIR, exist_ok=True)
    map_path = os.path.join(MAPS_DIR, map_name)

    return [
        LogInfo(msg=[
            f'Saving map as {map_path}.pgm / .yaml from topic {map_topic} '
            f'(timeout={timeout}s). SLAM must be running.'
        ]),
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                '-f', map_path,
                '-t', map_topic,
                '--free', '0.25',
                '--occ', '0.65',
                '--ros-args',
                '-p', 'map_subscribe_transient_local:=true',
                '-p', f'save_map_timeout:={timeout}',
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
        DeclareLaunchArgument(
            'map_topic',
            default_value='map',
            description='OccupancyGrid topic (slam_toolbox default: map)',
        ),
        DeclareLaunchArgument(
            'save_map_timeout',
            default_value='15.0',
            description='Seconds to wait for a /map message',
        ),
        OpaqueFunction(function=_setup),
    ])
