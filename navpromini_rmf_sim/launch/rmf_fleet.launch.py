#!/usr/bin/env python3
"""Fleet adapter entry — includes ``launch/include/adapters/fleet_adapter.launch.py``.

Requires RMF core + API (rmf_web) and namespaced Nav2 (rmf_sim start_nav:=true).
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory('navpromini_rmf_sim')
    fleet_launch = os.path.join(
        pkg, 'launch', 'include', 'adapters', 'fleet_adapter.launch.py'
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'fleet_config',
            default_value=os.path.join(
                pkg, 'site', 'fleet_config', 'navpromini.yaml'
            ),
        ),
        DeclareLaunchArgument(
            'nav_graph',
            default_value=os.path.join(pkg, 'site', 'nav_graphs', '0.yaml'),
        ),
        DeclareLaunchArgument(
            'spawn_poses',
            default_value=os.path.join(pkg, 'site', 'spawn_poses.yaml'),
        ),
        DeclareLaunchArgument('robot_names', default_value='robot1,robot2'),
        DeclareLaunchArgument('initial_map', default_value='L1'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'server_uri',
            default_value='http://localhost:8000/_internal',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fleet_launch),
            launch_arguments={
                'fleet_config': LaunchConfiguration('fleet_config'),
                'nav_graph': LaunchConfiguration('nav_graph'),
                'spawn_poses': LaunchConfiguration('spawn_poses'),
                'robot_names': LaunchConfiguration('robot_names'),
                'initial_map': LaunchConfiguration('initial_map'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'server_uri': LaunchConfiguration('server_uri'),
            }.items(),
        ),
    ])
