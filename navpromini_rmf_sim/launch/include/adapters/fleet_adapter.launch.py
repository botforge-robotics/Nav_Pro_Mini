#!/usr/bin/env python3
"""Fleet adapter launch: path_to_nav2_bridge + rmf_demos EasyFullControl."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _truthy(value: str) -> bool:
    return value.lower() in ('true', '1', 'yes')


def _setup(context, *args, **kwargs):
    pkg = get_package_share_directory('navpromini_rmf_sim')
    fleet_config = LaunchConfiguration('fleet_config').perform(context)
    nav_graph = LaunchConfiguration('nav_graph').perform(context)
    server_uri = LaunchConfiguration('server_uri').perform(context)
    use_sim = _truthy(LaunchConfiguration('use_sim_time').perform(context))
    spawn_poses = LaunchConfiguration('spawn_poses').perform(context)
    robots = LaunchConfiguration('robot_names').perform(context)
    initial_map = LaunchConfiguration('initial_map').perform(context)

    if not fleet_config:
        fleet_config = os.path.join(pkg, 'site', 'fleet_config', 'navpromini.yaml')
    if not nav_graph:
        nav_graph = os.path.join(pkg, 'site', 'nav_graphs', '0.yaml')
    if not spawn_poses:
        spawn_poses = os.path.join(pkg, 'site', 'spawn_poses.yaml')

    demos_share = get_package_share_directory('rmf_demos_fleet_adapter')
    return [
        LogInfo(msg=[
            f'Fleet adapter config={fleet_config} nav_graph={nav_graph} '
            f'server_uri={server_uri} use_sim_time={use_sim}'
        ]),
        Node(
            package='navpromini_rmf_sim',
            executable='path_to_nav2_bridge',
            name='path_to_nav2_bridge',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim,
                'robot_names': robots,
                'level_name': initial_map,
                'spawn_poses': spawn_poses,
            }],
        ),
        TimerAction(
            period=3.0,
            actions=[
                IncludeLaunchDescription(
                    FrontendLaunchDescriptionSource(
                        os.path.join(
                            demos_share, 'launch', 'fleet_adapter.launch.xml'
                        )
                    ),
                    launch_arguments={
                        'use_sim_time': 'true' if use_sim else 'false',
                        'config_file': fleet_config,
                        'nav_graph_file': nav_graph,
                        'server_uri': server_uri,
                    }.items(),
                ),
            ],
        ),
    ]


def generate_launch_description():
    pkg = get_package_share_directory('navpromini_rmf_sim')
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
        OpaqueFunction(function=_setup),
    ])
