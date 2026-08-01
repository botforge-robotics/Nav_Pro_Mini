#!/usr/bin/env python3
"""Mission Planner mapping wrapper (per botforge nav2_mission_planner docs).

Starts Nav2 navigation stack + slam_toolbox (sync/async).
App launch ref: navpromini_mission_planner/mapping_launch
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
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
        'sync',
        default_value='false',
        choices=['true', 'false'],
        description='Use synchronous SLAM (false = async, NavProMini default)',
    ),
    DeclareLaunchArgument(
        'autostart',
        default_value='true',
        choices=['true', 'false'],
        description='Automatically startup slam_toolbox / Nav2',
    ),
    DeclareLaunchArgument(
        'use_lifecycle_manager',
        default_value='false',
        choices=['true', 'false'],
        description='Enable bond connection during node activation',
    ),
    DeclareLaunchArgument(
        'slam_params_file',
        default_value=PathJoinSubstitution([
            get_package_share_directory('navpromini_mapping'),
            'config',
            'mapper_params_online_async.yaml',
        ]),
        description='Path to the SLAM Toolbox configuration file',
    ),
    DeclareLaunchArgument(
        'nav2_params_file',
        default_value=PathJoinSubstitution([
            get_package_share_directory('navpromini_navigation'),
            'config',
            'nav2_params.yaml',
        ]),
        description='Path to the Nav2 navigation parameters file',
    ),
]


def launch_setup(context, *args, **kwargs):
    sync = LaunchConfiguration('sync')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_lifecycle_manager = LaunchConfiguration('use_lifecycle_manager')
    slam_params = LaunchConfiguration('slam_params_file')
    nav2_params = LaunchConfiguration('nav2_params_file')

    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    launch_slam_sync = PathJoinSubstitution(
        [pkg_slam_toolbox, 'launch', 'online_sync_launch.py'])
    launch_slam_async = PathJoinSubstitution(
        [pkg_slam_toolbox, 'launch', 'online_async_launch.py'])
    launch_nav2 = PathJoinSubstitution(
        [pkg_nav2_bringup, 'launch', 'navigation_launch.py'])

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_nav2),
        launch_arguments=[
            ('use_sim_time', use_sim_time),
            ('params_file', nav2_params.perform(context)),
            ('autostart', autostart),
        ],
    )

    slam_sync = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_slam_sync),
        launch_arguments=[
            ('use_sim_time', use_sim_time),
            ('autostart', autostart),
            ('use_lifecycle_manager', use_lifecycle_manager),
            ('slam_params_file', slam_params.perform(context)),
        ],
        condition=IfCondition(sync),
    )

    slam_async = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_slam_async),
        launch_arguments=[
            ('use_sim_time', use_sim_time),
            ('autostart', autostart),
            ('use_lifecycle_manager', use_lifecycle_manager),
            ('slam_params_file', slam_params.perform(context)),
        ],
        condition=UnlessCondition(sync),
    )

    return [nav2, slam_sync, slam_async]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
