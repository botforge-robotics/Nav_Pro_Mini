#!/usr/bin/env python3
"""Filter lidar hits on the robot body (pillars, deck) using laser_filters."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('navpromini_controller')
    params_file = os.path.join(pkg, 'config', 'scan_filter.yaml')

    scan_in = LaunchConfiguration('scan_in')
    scan_out = LaunchConfiguration('scan_out')

    return LaunchDescription([
        DeclareLaunchArgument(
            'scan_in',
            default_value='/scan',
            description='Raw LaserScan from the lidar driver',
        ),
        DeclareLaunchArgument(
            'scan_out',
            default_value='/scan_filtered',
            description='Filtered scan for Nav2 / AMCL / SLAM',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=params_file,
            description='laser_filters scan_to_scan_filter_chain params',
        ),
        LogInfo(msg=[
            'laser_filters: ', scan_in, ' → ', scan_out,
            ' (body box + footprint filter)',
        ]),
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='scan_to_scan_filter_chain',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
            remappings=[
                ('scan', scan_in),
                ('scan_filtered', scan_out),
            ],
        ),
    ])
