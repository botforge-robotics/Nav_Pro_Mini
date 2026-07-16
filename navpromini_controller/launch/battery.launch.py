#!/usr/bin/env python3
"""Daly Smart BMS battery telemetry node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('navpromini_controller')
    default_params = os.path.join(pkg, 'config', 'battery_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('battery_params_file', default_value=default_params),
        DeclareLaunchArgument('serial_port', default_value='/dev/battery_bms'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='navpromini_controller',
            executable='battery_node',
            name='navpromini_battery',
            output='screen',
            parameters=[
                LaunchConfiguration('battery_params_file'),
                {
                    'serial_port': LaunchConfiguration('serial_port'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                },
            ],
        ),
    ])
