#!/usr/bin/env python3
"""Wheel odometry node (wheel_ticks → /odom + TF)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('navpromini_controller')
    default_params = os.path.join(pkg, 'config', 'odom_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'odom_params_file',
            default_value=default_params,
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='navpromini_controller',
            executable='odom_node',
            name='navpromini_odom',
            output='screen',
            parameters=[
                LaunchConfiguration('odom_params_file'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        ),
    ])
