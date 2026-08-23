#!/usr/bin/env python3
"""Launches the geometric self-filter node with config/self_filter.yaml.

Deliberately NOT wired into robot.launch.py or the existing
scan_to_scan_filter_chain by this commit — see the package README's
"Switching from the old filter" section for why this needs a side-by-side
RViz comparison before it replaces anything already running on the robot.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('navpromini_self_filter')
    default_config = os.path.join(pkg_share, 'config', 'self_filter.yaml')

    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to self_filter.yaml',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time',
    )

    node = Node(
        package='navpromini_self_filter',
        executable='self_filter_node',
        name='self_filter_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    return LaunchDescription([config_arg, use_sim_time_arg, node])
