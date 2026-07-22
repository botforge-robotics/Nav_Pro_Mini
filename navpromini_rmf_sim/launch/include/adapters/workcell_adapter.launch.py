#!/usr/bin/env python3
"""Workcell (dispenser/ingestor) adapter launch."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _truthy(value: str) -> bool:
    return value.lower() in ('true', '1', 'yes')


def _setup(context, *args, **kwargs):
    pkg = get_package_share_directory('navpromini_rmf_sim')
    nav_graph = LaunchConfiguration('nav_graph').perform(context)
    if not nav_graph:
        nav_graph = os.path.join(pkg, 'site', 'nav_graphs', '0.yaml')
    use_sim = _truthy(LaunchConfiguration('use_sim_time').perform(context))
    return [
        Node(
            package='navpromini_rmf_sim',
            executable='workcell_adapter',
            name='workcell_adapter',
            parameters=[{
                'use_sim_time': use_sim,
                'nav_graph_file': nav_graph,
            }],
            output='screen',
        ),
    ]


def generate_launch_description():
    pkg = get_package_share_directory('navpromini_rmf_sim')
    return LaunchDescription([
        DeclareLaunchArgument(
            'nav_graph',
            default_value=os.path.join(pkg, 'site', 'nav_graphs', '0.yaml'),
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=_setup),
    ])
