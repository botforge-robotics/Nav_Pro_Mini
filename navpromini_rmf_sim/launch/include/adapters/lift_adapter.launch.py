#!/usr/bin/env python3
"""Lift ownership: Gazebo liblift (node) + lift_supervisor (adapter).

Matches https://osrf.github.io/ros2multirobotbook/integration_lifts.html

  fleet / RMF  →  /adapter_lift_requests  →  lift_supervisor
               →  /lift_requests          →  Gazebo liblift (lift node)
               ←  /lift_states

Optional Python fallback (no Gazebo lift plugin): use_python_lift_adapter:=true
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _truthy(value: str) -> bool:
    return value.lower() in ('true', '1', 'yes')


def _setup(context, *args, **kwargs):
    if not _truthy(LaunchConfiguration('use_python_lift_adapter').perform(context)):
        return [
            LogInfo(msg=[
                'Lift adapter: Gazebo liblift owns /lift_states; '
                'lift_supervisor (rmf_fleet_adapter) mediates. '
                'No Python lift adapter. '
                '(Fallback: use_python_lift_adapter:=true)'
            ]),
        ]

    pkg = get_package_share_directory('navpromini_rmf_sim')
    nav_graph = LaunchConfiguration('nav_graph').perform(context)
    if not nav_graph:
        nav_graph = os.path.join(pkg, 'site', 'nav_graphs', '0.yaml')
    use_sim = _truthy(LaunchConfiguration('use_sim_time').perform(context))
    return [
        LogInfo(msg=[
            'WARNING: Python lift_adapter fallback — not the book Gazebo stack. '
            'Prefer lifts.*.plugins: true + liblift.so'
        ]),
        Node(
            package='navpromini_rmf_sim',
            executable='lift_adapter',
            name='lift_adapter',
            parameters=[{
                'use_sim_time': use_sim,
                'nav_graph_file': nav_graph,
                'initial_floor': LaunchConfiguration('initial_map').perform(
                    context
                ),
                'move_gazebo_doors': True,
                'move_gazebo_cabin': True,
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
        DeclareLaunchArgument('initial_map', default_value='L1'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'use_python_lift_adapter',
            default_value='false',
            description='Start Python set_pose lift stand-in (only if liblift off)',
        ),
        OpaqueFunction(function=_setup),
    ])
