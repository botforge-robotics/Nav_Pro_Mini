#!/usr/bin/env python3
"""Launch NavProMini in Gazebo Harmonic via navpromini_gazebo."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_navpromini_gazebo = get_package_share_directory('navpromini_gazebo')
    gazebo_launch_path = os.path.join(pkg_navpromini_gazebo, 'launch', 'gazebo.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world_name',
            default_value='empty',
            description='World to load: empty, office, cafe',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Launch RViz2 for visualization',
        ),
        DeclareLaunchArgument('x', default_value='', description='Initial X position (empty for world default)'),
        DeclareLaunchArgument('y', default_value='', description='Initial Y position (empty for world default)'),
        DeclareLaunchArgument('z', default_value='', description='Initial Z position (empty for world default)'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Initial yaw angle'),
        DeclareLaunchArgument(
            'use_arm',
            default_value='false',
            description='Spawn robot with modular arm attachment (true/false)',
        ),
        DeclareLaunchArgument(
            'arm_z',
            default_value='0.0198',
            description='Z offset of modular arm attachment above lidar in meters',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_path),
            launch_arguments={
                'world_name': LaunchConfiguration('world_name'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'use_rviz': LaunchConfiguration('use_rviz'),
                'x': LaunchConfiguration('x'),
                'y': LaunchConfiguration('y'),
                'z': LaunchConfiguration('z'),
                'yaw': LaunchConfiguration('yaw'),
                'use_arm': LaunchConfiguration('use_arm'),
                'arm_z': LaunchConfiguration('arm_z'),
            }.items(),
        ),
    ])
