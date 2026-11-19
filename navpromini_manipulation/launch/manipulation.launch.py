#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    step_delay_sec = LaunchConfiguration('step_delay_sec')
    enable_vision = LaunchConfiguration('enable_vision')
    start_navigation = LaunchConfiguration('start_navigation')
    map_name = LaunchConfiguration('map_name')

    pkg_nav = get_package_share_directory('navpromini_navigation')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation clock from Gazebo'),
        DeclareLaunchArgument('step_delay_sec', default_value='1.6',
                              description='Time interval between manipulation trajectory waypoints'),
        DeclareLaunchArgument('enable_vision', default_value='true',
                              description='Enable wrist camera color verification'),
        DeclareLaunchArgument('start_navigation', default_value='false',
                              description='Optionally start Nav2 navigation stack (map + AMCL + costmaps + planner)'),
        DeclareLaunchArgument('map_name', default_value='playground',
                              description='Map name to load in Nav2 (e.g. playground, cafe, office)'),

        # Optional Nav2 Stack Bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav, 'launch', 'navigation.launch.py')
            ),
            launch_arguments={
                'map_name': map_name,
                'use_sim_time': use_sim_time,
                'use_rviz': 'false',
            }.items(),
            condition=IfCondition(start_navigation),
        ),

        # Object Manipulation Executive Node
        Node(
            package='navpromini_manipulation',
            executable='object_manipulation_node',
            name='object_manipulation_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'step_delay_sec': step_delay_sec,
                'enable_vision_verification': enable_vision,
            }],
        ),
    ])
