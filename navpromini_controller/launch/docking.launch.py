#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='navpromini_controller',
            executable='camera_node',
            name='navpromini_camera',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='navpromini_controller',
            executable='dock_marker_node',
            name='navpromini_dock_marker',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='navpromini_controller',
            executable='dock_manager_node',
            name='navpromini_dock_manager',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
