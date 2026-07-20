#!/usr/bin/env python3
"""Joystick teleop for NavProMini -> /cmd_vel.

Publishes a steady /cmd_vel stream (joy autorepeat) so the ESP32
cmd_vel timeout does not chatter. Defaults match real-robot use
(use_sim_time:=false); pass true only for Gazebo.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('navpromini_teleop')

    joy_config = LaunchConfiguration('joy_config')
    joy_dev = LaunchConfiguration('joy_dev')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    use_sim_time = LaunchConfiguration('use_sim_time')

    config_file = [
        TextSubstitution(text=os.path.join(pkg, 'config', '')),
        joy_config,
        TextSubstitution(text='.config.yaml'),
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'joy_config',
            default_value='xbox',
            description='Joystick config name without extension (xbox, ps4)',
        ),
        DeclareLaunchArgument(
            'joy_dev',
            default_value='0',
            description='Joystick device id (js0 -> 0)',
        ),
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='cmd_vel',
            description='Velocity command topic',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='true only for Gazebo; false on real robot',
        ),
        LogInfo(msg=['NavProMini joystick teleop config=', joy_config]),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'device_id': joy_dev,
                # Wider deadzone reduces center-noise → less jerky cmd_vel
                'deadzone': 0.3,
                # Keep publishing while stick held (ESP32 stops if /cmd_vel gaps)
                'autorepeat_rate': 30.0,
                'sticky_buttons': False,
                'use_sim_time': use_sim_time,
            }],
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            parameters=[
                config_file,
                {
                    'publish_stamped_twist': False,
                    'use_sim_time': use_sim_time,
                },
            ],
            remappings=[('/cmd_vel', cmd_vel_topic)],
        ),
    ])
