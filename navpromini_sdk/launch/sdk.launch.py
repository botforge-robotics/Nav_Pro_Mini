#!/usr/bin/env python3
"""NavProMini SDK HTTP/WebSocket API server.

Purely additive: rosbridge (:9090) and the Flutter app keep working unchanged.
Port 8090 avoids the provisioning portal (:80) and web_video_server (:8081).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='8090',
                              description='TCP port for the REST/WebSocket API'),
        DeclareLaunchArgument('address', default_value='0.0.0.0',
                              description='Bind address'),
        DeclareLaunchArgument(
            'auth_token', default_value='',
            description='Bearer token required on every request. Empty = no auth.'),
        DeclareLaunchArgument('rosbridge_url', default_value='ws://127.0.0.1:9090',
                              description='Used to hold rosbridge client_count '
                                          'above zero while the SDK owns a launch'),
        Node(
            package='navpromini_sdk',
            executable='sdk_server',
            name='navpromini_sdk',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'address': LaunchConfiguration('address'),
                'auth_token': LaunchConfiguration('auth_token'),
                'rosbridge_url': LaunchConfiguration('rosbridge_url'),
            }],
        ),
    ])
