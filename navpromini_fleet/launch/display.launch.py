"""OLED/LED status display for ESP32 via micro-ROS topics."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('state', default_value='boot'),
        DeclareLaunchArgument('robot_name', default_value=''),
        DeclareLaunchArgument('ap_ssid', default_value=''),
        Node(
            package='navpromini_fleet',
            executable='status_display_node',
            name='navpro_status_display',
            output='screen',
            parameters=[{
                'state': LaunchConfiguration('state'),
                'robot_name': LaunchConfiguration('robot_name'),
                'ap_ssid': LaunchConfiguration('ap_ssid'),
                'refresh_hz': 0.5,
            }],
        ),
    ])
