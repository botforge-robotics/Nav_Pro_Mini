"""OLED/LED status display for ESP32 via micro-ROS topics."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('state', default_value='boot'),
        DeclareLaunchArgument('robot_name', default_value='_'),
        DeclareLaunchArgument('ap_ssid', default_value='_'),
        DeclareLaunchArgument('ap_password', default_value='navprosetup'),
        Node(
            package='navpromini_fleet',
            executable='status_display_node',
            name='navpro_status_display',
            output='screen',
            parameters=[{
                'state': LaunchConfiguration('state'),
                # '_' = unset (start_display omits empty args; node treats '_' as empty)
                'robot_name': LaunchConfiguration('robot_name'),
                'ap_ssid': LaunchConfiguration('ap_ssid'),
                'ap_password': LaunchConfiguration('ap_password'),
                'refresh_hz': 1.0,
            }],
        ),
    ])