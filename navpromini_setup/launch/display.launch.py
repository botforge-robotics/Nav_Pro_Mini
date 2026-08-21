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
            package='navpromini_setup',
            executable='status_display_node',
            name='navpro_status_display',
            output='screen',
            parameters=[{
                'state': LaunchConfiguration('state'),
                'robot_name': LaunchConfiguration('robot_name'),
                'ap_ssid': LaunchConfiguration('ap_ssid'),
                'ap_password': LaunchConfiguration('ap_password'),
                # 4Hz, not 1Hz. This is the rate the OLED/LED composer runs
                # at, so at 1Hz a charge-state change waited up to a second
                # for the next tick (plus the repost interval on top) — the
                # LED took 5-10s to react to the dock. The battery now
                # publishes at 4Hz, so match it.
                'refresh_hz': 4.0,
            }],
        ),
    ])
