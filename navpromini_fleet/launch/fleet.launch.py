"""Launch fleet agent nodes after provisioning (heartbeat + mode_manager)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_path = LaunchConfiguration('config_path')
    maps_dir = LaunchConfiguration('maps_dir')
    return LaunchDescription([
        DeclareLaunchArgument('config_path', default_value='/etc/navpro/fleet.yaml'),
        DeclareLaunchArgument('maps_dir', default_value='/var/lib/navpro/maps'),
        Node(
            package='navpromini_fleet',
            executable='heartbeat_node',
            name='navpro_heartbeat',
            output='screen',
            parameters=[{
                'config_path': config_path,
                'maps_dir': maps_dir,
                'period_s': 2.0,
            }],
        ),
        Node(
            package='navpromini_fleet',
            executable='mode_manager',
            name='navpro_mode_manager',
            output='screen',
            parameters=[{
                'config_path': config_path,
                'maps_dir': maps_dir,
            }],
        ),
        # RMF goals over zenoh topics (PoseStamped) → local Nav2 action.
        Node(
            package='navpromini_fleet',
            executable='nav_goal_relay',
            name='navpro_nav_goal_relay',
            output='screen',
        ),
    ])
