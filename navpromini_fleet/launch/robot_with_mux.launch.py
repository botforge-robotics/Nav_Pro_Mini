"""Hardware bringup + twist_mux so only one Twist reaches ESP32 /cmd_vel."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    fleet_share = get_package_share_directory('navpromini_fleet')
    ctrl_share = get_package_share_directory('navpromini_controller')
    mux_yaml = os.path.join(fleet_share, 'config', 'twist_mux.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('start_slam', default_value='false'),
        DeclareLaunchArgument('start_nav', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ctrl_share, 'launch', 'robot.launch.py')
            ),
            launch_arguments={
                'start_slam': LaunchConfiguration('start_slam'),
                'start_nav': LaunchConfiguration('start_nav'),
            }.items(),
        ),
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=[mux_yaml],
            # Publishes to cmd_vel → ESP32 micro-ROS subscription
        ),
    ])
