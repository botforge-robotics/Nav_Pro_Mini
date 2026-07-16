#!/usr/bin/env python3
"""RPLidar A1M8 bringup with URDF frame lidar_1."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    baudrate = LaunchConfiguration('baudrate')
    frame_id = LaunchConfiguration('frame_id')

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='RPLidar USB serial device on the Pi',
        ),
        DeclareLaunchArgument(
            'baudrate',
            default_value='115200',
            description='A1/A1M8 baud rate',
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='lidar_1',
            description='Must match URDF lidar link',
        ),
        Node(
            name='rplidar_composition',
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': serial_port,
                'serial_baudrate': baudrate,
                'frame_id': frame_id,
                'inverted': False,
                'angle_compensate': True,
            }],
        ),
    ])
