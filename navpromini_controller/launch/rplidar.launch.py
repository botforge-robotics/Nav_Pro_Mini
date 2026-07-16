#!/usr/bin/env python3
"""RPLidar A1M8 bringup with URDF frame lidar_1.

Resolves the serial port at launch time: prefers the stable udev
symlink (/dev/rplidar), then the CP2102 by-id path, then the configured
port. ttyUSB0/ttyUSB1 swap between boots, so a fixed name is unreliable.
"""

import glob
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# This robot's RPLidar uses a Silicon Labs CP2102 USB-UART adapter.
CP2102_BY_ID_GLOB = '/dev/serial/by-id/usb-Silicon_Labs_CP2102_*'
STABLE_SYMLINK = '/dev/rplidar'


def _resolve_port(configured: str) -> str:
    if os.path.exists(STABLE_SYMLINK):
        return STABLE_SYMLINK
    cp2102 = sorted(glob.glob(CP2102_BY_ID_GLOB))
    if cp2102:
        return cp2102[0]
    return configured


def _setup(context, *args, **kwargs):
    configured = LaunchConfiguration('serial_port').perform(context)
    port = _resolve_port(configured)

    actions = [LogInfo(msg=[f'RPLidar port: {port}'])]
    if not os.path.exists(port):
        actions.append(LogInfo(msg=[
            f'WARNING: lidar port {port} not found — check USB cable/power.'
        ]))

    actions.append(
        Node(
            name='rplidar_composition',
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': port,
                'serial_baudrate': LaunchConfiguration('baudrate'),
                'frame_id': LaunchConfiguration('frame_id'),
                'inverted': False,
                'angle_compensate': True,
            }],
        )
    )
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/rplidar',
            description='Fallback RPLidar device if /dev/rplidar and '
                        'CP2102 by-id path are absent',
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
        OpaqueFunction(function=_setup),
    ])
