#!/usr/bin/env python3
"""Start micro-ROS agent for ESP32 on Waveshare board (Pi GPIO UART).

Pi 5 + Waveshare General Driver: default /dev/ttyAMA0 @ 115200.
Flash ESP32 over Type-C USB; run agent on the stacked GPIO UART.
Uses Docker image (avoids broken host micro_ros_agent on some Jazzy installs).
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _setup(context, *args, **kwargs):
    device = LaunchConfiguration('serial_port').perform(context)
    baud = LaunchConfiguration('baudrate').perform(context)
    image = LaunchConfiguration('docker_image').perform(context)

    if not os.path.exists(device):
        return [
            LogInfo(msg=[
                f'WARNING: serial device {device} not found. '
                'Enable UART (raspi-config) and check wiring / device name.'
            ]),
        ]

    cmd = [
        'docker', 'run', '--rm', '--privileged', '--net=host',
        '-v', '/dev:/dev',
        image,
        'serial',
        '--dev', device,
        '--baudrate', baud,
        '-v6',
    ]

    return [
        LogInfo(msg=[f'micro-ROS agent: {device} @ {baud} ({image})']),
        ExecuteProcess(cmd=cmd, output='screen', name='micro_ros_agent'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyAMA0',
            description='Pi 5 GPIO UART to ESP32 (Waveshare stacked board). '
                        'Pi 4 often uses /dev/serial0.',
        ),
        DeclareLaunchArgument('baudrate', default_value='115200'),
        DeclareLaunchArgument(
            'docker_image',
            default_value='microros/micro-ros-agent:jazzy',
        ),
        OpaqueFunction(function=_setup),
    ])
