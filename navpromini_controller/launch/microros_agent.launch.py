#!/usr/bin/env python3
"""Start the native micro-ROS agent over the Pi GPIO UART."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _setup(context, *args, **kwargs):
    device = LaunchConfiguration('serial_port').perform(context)
    baud = LaunchConfiguration('baudrate').perform(context)

    if not os.path.exists(device):
        return [
            LogInfo(msg=[
                f'WARNING: serial device {device} not found. '
                'Enable UART (raspi-config) and check wiring / device name.'
            ]),
        ]

    if not os.access(device, os.R_OK | os.W_OK):
        return [
            LogInfo(msg=[
                f'ERROR: no permission for {device} (need dialout/tty access). '
                'Disable serial console (remove console=serial0 from cmdline), '
                f'then: sudo chmod 660 {device} && sudo chgrp dialout {device} '
                '(or reboot after udev/raspi-config).'
            ]),
        ]

    return [
        LogInfo(msg=[f'native micro-ROS agent: {device} @ {baud}']),
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
                'serial', '--dev', device, '--baudrate', baud,
            ],
            output='screen',
            name='micro_ros_agent',
        ),
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
        OpaqueFunction(function=_setup),
    ])
