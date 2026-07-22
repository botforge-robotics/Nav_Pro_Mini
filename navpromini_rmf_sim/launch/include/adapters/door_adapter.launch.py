#!/usr/bin/env python3
"""Document door ownership: Gazebo libdoor + door_supervisor (no Python node)."""

from launch import LaunchDescription
from launch.actions import LogInfo


def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg=[
            'Door adapter: Gazebo libdoor owns /door_states; '
            'door_supervisor (rmf_fleet_adapter) mediates. '
            'No Python door adapter is started.'
        ]),
    ])
