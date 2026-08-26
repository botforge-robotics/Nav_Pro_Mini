#!/usr/bin/env python3
"""NavProMini docking: camera -> AprilTag detection -> approach controller.

Starts:
  - camera_node     rear USB camera (raw + compressed + camera_info)
  - dock_tag_node   AprilTag 36h11 detection -> `dock_tag`
  - tag_dock_node   the `tag_dock` action, servos the tag to centre and
                    reverses in
  - dock_manager_node  the two actions clients should call: dock / undock

The IR-beacon and lidar docking paths this replaces have been removed. Both
were superseded by the tag: measured on this robot the tag gives 100%
detection with 0.3px repeatability, where the IR zone flapped several times a
second and the lidar dock fit was unusable beyond ~45cm.

Include this from wherever navigation_launch.launch.py is included, since
dock_manager_node relays to the controller started here.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='navpromini_controller',
            executable='camera_node',
            name='navpromini_camera',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='navpromini_controller',
            executable='dock_tag_node',
            name='navpromini_dock_tag',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='navpromini_controller',
            executable='tag_dock_node',
            name='navpromini_tag_dock',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='navpromini_controller',
            executable='dock_manager_node',
            name='navpromini_dock_manager',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
