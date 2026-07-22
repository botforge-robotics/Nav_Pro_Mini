#!/usr/bin/env python3
"""Top-level RMF multi-robot simulation (Gazebo cafe world + Nav2 + RViz)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from navpromini_rmf_sim.launch_utils import parse_robot_list


def _setup(context, *args, **kwargs):
    pkg = get_package_share_directory('navpromini_rmf_sim')
    robots = parse_robot_list(LaunchConfiguration('robot_names').perform(context))
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz').perform(context)
    start_building_map = LaunchConfiguration('start_building_map').perform(context)

    building = os.path.join(pkg, 'site', 'site.building.yaml')
    actions = [
        LogInfo(msg=[
            f'NavProMini RMF sim robots={robots} '
            f'world={LaunchConfiguration("world_name").perform(context)}'
        ]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'multi_robot_gazebo.launch.py')
            ),
            launch_arguments={
                'world_name': LaunchConfiguration('world_name'),
                'use_sim_time': use_sim_time,
                'robot_names': LaunchConfiguration('robot_names'),
                'spawn_poses': LaunchConfiguration('spawn_poses'),
            }.items(),
            condition=IfCondition(LaunchConfiguration('start_gazebo')),
        ),
        # Nav2 starts immediately; lifecycle managers + costmap
        # initial_transform_timeout wait for Gazebo odom TF (no TimerAction).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'multi_robot_nav.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'robot_names': LaunchConfiguration('robot_names'),
                'map_yaml': LaunchConfiguration('map_yaml'),
            }.items(),
            condition=IfCondition(LaunchConfiguration('start_nav')),
        ),
    ]

    if start_building_map.lower() in ('true', '1', 'yes'):
        use_sim = LaunchConfiguration('use_sim_time').perform(context) == 'true'
        actions.append(
            Node(
                package='rmf_building_map_tools',
                executable='building_map_server',
                name='building_map_server',
                arguments=[building],
                parameters=[{'use_sim_time': use_sim}],
                output='screen',
            )
        )

    # Doors: Gazebo libdoor owns /door_states (see launch/include/adapters/door_adapter).
    start_workcells = LaunchConfiguration('start_workcells').perform(context)
    cafe_alias = LaunchConfiguration('start_cafe_infra').perform(context)
    if cafe_alias.lower() in ('true', '1', 'yes'):
        start_workcells = 'true'
    elif cafe_alias.lower() in ('false', '0', 'no'):
        start_workcells = 'false'
    if start_workcells.lower() in ('true', '1', 'yes'):
        nav_graph = LaunchConfiguration('nav_graph').perform(context)
        if not nav_graph:
            nav_graph = os.path.join(pkg, 'site', 'nav_graphs', '0.yaml')
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        pkg, 'launch', 'include', 'adapters',
                        'workcell_adapter.launch.py',
                    )
                ),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'nav_graph': nav_graph,
                }.items(),
            )
        )

    if use_rviz.lower() in ('true', '1', 'yes'):
        rviz_cfg = os.path.join(pkg, 'config', 'rmf_sim.rviz')
        if not os.path.isfile(rviz_cfg):
            # Fall back to navpromini_navigation rviz if present
            try:
                nav_pkg = get_package_share_directory('navpromini_navigation')
                rviz_cfg = os.path.join(nav_pkg, 'rviz', 'navigation.rviz')
            except Exception:
                rviz_cfg = ''
        actions.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_cfg] if rviz_cfg else [],
                parameters=[{'use_sim_time': True}],
                output='screen',
            )
        )

    return actions


def generate_launch_description():
    pkg = get_package_share_directory('navpromini_rmf_sim')

    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value='cafe'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('robot_names', default_value='robot1,robot2'),
        DeclareLaunchArgument(
            'map_yaml',
            default_value=os.path.join(pkg, 'site', 'maps', 'L1', 'map.yaml'),
        ),
        DeclareLaunchArgument(
            'spawn_poses',
            default_value=os.path.join(pkg, 'site', 'spawn_poses.yaml'),
        ),
        DeclareLaunchArgument('start_gazebo', default_value='true'),
        DeclareLaunchArgument(
            'start_nav',
            default_value='false',
            description='Nav2 stack (off for fleet-server GUI workflow)',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='RViz (off for fleet-server GUI workflow)',
        ),
        DeclareLaunchArgument(
            'start_building_map',
            default_value='true',
            description='Start RMF building_map_server for fleet GUI /map',
        ),
        DeclareLaunchArgument(
            'nav_graph',
            default_value=os.path.join(pkg, 'site', 'nav_graphs', '0.yaml'),
            description='RMF nav graph used to discover workcell GUIDs',
        ),
        DeclareLaunchArgument(
            'start_workcells',
            default_value='false',
            description=(
                'Start dispenser/ingestor adapter. Keep false when using the '
                'two-terminal flow (rmf_web.launch.py starts workcells). '
                'Doors come from Gazebo libdoor + door_supervisor only.'
            ),
        ),
        # Back-compat alias
        DeclareLaunchArgument('start_cafe_infra', default_value=''),
        OpaqueFunction(function=_setup),
    ])
