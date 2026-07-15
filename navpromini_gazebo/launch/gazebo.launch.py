#!/usr/bin/env python3
"""Launch NavProMini in Gazebo Harmonic with selectable worlds."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

AVAILABLE_WORLDS = ('empty', 'office', 'cafe')

# Cafe Fuel model floor top ≈ 0.0948 + 0.19/2 = 0.19 m
WORLD_SPAWNS = {
    'empty':  {'x': '0.0', 'y': '0.0',  'z': '0.06'},
    'office': {'x': '0.0', 'y': '-6.5', 'z': '0.06'},   # lobby
    'cafe':   {'x': '0.0', 'y': '-3.0', 'z': '0.28'},   # on cafe floor
}


def _setup(context, *args, **kwargs):
    pkg_gazebo = get_package_share_directory('navpromini_gazebo')
    pkg_desc = get_package_share_directory('navpromini_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_name = LaunchConfiguration('world_name').perform(context).strip().lower()
    if world_name not in AVAILABLE_WORLDS:
        raise RuntimeError(
            f"Unknown world_name '{world_name}'. Choose one of: {', '.join(AVAILABLE_WORLDS)}"
        )

    world_path = os.path.join(pkg_gazebo, 'worlds', f'{world_name}.sdf')
    if not os.path.isfile(world_path):
        raise RuntimeError(f'World file not found: {world_path}')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    z = LaunchConfiguration('z').perform(context)
    yaw = LaunchConfiguration('yaw').perform(context)

    # Dynamic spawn: apply world defaults when user left launch defaults
    defaults_used = (x == '0.0' and y == '0.0' and z == '0.06')
    if defaults_used:
        spawn = WORLD_SPAWNS[world_name]
        x, y, z = spawn['x'], spawn['y'], spawn['z']
    elif world_name == 'cafe' and abs(float(z) - 0.06) < 1e-6:
        # Even if x/y overridden, keep robot above cafe floor
        z = WORLD_SPAWNS['cafe']['z']

    xacro_file = os.path.join(pkg_desc, 'urdf', 'NavProMini.xacro')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    resource_paths = [
        str(Path(pkg_desc).parent.resolve()),
        str(Path(pkg_gazebo).parent.resolve()),
    ]

    return [
        LogInfo(msg=[
            f'Launching world={world_name} spawn=({x}, {y}, {z}) '
            f'rviz={LaunchConfiguration("use_rviz").perform(context)} '
            f'file={world_path}'
        ]),
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.pathsep.join(resource_paths),
        ),
        SetEnvironmentVariable(
            name='GZ_SIM_SYSTEM_PLUGIN_PATH',
            value=os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': f'-r -v 1 {world_path}',
            }.items(),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'use_sim_time': use_sim_time,
                    'robot_description': robot_description,
                }
            ],
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=[
                '-name', 'NavProMini',
                '-topic', 'robot_description',
                '-x', x,
                '-y', y,
                '-z', z,
                '-Y', yaw,
            ],
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[
                {
                    'config_file': os.path.join(
                        pkg_gazebo, 'config', 'ros_gz_bridge.yaml'
                    ),
                    'use_sim_time': use_sim_time,
                }
            ],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_gazebo, 'config', 'navpromini_sim.rviz')],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_rviz),
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world_name',
            default_value='empty',
            description=f'World to load: {", ".join(AVAILABLE_WORLDS)}',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Launch Gazebo companion RViz (default: false). '
                        'Use use_rviz:=true to enable.',
        ),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.06'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        OpaqueFunction(function=_setup),
    ])
