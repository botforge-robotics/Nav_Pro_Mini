#!/usr/bin/env python3
"""Spawn multiple NavProMini robots in Gazebo (cafe world from site.building.yaml)."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue

from navpromini_rmf_sim.launch_utils import default_spawn_pairs, parse_robot_list


def _resolve_world(pkg_sim: str, pkg_gazebo: str, world_name: str) -> str:
    """Prefer site/generated/<name>.world, then navpromini_gazebo worlds."""
    candidates = [
        os.path.join(pkg_sim, 'site', 'generated', f'{world_name}.world'),
        os.path.join(pkg_sim, 'site', 'generated', f'{world_name}.sdf'),
        os.path.join(pkg_gazebo, 'worlds', f'{world_name}.sdf'),
        os.path.join(pkg_gazebo, 'worlds', f'{world_name}.world'),
    ]
    if os.path.isabs(world_name) and os.path.isfile(world_name):
        return world_name
    for path in candidates:
        if os.path.isfile(path):
            return path
    raise RuntimeError(
        f'World not found for world_name={world_name}. Tried:\n  '
        + '\n  '.join(candidates)
        + '\nRun: ros2 run navpromini_rmf_sim generate_site_assets'
    )


def _setup(context, *args, **kwargs):
    pkg_gazebo = get_package_share_directory('navpromini_gazebo')
    pkg_desc = get_package_share_directory('navpromini_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_sim = get_package_share_directory('navpromini_rmf_sim')

    world_name = LaunchConfiguration('world_name').perform(context).strip()
    world_path = _resolve_world(pkg_sim, pkg_gazebo, world_name)
    use_sim_time = LaunchConfiguration('use_sim_time')
    robots = parse_robot_list(LaunchConfiguration('robot_names').perform(context))
    spawn_delay = float(LaunchConfiguration('spawn_delay').perform(context))

    spawn_yaml = LaunchConfiguration('spawn_poses').perform(context)
    if not spawn_yaml:
        spawn_yaml = os.path.join(pkg_sim, 'site', 'spawn_poses.yaml')
    spawn_pairs = default_spawn_pairs(robots, spawn_yaml)

    xacro_file = os.path.join(pkg_desc, 'urdf', 'NavProMini.xacro')
    # Harmonic resolves model:// and package:// from these roots.
    resource_paths = [
        str(Path(pkg_desc).parent.resolve()),
        str(Path(pkg_gazebo).parent.resolve()),
        pkg_desc,
        os.path.join(pkg_sim, 'site', 'generated', 'models'),
        os.path.join(pkg_sim, 'site', 'generated'),
    ]

    # Book lift/door nodes: liblift.so / libdoor.so from rmf_building_sim_gz_plugins
    gz_plugin_dirs = []
    for pkg_name in (
        'rmf_building_sim_gz_plugins',
        'rmf_robot_sim_gz_plugins',
    ):
        try:
            gz_plugin_dirs.append(
                os.path.join(get_package_prefix(pkg_name), 'lib', pkg_name)
            )
        except Exception:
            pass
    existing_plugin_path = os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
    plugin_path = os.pathsep.join(
        [p for p in gz_plugin_dirs if p] +
        ([existing_plugin_path] if existing_plugin_path else [])
    )

    actions = [
        LogInfo(msg=[
            f'Multi-robot Gazebo world={world_path} robots={robots} '
            f'spawns={[(n, x, y, z) for n, x, y, z, _yaw in spawn_pairs]} '
            f'spawn_delay={spawn_delay}s'
        ]),
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.pathsep.join(resource_paths),
        ),
        SetEnvironmentVariable(
            name='GZ_SIM_SYSTEM_PLUGIN_PATH',
            value=plugin_path,
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': f'-r -v 1 {world_path}'}.items(),
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='clock_bridge',
            parameters=[
                {
                    'config_file': os.path.join(
                        pkg_sim, 'config', 'clock_bridge.yaml'
                    ),
                    'use_sim_time': use_sim_time,
                }
            ],
            output='screen',
        ),
    ]

    spawn_actions = []
    for name, x, y, z, yaw in spawn_pairs:
        model_name = f'NavProMini_{name}'
        robot_description = ParameterValue(
            Command([
                'xacro ', xacro_file,
                ' robot_namespace:=', name,
                ' frame_prefix:=', name, '/',
                ' topic_prefix:=/', name,
            ]),
            value_type=str,
        )
        # Absolute topic so ros_gz create does not race namespace remaps.
        desc_topic = f'/{name}/robot_description'
        spawn_actions.append(
            GroupAction([
                PushRosNamespace(name),
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    output='screen',
                    parameters=[
                        {
                            'use_sim_time': use_sim_time,
                            'robot_description': robot_description,
                            'frame_prefix': f'{name}/',
                        }
                    ],
                ),
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    output='screen',
                    arguments=[
                        '-world', 'sim_world',
                        '-name', model_name,
                        '-topic', desc_topic,
                        '-x', str(x),
                        '-y', str(y),
                        '-z', str(z),
                        '-Y', str(yaw),
                        '-allow_renaming', 'true',
                    ],
                ),
            ])
        )
        bridge_yaml = os.path.join(pkg_sim, 'config', 'bridges', f'{name}.yaml')
        if not os.path.isfile(bridge_yaml):
            raise RuntimeError(
                f'Missing bridge config {bridge_yaml}. '
                f'Add config/bridges/{name}.yaml or reduce robot_names.'
            )
        spawn_actions.append(
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name=f'{name}_bridge',
                parameters=[
                    {
                        'config_file': bridge_yaml,
                        'use_sim_time': use_sim_time,
                    }
                ],
                output='screen',
            )
        )

    # Wait for Gazebo server + world models before spawning robots.
    actions.append(TimerAction(period=spawn_delay, actions=spawn_actions))
    return actions


def generate_launch_description():
    pkg_sim = get_package_share_directory('navpromini_rmf_sim')
    return LaunchDescription([
        DeclareLaunchArgument(
            'world_name',
            default_value='cafe',
            description='empty|office|cafe — cafe uses site/generated/cafe.world',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'robot_names',
            default_value='robot1,robot2',
            description='Comma-separated robot namespaces (must match spawn_robot_name)',
        ),
        DeclareLaunchArgument(
            'spawn_poses',
            default_value=os.path.join(pkg_sim, 'site', 'spawn_poses.yaml'),
            description='YAML with per-robot x/y/z/yaw from generate_site_assets',
        ),
        DeclareLaunchArgument(
            'spawn_delay',
            default_value='4.0',
            description='Seconds to wait for Gazebo world before spawning robots',
        ),
        OpaqueFunction(function=_setup),
    ])
