#!/usr/bin/env python3
"""Nav2 bringup for multiple namespaced NavProMini robots on one shared map."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

from navpromini_rmf_sim.launch_utils import (
    default_spawn_pairs,
    parse_robot_list,
    write_namespaced_nav2_params,
)


def _setup(context, *args, **kwargs):
    pkg_nav = get_package_share_directory('navpromini_navigation')
    pkg_sim = get_package_share_directory('navpromini_rmf_sim')

    map_yaml = LaunchConfiguration('map_yaml').perform(context)
    if not os.path.isabs(map_yaml):
        map_yaml = os.path.join(pkg_sim, map_yaml)
    if not os.path.isfile(map_yaml):
        raise RuntimeError(f'Map yaml not found: {map_yaml}')

    params_file = LaunchConfiguration('params_file').perform(context)
    if not os.path.isabs(params_file):
        params_file = os.path.join(pkg_nav, params_file)

    spawn_poses = LaunchConfiguration('spawn_poses').perform(context)
    if not os.path.isabs(spawn_poses):
        spawn_poses = os.path.join(pkg_sim, spawn_poses)

    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    use_sim = use_sim_time.lower() in ('true', '1', 'yes')
    autostart = LaunchConfiguration('autostart').perform(context)
    robots = parse_robot_list(LaunchConfiguration('robot_names').perform(context))
    spawn_by_name = {
        name: (x, y, yaw)
        for name, x, y, _z, yaw in default_spawn_pairs(robots, spawn_poses)
    }

    # RMF building_map_server also publishes on /map (BuildingMap). Nav2 needs
    # a dedicated OccupancyGrid topic or costmaps never receive a map.
    nav2_map_topic = '/nav2_map'

    actions = [
        LogInfo(msg=[
            f'Multi-robot Nav2 map={map_yaml} robots={robots} '
            f'spawn_poses={spawn_poses} occupancy={nav2_map_topic}'
        ]),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {
                    'use_sim_time': use_sim,
                    'yaml_filename': map_yaml,
                    'topic_name': nav2_map_topic,
                }
            ],
            remappings=[('map', nav2_map_topic)],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[
                {
                    'use_sim_time': use_sim,
                    'autostart': autostart.lower() in ('true', '1', 'yes'),
                    'bond_timeout': 30.0,
                    'attempt_respawn_reconnection': True,
                    'node_names': ['map_server'],
                }
            ],
        ),
    ]

    nav_nodes = [
        'amcl',
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
        # collision_monitor omitted in sim: Gazebo gpu_lidar often has no
        # publisher on /robotN/scan, and an empty observation_sources list
        # crashes the node. Velocity smoother publishes directly to cmd_vel.
    ]

    # Point namespaced Nav2 nodes at the occupancy grid (not RMF /map).
    # Use global /tf + /tf_static so map→odom from sim_map_odom_tf is seen.
    map_remaps = [
        ('map', nav2_map_topic),
        ('/map', nav2_map_topic),
        ('tf', '/tf'),
        ('tf_static', '/tf_static'),
    ]

    # Lifecycle manager waits (via costmap initial_transform_timeout) for
    # Gazebo odom TF; bond must outlive that wait.
    lifecycle_params = {
        'use_sim_time': use_sim,
        'autostart': autostart.lower() in ('true', '1', 'yes'),
        'bond_timeout': 90.0,
        'bond_respawn_max_duration': 30.0,
        'attempt_respawn_reconnection': True,
        'node_names': nav_nodes,
    }

    # Sim: identity map→robotN/odom on /tf_static (AMCL does not broadcast).
    # Must be static — a dynamic /tf publisher races Gazebo/ff_tf stamps and
    # triggers TF_OLD_DATA (e.g. frame robotN/odom at time 14.25).
    if use_sim:
        # Un-namespaced map→odom for free_fleet / tools.
        actions.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='map_odom_static',
                output='screen',
                arguments=[
                    '--x', '0', '--y', '0', '--z', '0',
                    '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                    '--frame-id', 'map',
                    '--child-frame-id', 'odom',
                ],
                parameters=[{'use_sim_time': True}],
            )
        )
        for namespace in robots:
            actions.append(
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name=f'{namespace}_map_odom_static',
                    output='screen',
                    arguments=[
                        '--x', '0', '--y', '0', '--z', '0',
                        '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                        '--frame-id', 'map',
                        '--child-frame-id', f'{namespace}/odom',
                    ],
                    parameters=[{'use_sim_time': True}],
                )
            )

    for namespace in robots:
        init_xy_yaw = spawn_by_name.get(namespace)
        params = write_namespaced_nav2_params(
            params_file, namespace, use_sim, initial_pose=init_xy_yaw
        )
        scan_topic = f'/{namespace}/scan'
        actions.append(
            GroupAction([
                PushRosNamespace(namespace),
                Node(
                    package='nav2_amcl',
                    executable='amcl',
                    name='amcl',
                    output='screen',
                    parameters=[params],
                    remappings=[('scan', scan_topic), *map_remaps],
                ),
                Node(
                    package='nav2_controller',
                    executable='controller_server',
                    name='controller_server',
                    output='screen',
                    parameters=[params],
                    remappings=[
                        ('cmd_vel', 'cmd_vel_nav'),
                        ('scan', scan_topic),
                        ('odom', 'odom'),
                        *map_remaps,
                    ],
                ),
                Node(
                    package='nav2_smoother',
                    executable='smoother_server',
                    name='smoother_server',
                    output='screen',
                    parameters=[params],
                ),
                Node(
                    package='nav2_planner',
                    executable='planner_server',
                    name='planner_server',
                    output='screen',
                    parameters=[params],
                    remappings=[('scan', scan_topic), *map_remaps],
                ),
                Node(
                    package='nav2_behaviors',
                    executable='behavior_server',
                    name='behavior_server',
                    output='screen',
                    parameters=[params],
                    remappings=[('scan', scan_topic), *map_remaps],
                ),
                Node(
                    package='nav2_bt_navigator',
                    executable='bt_navigator',
                    name='bt_navigator',
                    output='screen',
                    parameters=[params],
                    remappings=map_remaps,
                ),
                Node(
                    package='nav2_waypoint_follower',
                    executable='waypoint_follower',
                    name='waypoint_follower',
                    output='screen',
                    parameters=[params],
                ),
                Node(
                    package='nav2_velocity_smoother',
                    executable='velocity_smoother',
                    name='velocity_smoother',
                    output='screen',
                    parameters=[params],
                    remappings=[
                        ('cmd_vel', 'cmd_vel_nav'),
                        # No collision_monitor in sim — drive Gazebo directly.
                        ('cmd_vel_smoothed', 'cmd_vel'),
                    ],
                ),
                # One manager per robot: activate in order (AMCL → controller…).
                # Costmaps block on odom TF via initial_transform_timeout.
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_navigation',
                    output='screen',
                    parameters=[lifecycle_params],
                ),
            ])
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument(
            'map_yaml',
            default_value='site/maps/L1/map.yaml',
            description='Nav2 map yaml (package-relative or absolute)',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value='config/nav2_params.yaml',
            description='Nav2 params relative to navpromini_navigation share',
        ),
        DeclareLaunchArgument(
            'robot_names',
            default_value='robot1,robot2',
        ),
        DeclareLaunchArgument(
            'spawn_poses',
            default_value='site/spawn_poses.yaml',
            description='Spawn/AMCL initial poses (package-relative or absolute)',
        ),
        OpaqueFunction(function=_setup),
    ])
