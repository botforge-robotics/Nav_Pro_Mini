#!/usr/bin/env python3
"""Nav2 bringup for NavProMini (localization + navigation + optional MoveIt 2 integration).

Pass only the map name (no path / extension). Full path is built from
the SLAM save directory: <ws>/src/navpromini_mapping/maps/<map_name>.yaml

When use_arm:=true:
  - Launches MoveIt 2 move_group node
  - Launches a unified RViz window with both Nav2 displays (map, costmaps, plans)
    and MoveIt MotionPlanning tools (arm interactive marker, trajectory preview, joints)
"""

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
from nav2_common.launch import RewrittenYaml

# Prefer NAVPRO_MAPS_DIR (production), then legacy workspace path.
MAPS_DIR = os.environ.get(
    'NAVPRO_MAPS_DIR',
    os.path.join(
        os.path.expanduser('~'),
        'NavProMini_ws',
        'src',
        'navpromini_mapping',
        'maps',
    ),
)

DEFAULT_INITIAL_POSES = {
    'cafe': {'x': '0.0', 'y': '-3.0', 'yaw': '0.0'},
    'office': {'x': '0.0', 'y': '-6.5', 'yaw': '0.0'},
    't202': {'x': '-0.13', 'y': '0.0', 'yaw': '0.0'},
}


def resolve_map_yaml(map_name: str) -> str:
    """Build absolute map yaml path from a bare map name."""
    name = map_name.strip()
    if name.endswith('.yaml'):
        name = name[:-5]
    if name.endswith('.pgm'):
        name = name[:-4]

    # Absolute path already provided
    if os.path.isabs(map_name) and map_name.endswith('.yaml'):
        return map_name

    candidates = [
        os.path.join(MAPS_DIR, f'{name}.yaml'),
        os.path.join(
            get_package_share_directory('navpromini_mapping'),
            'maps',
            f'{name}.yaml',
        ),
        os.path.join(
            get_package_share_directory('navpromini_navigation'),
            'maps',
            f'{name}.yaml',
        ),
    ]
    for path in candidates:
        if os.path.isfile(path):
            return path
    # Prefer src maps dir even if not yet present (clearer error from map_server)
    return candidates[0]


def _setup(context, *args, **kwargs):
    pkg = get_package_share_directory('navpromini_navigation')
    bringup_dir = get_package_share_directory('nav2_bringup')

    map_name = LaunchConfiguration('map_name').perform(context).strip()
    map_yaml = resolve_map_yaml(map_name)

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file').perform(context)
    use_rviz = LaunchConfiguration('use_rviz')
    autostart = LaunchConfiguration('autostart')
    use_arm = LaunchConfiguration('use_arm').perform(context).strip().lower() in ('true', '1')

    # Resolve initial pose if specified or auto-set from known maps (e.g. cafe)
    init_x = LaunchConfiguration('initial_pose_x').perform(context).strip()
    init_y = LaunchConfiguration('initial_pose_y').perform(context).strip()
    init_yaw = LaunchConfiguration('initial_pose_yaw').perform(context).strip()

    map_base = os.path.splitext(os.path.basename(map_yaml))[0].lower()
    if not init_x and map_base in DEFAULT_INITIAL_POSES:
        init_x = DEFAULT_INITIAL_POSES[map_base]['x']
        init_y = DEFAULT_INITIAL_POSES[map_base]['y']
        init_yaw = DEFAULT_INITIAL_POSES[map_base]['yaw']

    param_rewrites = {}
    if init_x:
        param_rewrites['amcl.ros__parameters.initial_pose.x'] = str(init_x)
        param_rewrites['amcl.ros__parameters.initial_pose.y'] = str(init_y)
        param_rewrites['amcl.ros__parameters.initial_pose.yaw'] = str(init_yaw)
        param_rewrites['amcl.ros__parameters.set_initial_pose'] = 'true'

    if param_rewrites:
        configured_params = RewrittenYaml(
            source_file=params_file,
            param_rewrites=param_rewrites,
            convert_types=True,
        )
    else:
        configured_params = params_file

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'slam': 'False',
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': configured_params,
            'autostart': autostart,
            'use_composition': 'False',
            'use_localization': 'True',
        }.items(),
    )

    actions = [
        LogInfo(msg=[
            f'NavProMini Nav2 — map_name={map_name} → {map_yaml} '
            f'(use_arm={use_arm}, sim_time={use_sim_time.perform(context)})'
        ]),
        bringup,
    ]

    # Auto-start scan filter in simulation to produce /scan_filtered from Gazebo /scan
    start_scan_filter = LaunchConfiguration('start_scan_filter').perform(context).strip().lower() in ('true', '1')
    if start_scan_filter:
        scan_filter = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('navpromini_controller'),
                    'launch',
                    'laser_scan_filter.launch.py',
                )
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        )
        actions.append(scan_filter)

    # Setup MoveIt 2 integration if use_arm is true
    if use_arm:
        from moveit_configs_utils import MoveItConfigsBuilder

        moveit_config = (
            MoveItConfigsBuilder('NavProMini', package_name='navpromini_moveit_config')
            .robot_description(file_path='config/NavProMini.urdf.xacro')
            .trajectory_execution(file_path='config/moveit_controllers.yaml')
            .planning_pipelines(pipelines=['ompl'])
            .to_moveit_configs()
        )

        move_group_node = Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[
                moveit_config.to_dict(),
                {'use_sim_time': use_sim_time},
            ],
        )
        actions.append(move_group_node)

        rviz_config_file = os.path.join(pkg, 'rviz', 'navigation_moveit.rviz')
        rviz_parameters = [
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': use_sim_time},
        ]
    else:
        rviz_config_file = os.path.join(pkg, 'rviz', 'navigation.rviz')
        rviz_parameters = [{'use_sim_time': use_sim_time}]

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=rviz_parameters,
        condition=IfCondition(use_rviz),
        output='screen',
    )
    actions.append(rviz_node)

    return actions


def generate_launch_description():
    pkg = get_package_share_directory('navpromini_navigation')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true',
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup Nav2 stack',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg, 'config', 'nav2_params.yaml'),
            description='Full path to the ROS2 parameters file to use for all launched nodes',
        ),
        DeclareLaunchArgument(
            'map_name',
            default_value='navpromini_map',
            description=(
                'Map name only (no path). Resolved to '
                f'{MAPS_DIR}/<map_name>.yaml'
            ),
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch Nav2 RViz (default: true). Use use_rviz:=false to disable.',
        ),
        DeclareLaunchArgument(
            'use_arm',
            default_value='false',
            description='Enable MoveIt 2 integration (launches move_group and loads unified Nav2+MoveIt RViz)',
        ),
        DeclareLaunchArgument(
            'start_scan_filter',
            default_value=LaunchConfiguration('use_sim_time'),
            description='Launch laser_scan_filter to produce /scan_filtered (default: matches use_sim_time)',
        ),
        DeclareLaunchArgument(
            'initial_pose_x',
            default_value='',
            description='Initial robot X position on the map (defaults to map-specific spawn, e.g. 0.0 for cafe)',
        ),
        DeclareLaunchArgument(
            'initial_pose_y',
            default_value='',
            description='Initial robot Y position on the map (defaults to map-specific spawn, e.g. -3.0 for cafe)',
        ),
        DeclareLaunchArgument(
            'initial_pose_yaw',
            default_value='',
            description='Initial robot heading yaw on the map (radians)',
        ),
        OpaqueFunction(function=_setup),
    ])
