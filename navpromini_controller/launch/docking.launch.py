#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

DOCK_WORLD_COORDS = {
    'empty':      {'x': '-0.7',   'y': '0.0',  'yaw': '0.0'},
    'office':     {'x': '-0.7',   'y': '-6.5', 'yaw': '0.0'},
    'cafe':       {'x': '-4.85',  'y': '-1.0', 'yaw': '0.0'},
    'playground': {'x': '-10.75', 'y': '-5.0', 'yaw': '0.0'},
}


def _setup(context, *args, **kwargs):
    world_name = LaunchConfiguration('world_name').perform(context).strip().lower()
    dock_x = LaunchConfiguration('dock_x').perform(context).strip()
    dock_y = LaunchConfiguration('dock_y').perform(context).strip()
    dock_yaw = LaunchConfiguration('dock_yaw').perform(context).strip()
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_camera = LaunchConfiguration('start_camera')
    cmd_vel_dock_topic = LaunchConfiguration('cmd_vel_dock_topic')
    require_charge_sense = LaunchConfiguration('require_charge_sense')
    start_docked = LaunchConfiguration('start_docked')

    if not dock_x and world_name in DOCK_WORLD_COORDS:
        coords = DOCK_WORLD_COORDS[world_name]
        dock_x = coords['x']
        if not dock_y:
            dock_y = coords['y']
        if not dock_yaw:
            dock_yaw = coords['yaw']

    return [
        Node(
            package='navpromini_controller',
            executable='camera_node',
            name='navpromini_camera',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(start_camera),
        ),
        Node(
            package='navpromini_controller',
            executable='dock_marker_node',
            name='navpromini_dock_marker',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            respawn=True,
        ),
        Node(
            package='navpromini_controller',
            executable='dock_manager_node',
            name='navpromini_dock_manager',
            output='screen',
            respawn=True,
            parameters=[{
                'use_sim_time': use_sim_time,
                'require_charge_sense': require_charge_sense,
                'start_docked': start_docked,
                'dock_x': dock_x,
                'dock_y': dock_y,
                'dock_yaw': dock_yaw,
            }],
            remappings=[('cmd_vel_dock', cmd_vel_dock_topic)],
        ),
    ]


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'world_name',
            default_value='',
            description='Simulation world name (empty, office, cafe, playground) for auto dock coords',
        ),
        DeclareLaunchArgument('dock_x', default_value='', description='Dock map X coordinate (empty for saved db/world default)'),
        DeclareLaunchArgument('dock_y', default_value='', description='Dock map Y coordinate (empty for saved db/world default)'),
        DeclareLaunchArgument('dock_yaw', default_value='', description='Dock map heading yaw (empty for saved db/world default)'),
        DeclareLaunchArgument(
            'start_camera',
            default_value=PythonExpression(["'false' if '", use_sim_time, "' == 'true' else 'true'"]),
            description='Start physical USB camera driver node (auto-disabled when use_sim_time is true)',
        ),
        DeclareLaunchArgument(
            'cmd_vel_dock_topic',
            default_value=PythonExpression(["'cmd_vel' if '", use_sim_time, "' == 'true' else 'cmd_vel_dock'"]),
            description='Dock velocity output topic (cmd_vel for sim, cmd_vel_dock for twist_mux on real robot)',
        ),
        DeclareLaunchArgument(
            'require_charge_sense',
            default_value=PythonExpression(["'false' if '", use_sim_time, "' == 'true' else 'true'"]),
            description='Require BMS charging contact to confirm dock (false in sim)',
        ),
        DeclareLaunchArgument(
            'start_docked',
            default_value=PythonExpression(["'true' if '", use_sim_time, "' == 'true' else 'false'"]),
            description='Initialize in docked state (defaults to true in simulation)',
        ),
        OpaqueFunction(function=_setup),
    ])
