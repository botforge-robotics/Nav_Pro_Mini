#!/usr/bin/env python3
"""Real-robot hardware bringup for NavProMini on Raspberry Pi 5.

Starts:
  - robot_state_publisher (URDF → tf_static + wheel TF from joint_states)
  - micro-ROS agent (Pi GPIO UART ↔ ESP32)
  - RPLidar A1M8 (/scan, frame lidar_1)
  - wheel odom node (/odom + TF odom→base_link)
  - Daly Smart BMS battery node (/battery/* via FTDI USB–RS485)

Optional:
  - slam (navpromini_mapping)
  - navigation (navpromini_navigation)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('navpromini_controller')
    pkg_desc = get_package_share_directory('navpromini_description')

    xacro_file = os.path.join(pkg_desc, 'urdf', 'NavProMini.xacro')
    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' lidar_yaw:=', LaunchConfiguration('lidar_yaw'),
        ]),
        value_type=str,
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    start_agent = LaunchConfiguration('start_agent')
    start_lidar = LaunchConfiguration('start_lidar')
    start_odom = LaunchConfiguration('start_odom')
    start_battery = LaunchConfiguration('start_battery')
    start_slam = LaunchConfiguration('start_slam')
    start_nav = LaunchConfiguration('start_nav')

    microros = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'microros_agent.launch.py')
        ),
        launch_arguments={
            'serial_port': LaunchConfiguration('microros_port'),
            'baudrate': LaunchConfiguration('microros_baud'),
        }.items(),
        condition=IfCondition(start_agent),
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'rplidar.launch.py')
        ),
        launch_arguments={
            'serial_port': LaunchConfiguration('lidar_port'),
            'baudrate': LaunchConfiguration('lidar_baud'),
            'frame_id': LaunchConfiguration('lidar_frame'),
        }.items(),
        condition=IfCondition(start_lidar),
    )

    odom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'odom.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(start_odom),
    )

    battery = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'battery.launch.py')
        ),
        launch_arguments={
            'serial_port': LaunchConfiguration('battery_port'),
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(start_battery),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('navpromini_mapping'),
                'launch',
                'slam.launch.py',
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_rviz': LaunchConfiguration('use_rviz'),
        }.items(),
        condition=IfCondition(start_slam),
    )

    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('navpromini_navigation'),
                'launch',
                'navigation.launch.py',
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map_name': LaunchConfiguration('map_name'),
            'use_rviz': LaunchConfiguration('use_rviz'),
        }.items(),
        condition=IfCondition(start_nav),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'microros_port',
            default_value='/dev/ttyAMA0',
            description='Pi 5 GPIO UART to ESP32 (Waveshare). Pi 4: /dev/serial0',
        ),
        DeclareLaunchArgument('microros_baud', default_value='115200'),
        DeclareLaunchArgument(
            'lidar_port',
            default_value='/dev/rplidar',
            description='RPLidar CP2102 stable udev device',
        ),
        DeclareLaunchArgument('lidar_baud', default_value='115200'),
        DeclareLaunchArgument('lidar_frame', default_value='lidar_1'),
        DeclareLaunchArgument(
            'lidar_yaw',
            default_value='0.0',
            description='lidar_1 yaw vs base_link (rad). '
                        'If scan is 90° off in RViz, try 1.5708 or -1.5708',
        ),
        DeclareLaunchArgument('start_agent', default_value='true'),
        DeclareLaunchArgument('start_lidar', default_value='true'),
        DeclareLaunchArgument('start_odom', default_value='true'),
        DeclareLaunchArgument(
            'start_battery',
            default_value='true',
            description='Daly Smart BMS over FTDI USB–RS485',
        ),
        DeclareLaunchArgument(
            'battery_port',
            default_value='/dev/battery_bms',
            description='FTDI USB–RS485 to Daly BMS (udev symlink)',
        ),
        DeclareLaunchArgument(
            'start_slam',
            default_value='false',
            description='Also launch slam_toolbox',
        ),
        DeclareLaunchArgument(
            'start_nav',
            default_value='false',
            description='Also launch Nav2 (needs map_name)',
        ),
        DeclareLaunchArgument('map_name', default_value='navpromini_map'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        LogInfo(msg=[
            'NavProMini real robot: use_sim_time=false | '
            'agent+lidar+odom+battery | ROS_DOMAIN_ID inherited from shell'
        ]),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
                # ESP32 micro-ROS stamps /joint_states with its own clock (~uptime).
                # Without this, chassis→leftWheel_1/rightWheel_1 TF is in ESP time while
                # odom→base_link is host time, so RViz Fixed Frame=odom warns on wheels.
                'ignore_timestamp': True,
            }],
        ),
        microros,
        lidar,
        odom,
        battery,
        slam,
        nav,
    ])
