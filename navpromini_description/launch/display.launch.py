import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share_dir = get_package_share_directory('navpromini_description')
    xacro_file = os.path.join(share_dir, 'urdf', 'navpromini.xacro')
    rviz_config_file = os.path.join(share_dir, 'config', 'display.rviz')

    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='True',
        description='Flag to enable joint_state_publisher_gui',
    )
    sim_arg = DeclareLaunchArgument(
        name='sim',
        default_value='false',
        description='If true, connect GUI sliders to Gazebo simulation (remapped to /joint_states_gui)',
    )
    use_arm_arg = DeclareLaunchArgument(
        name='use_arm',
        default_value='false',
        description='Spawn robot with modular arm attachment (true/false)',
    )
    arm_z_arg = DeclareLaunchArgument(
        name='arm_z',
        default_value='0.0198',
        description='Z offset of modular arm attachment above lidar in meters',
    )

    show_gui = LaunchConfiguration('gui')
    sim = LaunchConfiguration('sim')
    use_arm = LaunchConfiguration('use_arm')
    arm_z = LaunchConfiguration('arm_z')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' use_arm:=', use_arm, ' arm_z:=', arm_z]),
        value_type=str,
    )

    jsp_params = [
        {
            'robot_description': robot_description,
            'zeros.shoulder_lift': -1.57079632679,
            'zeros.elbow_flex': 1.57079632679,
        }
    ]

    # Offline mode: robot_state_publisher publishes /tf from offline joint_states
    robot_state_publisher_node = Node(
        condition=UnlessCondition(sim),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description}
        ],
    )

    joint_state_publisher_node = Node(
        condition=UnlessCondition(show_gui),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=jsp_params,
    )

    # Offline GUI: publishes to /joint_states
    joint_state_publisher_gui_offline = Node(
        condition=IfCondition(PythonExpression(["'", show_gui, "' == 'True' and '", sim, "' == 'false'"])),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=jsp_params,
    )

    # Sim GUI: publishes to /joint_states_gui so arm_teleop forwards to Gazebo in real-time
    joint_state_publisher_gui_sim = Node(
        condition=IfCondition(PythonExpression(["'", show_gui, "' == 'True' and '", sim, "' == 'true'"])),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=jsp_params,
        remappings=[('/joint_states', '/joint_states_gui')],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': sim}],
        output='screen',
    )

    return LaunchDescription([
        gui_arg,
        sim_arg,
        use_arm_arg,
        arm_z_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_offline,
        joint_state_publisher_gui_sim,
        rviz_node,
    ])
