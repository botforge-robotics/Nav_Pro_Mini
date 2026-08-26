"""rosbridge + rosapi + launch_manager.

Relocated from nav2_mission_planner.launch.py, trimmed to what's actually still
needed: the Flutter app (and any other ROS-native client) talks to the robot via
rosbridge/rosapi directly and independently of navpromini_sdk — see
navpromini-sdk-is-optional-not-gateway in project memory — so those two stay.
launch_manager provides LaunchWithArgs/StopLaunch/GetMapList/DeleteMap, called by
both the app and navpromini_sdk by service name; which package hosts it doesn't
matter to either caller.

Dropped in the move, since nothing calls them: web_video_server + the image
republisher (camera view is out of scope for the app rebuild) and
tf2_buffer_server (nothing subscribes to it).
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[
            {'send_action_goals_in_new_thread': True},
            {'call_services_in_new_thread': True},
        ],
        output='screen'
    )

    rosapi_node = Node(
        package='rosapi',
        executable='rosapi_node',
        name='rosapi',
        output='screen'
    )

    launch_manager_node = Node(
        package='navpromini_launch_manager',
        executable='launch_manager',
        name='launch_manager',
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        rosbridge_node,
        rosapi_node,
        launch_manager_node,
    ])
