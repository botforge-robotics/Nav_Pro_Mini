#!/usr/bin/env python3
"""Open-RMF web stack for NavProMini site maps (no Gazebo).

Starts:
  - building_map_server from this package's site.building.yaml
  - minimal RMF core (schedule / blockade / dispatcher / supervisors)
  - schedule websocket (default :8006) for dashboard trajectories
  - open-source rmf-web API server + dashboard (Docker by default)

Usage:
  source ~/rmf_ws/install/setup.bash
  source ~/NavProMini_ws/install/setup.bash
  ros2 launch navpromini_rmf_sim rmf_web.launch.py

If navpro-fleet-server containers are already on host network (ports 8000/8006),
either stop them or use a free domain/ports, e.g.:
  ROS_DOMAIN_ID=42 ros2 launch navpromini_rmf_sim rmf_web.launch.py \\
    websocket_port:=8016 api_url:=http://localhost:8010 \\
    server_uri:=http://localhost:8010/_internal

Dashboard (docker): http://localhost:3000  (admin / admin)
Dashboard (local):  http://localhost:5173  (admin / admin)
API docs:           http://localhost:8000/docs
"""

from __future__ import annotations

import os
import socket
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import (
    FrontendLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _truthy(value: str) -> bool:
    return value.lower() in ('true', '1', 'yes')


def _expand_path(raw: str) -> str:
    return os.path.expanduser(os.path.expandvars(raw.strip()))


def _port_open(port: int, host: str = '127.0.0.1') -> bool:
    try:
        with socket.create_connection((host, port), timeout=0.3):
            return True
    except OSError:
        return False


def _find_pnpm() -> str | None:
    candidates = [
        os.environ.get('PNPM_BIN', ''),
        'pnpm',
        os.path.expanduser('~/.local/share/pnpm/bin/pnpm'),
        os.path.expanduser('~/.local/bin/pnpm'),
    ]
    for raw in candidates:
        if not raw:
            continue
        path = Path(raw)
        if path.is_file() and os.access(path, os.X_OK):
            return str(path)
        # bare name — hope PATH works at ExecuteProcess time
        if raw == 'pnpm':
            return 'pnpm'
    return None


def _find_node_bin_dirs() -> list[str]:
    dirs: list[str] = []
    env_dir = os.environ.get('NODE_BIN_DIR', '').strip()
    if env_dir and Path(env_dir, 'node').is_file():
        dirs.append(env_dir)

    nvm_root = Path.home() / '.nvm' / 'versions' / 'node'
    if nvm_root.is_dir():
        versions = sorted(
            [p for p in nvm_root.iterdir() if p.is_dir()],
            key=lambda p: p.name,
            reverse=True,
        )
        for ver in versions:
            bin_dir = ver / 'bin'
            if (bin_dir / 'node').is_file():
                dirs.append(str(bin_dir))
                break

    for candidate in (
        Path.home() / '.local' / 'share' / 'pnpm' / 'nodejs_current' / 'bin',
        Path('/usr/bin'),
    ):
        if (candidate / 'node').is_file():
            dirs.append(str(candidate))
    return dirs


def _find_vite(rmf_web_dir: str) -> str | None:
    candidates = [
        Path(rmf_web_dir) / 'node_modules' / '.pnpm' / 'node_modules' / '.bin' / 'vite',
        Path(rmf_web_dir) / 'node_modules' / '.bin' / 'vite',
        Path(rmf_web_dir)
        / 'packages'
        / 'rmf-dashboard-framework'
        / 'node_modules'
        / '.bin'
        / 'vite',
    ]
    for path in candidates:
        if path.is_file() and os.access(path, os.X_OK):
            return str(path)
    return None


def _parse_url_port(url: str, default: int) -> int:
    # http://localhost:8000 or ws://localhost:8006
    try:
        after = url.rsplit('://', 1)[-1]
        hostport = after.split('/', 1)[0]
        if ':' in hostport:
            return int(hostport.rsplit(':', 1)[-1])
    except (TypeError, ValueError):
        pass
    return default


def _setup(context, *args, **kwargs):
    pkg = get_package_share_directory('navpromini_rmf_sim')
    building = _expand_path(LaunchConfiguration('building_file').perform(context))
    if not building:
        building = os.path.join(pkg, 'site', 'site.building.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim = _truthy(use_sim_time.perform(context))
    server_uri = LaunchConfiguration('server_uri').perform(context)
    web_mode = LaunchConfiguration('web_mode').perform(context).strip().lower()
    rmf_web_dir = _expand_path(LaunchConfiguration('rmf_web_dir').perform(context))
    ros_domain = os.environ.get('ROS_DOMAIN_ID', '0')
    rmw = os.environ.get('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
    api_image = LaunchConfiguration('api_image').perform(context)
    dashboard_image = LaunchConfiguration('dashboard_image').perform(context)
    api_url = LaunchConfiguration('api_url').perform(context)
    trajectory_url = LaunchConfiguration('trajectory_url').perform(context)
    websocket_port = LaunchConfiguration('websocket_port').perform(context).strip()
    api_port = _parse_url_port(api_url, 8000)
    traj_port = int(websocket_port) if websocket_port.isdigit() else _parse_url_port(
        trajectory_url, 8006
    )

    actions = [
        LogInfo(msg=[
            f'NavProMini RMF web: building={building} web_mode={web_mode} '
            f'server_uri={server_uri} ROS_DOMAIN_ID={ros_domain}'
        ]),
    ]

    if _port_open(api_port) or _port_open(traj_port):
        actions.append(LogInfo(msg=[
            f'Note: port {api_port} and/or {traj_port} already in use. '
            'If navpro-fleet-server Docker (navpro-rmf-api-sim / rmf-core) is running '
            'on host network, stop it or launch with a free ROS_DOMAIN_ID + ports.'
        ]))

    if _truthy(LaunchConfiguration('start_building_map').perform(context)):
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

    if _truthy(LaunchConfiguration('start_rmf_core').perform(context)):
        actions.extend([
            Node(
                package='rmf_traffic_ros2',
                executable='rmf_traffic_schedule',
                name='rmf_traffic_schedule_primary',
                parameters=[{'use_sim_time': use_sim}],
                output='both',
            ),
            Node(
                package='rmf_traffic_ros2',
                executable='rmf_traffic_blockade',
                name='rmf_traffic_blockade',
                parameters=[{'use_sim_time': use_sim}],
                output='both',
            ),
            Node(
                package='rmf_fleet_adapter',
                executable='door_supervisor',
                name='door_supervisor',
                parameters=[{'use_sim_time': use_sim}],
                output='screen',
            ),
            Node(
                package='rmf_fleet_adapter',
                executable='lift_supervisor',
                name='lift_supervisor',
                parameters=[{'use_sim_time': use_sim}],
                output='screen',
            ),
            Node(
                package='rmf_fleet_adapter',
                executable='mutex_group_supervisor',
                name='mutex_group_supervisor',
                parameters=[{'use_sim_time': use_sim}],
                output='screen',
            ),
            Node(
                package='rmf_task_ros2',
                executable='rmf_task_dispatcher',
                name='rmf_task_dispatcher',
                parameters=[{
                    'use_sim_time': use_sim,
                    'bidding_time_window': 2.0,
                    'use_unique_hex_string_with_task_id': True,
                    'server_uri': server_uri,
                }],
                output='screen',
            ),
        ])

    start_viz = _truthy(LaunchConfiguration('start_schedule_visualizer').perform(context))
    if start_viz and _port_open(traj_port):
        actions.append(LogInfo(msg=[
            f'Skipping schedule visualizer: port {traj_port} already in use. '
            f'Pass websocket_port:=8016 (and matching trajectory_url) or free the port.'
        ]))
        start_viz = False

    if start_viz:
        viz_share = get_package_share_directory('rmf_visualization')
        wait_secs = LaunchConfiguration('schedule_wait_secs').perform(context)
        actions.append(
            IncludeLaunchDescription(
                FrontendLaunchDescriptionSource(
                    os.path.join(viz_share, 'visualization.launch.xml')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'map_name': LaunchConfiguration('initial_map'),
                    'headless': 'true',
                    'websocket_port': str(traj_port),
                    'wait_secs': wait_secs,
                }.items(),
            )
        )

    # Adapters (demos-style launch/include/adapters/):
    #   door  → Gazebo libdoor + door_supervisor (no Python node)
    #   lift  → Gazebo liblift + lift_supervisor (book); Python only if
    #           use_python_lift_adapter:=true
    #   workcell → dispenser/ingestor
    #   fleet → path_to_nav2_bridge + EasyFullControl
    adapters = os.path.join(pkg, 'launch', 'include', 'adapters')
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(adapters, 'door_adapter.launch.py')
            ),
        )
    )
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(adapters, 'lift_adapter.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'nav_graph': LaunchConfiguration('nav_graph'),
                'initial_map': LaunchConfiguration('initial_map'),
                'use_python_lift_adapter': LaunchConfiguration(
                    'use_python_lift_adapter'
                ),
            }.items(),
        )
    )

    start_workcells = LaunchConfiguration('start_workcells').perform(context)
    cafe_alias = LaunchConfiguration('start_cafe_infra').perform(context)
    if cafe_alias:
        start_workcells = cafe_alias
    if _truthy(start_workcells):
        nav_graph = LaunchConfiguration('nav_graph').perform(context)
        if not nav_graph:
            nav_graph = os.path.join(pkg, 'site', 'nav_graphs', '0.yaml')
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(adapters, 'workcell_adapter.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'nav_graph': nav_graph,
                }.items(),
            )
        )

    if _truthy(LaunchConfiguration('start_fleet').perform(context)):
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(adapters, 'fleet_adapter.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'server_uri': server_uri,
                    'initial_map': LaunchConfiguration('initial_map'),
                }.items(),
            )
        )

    start_api = _truthy(LaunchConfiguration('start_api').perform(context))
    start_dashboard = _truthy(LaunchConfiguration('start_dashboard').perform(context))

    if start_api and _port_open(api_port):
        actions.append(LogInfo(msg=[
            f'Skipping API server: port {api_port} already in use '
            f'(reusing {api_url}). Set start_api:=false to silence this.'
        ]))
        start_api = False

    if start_api or start_dashboard:
        if web_mode == 'local':
            api_pkg = Path(rmf_web_dir) / 'packages' / 'api-server'
            dash_pkg = Path(rmf_web_dir) / 'packages' / 'rmf-dashboard-framework'
            venv_python = Path(rmf_web_dir) / '.venv' / 'bin' / 'python'
            vite_bin = _find_vite(rmf_web_dir)
            node_dirs = _find_node_bin_dirs()
            path_parts = node_dirs + [
                os.path.expanduser('~/.local/share/pnpm/bin'),
                os.environ.get('PATH', ''),
            ]
            path_env = ':'.join(p for p in path_parts if p)
            default_cfg = os.path.join(pkg, 'config', 'rmf_api_server_config.py')
            traj_ws = (
                trajectory_url
                if '://' in trajectory_url
                else f'ws://localhost:{traj_port}'
            )

            if start_api:
                if not api_pkg.is_dir():
                    actions.append(LogInfo(msg=[
                        f'rmf-web api-server not found at {api_pkg}. '
                        'Set rmf_web_dir:=~/rmf_ws/src/rmf-web or use web_mode:=docker'
                    ]))
                elif not venv_python.is_file():
                    actions.append(LogInfo(msg=[
                        f'rmf-web venv missing ({venv_python}). '
                        'From ~/rmf_ws/src/rmf-web run: pnpm install '
                        '(or use web_mode:=docker)'
                    ]))
                elif not os.path.isfile(default_cfg) and not os.environ.get(
                    'RMF_API_SERVER_CONFIG'
                ):
                    actions.append(LogInfo(msg=[
                        f'Missing API config at {default_cfg}'
                    ]))
                else:
                    cfg = os.environ.get('RMF_API_SERVER_CONFIG', default_cfg)
                    cache_dir = os.path.expanduser('~/.cache/navpromini_rmf_api/cache')
                    actions.append(
                        ExecuteProcess(
                            # Fresh DB path via navpromini config (avoids stale door rows).
                            # Bypass pnpm entirely.
                            cmd=[
                                'bash', '-lc',
                                f'mkdir -p "{cache_dir}" && '
                                f'export RMF_API_SERVER_CONFIG="{cfg}" && '
                                f'export RMF_API_USE_SIM_TIME="{str(use_sim).lower()}" && '
                                f'export RMF_API_SERVER_PORT="{api_port}" && '
                                f'export RMF_API_SERVER_PUBLIC_URL="{api_url}" && '
                                f'exec "{venv_python}" -m api_server',
                            ],
                            cwd=str(api_pkg),
                            output='screen',
                            additional_env={'PATH': path_env},
                        )
                    )

            if start_dashboard:
                if not dash_pkg.is_dir():
                    actions.append(LogInfo(msg=[
                        f'rmf-web dashboard not found at {dash_pkg}. '
                        'Set rmf_web_dir:=~/rmf_ws/src/rmf-web or use web_mode:=docker'
                    ]))
                elif not vite_bin:
                    actions.append(LogInfo(msg=[
                        'vite not found under rmf-web/node_modules. '
                        'From ~/rmf_ws/src/rmf-web run: pnpm install '
                        '(or use web_mode:=docker)'
                    ]))
                elif not node_dirs:
                    actions.append(LogInfo(msg=[
                        'node not found (need nvm or system node on PATH). '
                        'Or set NODE_BIN_DIR:=/path/to/node/bin'
                    ]))
                else:
                    # Call vite binary directly — never `pnpm exec` (triggers reinstall).
                    actions.append(
                        ExecuteProcess(
                            cmd=[
                                vite_bin,
                                '-c', 'examples/shared/vite.config.ts',
                                'examples/demo',
                            ],
                            cwd=str(dash_pkg),
                            output='screen',
                            additional_env={
                                'PATH': path_env,
                                'RMF_SERVER_URL': api_url,
                                'TRAJECTORY_SERVER_URL': traj_ws,
                            },
                        )
                    )
                    actions.append(LogInfo(msg=[
                        'Local dashboard: http://localhost:5173  (admin / admin)'
                    ]))
        else:
            # Docker (default): open-source images from ghcr.io/open-rmf/rmf-web
            if start_api:
                actions.append(
                    ExecuteProcess(
                        cmd=[
                            'docker', 'run', '--rm', '--network', 'host',
                            '--name', 'navpromini-rmf-api',
                            '-e', f'ROS_DOMAIN_ID={ros_domain}',
                            '-e', f'RMW_IMPLEMENTATION={rmw}',
                            api_image,
                        ],
                        output='screen',
                    )
                )
            if start_dashboard:
                actions.append(
                    ExecuteProcess(
                        cmd=[
                            'docker', 'run', '--rm', '--network', 'host',
                            '--name', 'navpromini-rmf-dashboard',
                            '-e', f'RMF_SERVER_URL={api_url}',
                            '-e', (
                                f'TRAJECTORY_SERVER_URL='
                                f'{trajectory_url if "://" in trajectory_url else f"ws://localhost:{traj_port}"}'
                            ),
                            dashboard_image,
                        ],
                        output='screen',
                    )
                )
                actions.append(LogInfo(msg=[
                    'Docker dashboard: http://localhost:3000  (admin / admin)'
                ]))

        actions.append(LogInfo(msg=[f'API docs: {api_url}/docs']))

    return actions


def generate_launch_description():
    pkg = get_package_share_directory('navpromini_rmf_sim')
    default_building = os.path.join(pkg, 'site', 'site.building.yaml')
    default_rmf_web = os.path.expanduser('~/rmf_ws/src/rmf-web')

    return LaunchDescription([
        DeclareLaunchArgument(
            'building_file',
            default_value=default_building,
            description='Path to traffic-editor building YAML (site maps)',
        ),
        DeclareLaunchArgument(
            'initial_map',
            default_value='L1',
            description='Initial floor for schedule visualizer',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='true when pairing with Gazebo/Nav2 (/clock)',
        ),
        DeclareLaunchArgument(
            'server_uri',
            default_value='http://localhost:8000/_internal',
            description='API internal URI for task dispatcher / fleet adapters',
        ),
        DeclareLaunchArgument(
            'api_url',
            default_value='http://localhost:8000',
            description='Public API URL for the dashboard',
        ),
        DeclareLaunchArgument(
            'trajectory_url',
            default_value='ws://localhost:8006',
            description='Schedule visualizer websocket URL',
        ),
        DeclareLaunchArgument(
            'websocket_port',
            default_value='8006',
            description='Trajectory server port (must be free)',
        ),
        DeclareLaunchArgument(
            'web_mode',
            default_value='docker',
            description='docker (ghcr images) or local (rmf-web venv + vite)',
        ),
        DeclareLaunchArgument(
            'rmf_web_dir',
            default_value=default_rmf_web,
            description='Path to open-rmf/rmf-web checkout (web_mode:=local)',
        ),
        DeclareLaunchArgument(
            'api_image',
            default_value='ghcr.io/open-rmf/rmf-web/api-server:latest',
        ),
        DeclareLaunchArgument(
            'dashboard_image',
            default_value='ghcr.io/open-rmf/rmf-web/demo-dashboard:latest',
        ),
        DeclareLaunchArgument('start_building_map', default_value='true'),
        DeclareLaunchArgument(
            'start_rmf_core',
            default_value='true',
            description='Traffic schedule, blockade, dispatcher, supervisors',
        ),
        DeclareLaunchArgument(
            'start_schedule_visualizer',
            default_value='true',
            description='Headless RMF viz (websocket for dashboard)',
        ),
        DeclareLaunchArgument(
            'schedule_wait_secs',
            default_value='30',
            description='Seconds for schedule visualizer to connect to traffic schedule',
        ),
        DeclareLaunchArgument(
            'nav_graph',
            default_value=os.path.join(
                get_package_share_directory('navpromini_rmf_sim'),
                'site', 'nav_graphs', '0.yaml',
            ),
            description='Nav graph for workcell GUID discovery',
        ),
        DeclareLaunchArgument(
            'start_workcells',
            default_value='false',
            description=(
                'Start site-agnostic dispenser/ingestor adapter from nav graph. '
                'Enable when running deliveries with sim (Gazebo owns doors).'
            ),
        ),
        DeclareLaunchArgument(
            'use_python_lift_adapter',
            default_value='false',
            description=(
                'Optional Python set_pose lift stand-in. Default false — use '
                'Gazebo liblift + lift_supervisor (book architecture).'
            ),
        ),
        DeclareLaunchArgument(
            'start_cafe_infra',
            default_value='',
            description='Deprecated alias for start_workcells',
        ),
        DeclareLaunchArgument(
            'start_fleet',
            default_value='true',
            description='Fleet adapter + PathRequest→Nav2 bridge',
        ),
        DeclareLaunchArgument('start_api', default_value='true'),
        DeclareLaunchArgument('start_dashboard', default_value='true'),
        OpaqueFunction(function=_setup),
    ])
