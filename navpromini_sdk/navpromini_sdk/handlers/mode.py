#!/usr/bin/env python3
"""Robot mode: idle / mapping / navigation.

Mode changes go through launch_manager (the same path the Flutter app uses) so
there is exactly ONE owner of the mapping and navigation launches. Two owners
would be a genuine hazard: slam_toolbox and AMCL both publish map->odom, and
two publishers of one TF edge make the transform tree non-deterministic.

THE client_count TRAP
---------------------
launch_manager subscribes to /client_count (published by rosbridge) and SIGINTs
every active launch when it reaches zero. That is sensible when a UI is the only
client — stop the robot when the operator disconnects — but fatal for an API:
an SDK-started navigation would die the moment the last browser closed.

The SDK therefore holds its own rosbridge WebSocket connection for as long as it
has launches running, so the count never falls to zero underneath it. See
`client_hold.py`. This is why mode changes are owned here rather than by
duplicating launch_manager's process handling.
"""

from __future__ import annotations

from nav2_mission_planner_interfaces.srv import LaunchWithArgs, StopLaunch

from .base import ApiError, BaseHandler
from .roscall import call_service

VALID_MODES = ('idle', 'mapping', 'navigation')

# Launch files that implement each mode, in navpromini_mission_planner — the
# thin wrappers that resolve `map:=office.yaml` to a full path.
_LAUNCH = {
    'mapping': ('navpromini_mission_planner', 'mapping_launch.launch.py'),
    'navigation': ('navpromini_mission_planner', 'navigation_launch.launch.py'),
}

# Node names unique to each launch, used by reconcile_mode() below to read
# the true mode off the ROS graph. Both are started with an empty namespace
# by navpromini_mission_planner's wrappers, so a plain name match is exact.
_AMCL_NODE = 'amcl'
_SLAM_NODE = 'slam_toolbox'


def _once(bridge, delay_sec: float, fn) -> None:
    """Run fn() once, delay_sec from now, on the executor's thread.

    rclpy timers are periodic; self-cancelling on first fire is the standard
    way to get a one-shot out of create_timer without pulling in a second
    timer API.
    """
    holder: dict = {}

    def _cb() -> None:
        holder['timer'].cancel()
        fn()

    holder['timer'] = bridge.create_timer(delay_sec, _cb)


def _seed_pose_from_dock(bridge, state) -> None:
    """Skip the manual '2D Pose Estimate' step when navigation starts right
    off the dock — the dock's saved pose already IS the robot's pose, to
    within the docking approach's own tolerance.

    Fires a few times over several seconds rather than once: AMCL is a
    lifecycle node, and navigation_launch.launch.py reporting success only
    means the *process* started, not that AMCL has a map yet and has
    activated its `/initialpose` subscription — that can take a few seconds,
    and a pose published before then is just dropped (the topic carries no
    transient_local durability to catch a late subscriber). Each attempt
    re-checks mode and dock status first, so a fast mode change or an
    undock mid-window can't seed a now-stale pose.
    """
    status = bridge.get('dock_status')
    if status not in ('charging', 'full'):
        return
    dock = bridge.get('dock_pose')
    if not dock:
        return
    x, y, theta = dock['x'], dock['y'], dock['theta']
    frame = dock.get('frame', 'map')

    def _attempt(n: int) -> None:
        if state.mode != 'navigation' or bridge.get('dock_status') not in ('charging', 'full'):
            return
        bridge.publish_initial_pose(x, y, theta, frame)
        bridge.get_logger().info(
            f'seeded AMCL initial pose from the dock ({x:.2f}, {y:.2f}) — '
            f'robot is docked, attempt {n}/3')

    for n, delay in enumerate((1.0, 3.0, 6.0), start=1):
        _once(bridge, delay, lambda n=n: _attempt(n))


def reconcile_mode(bridge, state) -> None:
    """Keep ModeState honest when navigation/mapping was started by someone
    else — the Flutter app talks to launch_manager directly, not through
    switch_mode() below, so a launch it starts (or stops) never touches
    ModeState and GET /mode just reports whatever the SDK itself last did,
    however long ago (observed: 'idle' after 37 hours of the robot actually
    running navigation).

    Poll the real ROS graph instead of trusting only our own launches:
    `/amcl` only exists while navigation_launch.launch.py is up, and
    `/slam_toolbox` only while mapping is — ground truth regardless of who
    started it. Called on a timer, not per-request, so GET /mode stays a
    cheap cache read.
    """
    if state.busy:
        return  # a switch_mode() we started is mid-flight; don't fight it
    names = set(bridge.get_node_names())
    if _AMCL_NODE in names:
        observed = 'navigation'
    elif _SLAM_NODE in names:
        observed = 'mapping'
    else:
        observed = 'idle'

    if observed == state.mode:
        return
    # launch_id and map are only known for launches the SDK itself started —
    # reporting a guessed map name here would be exactly the kind of
    # fabricated data GET /mode must not hand back. See the module docstring
    # in ros_bridge.py: null over an invented value.
    state.set(observed, None, None)
    bridge.get_logger().info(f'mode reconciled from ROS graph: {observed} '
                             '(started outside the SDK)')
    if observed == 'navigation':
        _seed_pose_from_dock(bridge, state)


async def switch_mode(h: BaseHandler, mode: str, map_name: str | None) -> dict:
    """Perform a mode change. Shared by POST /mode and POST /maps/{n}/activate.

    Extracted rather than having one handler call the other's HTTP method:
    invoking a RequestHandler method on a different handler instance couples
    them through request state and breaks as soon as either signature changes.
    """
    if mode not in VALID_MODES:
        raise ApiError(400, 'invalid_mode',
                       f'mode must be one of: {", ".join(VALID_MODES)}',
                       {'valid': list(VALID_MODES)})

    state = h.opts['mode_state']
    if state.busy:
        raise ApiError(409, 'mode_busy', 'A mode change is already in progress')

    if mode == 'navigation' and not map_name:
        map_name = h.opts['store'].current_map()
        if not map_name or map_name == 'default':
            raise ApiError(400, 'map_required',
                           'navigation mode needs a map — pass {"map": "<name>"}')

    state.busy = True
    try:
        # Always stop what is running first. Mapping and navigation are
        # mutually exclusive (both own map->odom), so overlapping them even
        # briefly corrupts the transform tree.
        if state.launch_id:
            req = StopLaunch.Request()
            req.unique_id = state.launch_id
            await call_service(h.bridge.cli_stop, req, 'stop_launch', timeout=90.0)
            state.set('idle', None, None)
            h.opts['client_hold'].release()

        if mode == 'idle':
            return {'mode': 'idle'}

        package, launch_file = _LAUNCH[mode]
        req = LaunchWithArgs.Request()
        req.package = package
        req.launch_file = launch_file
        req.arguments = f'map:={map_name}.yaml' if mode == 'navigation' else ''

        # Hold the rosbridge client count up BEFORE launching, so there is no
        # window where a zero count could kill the launch we just made.
        h.opts['client_hold'].acquire()
        resp = await call_service(h.bridge.cli_launch, req,
                                  'launch_with_args', timeout=60.0)
        if not resp.success:
            h.opts['client_hold'].release()
            raise ApiError(500, 'launch_failed', resp.message or 'launch failed',
                           {'package': package, 'launch_file': launch_file})

        state.set(mode, map_name, resp.unique_id)
        if mode == 'navigation':
            if map_name:
                h.opts['store'].set_current_map(map_name)
            _seed_pose_from_dock(h.bridge, state)
        return {'mode': mode, 'map': map_name, 'launch_id': resp.unique_id}
    finally:
        state.busy = False


class ModeHandler(BaseHandler):
    def get(self) -> None:
        state = self.opts['mode_state']
        self.send({
            'mode': state.mode,
            'map': state.map_name,
            'launch_id': state.launch_id,
            'since_sec': state.age(),
        })

    async def post(self) -> None:
        data = self.body(('mode',))
        result = await switch_mode(self, str(data['mode']).lower(), data.get('map'))
        self.send(result, status=200 if result['mode'] == 'idle' else 202)
