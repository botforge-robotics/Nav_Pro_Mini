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

from navpromini_launch_manager_interfaces.srv import LaunchWithArgs, StopLaunch
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters

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

    Fires a few times over the better part of a minute rather than once:
    AMCL is a lifecycle node, and navigation_launch.launch.py reporting
    success only means the *process* started, not that AMCL has a map yet
    and has activated its `/initialpose` subscription — a pose published
    before then is just dropped (the topic carries no transient_local
    durability to catch a late subscriber). Confirmed live on this
    hardware: AMCL doesn't even start warning "please set the initial
    pose" until ~18s after navigation_launch starts, so the retry window
    needs real margin past that, not just "a few seconds".

    Both the dock-status and dock-pose checks live *inside* `_attempt`,
    run fresh on every retry, rather than once up front before scheduling
    any of them — `dock_manager_node` (the thing that actually reports
    dock_status) is itself only just starting at the moment this function
    is first called (switch_mode calls it the instant launch_manager
    acknowledges navigation_launch *started*, long before any of its own
    nodes are up), so `bridge.get('dock_status')` is essentially
    guaranteed to be stale or None on that first synchronous check.
    Gating the retries themselves on that check — instead of only gating
    each individual publish — meant no retry was ever scheduled at all,
    silently, on every single boot: confirmed live (AMCL sat un-seeded for
    over two minutes, until a manual localize). A fast mode change or an
    undock mid-window still can't seed a now-stale pose: each attempt
    re-checks live state right before it publishes.
    """
    def _attempt(n: int, total: int) -> None:
        if state.mode != 'navigation':
            return
        if bridge.get('dock_status') not in ('charging', 'full'):
            return
        dock = bridge.get('dock_pose')
        if not dock:
            return
        x, y, theta = dock['x'], dock['y'], dock['theta']
        frame = dock.get('frame', 'map')
        bridge.publish_initial_pose(x, y, theta, frame)
        bridge.get_logger().info(
            f'seeded AMCL initial pose from the dock ({x:.2f}, {y:.2f}) — '
            f'robot is docked, attempt {n}/{total}')

    delays = (2.0, 5.0, 10.0, 18.0, 28.0)
    for n, delay in enumerate(delays, start=1):
        _once(bridge, delay, lambda n=n: _attempt(n, len(delays)))


def _resolve_active_map_name(bridge, state) -> None:
    """Best-effort: reads /map_server's own `yaml_filename` parameter and
    fills state.map_name in once it resolves.

    Called after reconcile_mode observes 'navigation' from a launch it
    didn't start itself, where the real map name is otherwise unknowable
    (see reconcile_mode's own comment on why it settles for None instead of
    guessing). Fire-and-forget via add_done_callback rather than awaited:
    reconcile_mode runs synchronously off a plain rclpy timer, not a
    tornado coroutine, so there is no event loop here to await onto — see
    roscall.py's module docstring for why that boundary matters.

    Retries a few times rather than checking once: this is typically called
    moments after reconcile_mode first observes the `amcl` node in the graph,
    but map_server's parameter service can still take a few more seconds to
    come up after that — a single check here used to just silently give up,
    leaving map_name stuck at None for the rest of the session. Each attempt
    re-checks mode first, same reasoning as _seed_pose_from_dock's retries.
    """
    resolved = {'done': False}

    def _attempt(n: int) -> None:
        if resolved['done'] or state.mode != 'navigation':
            return
        client = bridge.cli_map_server_param
        if not client.service_is_ready():
            return
        request = GetParameters.Request()
        request.names = ['yaml_filename']

        def _on_done(future) -> None:
            try:
                result = future.result()
            except Exception as exc:  # noqa: BLE001
                bridge.get_logger().warn(f'map_server param fetch failed: {exc}')
                return
            if not result.values:
                return
            value = result.values[0]
            if value.type != ParameterType.PARAMETER_STRING or not value.string_value:
                return
            file_name = value.string_value.rsplit('/', 1)[-1]
            map_name = file_name[:-5] if file_name.endswith('.yaml') else file_name
            # Only apply if still in the navigation state this fetch was kicked
            # off for — a mode change mid-flight (e.g. back to idle) shouldn't
            # retroactively stamp a map name onto it.
            if state.mode == 'navigation':
                state.map_name = map_name
                resolved['done'] = True
                bridge.get_logger().info(f'resolved running map: {map_name} (attempt {n}/4)')

        client.call_async(request).add_done_callback(_on_done)

    for n, delay in enumerate((1.0, 3.0, 6.0, 10.0), start=1):
        _once(bridge, delay, lambda n=n: _attempt(n))


# localization.lost / localization.recovered — how stale pose_map may get
# before a client should be told AMCL is no longer tracking. Same 2s cadence
# as reconcile_mode's own timer; a couple of missed cycles' worth of grace so
# a single slow AMCL update doesn't flap the event.
_LOCALIZATION_LOST_SEC = 5.0


def watch_localization(bridge, state) -> None:
    """Emit localization.lost / localization.recovered on pose_map staleness.

    Only meaningful in navigation mode — pose_map has no publisher (no AMCL)
    in idle/mapping, and an absent value there is normal, not a loss.
    """
    if state.mode != 'navigation':
        state.localization_lost = False
        return
    _value, age = bridge.get_with_age('pose_map')
    lost = age is None or age > _LOCALIZATION_LOST_SEC
    if lost == state.localization_lost:
        return
    state.localization_lost = lost
    bridge.emit_event('localization.lost' if lost else 'localization.recovered')


# How long a just-set mode is trusted over the ROS graph before reconcile_mode
# will call it "idle" on that basis alone. Covers the gap between a launch's
# process starting (switch_mode returns) and all of its nodes — AMCL/
# slam_toolbox included — actually registering in the graph, which can take
# several seconds for the full nav2/SLAM stack. Without this, a poll landing
# in that gap reads as "neither node exists yet" and wrongly bounces state
# through idle, wiping the map/launch_id switch_mode had just set correctly
# (observed live: map stuck at null for the rest of the session, since
# _resolve_active_map_name only gets kicked off again on the *next* genuine
# transition). Only gates the ambiguous "nothing observed" case — an
# unambiguous mapping<->navigation transition (the other node's name
# appearing) is trusted immediately, no grace needed.
_RECONCILE_IDLE_SETTLE_SEC = 15.0


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
    if (observed == 'idle' and state.mode != 'idle'
            and state.age() < _RECONCILE_IDLE_SETTLE_SEC):
        return  # ambiguous: genuinely idle, or still starting up? — see above
    # launch_id genuinely can't be recovered for a launch the SDK didn't
    # start itself — there's no ROS-level way to ask launch_manager "what's
    # the id of whatever's currently running" if we never issued it. map
    # CAN be recovered though (read straight off map_server's own
    # yaml_filename param, not guessed) — see _resolve_active_map_name,
    # kicked off just below rather than fabricating a value here. See the
    # module docstring in ros_bridge.py: null over an invented value, for
    # what's still genuinely unknowable (launch_id).
    state.set(observed, None, None)
    bridge.get_logger().info(f'mode reconciled from ROS graph: {observed} '
                             '(started outside the SDK)')
    if observed == 'idle':
        # Whatever launch was running (started outside the SDK, hence this
        # whole reconcile path) just went away — see switch_mode's own
        # invalidate call and RosBridge.invalidate's docstring for why a
        # stale pose_map/dock_status left behind reads as confidently
        # correct instead of unknown.
        bridge.invalidate('pose_map', 'dock_status')
    if observed == 'navigation':
        _seed_pose_from_dock(bridge, state)
        _resolve_active_map_name(bridge, state)


async def switch_mode(opts: dict, bridge, mode: str, map_name: str | None) -> dict:
    """Perform a mode change. Shared by POST /mode, POST /maps/{n}/activate,
    POST /mapping/finish, and the boot-time auto-navigation hook in server.py.

    Takes `opts`/`bridge` directly rather than a handler instance so it is
    callable from contexts with no in-flight HTTP request — the boot hook in
    particular runs before any client has connected.
    """
    if mode not in VALID_MODES:
        raise ApiError(400, 'invalid_mode',
                       f'mode must be one of: {", ".join(VALID_MODES)}',
                       {'valid': list(VALID_MODES)})

    state = opts['mode_state']
    if state.busy:
        raise ApiError(409, 'mode_busy', 'A mode change is already in progress')

    if mode == 'navigation' and not map_name:
        map_name = opts['store'].current_map()
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
            await call_service(bridge.cli_stop, req, 'stop_launch', timeout=90.0)
            state.set('idle', None, None)
            opts['client_hold'].release()
            # These belong to the launch instance that just stopped, not the
            # robot as a whole — left cached, they'd read as a perfectly
            # fresh, confidently wrong answer (LOCALIZED off a pose AMCL's
            # *previous* instance published, a dock state from the
            # dock_manager that just died) for as long as it takes whatever
            # starts next to publish its own first message. See
            # RosBridge.invalidate's docstring.
            bridge.invalidate('pose_map', 'dock_status')

        if mode == 'idle':
            return {'mode': 'idle'}

        package, launch_file = _LAUNCH[mode]
        req = LaunchWithArgs.Request()
        req.package = package
        req.launch_file = launch_file
        req.arguments = f'map:={map_name}.yaml' if mode == 'navigation' else ''

        # Hold the rosbridge client count up BEFORE launching, so there is no
        # window where a zero count could kill the launch we just made.
        opts['client_hold'].acquire()
        resp = await call_service(bridge.cli_launch, req,
                                  'launch_with_args', timeout=60.0)
        if not resp.success:
            opts['client_hold'].release()
            bridge.emit_event(f'{mode}.failed', {'message': resp.message or 'launch failed'})
            raise ApiError(500, 'launch_failed', resp.message or 'launch failed',
                           {'package': package, 'launch_file': launch_file})

        state.set(mode, map_name, resp.unique_id)
        bridge.emit_event(f'{mode}.started', {'map': map_name})
        if mode == 'navigation':
            if map_name:
                opts['store'].set_current_map(map_name)
                bridge.emit_event('map.loaded', {'name': map_name})
            _seed_pose_from_dock(bridge, state)
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
        result = await switch_mode(self.opts, self.bridge,
                                   str(data['mode']).lower(), data.get('map'))
        self.send(result, status=200 if result['mode'] == 'idle' else 202)


class FinishMappingHandler(BaseHandler):
    """Atomic 'FINISH_MAPPING' (doc §12): stop SLAM, save the map, switch to
    navigation on it — one robot-owned call instead of three client-issued
    ones. Shared by both the Flutter app and any other client that wants the
    same doc-specified MAPPING -> MAP_SAVING -> NAVIGATION transition without
    reimplementing the orchestration itself.
    """

    async def post(self) -> None:
        # Lazy import, matching maps.py's own lazy import of switch_mode in
        # ActivateMapHandler.post — keeps the two modules' mutual dependency
        # resolved at call time rather than at either module's load time.
        from . import maps as maps_handlers

        state = self.opts['mode_state']
        if state.mode != 'mapping':
            raise ApiError(409, 'not_mapping',
                           f'Not currently mapping (mode is {state.mode!r})')

        data = self.body(('name',))
        name = str(data['name']).strip()
        overwrite = bool(data.get('overwrite', False))

        await switch_mode(self.opts, self.bridge, 'idle', None)
        try:
            await maps_handlers.save_map(self.bridge, self.opts['store'], name, overwrite)
        except ApiError as exc:
            self.bridge.emit_event('mapping.failed', {'name': name, 'message': exc.message})
            raise
        self.bridge.emit_event('mapping.completed', {'name': name})
        result = await switch_mode(self.opts, self.bridge, 'navigation', name)
        self.send(result, status=202)
