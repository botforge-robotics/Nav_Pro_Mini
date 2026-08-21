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
        if mode == 'navigation' and map_name:
            h.opts['store'].set_current_map(map_name)
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
