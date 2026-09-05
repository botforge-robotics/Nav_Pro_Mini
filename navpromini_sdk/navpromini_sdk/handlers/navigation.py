#!/usr/bin/env python3
"""Navigation: send goals, track them, cancel, localize."""

from __future__ import annotations

import math
import time

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

from .base import ApiError, BaseHandler
from .roscall import call_service, ros_future, send_goal


class _GoalTracker:
    """Holds the one in-flight navigation goal.

    Single-goal by design: a robot can only drive to one place at a time, so
    accepting a second goal silently would leave the caller unable to tell
    which one is actually running. A new goal supersedes the old one only when
    asked explicitly.
    """

    def __init__(self) -> None:
        self.handle = None
        self.target: dict | None = None
        self.started_at: float | None = None
        self.state = 'idle'
        self.message = ''

    def begin(self, handle, target: dict) -> None:
        self.handle = handle
        self.target = target
        self.started_at = time.time()
        self.state = 'active'
        self.message = ''

    def finish(self, state: str, message: str = '') -> None:
        self.state = state
        self.message = message
        self.handle = None

    def snapshot(self) -> dict:
        return {
            'state': self.state,
            'target': self.target,
            'message': self.message,
            'elapsed_sec': (round(time.time() - self.started_at, 1)
                            if self.started_at else None),
        }


TRACKER = _GoalTracker()


def resolve_target(store, data: dict) -> dict:
    """Waypoint name or raw x/y/theta -> a target dict. Shared by GotoHandler
    and the mission runner's `navigate` step (missions.py)."""
    if 'waypoint' in data:
        wp = store.get_waypoint(str(data['waypoint']))
        if wp is None:
            raise ApiError(404, 'waypoint_not_found',
                           f"No waypoint named {data['waypoint']!r}",
                           {'waypoint': data['waypoint']})
        x, y, theta = wp['x'], wp['y'], wp.get('theta', 0.0)
        return {'waypoint': wp['name'], 'x': x, 'y': y, 'theta': theta}
    missing = [f for f in ('x', 'y') if f not in data]
    if missing:
        raise ApiError(400, 'missing_field',
                       'Provide either "waypoint" or both "x" and "y"',
                       {'missing': missing})
    x, y = float(data['x']), float(data['y'])
    theta = float(data.get('theta', 0.0))
    return {'x': x, 'y': y, 'theta': theta}


async def send_navigate_goal(bridge, target: dict):
    """Build and send the nav goal, returning the accepted handle.

    Routed through dock_manager's `undock` action rather than bt_navigator's
    `navigate_to_pose`. That action undocks first if the robot is on the
    charger, then navigates — so a docked robot cannot be told to drive away
    while still physically connected. See the dock_manager module docstring.

    Raises ApiError (action unavailable / goal rejected) rather than
    swallowing it — a caller sending an interactive goal needs that to
    surface synchronously as an HTTP error, not silently vanish into a
    background task. See navigate_to() for the awaited-result counterpart.
    """
    x, y, theta = target['x'], target['y'], target.get('theta', 0.0)
    goal = NavigateToPose.Goal()
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(theta / 2.0)
    pose.pose.orientation.w = math.cos(theta / 2.0)
    goal.pose = pose

    handle = await send_goal(goal=goal, action_client=bridge.act_navigate, name='navigation')
    TRACKER.begin(handle, target)
    bridge.emit_event('navigation.started', {'target': target})
    return handle


async def await_navigate_result(bridge, handle, target: dict, timeout: float = 3600.0) -> dict:
    """Await an already-sent goal's outcome. Never raises; returns
    {'ok': bool, 'message': str} so callers (background task or mission
    runner) don't need their own try/except around ROS-layer exceptions."""
    try:
        wrapped = await ros_future(handle.get_result_async(), timeout=timeout)
        status = getattr(wrapped, 'status', None)
        if status == 4:
            TRACKER.finish('succeeded')
            bridge.emit_event('navigation.completed', {'target': target})
            return {'ok': True, 'message': ''}
        if status == 5:
            TRACKER.finish('canceled', 'Goal was canceled')
            bridge.emit_event('navigation.cancelled', {'target': target})
            return {'ok': False, 'message': 'Goal was canceled'}
        message = f'Navigation ended with status {status}'
        TRACKER.finish('failed', message)
        bridge.emit_event('navigation.failed', {'target': target, 'message': message})
        return {'ok': False, 'message': message}
    except Exception as exc:  # noqa: BLE001
        TRACKER.finish('failed', str(exc))
        bridge.emit_event('navigation.failed', {'target': target, 'message': str(exc)})
        return {'ok': False, 'message': str(exc)}


async def navigate_to(bridge, target: dict, timeout: float = 3600.0) -> dict:
    """send_navigate_goal + await_navigate_result, fully awaited.

    For the mission runner's `navigate` step, whose steps run one at a time
    by design — unlike GotoHandler, which sends synchronously (so a rejection
    reaches the caller) but backgrounds the result wait (see its own comment).
    Returns {'ok': False, 'message': ...} on a send-side ApiError too, rather
    than raising, so a mission step failure is just "this step failed",
    not an unhandled exception in the runner's loop.
    """
    try:
        handle = await send_navigate_goal(bridge, target)
    except ApiError as exc:
        bridge.emit_event('navigation.failed', {'target': target, 'message': exc.message})
        return {'ok': False, 'message': exc.message}
    return await await_navigate_result(bridge, handle, target, timeout)


class GotoHandler(BaseHandler):
    """Send a navigation goal, by coordinates or by saved waypoint name."""

    async def post(self) -> None:
        data = self.body()
        target = resolve_target(self.opts['store'], data)

        if TRACKER.state == 'active' and not data.get('replace', False):
            raise ApiError(409, 'goal_active',
                           'A navigation goal is already running. Cancel it, or '
                           'resend with {"replace": true}.',
                           {'current': TRACKER.snapshot()})
        if TRACKER.state == 'active' and TRACKER.handle is not None:
            await ros_future(TRACKER.handle.cancel_goal_async(), timeout=5.0)

        handle = await send_navigate_goal(self.bridge, target)

        # Resolve the result in the background so the HTTP call returns as soon
        # as the goal is accepted. Callers poll /navigation/status or watch the
        # event stream; holding the request open for a multi-minute drive would
        # tie up a connection and hit every proxy timeout in between.
        self.opts['spawn'](await_navigate_result(self.bridge, handle, target))
        self.send({'accepted': True, 'target': target}, status=202)


class StatusHandler(BaseHandler):
    def get(self) -> None:
        snap = TRACKER.snapshot()
        pose, _age = self.bridge.get_with_age('pose_map')
        if pose and snap['target']:
            snap['distance_remaining'] = round(
                math.dist((pose['x'], pose['y']),
                          (snap['target']['x'], snap['target']['y'])), 3)
        self.send(snap)


async def cancel_active_goal(bridge, reason: str = 'canceled') -> bool:
    if TRACKER.state != 'active' or TRACKER.handle is None:
        return False
    try:
        await ros_future(TRACKER.handle.cancel_goal_async(), timeout=5.0)
    except Exception:
        pass
    TRACKER.finish('canceled', reason)
    bridge.emit_event('navigation.cancelled', {'reason': reason})
    return True


class CancelHandler(BaseHandler):
    async def delete(self) -> None:
        canceled = await cancel_active_goal(self.bridge, 'Canceled by API request')
        if not canceled:
            self.send({'canceled': False, 'reason': 'no active goal'})
            return
        self.send({'canceled': True})


class GlobalRelocalizeHandler(BaseHandler):
    """Disperse AMCL particles across the map for global relocalization."""

    async def post(self) -> None:
        from std_srvs.srv import Empty
        req = Empty.Request()
        await call_service(self.bridge.cli_global_loc, req, 'reinitialize_global_localization', timeout=5.0)
        self.send({'status': 'ok', 'message': 'AMCL particles dispersed across map'})


class LocalizeHandler(BaseHandler):
    """Seed AMCL with an initial pose (the UI's '2D Pose Estimate')."""

    def post(self) -> None:
        data = self.body(('x', 'y'))
        x, y = float(data['x']), float(data['y'])
        theta = float(data.get('theta', 0.0))
        self.bridge.publish_initial_pose(x, y, theta)
        self.send({'x': x, 'y': y, 'theta': theta})


class PathHandler(BaseHandler):
    def get(self) -> None:
        self.send(self.cached('plan', 'planned path'))
