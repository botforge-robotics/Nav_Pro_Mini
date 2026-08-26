#!/usr/bin/env python3
"""Docking and undocking, via dock_manager's two action servers."""

from __future__ import annotations

import math
import time

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import DockRobot, NavigateToPose

from .base import ApiError, BaseHandler
from .roscall import ros_future, send_goal


class _DockTracker:
    def __init__(self) -> None:
        self.handle = None
        self.state = 'idle'
        self.message = ''
        self.started_at: float | None = None

    def begin(self, handle, what: str) -> None:
        self.handle = handle
        self.state = what
        self.message = ''
        self.started_at = time.time()

    def finish(self, state: str, message: str = '') -> None:
        self.state = state
        self.message = message
        self.handle = None


TRACKER = _DockTracker()


async def send_dock_goal(bridge, navigate_to_staging: bool = True):
    """Build and send the dock goal, returning the accepted handle.

    Raises ApiError synchronously (action unavailable / goal rejected) — see
    navigation.py's send_navigate_goal for why that's kept separate from the
    awaited-result half.
    """
    goal = DockRobot.Goal()
    goal.dock_type = 'simple_charging_dock'
    # navigate_to_staging_pose drives to the standoff first. Skipping it only
    # makes sense when the robot is already parked in front of the dock;
    # default to the safe behaviour.
    goal.navigate_to_staging_pose = bool(navigate_to_staging)
    goal.use_dock_id = True     # use the robot's own saved dock pose

    handle = await send_goal(goal=goal, action_client=bridge.act_dock, name='dock')
    TRACKER.begin(handle, 'docking')
    bridge.emit_event('dock.started')
    return handle


async def await_dock_result(bridge, handle, timeout: float = 600.0) -> dict:
    """Await an already-sent dock goal's outcome. Never raises."""
    try:
        wrapped = await ros_future(handle.get_result_async(), timeout=timeout)
        result = getattr(wrapped, 'result', None)
        ok = bool(getattr(result, 'success', False))
        message = getattr(result, 'error_msg', '') or ''
        TRACKER.finish('docked' if ok else 'failed', message)
        bridge.emit_event('dock.completed' if ok else 'dock.failed',
                          {'message': message} if not ok else {})
        return {'ok': ok, 'message': message}
    except Exception as exc:  # noqa: BLE001
        TRACKER.finish('failed', str(exc))
        bridge.emit_event('dock.failed', {'message': str(exc)})
        return {'ok': False, 'message': str(exc)}


async def dock_robot(bridge, navigate_to_staging: bool = True, timeout: float = 600.0) -> dict:
    """send_dock_goal + await_dock_result, fully awaited — for the mission
    runner's `dock` step (missions.py)."""
    if TRACKER.state in ('docking', 'undocking'):
        return {'ok': False, 'message': f'A {TRACKER.state} operation is already running'}
    try:
        handle = await send_dock_goal(bridge, navigate_to_staging)
    except ApiError as exc:
        bridge.emit_event('dock.failed', {'message': exc.message})
        return {'ok': False, 'message': exc.message}
    return await await_dock_result(bridge, handle, timeout)


async def send_undock_goal(bridge):
    """Build and send the undock goal, returning the accepted handle.

    Split from await_undock_result the same way send_dock_goal/
    await_dock_result are — the FIRST version of this (undock_robot alone,
    entirely spawned by UndockHandler) was a real regression: it meant a
    rejected goal — action server down, robot already busy — never reached
    the HTTP response at all, which just came back `{"accepted": true}`
    regardless. A client (the app or a raw API caller) had no way to tell
    "your undock request failed outright" from "it's in progress", which
    read as the whole endpoint simply not responding. Raises ApiError
    synchronously on rejection, same as send_dock_goal.
    """
    goal = NavigateToPose.Goal()
    goal.pose = PoseStamped()   # empty frame_id + zero quaternion = undock only

    handle = await send_goal(goal=goal, action_client=bridge.act_navigate, name='undock')
    TRACKER.begin(handle, 'undocking')
    bridge.emit_event('dock.started', {'operation': 'undock'})
    return handle


async def await_undock_result(bridge, handle, timeout: float = 180.0) -> dict:
    """Await an already-sent undock goal's outcome. Never raises."""
    try:
        wrapped = await ros_future(handle.get_result_async(), timeout=timeout)
        status = getattr(wrapped, 'status', None)
        ok = status == 4
        message = '' if ok else f'ended with status {status}'
        TRACKER.finish('undocked' if ok else 'failed', message)
        bridge.emit_event('dock.completed' if ok else 'dock.failed',
                          {'operation': 'undock',
                           **({'message': message} if not ok else {})})
        return {'ok': ok, 'message': message}
    except Exception as exc:  # noqa: BLE001
        TRACKER.finish('failed', str(exc))
        bridge.emit_event('dock.failed', {'operation': 'undock', 'message': str(exc)})
        return {'ok': False, 'message': str(exc)}


async def undock_robot(bridge, timeout: float = 180.0) -> dict:
    """send_undock_goal + await_undock_result, fully awaited — for the
    mission runner's `undock` step (missions.py)."""
    if TRACKER.state in ('docking', 'undocking'):
        return {'ok': False, 'message': f'A {TRACKER.state} operation is already running'}
    try:
        handle = await send_undock_goal(bridge)
    except ApiError as exc:
        bridge.emit_event('dock.failed', {'operation': 'undock', 'message': exc.message})
        return {'ok': False, 'message': exc.message}
    return await await_undock_result(bridge, handle, timeout)


class DockHandler(BaseHandler):
    """Start an autonomous dock.

    Returns as soon as dock_manager accepts, not when the robot is charging —
    a dock takes tens of seconds. Watch /dock/status or the event stream.
    """

    async def post(self) -> None:
        data = self.body()
        if TRACKER.state in ('docking', 'undocking'):
            raise ApiError(409, 'dock_busy',
                           f'A {TRACKER.state} operation is already running')

        navigate_to_staging = bool(data.get('navigate_to_staging', True))
        handle = await send_dock_goal(self.bridge, navigate_to_staging)
        self.opts['spawn'](await_dock_result(self.bridge, handle))
        self.send({'accepted': True,
                   'navigate_to_staging': navigate_to_staging}, status=202)


class UndockHandler(BaseHandler):
    """Undock and stop.

    Sends the undock action with no goal pose. dock_manager treats an absent
    pose as "undock only" and does not navigate afterwards — deliberately, so
    an empty request cannot be read as "drive to the map origin", which is the
    dock itself once the map was made while docked.
    """

    async def post(self) -> None:
        if TRACKER.state in ('docking', 'undocking'):
            raise ApiError(409, 'dock_busy',
                           f'A {TRACKER.state} operation is already running')
        handle = await send_undock_goal(self.bridge)
        self.opts['spawn'](await_undock_result(self.bridge, handle))
        self.send({'accepted': True}, status=202)


class DockCancelHandler(BaseHandler):
    """Cancel an in-progress dock/undock goal. Mirrors navigation.py's own
    CancelHandler exactly, against this module's own TRACKER — dock/undock
    goals are tracked separately from plain navigation goals (see
    _DockTracker above), so navigation's cancel route can't reach these."""

    async def delete(self) -> None:
        if TRACKER.state not in ('docking', 'undocking') or TRACKER.handle is None:
            self.send({'canceled': False, 'reason': 'no active dock operation'})
            return
        await ros_future(TRACKER.handle.cancel_goal_async(), timeout=5.0)
        TRACKER.finish('failed', 'Canceled by API request')
        self.send({'canceled': True})


class DockStatusHandler(BaseHandler):
    """Combined view: what dock_manager reports and what the battery proves.

    Both are included because they can legitimately disagree — a robot pushed
    onto the dock by hand is charging while dock_manager still says 'undocked',
    and charging current is the ground truth for physical connection.
    """

    def get(self) -> None:
        battery = self.bridge.get('battery') or {}
        tag = self.bridge.get('dock_tag')
        dock_status, age = self.bridge.get_with_age('dock_status')
        self.send({
            'state': dock_status or 'unknown',
            'age_sec': age,
            'operation': TRACKER.state,
            'message': TRACKER.message,
            'charging': bool(battery.get('charging')),
            'battery_status': battery.get('status'),
            'tag_visible': bool(tag.get('visible')) if tag else False,
        })


class DockPoseHandler(BaseHandler):
    """Read or set where the robot believes its dock is."""

    def get(self) -> None:
        value, age = self.bridge.get_with_age('dock_pose')
        if value is None:
            raise ApiError(404, 'no_dock_pose',
                           'No dock pose is known. Set one, or map the area with '
                           'the robot docked so the map origin defines it.')
        self.send({'data': value, 'age_sec': age})

    def put(self) -> None:
        data = self.body(('x', 'y'))
        x, y = float(data['x']), float(data['y'])
        theta = float(data.get('theta', 0.0))
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.bridge.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.z = math.sin(theta / 2.0)
        msg.pose.orientation.w = math.cos(theta / 2.0)
        self.opts['dock_pose_pub'].publish(msg)
        self.send({'x': x, 'y': y, 'theta': theta})
