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

        goal = DockRobot.Goal()
        goal.dock_type = 'simple_charging_dock'
        # navigate_to_staging_pose drives to the standoff first. Skipping it
        # only makes sense when the robot is already parked in front of the
        # dock; default to the safe behaviour.
        goal.navigate_to_staging_pose = bool(data.get('navigate_to_staging', True))
        goal.use_dock_id = True     # use the robot's own saved dock pose

        handle = await send_goal(goal=goal, action_client=self.bridge.act_dock,
                                 name='dock')
        TRACKER.begin(handle, 'docking')

        async def _await() -> None:
            try:
                wrapped = await ros_future(handle.get_result_async(), timeout=600.0)
                result = getattr(wrapped, 'result', None)
                ok = bool(getattr(result, 'success', False))
                TRACKER.finish('docked' if ok else 'failed',
                               getattr(result, 'error_msg', '') or '')
            except Exception as exc:  # noqa: BLE001
                TRACKER.finish('failed', str(exc))

        self.opts['spawn'](_await())
        self.send({'accepted': True,
                   'navigate_to_staging': goal.navigate_to_staging_pose}, status=202)


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

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()   # empty frame_id + zero quaternion = undock only

        handle = await send_goal(goal=goal, action_client=self.bridge.act_navigate,
                                 name='undock')
        TRACKER.begin(handle, 'undocking')

        async def _await() -> None:
            try:
                wrapped = await ros_future(handle.get_result_async(), timeout=180.0)
                status = getattr(wrapped, 'status', None)
                TRACKER.finish('undocked' if status == 4 else 'failed',
                               '' if status == 4 else f'ended with status {status}')
            except Exception as exc:  # noqa: BLE001
                TRACKER.finish('failed', str(exc))

        self.opts['spawn'](_await())
        self.send({'accepted': True}, status=202)


class DockStatusHandler(BaseHandler):
    """Combined view: what dock_manager reports and what the battery proves.

    Both are included because they can legitimately disagree — a robot pushed
    onto the dock by hand is charging while dock_manager still says 'undocked',
    and charging current is the ground truth for physical connection.
    """

    def get(self) -> None:
        battery = self.bridge.get('battery') or {}
        tag = self.bridge.get('dock_tag')
        self.send({
            'state': self.bridge.get('dock_status') or 'unknown',
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
