#!/usr/bin/env python3
"""Navigation: send goals, track them, cancel, localize."""

from __future__ import annotations

import math
import time

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

from .base import ApiError, BaseHandler
from .roscall import ros_future, send_goal


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


class GotoHandler(BaseHandler):
    """Send a navigation goal, by coordinates or by saved waypoint name.

    Routed through dock_manager's `undock` action rather than bt_navigator's
    `navigate_to_pose`. That action undocks first if the robot is on the
    charger, then navigates — so a docked robot cannot be told to drive away
    while still physically connected. See the dock_manager module docstring.
    """

    async def post(self) -> None:
        data = self.body()
        store = self.opts['store']

        if 'waypoint' in data:
            wp = store.get_waypoint(str(data['waypoint']))
            if wp is None:
                raise ApiError(404, 'waypoint_not_found',
                               f"No waypoint named {data['waypoint']!r}",
                               {'waypoint': data['waypoint']})
            x, y, theta = wp['x'], wp['y'], wp.get('theta', 0.0)
            target = {'waypoint': wp['name'], 'x': x, 'y': y, 'theta': theta}
        else:
            missing = [f for f in ('x', 'y') if f not in data]
            if missing:
                raise ApiError(400, 'missing_field',
                               'Provide either "waypoint" or both "x" and "y"',
                               {'missing': missing})
            x, y = float(data['x']), float(data['y'])
            theta = float(data.get('theta', 0.0))
            target = {'x': x, 'y': y, 'theta': theta}

        if TRACKER.state == 'active' and not data.get('replace', False):
            raise ApiError(409, 'goal_active',
                           'A navigation goal is already running. Cancel it, or '
                           'resend with {"replace": true}.',
                           {'current': TRACKER.snapshot()})
        if TRACKER.state == 'active' and TRACKER.handle is not None:
            await ros_future(TRACKER.handle.cancel_goal_async(), timeout=5.0)

        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(theta / 2.0)
        pose.pose.orientation.w = math.cos(theta / 2.0)
        goal.pose = pose

        handle = await send_goal(goal=goal, action_client=self.bridge.act_navigate,
                                 name='navigation')
        TRACKER.begin(handle, target)

        # Resolve the result in the background so the HTTP call returns as soon
        # as the goal is accepted. Callers poll /navigation/status or watch the
        # event stream; holding the request open for a multi-minute drive would
        # tie up a connection and hit every proxy timeout in between.
        async def _await_result() -> None:
            try:
                wrapped = await ros_future(handle.get_result_async(), timeout=3600.0)
                status = getattr(wrapped, 'status', None)
                if status == 4:
                    TRACKER.finish('succeeded')
                elif status == 5:
                    TRACKER.finish('canceled', 'Goal was canceled')
                else:
                    TRACKER.finish('failed', f'Navigation ended with status {status}')
            except Exception as exc:  # noqa: BLE001
                TRACKER.finish('failed', str(exc))

        self.opts['spawn'](_await_result())
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


class CancelHandler(BaseHandler):
    async def delete(self) -> None:
        if TRACKER.state != 'active' or TRACKER.handle is None:
            self.send({'canceled': False, 'reason': 'no active goal'})
            return
        await ros_future(TRACKER.handle.cancel_goal_async(), timeout=5.0)
        TRACKER.finish('canceled', 'Canceled by API request')
        self.send({'canceled': True})


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
