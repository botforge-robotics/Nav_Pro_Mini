#!/usr/bin/env python3
"""Direct motion control (teleop-equivalent)."""

from __future__ import annotations

import math

from geometry_msgs.msg import Point
from nav2_msgs.action import DriveOnHeading, Spin

from .base import ApiError, BaseHandler
from .roscall import send_goal

# Clamps matching the robot's configured navigation limits. The API must not be
# a way around the speed limits the rest of the stack respects — a caller
# asking for 5 m/s gets a clear 400, not a robot that tries.
MAX_LINEAR = 0.35
MAX_ANGULAR = 1.2


def _clamped(value: float, limit: float, field: str) -> float:
    try:
        v = float(value)
    except (TypeError, ValueError):
        raise ApiError(400, 'invalid_field', f'{field} must be a number')
    if not math.isfinite(v):
        raise ApiError(400, 'invalid_field', f'{field} must be finite')
    if abs(v) > limit:
        raise ApiError(400, 'out_of_range',
                       f'{field} must be within +/-{limit}',
                       {'field': field, 'limit': limit, 'given': v})
    return v


class VelocityHandler(BaseHandler):
    """Continuous velocity command.

    Published on cmd_vel_teleop, so twist_mux applies the same priority and
    timeout as the joystick: the command expires after ~0.5s of silence. That
    is deliberate — an HTTP client that crashes mid-drive must not leave the
    robot moving, so callers repeat this call at ~5Hz to keep moving.
    """

    def post(self) -> None:
        data = self.body()
        linear = _clamped(data.get('linear', 0.0), MAX_LINEAR, 'linear')
        angular = _clamped(data.get('angular', 0.0), MAX_ANGULAR, 'angular')
        self.bridge.publish_cmd_vel(linear, angular)
        self.send({'linear': linear, 'angular': angular,
                   'expires_in_sec': 0.5})


class StopHandler(BaseHandler):
    def post(self) -> None:
        self.bridge.publish_cmd_vel(0.0, 0.0)
        self.send({'stopped': True})


class MoveHandler(BaseHandler):
    """Drive a fixed distance and stop. Negative distance reverses."""

    async def post(self) -> None:
        data = self.body(('distance',))
        distance = _clamped(data.get('distance'), 5.0, 'distance')
        speed = abs(_clamped(data.get('speed', 0.1), MAX_LINEAR, 'speed')) or 0.1

        goal = DriveOnHeading.Goal()
        goal.target = Point(x=abs(distance), y=0.0, z=0.0)
        goal.speed = speed
        allowance = abs(distance) / speed * 4.0 + 10.0
        goal.time_allowance.sec = int(allowance)

        action = self.bridge.act_drive
        await send_goal(goal=goal, action_client=action, name='drive_on_heading')
        self.send({'accepted': True, 'distance': distance, 'speed': speed})


class RotateHandler(BaseHandler):
    """Rotate in place by an angle in radians. Positive is counter-clockwise."""

    async def post(self) -> None:
        data = self.body(('angle',))
        angle = _clamped(data.get('angle'), 2 * math.pi, 'angle')
        goal = Spin.Goal()
        goal.target_yaw = angle
        goal.time_allowance.sec = int(abs(angle) / 0.3 * 4.0 + 10.0)
        await send_goal(goal=goal, action_client=self.bridge.act_spin, name='spin')
        self.send({'accepted': True, 'angle': angle})
