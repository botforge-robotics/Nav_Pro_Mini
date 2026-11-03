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
MAX_LINEAR = 0.50
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


def apply_live_motion_parameters(node, settings: dict) -> None:
    """Asynchronously update active ROS 2 nodes with dynamic motion parameters."""
    if node is None:
        return
    try:
        from rcl_interfaces.msg import Parameter as RclParam, ParameterType, ParameterValue
        from rcl_interfaces.srv import SetParameters
    except ImportError:
        return

    def p_double(name: str, val: float) -> RclParam:
        return RclParam(
            name=name,
            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(val)),
        )

    def p_double_array(name: str, vals: list) -> RclParam:
        return RclParam(
            name=name,
            value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                double_array_value=[float(x) for x in vals],
            ),
        )

    max_lin = float(settings.get('max_linear_speed', 0.35))
    min_lin = float(settings.get('min_linear_speed', 0.05))
    max_ang = float(settings.get('max_angular_speed', 0.30))
    min_ang = float(settings.get('min_angular_speed', 0.10))
    acc_lin = float(settings.get('linear_accel', 0.088))
    dec_lin = float(settings.get('linear_decel', -0.175))
    acc_ang = float(settings.get('angular_accel', 0.50))
    dec_ang = -1.0 * abs(acc_ang) * 1.4

    # 1. velocity_smoother
    try:
        vs_client = node.create_client(SetParameters, '/velocity_smoother/set_parameters')
        if vs_client.service_is_ready() or vs_client.wait_for_service(timeout_sec=0.5):
            req = SetParameters.Request()
            req.parameters = [
                p_double_array('max_velocity', [max_lin, 0.0, max_ang]),
                p_double_array('min_velocity', [-0.08, 0.0, -max_ang]),
                p_double_array('max_accel', [acc_lin, 0.0, acc_ang]),
                p_double_array('max_decel', [dec_lin, 0.0, dec_ang]),
            ]
            vs_client.call_async(req)
    except Exception as e:
        node.get_logger().debug(f'Could not set velocity_smoother params: {e}')

    # 2. controller_server
    try:
        cs_client = node.create_client(SetParameters, '/controller_server/set_parameters')
        if cs_client.service_is_ready() or cs_client.wait_for_service(timeout_sec=0.5):
            req = SetParameters.Request()
            req.parameters = [
                p_double('FollowPath.desired_linear_vel', max_lin),
                p_double('FollowPath.min_approach_linear_velocity', min_lin),
                p_double('FollowPath.rotate_to_heading_angular_vel', max_ang),
                p_double('FollowPath.max_angular_accel', acc_ang),
            ]
            cs_client.call_async(req)
    except Exception as e:
        node.get_logger().debug(f'Could not set controller_server params: {e}')

    # 3. behavior_server
    try:
        bs_client = node.create_client(SetParameters, '/behavior_server/set_parameters')
        if bs_client.service_is_ready() or bs_client.wait_for_service(timeout_sec=0.5):
            req = SetParameters.Request()
            req.parameters = [
                p_double('max_rotational_vel', max_ang),
                p_double('min_rotational_vel', min_ang),
                p_double('rotational_acc_lim', acc_ang),
            ]
            bs_client.call_async(req)
    except Exception as e:
        node.get_logger().debug(f'Could not set behavior_server params: {e}')

    # 4. navpromini_dock_manager
    try:
        dm_client = node.create_client(SetParameters, '/navpromini_dock_manager/set_parameters')
        if dm_client.service_is_ready() or dm_client.wait_for_service(timeout_sec=0.5):
            req = SetParameters.Request()
            req.parameters = [
                p_double('undock_speed', float(settings.get('undock_speed', 0.13))),
                p_double('dock_speed', float(settings.get('dock_speed', 0.13))),
            ]
            dm_client.call_async(req)
    except Exception as e:
        node.get_logger().debug(f'Could not set dock_manager params: {e}')


class MotionSettingsHandler(BaseHandler):
    """Query or update motion and docking tuning parameters stored in robot SQLite database."""

    def get(self) -> None:
        try:
            from navpromini_controller.motion_config import (
                DEFAULT_MOTION_SETTINGS,
                SAFETY_LIMITS,
                get_motion_settings,
            )
            settings = get_motion_settings()
            defaults = DEFAULT_MOTION_SETTINGS
            limits = SAFETY_LIMITS
        except Exception as e:
            self.fail(500, 'motion_config_error', f'Could not read motion configuration: {e}')
            return

        self.send({
            'success': True,
            'settings': settings,
            'defaults': defaults,
            'safety_limits': limits,
        })

    def put(self) -> None:
        try:
            from navpromini_controller.motion_config import save_motion_settings
        except Exception as e:
            self.fail(500, 'motion_config_error', f'Could not load motion configuration: {e}')
            return

        data = self.body()
        try:
            saved = save_motion_settings(data)
            apply_live_motion_parameters(self.bridge, saved)
            self.send({
                'success': True,
                'settings': saved,
                'message': 'Motion parameters persisted to robot database and applied to live nodes.',
            })
        except Exception as e:
            self.fail(400, 'save_failed', f'Failed to save motion settings: {e}')


class MotionSettingsRestoreHandler(BaseHandler):
    """Restore all motion settings to their calibrated baseline defaults in the robot SQLite database."""

    def post(self) -> None:
        try:
            from navpromini_controller.motion_config import (
                DEFAULT_MOTION_SETTINGS,
                restore_default_motion_settings,
            )
            restored = restore_default_motion_settings()
            apply_live_motion_parameters(self.bridge, restored)
            self.send({
                'success': True,
                'settings': restored,
                'defaults': DEFAULT_MOTION_SETTINGS,
                'message': 'Motion settings restored to defaults in robot database.',
            })
        except Exception as e:
            self.fail(500, 'restore_failed', f'Failed to restore default motion settings: {e}')

