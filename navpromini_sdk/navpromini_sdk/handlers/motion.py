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
        if (abs(linear) > 0.001 or abs(angular) > 0.001) and self.bridge.is_in_dock_zone:
            raise ApiError(409, 'robot_in_dock_zone',
                           'Manual teleop is disabled in the dock zone to protect the dock. Please undock first.')
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
        if self.bridge.is_in_dock_zone:
            raise ApiError(409, 'robot_in_dock_zone',
                           'Manual motion is disabled in the dock zone to protect the dock. Please undock first.')
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
        if self.bridge.is_in_dock_zone:
            raise ApiError(409, 'robot_in_dock_zone',
                           'Manual rotation is disabled in the dock zone to protect the dock. Please undock first.')
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

    def p_bool(name: str, val: bool) -> RclParam:
        return RclParam(
            name=name,
            value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=bool(val)),
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

    def _get_client(attr_name: str, srv_name: str):
        cli = getattr(node, attr_name, None)
        if cli is None:
            cli = node.create_client(SetParameters, srv_name)
            setattr(node, attr_name, cli)
        return cli

    # 1. velocity_smoother
    try:
        vs_client = _get_client('_cli_vs_param', '/velocity_smoother/set_parameters')
        req = SetParameters.Request()
        req.parameters = [
            p_double_array('max_velocity', [max_lin, 0.0, max_ang]),
            p_double_array('min_velocity', [-0.10, 0.0, -max_ang]),
            p_double_array('max_accel', [acc_lin, 0.0, acc_ang]),
            p_double_array('max_decel', [dec_lin, 0.0, dec_ang]),
        ]
        vs_client.call_async(req)
    except Exception as e:
        node.get_logger().debug(f'Could not set velocity_smoother params: {e}')

    # 2. controller_server
    try:
        cs_client = _get_client('_cli_cs_param', '/controller_server/set_parameters')
        req = SetParameters.Request()
        req.parameters = [
            p_double('controller_frequency', 15.0),
            p_double('FollowPath.desired_linear_vel', max_lin),
            p_double('FollowPath.min_approach_linear_velocity', 0.12),
            p_double('FollowPath.approach_velocity_scaling_dist', 0.30),
            p_double('FollowPath.rotate_to_heading_angular_vel', max_ang),
            p_double('FollowPath.max_angular_accel', acc_ang),
            p_double('FollowPath.rotate_to_heading_min_angle', 0.85),
            p_double('FollowPath.lookahead_dist', 0.28),
            p_double('FollowPath.min_lookahead_dist', 0.20),
            p_double('FollowPath.max_lookahead_dist', 0.45),
            p_double('FollowPath.lookahead_time', 1.0),
            p_bool('FollowPath.use_fixed_curvature_lookahead', True),
            p_double('FollowPath.curvature_lookahead_dist', 0.35),
            p_double('FollowPath.regulated_linear_scaling_min_radius', 0.70),
            p_double('FollowPath.regulated_linear_scaling_min_speed', 0.12),
            p_bool('FollowPath.use_cost_regulated_linear_velocity_scaling', False),
            p_double('general_goal_checker.xy_goal_tolerance', 0.12),
            p_double('general_goal_checker.yaw_goal_tolerance', 0.25),
        ]
        cs_client.call_async(req)
    except Exception as e:
        node.get_logger().debug(f'Could not set controller_server params: {e}')

    # 3. route_server
    try:
        rs_client = _get_client('_cli_rs_param', '/route_server/set_parameters')
        req = SetParameters.Request()
        req.parameters = [
            p_double('radius_to_achieve_node', 0.35),
            p_double('boundary_radius_to_achieve_node', 0.25),
        ]
        rs_client.call_async(req)
    except Exception as e:
        node.get_logger().debug(f'Could not set route_server params: {e}')

    # 4. behavior_server
    try:
        bs_client = _get_client('_cli_bs_param', '/behavior_server/set_parameters')
        req = SetParameters.Request()
        req.parameters = [
            p_double('max_rotational_vel', max_ang),
            p_double('min_rotational_vel', min_ang),
            p_double('rotational_acc_lim', acc_ang),
        ]
        bs_client.call_async(req)
    except Exception as e:
        node.get_logger().debug(f'Could not set behavior_server params: {e}')

    # 5. navpromini_dock_manager
    try:
        dm_client = _get_client('_cli_dm_param', '/navpromini_dock_manager/set_parameters')
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
                SAFETY_LIMITS,
                get_default_motion_settings,
                get_motion_settings,
            )
            settings = get_motion_settings()
            defaults = get_default_motion_settings()
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
    """Restore all motion settings to their default speeds in the robot database."""

    def post(self) -> None:
        try:
            from navpromini_controller.motion_config import (
                get_default_motion_settings,
                restore_default_motion_settings,
            )
            restored = restore_default_motion_settings()
            defaults = get_default_motion_settings()
            apply_live_motion_parameters(self.bridge, restored)
            self.send({
                'success': True,
                'settings': restored,
                'defaults': defaults,
                'message': 'Motion settings restored to defaults.',
            })
        except Exception as e:
            self.fail(500, 'restore_failed', f'Failed to restore default motion settings: {e}')


class MotionSettingsSetDefaultHandler(BaseHandler):
    """Save present motion settings as the new default speeds in the robot database."""

    def post(self) -> None:
        try:
            from navpromini_controller.motion_config import (
                set_current_as_default_motion_settings,
            )
            defaults = set_current_as_default_motion_settings()
            self.send({
                'success': True,
                'defaults': defaults,
                'message': 'Present motion settings saved as default speeds.',
            })
        except Exception as e:
            self.fail(500, 'set_default_failed', f'Failed to save present settings as defaults: {e}')

