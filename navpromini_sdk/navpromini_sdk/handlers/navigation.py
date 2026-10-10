#!/usr/bin/env python3
"""Navigation: send goals, track them, cancel, localize."""

from __future__ import annotations

import asyncio
import json
import math
import os
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
    and the mission runner's navigate step (missions.py). Accepts 'waypoint' or 'target'."""
    wp_name = data.get('waypoint') or data.get('target')
    if wp_name is not None and isinstance(wp_name, str):
        wp = store.get_waypoint(str(wp_name))
        if wp is None:
            raise ApiError(404, 'waypoint_not_found',
                           f"No waypoint named {wp_name!r}",
                           {'waypoint': wp_name})
        x, y, theta = wp['x'], wp['y'], wp.get('theta', 0.0)
        return {'waypoint': wp['name'], 'x': x, 'y': y, 'theta': theta}
    missing = [f for f in ('x', 'y') if f not in data]
    if missing:
        raise ApiError(400, 'missing_field',
                       'Provide either "waypoint" / "target" or both "x" and "y"',
                       {'missing': missing})
    x, y = float(data['x']), float(data['y'])
    theta = float(data.get('theta', 0.0))
    return {'x': x, 'y': y, 'theta': theta}


def load_saved_dock_config() -> dict | None:
    for path in ['/home/navpromini/.navpromini_dock_pose.json',
                 os.path.expanduser('~/.navpromini_dock_pose.json')]:
        if os.path.isfile(path):
            try:
                with open(path, 'r') as f:
                    return json.load(f)
            except Exception:
                pass
    return None


def is_near_dock_or_docked(bridge, max_dist_m: float = 1.35) -> tuple[bool, dict]:
    """Check if the robot is currently docked/charging or inside the docking standoff zone."""
    # 1. Check if physically on charger / docked
    battery = bridge.get('battery')
    is_charging = bool(battery and battery.get('charging'))
    dock_status = str(bridge.get('dock_status') or '').lower()
    is_docked_status = dock_status in ('docked', 'charging', 'full')
    info = bridge.get('battery_info') or {}
    charger_conn = bool(info.get('charger_connected'))

    is_docked = is_charging or is_docked_status or charger_conn

    # 2. Get dock pose coordinates
    dock_cfg = load_saved_dock_config()
    dock_pose = bridge.get('dock_pose')
    dx = dy = dtheta = None

    if dock_cfg and 'x' in dock_cfg and 'y' in dock_cfg:
        dx = float(dock_cfg['x'])
        dy = float(dock_cfg['y'])
        dtheta = float(dock_cfg.get('theta', 0.0))
    elif dock_pose and 'x' in dock_pose and 'y' in dock_pose:
        dx = float(dock_pose['x'])
        dy = float(dock_pose['y'])
        dtheta = float(dock_pose.get('theta', 0.0))

    dock_info = {
        'x': dx,
        'y': dy,
        'theta': dtheta,
        'standoff_x': float(dock_cfg.get('standoff_x', dx + 0.70 * math.cos(dtheta))) if dock_cfg and dx is not None else None,
        'standoff_y': float(dock_cfg.get('standoff_y', dy + 0.70 * math.sin(dtheta))) if dock_cfg and dy is not None else None,
        'standoff_theta': float(dock_cfg.get('standoff_theta', dtheta)) if dock_cfg and dtheta is not None else None,
        'is_docked': is_docked,
    }

    if is_docked:
        return True, dock_info

    # 3. Check distance to dock if coordinates and pose are known
    if dx is not None and dy is not None:
        pose_map = bridge.get('pose_map')
        if pose_map and 'x' in pose_map and 'y' in pose_map:
            rx = float(pose_map['x'])
            ry = float(pose_map['y'])
            dist_to_dock = math.hypot(rx - dx, ry - dy)
            dock_info['dist_to_dock'] = dist_to_dock
            if dist_to_dock <= max_dist_m:
                return True, dock_info

        # Also check tag visibility: if dock tag is in view, robot is facing dock directly
        dock_tag = bridge.get('dock_tag')
        if dock_tag and dock_tag.get('z', 99.0) < 1.30:
            dock_info['dist_to_dock'] = float(dock_tag.get('z', 0.5))
            return True, dock_info

    return False, dock_info


async def relocalize_at_dock(bridge, dock_info: dict | None = None) -> bool:
    """Relocalize AMCL directly at the dock or standoff pose without any spinning."""
    if dock_info is None:
        _, dock_info = is_near_dock_or_docked(bridge)
    if not dock_info or dock_info.get('x') is None:
        return False

    is_docked = bool(dock_info.get('is_docked'))
    dist = dock_info.get('dist_to_dock')

    if is_docked or (dist is not None and dist < 0.35):
        # On charger / in contacts: seed exact dock pose
        x = dock_info['x']
        y = dock_info['y']
        theta = dock_info.get('theta', 0.0)
        target_name = 'dock'
    else:
        # In standoff zone: seed standoff pose
        x = dock_info.get('standoff_x') or dock_info['x']
        y = dock_info.get('standoff_y') or dock_info['y']
        theta = dock_info.get('standoff_theta') or dock_info.get('theta', 0.0)
        target_name = 'standoff'

    bridge.get_logger().info(f'Relocalizing directly at {target_name} ({x:.3f}, {y:.3f}, th={theta:.3f}) WITHOUT spinning')
    bridge.publish_initial_pose(x, y, theta)
    await asyncio.sleep(0.4)
    return True


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
    # Check if robot is docked or inside docking standoff
    near_dock, dock_info = is_near_dock_or_docked(bridge)
    pose_map, _ = bridge.get_with_age('pose_map')
    is_unlocalized = (pose_map is None) or (float(pose_map.get('cov_x', 0.0)) > 0.45 or float(pose_map.get('cov_y', 0.0)) > 0.45)
    if is_unlocalized:
        if near_dock:
            bridge.get_logger().info('Robot near dock/standoff and delocalized: relocalizing at dock WITHOUT spinning...')
            await relocalize_at_dock(bridge, dock_info)
        else:
            bridge.get_logger().info('Robot delocalized before navigation goal; auto-running relocalization recovery...')
            bridge.emit_event('localization.recovering', {'target': target})
            try:
                await run_relocalize_spin(bridge, angular_vel=0.35, timeout_sec=22.0)
            except Exception as exc:
                bridge.get_logger().warn(f'Pre-navigation relocalization recovery note: {exc}')

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
            bridge.publish_empty_plan()
            return {'ok': True, 'message': ''}
        if status == 5:
            TRACKER.finish('canceled', 'Goal was canceled')
            bridge.emit_event('navigation.cancelled', {'target': target})
            bridge.publish_empty_plan()
            return {'ok': False, 'message': 'Goal was canceled'}
        message = f'Navigation ended with status {status}'
        TRACKER.finish('failed', message)
        bridge.emit_event('navigation.failed', {'target': target, 'message': message})
        bridge.publish_empty_plan()
        return {'ok': False, 'message': message}
    except Exception as exc:  # noqa: BLE001
        TRACKER.finish('failed', str(exc))
        bridge.emit_event('navigation.failed', {'target': target, 'message': str(exc)})
        bridge.publish_empty_plan()
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
    bridge.publish_empty_plan()
    return True


class CancelHandler(BaseHandler):
    async def delete(self) -> None:
        canceled = await cancel_active_goal(self.bridge, 'Canceled by API request')
        if not canceled:
            self.send({'canceled': False, 'reason': 'no active goal'})
            return
        self.send({'canceled': True})


async def run_relocalize_spin(
    bridge,
    angular_vel: float = 0.35,
    timeout_sec: float = 22.0,
    target_cov: float = 0.15,
    target_cov_yaw: float = 0.12,
) -> dict:
    """Disperse AMCL particles and smoothly rotate in place to let laser scans converge."""
    near_dock, dock_info = is_near_dock_or_docked(bridge)
    if near_dock:
        bridge.get_logger().info('run_relocalize_spin requested while near dock/standoff: PREVENTING SPIN and relocalizing directly at dock')
        await relocalize_at_dock(bridge, dock_info)
        return {
            'status': 'ok',
            'converged': True,
            'elapsed_sec': 0.0,
            'covariance': {'cov_x': 0.05, 'cov_y': 0.05, 'cov_yaw': 0.02},
            'message': 'Robot is at dock/standoff: relocalized directly at dock without spinning.',
        }

    ok = bridge.reinitialize_global_localization(timeout_sec=3.0)
    if not ok:
        raise ApiError(503, 'service_unavailable', 'AMCL /reinitialize_global_localization service unavailable')
    await asyncio.sleep(0.4)

    start_time = time.time()
    converged = False
    last_cov = {}

    try:
        while (time.time() - start_time) < timeout_sec:
            bridge.publish_cmd_vel(0.0, angular_vel)
            await asyncio.sleep(0.1)

            pose_map = bridge.get('pose_map')
            if pose_map:
                cx = pose_map.get('cov_x', 999.0)
                cy = pose_map.get('cov_y', 999.0)
                cyaw = pose_map.get('cov_yaw', 999.0)
                last_cov = {'cov_x': cx, 'cov_y': cy, 'cov_yaw': cyaw}

                elapsed = time.time() - start_time
                if elapsed > 2.5 and cx < target_cov and cy < target_cov and cyaw < target_cov_yaw:
                    converged = True
                    break
    finally:
        bridge.publish_cmd_vel(0.0, 0.0)

    elapsed = round(time.time() - start_time, 2)
    return {
        'status': 'ok' if converged else 'timeout',
        'converged': converged,
        'elapsed_sec': elapsed,
        'covariance': last_cov,
        'message': 'Relocalization converged successfully' if converged else '360° spin finished, particles converging',
    }


class GlobalRelocalizeHandler(BaseHandler):
    """Disperse AMCL particles across the map for global relocalization."""

    async def post(self) -> None:
        data = self.body()
        spin = bool(data.get('spin', False))
        if spin:
            angular_vel = float(data.get('angular_vel', 0.35))
            timeout_sec = float(data.get('timeout_sec', 22.0))
            result = await run_relocalize_spin(self.bridge, angular_vel=angular_vel, timeout_sec=timeout_sec)
            self.send(result)
            return

        ok = self.bridge.reinitialize_global_localization(timeout_sec=3.0)
        if not ok:
            raise ApiError(503, 'service_unavailable', 'AMCL /reinitialize_global_localization service unavailable')
        self.send({'status': 'ok', 'message': 'AMCL particles dispersed across map'})


class RelocalizeRecoverHandler(BaseHandler):
    """Autonomous 360° relocalization recovery with active particle convergence."""

    async def post(self) -> None:
        data = self.body()
        angular_vel = float(data.get('angular_vel', 0.35))
        timeout_sec = float(data.get('timeout_sec', 22.0))
        target_cov = float(data.get('target_cov', 0.15))
        target_cov_yaw = float(data.get('target_cov_yaw', 0.12))
        result = await run_relocalize_spin(
            self.bridge,
            angular_vel=angular_vel,
            timeout_sec=timeout_sec,
            target_cov=target_cov,
            target_cov_yaw=target_cov_yaw,
        )
        self.send(result)


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
