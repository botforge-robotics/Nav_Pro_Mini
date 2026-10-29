#!/usr/bin/env python3
"""Navigation: send goals, track them, cancel, localize."""

from __future__ import annotations

import asyncio
import json
import math
import os
import time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import (
    ComputePathToPose,
    ComputeRoute,
    FollowPath,
    NavigateThroughPoses,
    NavigateToPose,
)

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


def find_nearest_road_node(store, x: float, y: float, max_distance: float = 50.0) -> int | None:
    """Find the nearest road network node ID to (x, y) coordinates."""
    try:
        rn = store.get_road_network()
        if not rn or not rn.get('features'):
            from .road_network import read_roadnet_file
            cur_map = store.current_map() or 'default'
            rn = read_roadnet_file(cur_map)
        if not rn or not rn.get('features'):
            return None
        best_id = None
        best_dist = float('inf')
        for f in rn.get('features', []):
            if f.get('geometry', {}).get('type') == 'Point':
                coords = f.get('geometry', {}).get('coordinates', [])
                if len(coords) >= 2:
                    props = f.get('properties', {})
                    raw_id = props.get('id', f.get('id'))
                    if raw_id is None:
                        continue
                    try:
                        nid = int(str(raw_id).replace('node_', ''))
                    except (ValueError, TypeError):
                        continue
                    dist = math.hypot(float(coords[0]) - x, float(coords[1]) - y)
                    if dist < best_dist and dist <= max_distance:
                        best_dist = dist
                        best_id = nid
        return best_id
    except Exception:
        return None


def resolve_target(store, data: dict) -> dict:
    """Waypoint name, raw x/y/theta, or road network node ID -> a target dict. Shared by
    GotoHandler, RouteHandler, and the mission runner's navigate step (missions.py)."""
    target: dict = {
        'routing_mode': data.get('routing_mode', 'free_space'),
        'reroute_policy': data.get('reroute_policy', 'wait_then_reroute'),
        'wait_timeout_sec': float(data.get('wait_timeout_sec', 5.0)),
    }
    if 'goal_id' in data and data['goal_id'] is not None:
        raw_gid = str(data['goal_id']).replace('node_', '')
        try:
            target['goal_id'] = int(raw_gid)
        except (ValueError, TypeError):
            pass

    wp_name = data.get('waypoint') or data.get('target')
    if wp_name is not None and isinstance(wp_name, str):
        wp = store.get_waypoint(str(wp_name))
        if wp is None:
            raise ApiError(404, 'waypoint_not_found',
                           f"No waypoint named {wp_name!r}",
                           {'waypoint': wp_name})
        target['waypoint'] = wp['name']
        target['x'] = float(wp['x'])
        target['y'] = float(wp['y'])
        target['theta'] = float(wp.get('theta', 0.0))
        if target.get('routing_mode') == 'road_network' and target.get('goal_id') is None:
            nearest_id = find_nearest_road_node(store, target['x'], target['y'])
            if nearest_id is not None:
                target['goal_id'] = nearest_id
        return target

    if target.get('routing_mode') == 'road_network' and target.get('goal_id') is not None:
        if 'x' not in target or 'y' not in target:
            try:
                rn = store.get_road_network() if store else None
                if not rn:
                    from .road_network import read_roadnet_file
                    cur_map = (store.current_map() if store else None) or 'f202'
                    rn = read_roadnet_file(cur_map)
                if rn and 'features' in rn:
                    for f in rn['features']:
                        geom = f.get('geometry', {})
                        props = f.get('properties', {})
                        f_id = f.get('id')
                        props_id = props.get('id')
                        gid = target['goal_id']
                        matches = (
                            f_id == gid or
                            f_id == f'node_{gid}' or
                            props_id == gid or
                            props.get('name') == f'Node {gid}' or
                            str(f_id) == str(gid) or
                            str(props_id) == str(gid)
                        )
                        if matches and geom.get('type') == 'Point':
                            coords = geom.get('coordinates', [])
                            if len(coords) >= 2:
                                target['x'] = float(coords[0])
                                target['y'] = float(coords[1])
                                target['theta'] = float(props.get('theta', 0.0))
                                break
            except Exception:
                pass
        return target

    missing = [f for f in ('x', 'y') if f not in data]
    if missing:
        raise ApiError(400, 'missing_field',
                       'Provide either "waypoint" / "target", "goal_id", or both "x" and "y"',
                       {'missing': missing})
    target['x'] = float(data['x'])
    target['y'] = float(data['y'])
    target['theta'] = float(data.get('theta', 0.0))
    if target.get('routing_mode') == 'road_network' and target.get('goal_id') is None:
        nearest_id = find_nearest_road_node(store, target['x'], target['y'])
        if nearest_id is not None:
            target['goal_id'] = nearest_id
    return target


def load_saved_dock_config() -> dict | None:
    candidates = [
        '/home/navpromini/.navpromini_dock_pose.json',
        '/root/.navpromini_dock_pose.json',
        os.path.expanduser('~/.navpromini_dock_pose.json'),
    ]
    existing = [p for p in set(candidates) if os.path.isfile(p)]
    existing.sort(key=lambda p: os.path.getmtime(p), reverse=True)
    for path in existing:
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
        'standoff_x': (dx + 0.70 * math.cos(dtheta)) if dx is not None else None,
        'standoff_y': (dy + 0.70 * math.sin(dtheta)) if dy is not None else None,
        'standoff_theta': dtheta if dx is not None else None,
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


async def _watch_wait_and_reroute(bridge, handle, timeout_sec: float = 5.0):
    """Monitors if robot is blocked/stationary while road route tracking is active.
    If stationary for >= timeout_sec, triggers RouteServer ReroutingService."""
    stationary_start = None
    try:
        while TRACKER.state == 'active' and TRACKER.handle is handle:
            await asyncio.sleep(0.5)
            vel = bridge.get('velocity') or {}
            vx = abs(float(vel.get('linear_x', vel.get('linear', 0.0))))
            wz = abs(float(vel.get('angular_z', vel.get('angular', 0.0))))
            is_stopped = (vx < 0.02 and wz < 0.04)

            if is_stopped:
                now = time.time()
                if stationary_start is None:
                    stationary_start = now
                elif now - stationary_start >= timeout_sec:
                    bridge.get_logger().info(f'Robot stopped on road for {timeout_sec:.1f}s; requesting RouteServer reroute...')
                    bridge.emit_event('navigation.rerouting', {'reason': 'obstacle_wait_timeout', 'timeout_sec': timeout_sec})
                    try:
                        from std_srvs.srv import Trigger
                        req = Trigger.Request()
                        await call_service(bridge.cli_reroute, req, 'reroute', timeout=3.0)
                    except Exception as exc:
                        bridge.get_logger().warn(f'Reroute trigger call note: {exc}')
                    # Backoff so we do not spam reroute requests every 0.5s
                    stationary_start = time.time() + 6.0
            else:
                stationary_start = None
    except Exception:
        pass


async def send_route_goal(bridge, target: dict, store=None):
    """Send a Road Network goal: plans via RouteServer's ComputeRoute,
    stitches first-mile (free space -> entry node) and last-mile paths if needed,
    and executes dense trajectory following along the road network via FollowPath."""
    # 1. Undock if docked
    near_dock, dock_info = is_near_dock_or_docked(bridge)
    if dock_info.get('is_docked'):
        bridge.get_logger().info('Robot is currently docked; automatically undocking before road route...')
        try:
            from .docking import undock_robot
            undock_res = await undock_robot(bridge, timeout=60.0)
            if not undock_res.get('ok'):
                bridge.get_logger().warn(f"Auto-undock note: {undock_res.get('message')}")
        except Exception as exc:
            bridge.get_logger().warn(f"Auto-undock before road route encountered error: {exc}")

    # 2. Localization check
    pose_map, _ = bridge.get_with_age('pose_map')
    is_unlocalized = (pose_map is None) or (float(pose_map.get('cov_x', 0.0)) > 0.45 or float(pose_map.get('cov_y', 0.0)) > 0.45)
    if is_unlocalized:
        if near_dock:
            bridge.get_logger().info('Robot near dock and delocalized: relocalizing at dock...')
            await relocalize_at_dock(bridge, dock_info)
        else:
            bridge.get_logger().info('Robot delocalized before road route; auto-running relocalization recovery...')
            bridge.emit_event('localization.recovering', {'target': target})
            try:
                await run_relocalize_spin(bridge, angular_vel=0.35, timeout_sec=22.0)
            except Exception as exc:
                bridge.get_logger().warn(f'Pre-route relocalization recovery note: {exc}')

    # 3. Configure reroute on collision parameter
    policy = target.get('reroute_policy', 'wait_then_reroute')
    if policy == 'immediate':
        bridge.set_route_reroute_on_collision(True)
    else:
        bridge.set_route_reroute_on_collision(False)

    # 4. Ensure road network graph is loaded in RouteServer
    try:
        from .road_network import load_route_graph_into_route_server
        cur_map = bridge.get('current_map') or (store.current_map() if store else None) or 'f202'
        await load_route_graph_into_route_server(bridge, cur_map, store)
    except Exception as exc:
        bridge.get_logger().warn(f'Ensure road graph before route: {exc}')

    # 5. Plan the route along the road network using RouteServer's ComputeRoute
    if 'x' not in target or 'y' not in target:
        raise ApiError(400, 'missing_field', 'Road network navigation requires valid goal_id or destination coordinates (x, y)')

    tx = float(target['x'])
    ty = float(target['y'])
    target_theta = float(target.get('theta', 0.0))

    compute_goal = ComputeRoute.Goal()
    compute_goal.use_poses = True
    compute_goal.use_start = False
    compute_goal.goal.header.frame_id = 'map'
    compute_goal.goal.pose.position.x = tx
    compute_goal.goal.pose.position.y = ty
    compute_goal.goal.pose.orientation.z = math.sin(target_theta / 2.0)
    compute_goal.goal.pose.orientation.w = math.cos(target_theta / 2.0)

    try:
        route_handle = await send_goal(goal=compute_goal, action_client=bridge.act_compute_route, name='compute_route')
        route_res = await ros_future(route_handle.get_result_async(), timeout=10.0)
    except Exception as exc:
        bridge.get_logger().warn(f'Road network ComputeRoute failed ({exc}); falling back to free space navigation.')
        fb_target = dict(target)
        fb_target['routing_mode'] = 'free_space'
        return await send_navigate_goal(bridge, fb_target, store=store)

    if (not route_res or getattr(route_res, 'status', None) != 4 or
            getattr(route_res.result, 'error_code', 0) != 0 or
            not getattr(route_res.result, 'path', None) or
            not route_res.result.path.poses):
        err_code = getattr(route_res.result, 'error_code', 'unknown') if (route_res and hasattr(route_res, 'result')) else 'no_result'
        bridge.get_logger().warn(f'Road network routing unavailable (error_code={err_code}); falling back to free space navigation.')
        fb_target = dict(target)
        fb_target['routing_mode'] = 'free_space'
        return await send_navigate_goal(bridge, fb_target, store=store)

    route_path = route_res.result.path
    pose_map, _ = bridge.get_with_age('pose_map')
    rx = float(pose_map['x']) if (pose_map and 'x' in pose_map) else float(route_path.poses[0].pose.position.x)
    ry = float(pose_map['y']) if (pose_map and 'y' in pose_map) else float(route_path.poses[0].pose.position.y)

    first_route_pose = route_path.poses[0].pose.position
    dist_to_start = math.hypot(rx - first_route_pose.x, ry - first_route_pose.y)

    full_poses: list[PoseStamped] = list(route_path.poses)

    # 6. First-mile: if robot is in free space away from the road network entry (> 0.35m),
    # compute collision-free approach path from current pose to road network entry.
    if dist_to_start > 0.35:
        first_mile_goal = ComputePathToPose.Goal()
        first_mile_goal.use_start = False
        first_mile_goal.goal.header.frame_id = 'map'
        first_mile_goal.goal.header.stamp = bridge.get_clock().now().to_msg()
        first_mile_goal.goal.pose = route_path.poses[0].pose
        try:
            fm_handle = await send_goal(goal=first_mile_goal, action_client=bridge.act_compute_path_to_pose, name='first_mile_path')
            fm_res = await ros_future(fm_handle.get_result_async(), timeout=5.0)
            if fm_res and getattr(fm_res, 'status', None) == 4 and fm_res.result.path.poses:
                full_poses = list(fm_res.result.path.poses) + list(route_path.poses[1:])
                bridge.get_logger().info(f'Stitched first-mile ({len(fm_res.result.path.poses)} poses) with road network ({len(route_path.poses)} poses)')
        except Exception as exc:
            bridge.get_logger().warn(f'First-mile path computation note: {exc}')

    # 7. Last-mile: if destination target is away from the road network exit (> 0.35m),
    # compute collision-free path from road exit to final target coordinates.
    last_route_pose = full_poses[-1].pose.position
    dist_to_target = math.hypot(tx - last_route_pose.x, ty - last_route_pose.y)
    if dist_to_target > 0.35:
        last_mile_goal = ComputePathToPose.Goal()
        last_mile_goal.use_start = True
        last_mile_goal.start = full_poses[-1]
        last_mile_goal.start.header.frame_id = 'map'
        last_mile_goal.goal.header.frame_id = 'map'
        last_mile_goal.goal.header.stamp = bridge.get_clock().now().to_msg()
        last_mile_goal.goal.pose.position.x = tx
        last_mile_goal.goal.pose.position.y = ty
        last_mile_goal.goal.pose.orientation.z = math.sin(target_theta / 2.0)
        last_mile_goal.goal.pose.orientation.w = math.cos(target_theta / 2.0)
        try:
            lm_handle = await send_goal(goal=last_mile_goal, action_client=bridge.act_compute_path_to_pose, name='last_mile_path')
            lm_res = await ros_future(lm_handle.get_result_async(), timeout=5.0)
            if lm_res and getattr(lm_res, 'status', None) == 4 and lm_res.result.path.poses:
                full_poses = full_poses + list(lm_res.result.path.poses[1:])
                bridge.get_logger().info(f'Stitched last-mile ({len(lm_res.result.path.poses)} poses) to target')
        except Exception as exc:
            bridge.get_logger().warn(f'Last-mile path computation note: {exc}')

    # Ensure all poses have valid frame_id and timestamp
    now_msg = bridge.get_clock().now().to_msg()
    for p in full_poses:
        p.header.frame_id = 'map'
        p.header.stamp = now_msg

    # Apply target final orientation if explicitly provided
    if target_theta != 0.0:
        full_poses[-1].pose.orientation.z = math.sin(target_theta / 2.0)
        full_poses[-1].pose.orientation.w = math.cos(target_theta / 2.0)

    final_path = Path()
    final_path.header.frame_id = 'map'
    final_path.header.stamp = now_msg
    final_path.poses = full_poses

    # Publish plan to /plan immediately for frontend visualization
    bridge.publish_plan(final_path)

    # 8. FollowPath action execution along the dense road network
    follow_goal = FollowPath.Goal()
    follow_goal.path = final_path
    follow_goal.controller_id = 'FollowPath'
    follow_goal.goal_checker_id = 'general_goal_checker'
    follow_goal.progress_checker_id = 'progress_checker'

    handle = await send_goal(goal=follow_goal, action_client=bridge.act_follow_path, name='road_network_follow_path')

    TRACKER.begin(handle, target)
    bridge.emit_event('navigation.started', {
        'target': target,
        'routing_mode': 'road_network',
        'path_poses_count': len(final_path.poses),
        'reroute_policy': policy,
    })

    if policy == 'wait_then_reroute':
        timeout_sec = float(target.get('wait_timeout_sec', 5.0))
        asyncio.create_task(_watch_wait_and_reroute(bridge, handle, timeout_sec))

    return handle


async def send_navigate_goal(bridge, target: dict, store=None):
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
    if target.get('routing_mode') == 'road_network':
        return await send_route_goal(bridge, target, store=store)

    x, y, theta = target['x'], target['y'], target.get('theta', 0.0)
    # Check if robot is docked or inside docking standoff
    near_dock, dock_info = is_near_dock_or_docked(bridge)
    if dock_info.get('is_docked'):
        bridge.get_logger().info('Robot is currently docked on charger; automatically undocking before sending navigation goal...')
        try:
            from .docking import undock_robot
            undock_res = await undock_robot(bridge, timeout=60.0)
            if not undock_res.get('ok'):
                bridge.get_logger().warn(f"Auto-undock note: {undock_res.get('message')}")
        except Exception as exc:
            bridge.get_logger().warn(f"Auto-undock before nav goal encountered error: {exc}")

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

        handle = await send_navigate_goal(self.bridge, target, store=self.opts['store'])

        # Resolve the result in the background so the HTTP call returns as soon
        # as the goal is accepted. Callers poll /navigation/status or watch the
        # event stream; holding the request open for a multi-minute drive would
        # tie up a connection and hit every proxy timeout in between.
        self.opts['spawn'](await_navigate_result(self.bridge, handle, target))
        self.send({'accepted': True, 'target': target}, status=202)


class RouteHandler(BaseHandler):
    """Send a Road Network goal explicitly."""

    async def post(self) -> None:
        data = self.body()
        data['routing_mode'] = 'road_network'
        target = resolve_target(self.opts['store'], data)

        if TRACKER.state == 'active' and not data.get('replace', False):
            raise ApiError(409, 'goal_active',
                           'A navigation goal is already running. Cancel it, or '
                           'resend with {"replace": true}.',
                           {'current': TRACKER.snapshot()})
        if TRACKER.state == 'active' and TRACKER.handle is not None:
            await ros_future(TRACKER.handle.cancel_goal_async(), timeout=5.0)

        handle = await send_route_goal(self.bridge, target, store=self.opts['store'])
        self.opts['spawn'](await_navigate_result(self.bridge, handle, target))
        self.send({'accepted': True, 'target': target, 'routing_mode': 'road_network'}, status=202)


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
    angular_vel: float = 0.38,
    max_tries: int = 3,
    timeout_sec: float = 50.0,
    target_cov: float = 0.15,
    target_cov_yaw: float = 0.12,
) -> dict:
    """Disperse AMCL particles and autonomously rotate in place across multiple scan tries until converged."""
    near_dock, dock_info = is_near_dock_or_docked(bridge)
    if dock_info.get('is_docked'):
        bridge.get_logger().info('run_relocalize_spin: robot is physically docked on charger, automatically undocking before global scan...')
        try:
            from .docking import undock_robot
            undock_res = await undock_robot(bridge, timeout=30.0)
            if not undock_res.get('ok'):
                bridge.get_logger().warn(f"Auto-undock note: {undock_res.get('message')}")
        except Exception as exc:
            bridge.get_logger().warn(f'Auto-undock before global scan failed: {exc}')

    ok = bridge.reinitialize_global_localization(timeout_sec=3.0)
    if not ok:
        raise ApiError(503, 'service_unavailable', 'AMCL /reinitialize_global_localization service unavailable')
    await asyncio.sleep(0.5)

    start_time = time.time()
    converged = False
    last_cov = {}
    current_try = 1

    # Duration of one 360-degree rotation (2*pi / angular_vel)
    abs_speed = max(0.15, abs(angular_vel))
    cycle_duration = (2.0 * math.pi) / abs_speed  # ~16.5 seconds per 360-deg rotation

    # Alternating direction pattern: CCW (+), CW (-), CCW (+)
    directions = [1.0, -1.0, 1.0]

    bridge.get_logger().info(
        f'Starting autonomous relocalization: max_tries={max_tries}, speed={abs_speed:.2f} rad/s, cycle_duration={cycle_duration:.1f}s'
    )

    try:
        for try_idx in range(max(1, max_tries)):
            current_try = try_idx + 1
            direction = directions[try_idx % len(directions)]
            target_w = direction * abs_speed
            cycle_start = time.time()

            bridge.get_logger().info(
                f'Relocalization scan cycle {current_try}/{max_tries} started (w={target_w:+.2f} rad/s)'
            )

            while (time.time() - cycle_start) < cycle_duration:
                if (time.time() - start_time) >= timeout_sec:
                    break

                bridge.publish_cmd_vel(0.0, target_w)
                await asyncio.sleep(0.1)

                pose_map = bridge.get('pose_map')
                if pose_map:
                    cx = float(pose_map.get('cov_x', 999.0))
                    cy = float(pose_map.get('cov_y', 999.0))
                    cyaw = float(pose_map.get('cov_yaw', 999.0))
                    last_cov = {'cov_x': cx, 'cov_y': cy, 'cov_yaw': cyaw}

                    elapsed = time.time() - start_time
                    if elapsed > 2.5 and cx < target_cov and cy < target_cov and cyaw < target_cov_yaw:
                        converged = True
                        bridge.get_logger().info(
                            f'AMCL converged successfully on try {current_try}/{max_tries} in {elapsed:.1f}s: '
                            f'cov_x={cx:.3f}, cov_y={cy:.3f}, cov_yaw={cyaw:.3f}'
                        )
                        break

            if converged:
                break

            # Brief deceleration before reversing direction
            if current_try < max_tries and (time.time() - start_time) < timeout_sec:
                bridge.publish_cmd_vel(0.0, 0.0)
                await asyncio.sleep(0.4)

    finally:
        bridge.publish_cmd_vel(0.0, 0.0)

    elapsed = round(time.time() - start_time, 2)
    return {
        'status': 'ok' if converged else 'timeout',
        'converged': converged,
        'tries': current_try,
        'elapsed_sec': elapsed,
        'covariance': last_cov,
        'message': f'Relocalized successfully in {elapsed}s ({current_try} scan cycles). AMCL converged.'
                   if converged else
                   f'Relocalization timed out after {current_try} scan cycles ({elapsed}s).',
    }


class GlobalRelocalizeHandler(BaseHandler):
    """Disperse AMCL particles across the map and autonomously recover position."""

    async def post(self) -> None:
        data = self.body()
        spin = bool(data.get('spin', True))
        if spin:
            angular_vel = float(data.get('angular_vel', 0.38))
            max_tries = int(data.get('max_tries', 3))
            timeout_sec = float(data.get('timeout_sec', 50.0))
            target_cov = float(data.get('target_cov', 0.15))
            target_cov_yaw = float(data.get('target_cov_yaw', 0.12))
            result = await run_relocalize_spin(
                self.bridge,
                angular_vel=angular_vel,
                max_tries=max_tries,
                timeout_sec=timeout_sec,
                target_cov=target_cov,
                target_cov_yaw=target_cov_yaw,
            )
            self.send(result)
            return

        ok = self.bridge.reinitialize_global_localization(timeout_sec=3.0)
        if not ok:
            raise ApiError(503, 'service_unavailable', 'AMCL /reinitialize_global_localization service unavailable')
        self.send({'status': 'ok', 'message': 'AMCL particles dispersed across map'})


class RelocalizeRecoverHandler(BaseHandler):
    """Autonomous relocalization recovery with active multi-try particle convergence."""

    async def post(self) -> None:
        data = self.body()
        angular_vel = float(data.get('angular_vel', 0.38))
        max_tries = int(data.get('max_tries', 3))
        timeout_sec = float(data.get('timeout_sec', 50.0))
        target_cov = float(data.get('target_cov', 0.15))
        target_cov_yaw = float(data.get('target_cov_yaw', 0.12))
        result = await run_relocalize_spin(
            self.bridge,
            angular_vel=angular_vel,
            max_tries=max_tries,
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
