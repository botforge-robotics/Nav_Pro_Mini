#!/usr/bin/env python3
"""Mission Manager (doc §29): missions as a first-class, robot-owned
abstraction — `{id, name, steps: [...], loop_count, loop_forever}` — instead
of client-side-only orchestration. Step schema: navigate / wait / dock /
undock / call_service / call_action.

Steps reuse the exact same goal-sending code paths as the interactive
endpoints (navigate_to() from navigation.py, dock_robot()/undock_robot() from
docking.py) so a mission's `navigate` step behaves identically to a direct
POST /navigation/goto — same TRACKER, same events, same dock_manager
undock-first behaviour — rather than a second, subtly different
implementation. `call_service`/`call_action` are the generic escape hatch —
any ROS service or action by name+type — resolved via rosidl_runtime_py the
same way `ros2 service call`/`ros2 action send_goal` do; a bad type string is
rejected at *save* time (_validate_steps), not discovered mid-run.

One mission runs at a time (RUNNER is a module-level singleton), same
reasoning as _GoalTracker/_DockTracker in navigation.py/docking.py: the robot
can only actually do one thing.

Looping: `loop_count` (>=1, default 1) and `loop_forever` repeat the whole
step list, not an individual step — a failure or cancel ends the mission
outright rather than stranding a `loop_forever` mission on a permanently
broken step. RUNNER.snapshot()'s `loop_index`/`loop_total` report progress
across laps the same way `step_index` reports progress within one.

Pause/resume: there is no way to pause a NavigateToPose or DockRobot goal
mid-flight at the level this SDK operates on (that's a Nav2/BT concept, not
exposed here). "Pause" therefore cancels whatever step is in flight and holds
before starting the *same* step again on resume — the mission does not lose
its place, but a paused navigate restarts that leg rather than freezing in
place. Documented here rather than silently pretending otherwise.
"""

from __future__ import annotations

import asyncio
import json
import time
from typing import Any

from rclpy.action import ActionClient
from rosidl_runtime_py import set_message_fields
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_action, get_service

import os
import uuid
import requests
from .base import ApiError, BaseHandler
from .docking import dock_robot, undock_robot
from .mission_graph import (
    NODE_CATALOG,
    evaluate_condition_safely,
    resolve_template_value,
    validate_graph_mission,
)
from .navigation import cancel_active_goal, navigate_to
from .roscall import call_service, ros_future, send_goal

LOW_BATTERY_DOCK_PERCENT = float(os.environ.get('NAVPRO_LOW_BATT_DOCK_PCT', 5.0))
RESUME_BATTERY_PERCENT = float(os.environ.get('NAVPRO_RESUME_BATT_PCT', 95.0))

VALID_STEP_TYPES = ('navigate', 'wait', 'dock', 'undock', 'call_service', 'call_action', 'call_api')


def _validate_steps(steps: Any) -> list[dict]:
    if not isinstance(steps, list) or not steps:
        raise ApiError(400, 'invalid_field', 'steps must be a non-empty list')
    for i, step in enumerate(steps):
        if not isinstance(step, dict) or step.get('type') not in VALID_STEP_TYPES:
            raise ApiError(400, 'invalid_step',
                           f'step {i} must have "type" in {list(VALID_STEP_TYPES)}',
                           {'index': i})
        stype = step['type']
        if stype == 'navigate' and 'target' not in step and 'waypoint' not in step and not ('x' in step and 'y' in step):
            raise ApiError(400, 'invalid_step',
                           f'step {i}: navigate needs "waypoint" / "target" (a waypoint name) '
                           'or "x"/"y"', {'index': i})
        if stype == 'wait' and 'duration' not in step and 'duration_sec' not in step:
            raise ApiError(400, 'invalid_step', f'step {i}: wait needs "duration" or "duration_sec"',
                           {'index': i})
        if stype == 'call_service':
            if 'service' not in step or 'service_type' not in step:
                raise ApiError(400, 'invalid_step',
                               f'step {i}: call_service needs "service" and "service_type"',
                               {'index': i})
            # Resolved here, not just at run time — a typo'd type string should
            # fail the save, not surface days later when the mission actually runs.
            try:
                get_service(step['service_type'])
            except Exception as exc:  # noqa: BLE001
                raise ApiError(400, 'invalid_step',
                               f'step {i}: unknown service_type {step["service_type"]!r} '
                               f'({exc})', {'index': i})
        if stype == 'call_action':
            if ('action' not in step and 'action_name' not in step) or 'action_type' not in step:
                raise ApiError(400, 'invalid_step',
                               f'step {i}: call_action needs "action" (or "action_name") and "action_type"',
                               {'index': i})
            try:
                get_action(step['action_type'])
            except Exception as exc:  # noqa: BLE001
                raise ApiError(400, 'invalid_step',
                               f'step {i}: unknown action_type {step["action_type"]!r} '
                               f'({exc})', {'index': i})
        if stype == 'call_api':
            if 'url' not in step or not isinstance(step['url'], str) or not step['url'].strip():
                raise ApiError(400, 'invalid_step',
                               f'step {i}: call_api needs non-empty string "url"',
                               {'index': i})
            url = step['url'].strip()
            if not (url.startswith('http://') or url.startswith('https://')):
                raise ApiError(400, 'invalid_step',
                               f'step {i}: call_api "url" must start with http:// or https://',
                               {'index': i})
            method = step.get('method', 'POST')
            if not isinstance(method, str) or method.upper() not in ('GET', 'POST', 'PUT', 'DELETE', 'PATCH', 'HEAD'):
                raise ApiError(400, 'invalid_step',
                               f'step {i}: call_api invalid method {method!r}',
                               {'index': i})
            if 'headers' in step and step['headers'] is not None and not isinstance(step['headers'], dict):
                raise ApiError(400, 'invalid_step',
                               f'step {i}: call_api "headers" must be a dict',
                               {'index': i})
            timeout = step.get('timeout', step.get('timeout_sec'))
            if timeout is not None:
                try:
                    if float(timeout) <= 0:
                        raise ValueError()
                except (TypeError, ValueError):
                    raise ApiError(400, 'invalid_step',
                                   f'step {i}: call_api "timeout" must be positive number',
                                   {'index': i})
    return steps


def _validate_loop(data: dict) -> tuple[bool, int]:
    loop_forever = bool(data.get('loop_forever', False))
    loop_count = data.get('loop_count', 1)
    try:
        loop_count = int(loop_count)
    except (TypeError, ValueError):
        raise ApiError(400, 'invalid_field', 'loop_count must be an integer')
    if loop_count < 1:
        raise ApiError(400, 'invalid_field', 'loop_count must be >= 1')
    return loop_forever, loop_count


# Cached rclpy clients for call_service/call_action steps — created once per
# (type, name) pair and reused across mission runs and loop iterations,
# rather than leaking a new client every time a step executes.
_SERVICE_CLIENTS: dict[tuple[str, str], Any] = {}
_ACTION_CLIENTS: dict[tuple[str, str], Any] = {}


def _get_service_client(bridge, service_type: str, service_name: str):
    srv_cls = get_service(service_type)
    key = (service_type, service_name)
    client = _SERVICE_CLIENTS.get(key)
    if client is None:
        client = bridge.create_client(srv_cls, service_name, callback_group=bridge._cb)
        _SERVICE_CLIENTS[key] = client
    return client, srv_cls


def _get_action_client(bridge, action_type: str, action_name: str):
    action_cls = get_action(action_type)
    key = (action_type, action_name)
    client = _ACTION_CLIENTS.get(key)
    if client is None:
        client = ActionClient(bridge, action_cls, action_name, callback_group=bridge._cb)
        _ACTION_CLIENTS[key] = client
    return client, action_cls


class _MissionRunner:
    """State of the one mission that may be running right now."""

    def __init__(self) -> None:
        self.mission_id: str | None = None
        self.state = 'idle'   # idle | running | paused | completed | failed | canceled | waiting_for_user
        self.step_index = -1
        self.active_node_id: str | None = None
        self.active_node_type: str | None = None
        self.active_interaction: dict | None = None
        self.interaction_future: asyncio.Future | None = None
        self.context: dict = {}
        # loop_total is None for a loop_forever mission (there is no total to
        # report), and 1 for a plain, non-repeating mission — so a client can
        # tell "not looping" from "looping forever" from "lap 2 of 5" without
        # a separate flag.
        self.loop_index = 0
        self.loop_total: int | None = None
        self.message = ''
        self.started_at: float | None = None
        self.cancel_requested = False
        self.pause_requested = False
        self.pause_reason: str | None = None

    def snapshot(self) -> dict:
        return {
            'mission_id': self.mission_id,
            'state': self.state,
            'status': self.state,
            'step_index': self.step_index,
            'active_node_id': self.active_node_id,
            'active_node_type': self.active_node_type,
            'active_interaction': self.active_interaction,
            'loop_index': self.loop_index,
            'loop_total': self.loop_total,
            'message': self.message,
            'pause_reason': self.pause_reason,
            'context': self.context,
            'elapsed_sec': (round(time.time() - self.started_at, 1)
                            if self.started_at else None),
        }


RUNNER = _MissionRunner()


async def _run_step(bridge, store, step: dict) -> tuple[bool, str]:
    stype = step['type']
    if stype == 'navigate':
        target = step.get('waypoint') or step.get('target')
        if isinstance(target, str):
            wp = store.get_waypoint(target)
            if wp is None:
                return False, f'No waypoint named {target!r}'
            target_dict = {'waypoint': wp['name'], 'x': wp['x'], 'y': wp['y'],
                           'theta': wp.get('theta', 0.0)}
        else:
            target_dict = {'x': float(step['x']), 'y': float(step['y']),
                           'theta': float(step.get('theta', 0.0))}
        result = await navigate_to(bridge, target_dict)
        return result['ok'], result['message']
    if stype == 'wait':
        dur = float(step.get('duration', step.get('duration_sec', 0.0)))
        await asyncio.sleep(dur)
        return True, ''
    if stype == 'dock':
        result = await dock_robot(bridge, bool(step.get('navigate_to_staging', True)))
        return result['ok'], result['message']
    if stype == 'undock':
        result = await undock_robot(bridge)
        return result['ok'], result['message']
    if stype == 'call_service':
        ignore_error = bool(step.get('ignore_error', False))
        timeout = float(step.get('timeout', step.get('timeout_sec', 15.0)))
        try:
            client, srv_cls = _get_service_client(bridge, step['service_type'], step['service'])
            request = srv_cls.Request()
            req_data = step.get('request') or step.get('args') or step.get('payload') or {}
            set_message_fields(request, req_data)
            response = await call_service(client, request, step['service'], timeout=timeout)
            return True, json.dumps(message_to_ordereddict(response))[:500]
        except Exception as exc:  # noqa: BLE001
            bridge.get_logger().error(f"call_service step failed: {exc}")
            if ignore_error:
                return True, str(exc)
            return False, str(exc)
    if stype == 'call_action':
        ignore_error = bool(step.get('ignore_error', False))
        timeout = float(step.get('timeout', step.get('timeout_sec', 300.0)))
        try:
            action_name = step.get('action') or step.get('action_name')
            client, action_cls = _get_action_client(bridge, step['action_type'], action_name)
            goal = action_cls.Goal()
            goal_data = step.get('goal') or step.get('args') or step.get('payload') or {}
            set_message_fields(goal, goal_data)
            handle = await send_goal(action_client=client, goal=goal, name=action_name)
            wrapped = await ros_future(handle.get_result_async(), timeout=timeout)
            status = getattr(wrapped, 'status', None)
            if status == 4:  # GoalStatus.STATUS_SUCCEEDED
                result_dict = message_to_ordereddict(getattr(wrapped, 'result', None))
                return True, json.dumps(result_dict)[:500]
            msg = f'action ended with status {status}'
            if ignore_error:
                return True, msg
            return False, msg
        except Exception as exc:  # noqa: BLE001
            bridge.get_logger().error(f"call_action step failed: {exc}")
            if ignore_error:
                return True, str(exc)
            return False, str(exc)
    if stype == 'call_api':
        url = str(step['url']).strip()
        method = str(step.get('method', 'POST')).upper()
        headers = dict(step.get('headers') or {})
        payload = step.get('payload')
        if payload is None:
            payload = step.get('body')
        if payload is None:
            payload = step.get('json')
        timeout = float(step.get('timeout', step.get('timeout_sec', 15.0)))
        ignore_error = bool(step.get('ignore_error', False))

        def _do_request():
            req_kwargs = {'headers': headers, 'timeout': timeout}
            if payload is not None:
                if isinstance(payload, (dict, list)):
                    req_kwargs['json'] = payload
                elif isinstance(payload, str):
                    req_kwargs['data'] = payload
                else:
                    req_kwargs['json'] = payload
            resp = requests.request(method, url, **req_kwargs)
            return resp.status_code, resp.text[:500]

        try:
            status_code, resp_text = await asyncio.to_thread(_do_request)
            ok = (200 <= status_code < 300)
            msg = f"HTTP {status_code}: {resp_text}"
            if not ok:
                bridge.get_logger().warn(f"call_api step returned {status_code}: {resp_text}")
                if ignore_error:
                    return True, msg
                return False, msg
            bridge.get_logger().info(f"call_api step succeeded: {msg}")
            return True, msg
        except Exception as exc:
            bridge.get_logger().error(f"call_api step failed with error: {exc}")
            if ignore_error:
                return True, str(exc)
            return False, f"HTTP request failed: {exc}"
    return False, f'Unknown step type {stype!r}'  # unreachable — validated on save


async def _handle_low_battery_dock_and_resume(bridge, opts, mission: dict) -> None:
    """Auto-dock on low battery, wait on charger until charged, and auto-resume."""
    RUNNER.state = 'charging_paused'
    batt = bridge.get('battery') or {}
    pct = float(batt.get('percentage') or 0.0)
    bridge.emit_event('mission.battery_low_pause', {
        'mission_id': mission['id'],
        'step_index': RUNNER.step_index,
        'loop_index': RUNNER.loop_index,
        'percentage': pct,
    })

    bridge.get_logger().info(
        f"Mission {mission['id']}: navigating to dock to recharge (battery at {pct:.1f}%)..."
    )
    docked = False
    for attempt in range(2):
        if RUNNER.cancel_requested:
            RUNNER.state = 'canceled'
            RUNNER.pause_reason = None
            return
        res = await dock_robot(bridge, navigate_to_staging=True)
        if res.get('ok'):
            docked = True
            break
        bridge.get_logger().warn(f"Auto-dock attempt {attempt + 1} failed: {res.get('message')}")
        await asyncio.sleep(3.0)

    if not docked:
        bridge.get_logger().error(f"Mission {mission['id']}: auto-dock failed! Stopping robot in place.")
        bridge.publish_cmd_vel(0.0, 0.0)
        bridge.emit_event('mission.dock_failed', {
            'mission_id': mission['id'],
            'percentage': pct,
            'message': 'Failed to reach dock automatically',
        })
        RUNNER.state = 'paused'
        RUNNER.message = 'Low battery auto-dock failed'
        # Wait for operator intervention — do not resume or drain battery
        while RUNNER.pause_requested and not RUNNER.cancel_requested:
            await asyncio.sleep(1.0)
        return

    bridge.emit_event('mission.docked_for_charge', {
        'mission_id': mission['id'],
        'percentage': pct,
    })

    bridge.get_logger().info(
        f"Mission {mission['id']}: docked successfully. Recharging until >= {RESUME_BATTERY_PERCENT}%..."
    )
    while True:
        if RUNNER.cancel_requested:
            # User cancelled mission while robot is charging on the dock!
            # Robot STAYS on the charger. Mission cancels cleanly.
            RUNNER.state = 'canceled'
            RUNNER.pause_reason = None
            bridge.get_logger().info(f"Mission {mission['id']} canceled by user while on charger. Robot staying docked.")
            return

        # If user explicitly pressed "Resume" in the UI / API, honor it
        if not RUNNER.pause_requested:
            bridge.get_logger().info(f"Mission {mission['id']} manually resumed by user while on charger.")
            break

        batt = bridge.get('battery') or {}
        cur_pct = batt.get('percentage')
        status = batt.get('status', '')

        # Check if battery reached target resume percentage or full
        if cur_pct is not None and (cur_pct >= RESUME_BATTERY_PERCENT or status == 'full'):
            bridge.get_logger().info(f"Mission {mission['id']}: battery charged ({cur_pct:.1f}%). Ready to resume.")
            bridge.emit_event('mission.battery_charged', {
                'mission_id': mission['id'],
                'percentage': cur_pct,
            })
            break

        await asyncio.sleep(2.0)

    if RUNNER.cancel_requested:
        RUNNER.state = 'canceled'
        RUNNER.pause_reason = None
        return

    bridge.get_logger().info(f"Mission {mission['id']}: undocking to resume step {RUNNER.step_index}...")
    undock_res = await undock_robot(bridge)
    if not undock_res.get('ok'):
        bridge.get_logger().warn(f"Undock reported: {undock_res.get('message')}")

    # Settle for 2 seconds before resuming navigation
    await asyncio.sleep(2.0)

    if RUNNER.cancel_requested:
        RUNNER.state = 'canceled'
        RUNNER.pause_reason = None
        return

    RUNNER.pause_requested = False
    RUNNER.pause_reason = None
    RUNNER.state = 'running'
    bridge.emit_event('mission.resumed', {
        'mission_id': mission['id'],
        'step_index': RUNNER.step_index,
        'reason': 'charge_completed',
    })
    bridge.get_logger().info(f"Mission {mission['id']}: resumed successfully at step {RUNNER.step_index}.")


async def _execute_graph_node(bridge, opts, node: dict, context: dict, mission: Optional[dict] = None) -> Tuple[bool, str, str]:
    """Execute a single graph node."""
    store = opts['store']
    ntype = node.get('type')
    params = node.get('params', {})

    if ntype == 'start':
        return True, 'next', 'Started'

    if ntype in ('navigate', 'navigate_waypoint'):
        target = params.get('waypoint') or params.get('target')
        if isinstance(target, str):
            target = resolve_template_value(target, context)
            wp = store.get_waypoint(target)
            if wp is None:
                return False, 'failed', f"Waypoint {target!r} not found"
            target_dict = {'waypoint': wp['name'], 'x': wp['x'], 'y': wp['y'], 'theta': wp.get('theta', 0.0)}
        else:
            target_dict = {
                'x': float(params.get('x', 0.0)),
                'y': float(params.get('y', 0.0)),
                'theta': float(params.get('theta', 0.0)),
            }
        res = await navigate_to(bridge, target_dict)
        return (True, 'arrived', res.get('message', '')) if res.get('ok') else (False, 'failed', res.get('message', ''))

    if ntype == 'navigate_coordinates':
        try:
            x_val = float(resolve_template_value(params.get('x', 0.0), context))
            y_val = float(resolve_template_value(params.get('y', 0.0), context))
            th_val = float(resolve_template_value(params.get('theta', 0.0), context))
        except (ValueError, TypeError) as conv_err:
            return False, 'failed', f"Invalid coordinate value: {conv_err}"
        target_dict = {'x': x_val, 'y': y_val, 'theta': th_val}
        res = await navigate_to(bridge, target_dict)
        return (True, 'arrived', res.get('message', '')) if res.get('ok') else (False, 'failed', res.get('message', ''))

    if ntype in ('wait', 'wait_timer'):
        dur = float(params.get('duration_sec', params.get('duration', 5.0)))
        await asyncio.sleep(dur)
        return True, 'next', f"Waited {dur}s"

    if ntype in ('end', 'mission_end'):
        status = str(params.get('status', 'success')).lower()
        msg = resolve_template_value(params.get('message', 'Mission completed'), context)
        dock_on_end = bool(params.get('dock_on_end', False))
        sound = params.get('sound')
        if sound:
            try:
                bridge.publish_led_command("blink,0,255,0")
            except Exception:
                pass
        try:
            bridge.publish_cmd_vel(0.0, 0.0)
        except Exception:
            pass
        if dock_on_end:
            bridge.get_logger().info(f"Mission end node {node.get('id')}: auto-docking robot...")
            try:
                await dock_robot(bridge, True)
            except Exception as e:
                bridge.get_logger().warn(f"Auto-dock on mission end failed: {e}")
        return True, 'completed', msg

    if ntype in ('loop', 'loop_counter'):
        count = int(params.get('count', 3))
        var_name = str(params.get('variable_name', 'loop_index')).strip() or 'loop_index'
        cond = params.get('condition')
        max_iter = int(params.get('max_iterations', 50))
        loop_state = context.setdefault('loop_state', {})
        curr_iter = loop_state.get(node['id'], 0)

        cond_ok = True
        if cond and str(cond).strip():
            try:
                cond_ok = evaluate_condition_safely(cond, context)
            except Exception as e:
                bridge.get_logger().error(f"Loop condition error in {node.get('id')}: {e}")
                cond_ok = False

        if curr_iter < count and curr_iter < max_iter and cond_ok:
            loop_state[node['id']] = curr_iter + 1
            context['variables'][var_name] = curr_iter
            return True, 'loop_body', f"Loop iteration {curr_iter + 1}/{count}"
        else:
            loop_state[node['id']] = 0
            return True, 'completed', f"Loop completed ({count} iterations)"

    if ntype == 'patrol_loop':
        waypoints_list = params.get('waypoints') or []
        laps = int(params.get('laps', 1))
        dwell_sec = float(params.get('dwell_sec', 2.0))
        store = opts['store']

        if not waypoints_list:
            return False, 'failed', "No waypoints provided for patrol loop"

        lap = 0
        while laps == 0 or lap < laps:
            lap += 1
            for wp_name in waypoints_list:
                if RUNNER.cancel_requested:
                    return False, 'interrupted', "Patrol cancelled by operator"
                wp = store.get_waypoint(wp_name)
                if not wp:
                    return False, 'failed', f"Patrol waypoint {wp_name!r} not found"
                target_dict = {'waypoint': wp['name'], 'x': wp['x'], 'y': wp['y'], 'theta': wp.get('theta', 0.0)}
                res = await navigate_to(bridge, target_dict)
                if not res.get('ok'):
                    return False, 'failed', f"Failed navigation to {wp_name}: {res.get('message')}"
                if dwell_sec > 0:
                    await asyncio.sleep(dwell_sec)
        return True, 'completed', f"Completed {lap} patrol laps"

    if ntype == 'battery_guard':
        min_pct = float(params.get('min_battery_pct', 20.0))
        require_charging = bool(params.get('require_charging', False))
        batt = bridge.get('battery') or {}
        pct = float(batt.get('percentage', 100.0))
        charging = bool(batt.get('charging')) or batt.get('status') in ('charging', 'full')

        if pct >= min_pct and (not require_charging or charging):
            return True, 'ok', f"Battery OK: {pct:.1f}% >= {min_pct}%"
        else:
            return True, 'low_battery', f"Low battery: {pct:.1f}% < {min_pct}% (charging: {charging})"

    if ntype == 'dock':
        res = await dock_robot(bridge, bool(params.get('navigate_to_staging', True)))
        return (True, 'docked', res.get('message', '')) if res.get('ok') else (False, 'failed', res.get('message', ''))

    if ntype == 'undock':
        res = await undock_robot(bridge)
        return (True, 'undocked', res.get('message', '')) if res.get('ok') else (False, 'failed', res.get('message', ''))

    if ntype == 'condition':
        expr = params.get('expression', 'True')
        try:
            eval_result = evaluate_condition_safely(expr, context)
            port = 'true' if eval_result else 'false'
            return True, port, f"Condition evaluated to {eval_result}"
        except Exception as e:
            bridge.get_logger().error(f"Condition evaluation error in {node.get('id')}: {e}")
            return False, 'false', str(e)

    if ntype == 'set_variable':
        raw_key = params.get('key') or params.get('name') or params.get('variable')
        val = params.get('value')
        if raw_key:
            key = str(resolve_template_value(raw_key, context)).strip()
            val = resolve_template_value(val, context)
            context['variables'][key] = val
        return True, 'next', f"Set {raw_key}"

    if ntype == 'call_api':
        raw_url = params.get('url', '')
        url = str(resolve_template_value(raw_url, context)).strip()
        method = str(params.get('method', 'POST')).upper()
        
        headers_val = resolve_template_value(params.get('headers') or {}, context)
        if isinstance(headers_val, str):
            try:
                headers = json.loads(headers_val)
            except Exception:
                headers = {}
        else:
            headers = headers_val if isinstance(headers_val, dict) else {}
            
        bearer = str(resolve_template_value(params.get('bearer_token', ''), context)).strip()
        if bearer:
            headers['Authorization'] = f"Bearer {bearer}"
            
        payload_val = resolve_template_value(params.get('payload') or params.get('body'), context)
        if isinstance(payload_val, str) and payload_val.strip():
            try:
                payload = json.loads(payload_val)
            except Exception:
                payload = payload_val
        else:
            payload = payload_val

        timeout = float(params.get('timeout_sec', params.get('timeout', 15.0)))
        ignore_error = bool(params.get('ignore_error', False))

        def _do_request():
            req_kwargs = {'headers': headers, 'timeout': timeout}
            if payload is not None:
                if isinstance(payload, (dict, list)):
                    req_kwargs['json'] = payload
                else:
                    req_kwargs['data'] = str(payload)
            resp = requests.request(method, url, **req_kwargs)
            return resp.status_code, resp.text[:1000]

        try:
            status_code, resp_text = await asyncio.to_thread(_do_request)
            ok = (200 <= status_code < 300)
            parsed_json = None
            try:
                parsed_json = json.loads(resp_text)
                context['api_responses'][node['id']] = parsed_json
            except Exception:
                context['api_responses'][node['id']] = {'raw': resp_text, 'status_code': status_code}

            out_var = params.get('output_variable') or params.get('variable_name') or params.get('store_to')
            if out_var:
                out_val = parsed_json if parsed_json is not None else {'raw': resp_text, 'status_code': status_code}
                context['variables'][str(out_var).strip()] = out_val

            if ok or ignore_error:
                return True, 'success', f"HTTP {status_code}"
            return False, 'failure', f"HTTP {status_code}: {resp_text[:100]}"
        except Exception as e:
            bridge.get_logger().error(f"API call failed: {e}")
            if ignore_error:
                return True, 'success', str(e)
            return False, 'failure', str(e)

    if ntype == 'call_service':
        ignore_error = bool(params.get('ignore_error', False))
        timeout = float(params.get('timeout_sec', params.get('timeout', 15.0)))
        try:
            raw_srv = params.get('service_name') or params.get('service')
            srv_name = str(resolve_template_value(raw_srv, context)).strip()
            client, srv_cls = _get_service_client(bridge, params['service_type'], srv_name)
            request = srv_cls.Request()
            req_data = resolve_template_value(params.get('payload') or params.get('request') or params.get('args') or {}, context)
            if isinstance(req_data, str):
                try:
                    req_data = json.loads(req_data)
                except Exception:
                    req_data = {}
            set_message_fields(request, req_data)
            response = await call_service(client, request, srv_name, timeout=timeout)
            resp_dict = message_to_ordereddict(response)
            out_var = params.get('output_variable') or params.get('variable_name') or params.get('store_to')
            if out_var:
                context['variables'][str(out_var).strip()] = resp_dict
            return True, 'success', json.dumps(resp_dict)[:500]
        except Exception as exc:
            bridge.get_logger().error(f"call_service node failed: {exc}")
            if ignore_error:
                return True, 'success', str(exc)
            return False, 'failure', str(exc)

    if ntype == 'call_action':
        ignore_error = bool(params.get('ignore_error', False))
        timeout = float(params.get('timeout_sec', params.get('timeout', 300.0)))
        try:
            raw_act = params.get('action_name') or params.get('action')
            act_name = str(resolve_template_value(raw_act, context)).strip()
            client, act_cls = _get_action_client(bridge, params['action_type'], act_name)
            goal = act_cls.Goal()
            goal_data = resolve_template_value(params.get('payload') or params.get('goal') or {}, context)
            if isinstance(goal_data, str):
                try:
                    goal_data = json.loads(goal_data)
                except Exception:
                    goal_data = {}
            set_message_fields(goal, goal_data)
            ok, result = await send_goal(client, goal, act_name, timeout=timeout)
            res_dict = message_to_ordereddict(result)
            out_var = params.get('output_variable') or params.get('variable_name') or params.get('store_to')
            if out_var:
                context['variables'][str(out_var).strip()] = res_dict
            if ok or ignore_error:
                return True, 'succeeded', json.dumps(res_dict)[:500]
            return False, 'failed', str(result)
        except Exception as exc:
            bridge.get_logger().error(f"call_action node failed: {exc}")
            if ignore_error:
                return True, 'succeeded', str(exc)
            return False, 'failed', str(exc)

    if ntype in ('ui_interaction', 'ui_choice'):
        interaction_id = f"ui_{int(time.time()*1000)}_{uuid.uuid4().hex[:6]}"
        timeout_sec = float(params.get('timeout_sec', 60.0))
        default_option = str(params.get('default_option', 'timeout')).lower()
        subtype = 'choice' if ntype == 'ui_choice' else str(params.get('subtype', 'dynamic_form')).lower()
        target = str(params.get('target', 'robot_screen')).lower()
        if target not in ('robot_screen', 'operator_app', 'both'):
            target = 'robot_screen'

        raw_opts = params.get('options') or params.get('choices') or params.get('buttons') or (['Yes', 'No'] if (ntype == 'ui_choice' or subtype in ('choice', 'choices')) else [])
        opts_list = resolve_template_value(raw_opts, context)
        raw_media = params.get('media_url') or params.get('image_url')
        media_url = resolve_template_value(raw_media, context) if raw_media else None

        interaction_data = {
            'interaction_id': interaction_id,
            'mission_id': RUNNER.mission_id,
            'node_id': node['id'],
            'subtype': subtype,
            'interaction_type': subtype,
            'type': subtype,
            'target': target,
            'title': resolve_template_value(params.get('title', 'Operator Input'), context),
            'message': resolve_template_value(params.get('message', ''), context),
            'fields': resolve_template_value(params.get('fields', []), context),
            'options': opts_list,
            'choices': opts_list,
            'buttons': opts_list,
            'media_url': media_url,
            'image_url': media_url,
            'speech_text': resolve_template_value(params.get('speech_text'), context),
            'timeout_sec': timeout_sec,
            'default_option': default_option,
            'started_at': time.time(),
        }

        RUNNER.active_interaction = interaction_data
        RUNNER.state = 'waiting_for_user'
        bridge.emit_event('mission.ui_interaction', interaction_data)

        loop = asyncio.get_running_loop()
        future: asyncio.Future = loop.create_future()
        RUNNER.interaction_future = future

        try:
            resp_data = await asyncio.wait_for(future, timeout=timeout_sec)
            action = str(resp_data.get('action', 'submit')).strip()
            selected = str(resp_data.get('selected', '')).strip()
            form_data = resp_data.get('form_data') or resp_data.get('data') or {}

            context['forms'][node['id']] = form_data
            context['form'] = form_data
            context['form_data'].update(form_data)
            if isinstance(form_data, dict):
                context['variables'].update(form_data)
                context['variables']['last_form'] = form_data
                context['variables']['last_form_data'] = form_data
                context['variables'][f"{node['id']}_form"] = form_data

            chosen_val = None
            if subtype in ('choice', 'choices') or ntype == 'ui_choice':
                chosen = selected if (selected and selected.lower() != 'selected') else action
                chosen_val = chosen
                output_port = str(chosen).strip().lower()
                context['variables']['selected_choice'] = chosen
                context['variables']['last_choice'] = chosen
                context['variables']['choice'] = chosen
                context['variables'][f"{node['id']}_choice"] = chosen
            elif action.lower() in ('cancel', 'skip'):
                output_port = 'cancelled'
            elif subtype in ('kiosk', 'destination_picker'):
                chosen_dest = resp_data.get('destination') or form_data.get('destination') or selected
                chosen_val = chosen_dest
                if chosen_dest:
                    context['variables']['selected_destination'] = chosen_dest
                    context['variables']['destination'] = chosen_dest
                    context['variables']['target'] = chosen_dest
                    context['variables']['waypoint'] = chosen_dest
                output_port = 'selected'
            else:
                output_port = 'submitted'

            out_var = params.get('output_variable') or params.get('variable_name') or params.get('store_to')
            if out_var:
                out_var_key = str(out_var).strip()
                if chosen_val is not None:
                    context['variables'][out_var_key] = chosen_val
                else:
                    context['variables'][out_var_key] = form_data

            context['variables'][f"{node['id']}_response"] = resp_data
            bridge.emit_event('mission.ui_interaction_dismissed', {
                'interaction_id': interaction_id,
                'node_id': node['id'],
                'action': action,
                'output_port': output_port
            })
            return True, output_port, f"User response: {output_port}"
        except asyncio.TimeoutError:
            bridge.get_logger().info(f"UI interaction {interaction_id} timed out after {timeout_sec}s.")
            bridge.emit_event('mission.ui_interaction_dismissed', {
                'interaction_id': interaction_id,
                'node_id': node['id'],
                'action': 'timeout',
                'output_port': default_option
            })
            return True, default_option, "Timed out waiting for operator"
        finally:
            RUNNER.active_interaction = None
            RUNNER.interaction_future = None
            if RUNNER.state == 'waiting_for_user':
                RUNNER.state = 'running'
            bridge.emit_event('mission.ui_interaction_dismissed', {'interaction_id': interaction_id})

    if ntype == 'ui_media':
        url = resolve_template_value(params.get('url', ''), context)
        media_type = params.get('media_type', 'image')
        duration_sec = float(params.get('duration_sec', 15.0))
        target = str(params.get('target', 'robot_screen')).lower()
        if target not in ('robot_screen', 'operator_app', 'both'):
            target = 'robot_screen'
        show_skip = bool(params.get('show_skip', True))

        interaction_id = f"media_{int(time.time()*1000)}_{uuid.uuid4().hex[:6]}"
        interaction_data = {
            'interaction_id': interaction_id,
            'mission_id': RUNNER.mission_id,
            'node_id': node['id'],
            'subtype': 'media_display',
            'target': target,
            'title': resolve_template_value(params.get('title', 'Media Display'), context),
            'media_url': url,
            'media_type': media_type,
            'duration_sec': duration_sec,
            'show_skip': show_skip,
            'options': ['Skip'] if show_skip else [],
            'timeout_sec': duration_sec if duration_sec > 0 else 3600.0,
            'default_option': 'completed',
            'started_at': time.time(),
        }

        RUNNER.active_interaction = interaction_data
        RUNNER.state = 'waiting_for_user'
        bridge.emit_event('mission.ui_interaction', interaction_data)

        loop = asyncio.get_running_loop()
        future: asyncio.Future = loop.create_future()
        RUNNER.interaction_future = future

        try:
            if duration_sec > 0:
                resp_data = await asyncio.wait_for(future, timeout=duration_sec)
                action = resp_data.get('action', 'skip')
                port = 'skipped' if action in ('skip', 'cancel') else 'completed'
            else:
                resp_data = await future
                port = 'completed'
            return True, port, f"Media display: {port}"
        except asyncio.TimeoutError:
            return True, 'completed', "Media display completed"
        finally:
            RUNNER.active_interaction = None
            RUNNER.interaction_future = None
            if RUNNER.state == 'waiting_for_user':
                RUNNER.state = 'running'
            bridge.emit_event('mission.ui_interaction_resolved', {'interaction_id': interaction_id})

    if ntype == 'ui_speech':
        text = resolve_template_value(params.get('text', ''), context)
        if text:
            try:
                bridge.publish_display_text(text[:32])
            except Exception:
                pass
            bridge.get_logger().info(f"[TTS Announcement]: {text}")
            bridge.emit_event('mission.speech', {'text': text, 'node_id': node['id']})
            if bool(params.get('wait_completion', True)):
                speech_dur = min(15.0, max(1.5, len(text) / 12.0 + 0.8))
                await asyncio.sleep(speech_dur)
        return True, 'done', 'Speech completed'

    if ntype == 'notify':
        oled = params.get('oled_text')
        led = params.get('led_cmd')
        if oled:
            bridge.publish_display_text(resolve_template_value(oled, context))
        if led:
            bridge.publish_led_command(led)
        return True, 'next', 'Notification sent'

    if ntype == 'publish_topic':
        raw_topic = params.get('topic_name') or params.get('topic')
        topic_name = str(resolve_template_value(raw_topic, context)).strip()
        raw_type = params.get('message_type') or params.get('type')
        msg_type = str(resolve_template_value(raw_type, context)).strip()
        payload_val = resolve_template_value(params.get('payload', '{}'), context)
        try:
            if isinstance(payload_val, str):
                payload = json.loads(payload_val)
            else:
                payload = payload_val
            
            # Simple workaround: we use an ephemeral publisher for custom types
            from rclpy.serialization import serialize_message
            import importlib
            parts = msg_type.split('/')
            if len(parts) == 3:
                pkg, _, msg_name = parts
            else:
                pkg, msg_name = parts[0], parts[1]
            module = importlib.import_module(f"{pkg}.msg")
            msg_cls = getattr(module, msg_name)
            
            pub = bridge.create_publisher(msg_cls, topic_name, 10)
            msg = msg_cls()
            set_message_fields(msg, payload)
            pub.publish(msg)
            bridge.destroy_publisher(pub)
            return True, 'success', f"Published to {topic_name}"
        except Exception as e:
            bridge.get_logger().error(f"publish_topic failed: {e}")
            return False, 'failed', str(e)

    if ntype == 'relocalize':
        mode = params.get('mode', 'global_scan')
        try:
            from rclpy.action import ActionClient
            from rclpy.task import Future
            from action_msgs.msg import GoalStatus
            from botforge_interfaces.srv import Relocalize
            
            # Try to find a service first (assuming a Relocalize service might exist, otherwise just fake it for now)
            # Actually, standard Nav2 has amcl global localization service
            if mode == 'global_scan':
                client, srv_cls = _get_service_client(bridge, 'std_srvs/srv/Empty', '/reinitialize_global_localization')
                request = srv_cls.Request()
                await call_service(client, request, '/reinitialize_global_localization', timeout=10.0)
            
            return True, 'done', f"Relocalization ({mode}) completed"
        except Exception as e:
            bridge.get_logger().error(f"Relocalize failed: {e}")
            return False, 'failed', str(e)

    if ntype == 'jog_motion':
        dur = float(params.get('duration_sec', 1.0))
        linear_vel = float(params.get('linear_vel', 0.0))
        angular_vel = float(params.get('angular_vel', 0.0))
        
        try:
            # Publish twist continuously for duration
            start_time = time.time()
            while time.time() - start_time < dur and not RUNNER.cancel_requested:
                bridge.publish_cmd_vel(linear_vel, angular_vel)
                await asyncio.sleep(0.1)
            bridge.publish_cmd_vel(0.0, 0.0)
            return True, 'done', f"Jogged for {dur}s"
        except Exception as e:
            bridge.publish_cmd_vel(0.0, 0.0)
            return False, 'failed', str(e)

    if ntype == 'emergency_stop':
        bridge.publish_cmd_vel(0.0, 0.0)
        sound = bool(params.get('sound_alert', True))
        if sound:
            try:
                bridge.publish_led_command("blink,255,0,0")
            except:
                pass
        # Cancel any active navigation
        await cancel_active_goal(bridge, "Emergency stop")
        return True, 'stopped', "Emergency stop executed"

    if ntype == 'cancel_navigation':
        halt_type = params.get('halt_type', 'abort_goal')
        if halt_type == 'zero_vel':
            bridge.publish_cmd_vel(0.0, 0.0)
        await cancel_active_goal(bridge, "Emergency stop")
        return True, 'done', "Navigation cancelled"

    if ntype in ('switch_mission', 'redirect_mission'):
        target_id = params.get('target_mission_id') or params.get('mission_id')
        transfer_context = bool(params.get('transfer_context', True))
        if not target_id:
            return False, 'failed', "No target mission ID specified"
        target_m = store.get_mission(target_id)
        if not target_m:
            return False, 'failed', f"Target mission {target_id!r} not found"

        curr_map = (mission.get('map') if mission else None) or store.current_map()
        target_map = target_m.get('map')
        if target_map and curr_map and target_map != curr_map:
            return False, 'failed', f"Map mismatch: current={curr_map!r}, target={target_map!r}"

        context.setdefault('system', {})['previous_mission_id'] = mission.get('id') if mission else None
        RUNNER.switch_target = target_m
        RUNNER.switch_context = dict(context) if transfer_context else None
        bridge.get_logger().info(f"Mission redirection staged: {RUNNER.mission_id} -> {target_id}")
        return True, 'out', f"Redirecting to mission {target_id}"

    return False, 'abort', f"Unknown node type: {ntype!r}"


async def _run_graph_mission(bridge, opts, mission: dict, initial_context: Optional[dict] = None) -> None:
    """Execute a visual graph-based mission with branches and UI interactions."""
    nodes = mission['nodes']
    edges = mission.get('edges', [])
    entrypoint = mission.get('entrypoint') or (nodes[0]['id'] if nodes else None)

    nodes_by_id = {n['id']: n for n in nodes}
    if not entrypoint or entrypoint not in nodes_by_id:
        entrypoint = nodes[0]['id']

    RUNNER.mission_id = mission['id']
    RUNNER.state = 'running'
    RUNNER.started_at = time.time()
    RUNNER.context = dict(initial_context) if initial_context is not None else {
        'variables': {},
        'form': {},
        'forms': {},
        'form_data': {},
        'api_responses': {},
        'system': {},
        'history': [],
    }
    RUNNER.context.setdefault('variables', {})
    RUNNER.context.setdefault('form', {})
    RUNNER.context.setdefault('forms', {})
    RUNNER.context.setdefault('form_data', {})
    RUNNER.context.setdefault('api_responses', {})
    RUNNER.context.setdefault('system', {})
    RUNNER.context.setdefault('history', [])
    RUNNER.active_interaction = None
    RUNNER.interaction_future = None
    RUNNER.cancel_requested = False
    RUNNER.pause_requested = False
    RUNNER.pause_reason = None

    bridge.emit_event('mission.started', {
        'mission_id': mission['id'],
        'type': 'graph',
        'entrypoint': entrypoint,
    })

    current_node_id = entrypoint
    visited_count = 0
    max_transitions = int(mission.get('settings', {}).get('max_transitions', 500))

    stop_battery_monitor = asyncio.Event()

    async def _battery_watcher():
        while not stop_battery_monitor.is_set():
            batt = bridge.get('battery') or {}
            pct = batt.get('percentage')
            is_charging = bool(batt.get('charging')) or batt.get('status') in ('charging', 'full')
            if pct is not None and pct <= LOW_BATTERY_DOCK_PERCENT and not is_charging:
                if RUNNER.state in ('running', 'waiting_for_user') and not RUNNER.pause_requested:
                    bridge.get_logger().warn(
                        f"Mission {RUNNER.mission_id}: battery low ({pct:.1f}% <= {LOW_BATTERY_DOCK_PERCENT}%)! "
                        "Pausing mission for auto-dock & recharge."
                    )
                    RUNNER.pause_requested = True
                    RUNNER.pause_reason = 'low_battery'
                    try:
                        await cancel_active_goal(bridge, reason='low_battery')
                    except Exception as e:
                        bridge.get_logger().error(f"Error canceling active goal on low battery: {e}")
            await asyncio.sleep(1.0)

    monitor_task = asyncio.create_task(_battery_watcher())

    try:
        while current_node_id and visited_count < max_transitions:
            visited_count += 1
            node = nodes_by_id.get(current_node_id)
            if not node:
                RUNNER.state = 'failed'
                RUNNER.message = f"Node {current_node_id!r} not found in graph"
                bridge.emit_event('mission.failed', {'mission_id': mission['id'], 'message': RUNNER.message})
                return

            RUNNER.active_node_id = current_node_id
            RUNNER.active_node_type = node.get('type')
            RUNNER.context['history'].append(current_node_id)

            batt = bridge.get('battery') or {}
            RUNNER.context['system']['battery_pct'] = batt.get('percentage', 0.0)
            RUNNER.context['system']['is_charging'] = bool(batt.get('charging'))

            if RUNNER.cancel_requested:
                RUNNER.state = 'canceled'
                bridge.emit_event('mission.canceled', {'mission_id': mission['id'], 'node_id': current_node_id})
                return

            if RUNNER.pause_requested:
                if RUNNER.pause_reason == 'low_battery':
                    await _handle_low_battery_dock_and_resume(bridge, opts, mission)
                    if RUNNER.cancel_requested:
                        RUNNER.state = 'canceled'
                        return
                else:
                    RUNNER.state = 'paused'
                    bridge.emit_event('mission.paused', {'mission_id': mission['id'], 'node_id': current_node_id})
                    while RUNNER.pause_requested and not RUNNER.cancel_requested:
                        await asyncio.sleep(0.2)
                    if RUNNER.cancel_requested:
                        RUNNER.state = 'canceled'
                        return
                    RUNNER.state = 'running'
                    bridge.emit_event('mission.resumed', {'mission_id': mission['id'], 'node_id': current_node_id})

            bridge.emit_event('mission.node_started', {
                'mission_id': mission['id'],
                'node_id': current_node_id,
                'node_type': node.get('type'),
                'label': node.get('label') or node.get('title') or current_node_id,
            })

            ok, output_port, message = await _execute_graph_node(bridge, opts, node, RUNNER.context, mission)

            bridge.emit_event('mission.node_completed', {
                'mission_id': mission['id'],
                'node_id': current_node_id,
                'output_port': output_port,
                'ok': ok,
                'message': message,
            })

            if not ok and output_port == 'abort':
                RUNNER.state = 'failed'
                RUNNER.message = message
                bridge.emit_event('mission.failed', {'mission_id': mission['id'], 'message': message, 'node_id': current_node_id})
                return

            if node.get('type') in ('switch_mission', 'redirect_mission') and ok:
                target_m = getattr(RUNNER, 'switch_target', None)
                if target_m:
                    RUNNER.switch_target = None
                    new_context = getattr(RUNNER, 'switch_context', None)
                    RUNNER.switch_context = None
                    bridge.get_logger().info(f"Mission redirection executing: {mission['id']} -> {target_m['id']}")
                    bridge.emit_event('mission.completed', {
                        'mission_id': mission['id'],
                        'message': f"Redirected to mission {target_m['id']}",
                    })
                    stop_battery_monitor.set()
                    monitor_task.cancel()
                    await _run_mission(bridge, opts, target_m, initial_context=new_context)
                    return

            if node.get('type') in ('end', 'mission_end'):
                status = str(node.get('params', {}).get('status', 'success')).lower()
                if status in ('failed', 'aborted'):
                    RUNNER.state = 'failed'
                    RUNNER.message = message
                    bridge.emit_event('mission.failed', {'mission_id': mission['id'], 'message': message, 'node_id': current_node_id})
                else:
                    RUNNER.state = 'completed'
                    RUNNER.message = message
                    bridge.emit_event('mission.completed', {'mission_id': mission['id'], 'message': message})
                return

            matching_edges = [
                e for e in edges
                if e.get('from_node') == current_node_id and str(e.get('from_port', '')).lower() == str(output_port).lower()
            ]
            if not matching_edges:
                matching_edges = [
                    e for e in edges
                    if e.get('from_node') == current_node_id and str(e.get('from_port', '')).lower() in ('next', 'out')
                ]

            if not matching_edges:
                bridge.get_logger().info(f"Mission {mission['id']}: terminal node reached at {current_node_id} (port: {output_port})")
                current_node_id = None
                break

            next_edge = matching_edges[0]
            current_node_id = next_edge.get('to_node')

        if visited_count >= max_transitions:
            RUNNER.state = 'failed'
            RUNNER.message = f"Exceeded maximum node transitions ({max_transitions})"
            bridge.emit_event('mission.failed', {'mission_id': mission['id'], 'message': RUNNER.message})
            return

        RUNNER.state = 'completed'
        bridge.emit_event('mission.completed', {'mission_id': mission['id']})
    finally:
        stop_battery_monitor.set()
        monitor_task.cancel()
        try:
            await monitor_task
        except asyncio.CancelledError:
            pass


async def _run_mission(bridge, opts, mission: dict, initial_context: Optional[dict] = None) -> None:
    store = opts['store']
    current_map = store.current_map()
    mission_map = mission.get('map')
    if mission_map and mission_map != current_map:
        RUNNER.mission_id = mission['id']
        RUNNER.state = 'failed'
        RUNNER.message = (f'Map mismatch: mission requires {mission_map!r}, '
                          f'active map is {current_map!r}')
        bridge.emit_event('mission.failed', {
            'mission_id': mission['id'],
            'step_index': 0,
            'message': RUNNER.message,
        })
        return

    if 'nodes' in mission and mission['nodes']:
        await _run_graph_mission(bridge, opts, mission, initial_context=initial_context)
        return

    steps = mission.get('steps', [])
    loop_forever = bool(mission.get('loop_forever', False))
    loop_count = max(1, int(mission.get('loop_count', 1)))

    RUNNER.mission_id = mission['id']
    RUNNER.state = 'running'
    RUNNER.step_index = 0
    RUNNER.loop_index = 0
    RUNNER.loop_total = None if loop_forever else loop_count
    RUNNER.message = ''
    RUNNER.started_at = time.time()
    RUNNER.cancel_requested = False
    RUNNER.pause_requested = False
    RUNNER.pause_reason = None
    bridge.emit_event('mission.started', {'mission_id': mission['id']})

    stop_battery_monitor = asyncio.Event()

    async def _battery_watcher():
        while not stop_battery_monitor.is_set():
            batt = bridge.get('battery') or {}
            pct = batt.get('percentage')
            is_charging = bool(batt.get('charging')) or batt.get('status') in ('charging', 'full')
            if pct is not None and pct <= LOW_BATTERY_DOCK_PERCENT and not is_charging:
                if RUNNER.state == 'running' and not RUNNER.pause_requested:
                    bridge.get_logger().warn(
                        f"Mission {RUNNER.mission_id}: battery critically low ({pct:.1f}% <= {LOW_BATTERY_DOCK_PERCENT}%)! "
                        "Pausing mission for auto-dock & recharge."
                    )
                    RUNNER.pause_requested = True
                    RUNNER.pause_reason = 'low_battery'
                    try:
                        await cancel_active_goal(bridge, reason='low_battery')
                    except Exception as e:
                        bridge.get_logger().error(f"Error canceling active goal on low battery: {e}")
            await asyncio.sleep(1.0)

    monitor_task = asyncio.create_task(_battery_watcher())

    try:
        iteration = 0
        while loop_forever or iteration < loop_count:
            RUNNER.loop_index = iteration
            RUNNER.step_index = 0
            while RUNNER.step_index < len(steps):
                if RUNNER.cancel_requested:
                    RUNNER.state = 'canceled'
                    RUNNER.pause_reason = None
                    bridge.emit_event('mission.canceled', {'mission_id': mission['id'],
                                                            'step_index': RUNNER.step_index})
                    return

                if RUNNER.pause_requested:
                    if RUNNER.pause_reason == 'low_battery':
                        await _handle_low_battery_dock_and_resume(bridge, opts, mission)
                        if RUNNER.cancel_requested:
                            RUNNER.state = 'canceled'
                            RUNNER.pause_reason = None
                            bridge.emit_event('mission.canceled', {'mission_id': mission['id'],
                                                                    'step_index': RUNNER.step_index})
                            return
                        continue

                    RUNNER.state = 'paused'
                    bridge.emit_event('mission.paused', {'mission_id': mission['id'],
                                                          'step_index': RUNNER.step_index})
                    while RUNNER.pause_requested and not RUNNER.cancel_requested:
                        await asyncio.sleep(0.2)
                    if RUNNER.cancel_requested:
                        RUNNER.state = 'canceled'
                        bridge.emit_event('mission.canceled', {'mission_id': mission['id'],
                                                                'step_index': RUNNER.step_index})
                        return
                    RUNNER.state = 'running'
                    bridge.emit_event('mission.resumed', {'mission_id': mission['id'],
                                                           'step_index': RUNNER.step_index})

                ok, message = await _run_step(bridge, store, steps[RUNNER.step_index])
                if not ok:
                    if RUNNER.pause_reason == 'low_battery':
                        # Interrupted by low battery monitor mid-step; loop back to enter dock & recharge handler
                        continue
                    if RUNNER.cancel_requested:
                        RUNNER.state = 'canceled'
                        bridge.emit_event('mission.canceled', {'mission_id': mission['id'],
                                                                'step_index': RUNNER.step_index})
                        return
                    if RUNNER.pause_requested:
                        continue

                    RUNNER.state = 'failed'
                    RUNNER.message = message
                    bridge.emit_event('mission.failed', {'mission_id': mission['id'],
                                                          'step_index': RUNNER.step_index,
                                                          'message': message})
                    return
                RUNNER.step_index += 1

            iteration += 1
            if loop_forever or iteration < loop_count:
                bridge.emit_event('mission.lap_completed', {'mission_id': mission['id'],
                                                             'loop_index': RUNNER.loop_index})

        RUNNER.state = 'completed'
        RUNNER.step_index = max(0, len(steps) - 1)
        bridge.emit_event('mission.completed', {'mission_id': mission['id']})
    finally:
        stop_battery_monitor.set()
        monitor_task.cancel()
        try:
            await monitor_task
        except asyncio.CancelledError:
            pass


class MissionsHandler(BaseHandler):
    def get(self) -> None:
        map_name = self.get_argument('map', None)
        self.send({'missions': self.opts['store'].list_missions(map_name)})

    def post(self) -> None:
        """Create or replace a mission definition. Does not start it — see
        MissionControlHandler's `start` action."""
        data = self.body(('id',))
        mission_id = str(data['id']).strip()
        if not mission_id:
            raise ApiError(400, 'invalid_field', 'id must not be empty')

        map_name = data.get('map')
        if map_name is not None:
            map_name = str(map_name).strip() or None
        if not map_name:
            map_name = self.opts['store'].current_map()

        mission: dict[str, Any] = {
            'id': mission_id,
            'name': str(data.get('name') or mission_id),
            'map': map_name,
        }

        if 'nodes' in data and data['nodes']:
            nodes, edges, entrypoint = validate_graph_mission(data)
            mission['nodes'] = nodes
            mission['edges'] = edges
            mission['entrypoint'] = entrypoint
            mission['settings'] = data.get('settings', {})
            mission['steps'] = data.get('steps', [])
            mission['type'] = 'graph'
        elif 'steps' in data:
            steps = _validate_steps(data['steps'])
            loop_forever, loop_count = _validate_loop(data)
            mission['steps'] = steps
            mission['loop_forever'] = loop_forever
            mission['loop_count'] = loop_count
            mission['type'] = 'linear'
        else:
            raise ApiError(400, 'invalid_field', 'Mission must contain either "nodes" or "steps"')

        self.opts['store'].put_mission(mission)
        self.send({'mission': mission}, status=201)


class MissionHandler(BaseHandler):
    def get(self, mission_id: str) -> None:
        mission = self.opts['store'].get_mission(mission_id)
        if mission is None:
            raise ApiError(404, 'mission_not_found', f'No mission named {mission_id!r}')
        self.send({'mission': mission})

    def delete(self, mission_id: str) -> None:
        if RUNNER.mission_id == mission_id and RUNNER.state in ('running', 'paused', 'charging_paused'):
            raise ApiError(409, 'mission_active',
                           'Cancel the running mission before deleting it')
        if not self.opts['store'].delete_mission(mission_id):
            raise ApiError(404, 'mission_not_found', f'No mission named {mission_id!r}')
        self.send({'deleted': True, 'id': mission_id})


class MissionStatusHandler(BaseHandler):
    def get(self) -> None:
        self.send(RUNNER.snapshot())


class MissionControlHandler(BaseHandler):
    """POST /missions/{id}/start|pause|resume|cancel."""

    async def post(self, mission_id: str, action: str) -> None:
        if action == 'start':
            if RUNNER.state in ('running', 'paused', 'charging_paused'):
                raise ApiError(409, 'mission_active',
                               f'Mission {RUNNER.mission_id!r} is already {RUNNER.state}')
            mission = self.opts['store'].get_mission(mission_id)
            if mission is None:
                raise ApiError(404, 'mission_not_found', f'No mission named {mission_id!r}')

            current_map = self.opts['store'].current_map()
            mission_map = mission.get('map')
            if mission_map and mission_map != current_map:
                raise ApiError(409, 'map_mismatch',
                               f'Mission {mission_id!r} requires map {mission_map!r}, '
                               f'but active map is {current_map!r}. Switch map before starting.',
                               {'required_map': mission_map, 'active_map': current_map})

            self.opts['spawn'](_run_mission(self.bridge, self.opts, mission))
            self.send({'accepted': True, 'mission_id': mission_id}, status=202)
            return

        if RUNNER.mission_id != mission_id or RUNNER.state not in ('running', 'paused', 'charging_paused'):
            raise ApiError(409, 'mission_not_active',
                           f'Mission {mission_id!r} is not currently active')
        if action == 'pause':
            RUNNER.pause_requested = True
            RUNNER.pause_reason = 'user_requested'
        elif action == 'resume':
            RUNNER.pause_requested = False
            RUNNER.pause_reason = None
        elif action == 'cancel':
            RUNNER.cancel_requested = True
        else:
            raise ApiError(404, 'not_found', f'Unknown mission action {action!r}')
        self.send(RUNNER.snapshot())


class ActiveUiInteractionHandler(BaseHandler):
    """GET /api/v1/missions/active_ui_interaction"""

    def get(self) -> None:
        inter = RUNNER.active_interaction
        self.send({
            'active': inter is not None,
            'active_interaction': inter,
            'interaction': inter,
        })


class UiResponseHandler(BaseHandler):
    """POST /api/v1/missions/ui_response"""

    def post(self) -> None:
        data = self.body(('interaction_id',))
        interaction_id = str(data.get('interaction_id', '')).strip()
        if not RUNNER.active_interaction or RUNNER.active_interaction.get('interaction_id') != interaction_id:
            raise ApiError(404, 'no_matching_interaction',
                           f'No active interaction matching id {interaction_id!r}')

        if RUNNER.interaction_future and not RUNNER.interaction_future.done():
            RUNNER.interaction_future.set_result(data)
            self.send({'accepted': True, 'interaction_id': interaction_id})
        else:
            raise ApiError(409, 'interaction_closed', 'Interaction has already completed or timed out')


class NodeTypesHandler(BaseHandler):
    """GET /api/v1/missions/node_types"""

    def get(self) -> None:
        self.send({'node_types': NODE_CATALOG})

