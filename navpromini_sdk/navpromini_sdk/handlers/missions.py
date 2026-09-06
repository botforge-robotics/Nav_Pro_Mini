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
import requests
from .base import ApiError, BaseHandler
from .docking import dock_robot, undock_robot
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
        if stype == 'navigate' and 'target' not in step and not ('x' in step and 'y' in step):
            raise ApiError(400, 'invalid_step',
                           f'step {i}: navigate needs "target" (a waypoint name) '
                           'or "x"/"y"', {'index': i})
        if stype == 'wait' and 'duration' not in step:
            raise ApiError(400, 'invalid_step', f'step {i}: wait needs "duration"',
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
            if 'action' not in step or 'action_type' not in step:
                raise ApiError(400, 'invalid_step',
                               f'step {i}: call_action needs "action" and "action_type"',
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
        self.state = 'idle'   # idle | running | paused | completed | failed | canceled
        self.step_index = -1
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
            'step_index': self.step_index,
            'loop_index': self.loop_index,
            'loop_total': self.loop_total,
            'message': self.message,
            'pause_reason': self.pause_reason,
            'elapsed_sec': (round(time.time() - self.started_at, 1)
                            if self.started_at else None),
        }


RUNNER = _MissionRunner()


async def _run_step(bridge, store, step: dict) -> tuple[bool, str]:
    stype = step['type']
    if stype == 'navigate':
        target = step.get('target')
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
        await asyncio.sleep(float(step.get('duration', 0.0)))
        return True, ''
    if stype == 'dock':
        result = await dock_robot(bridge, bool(step.get('navigate_to_staging', True)))
        return result['ok'], result['message']
    if stype == 'undock':
        result = await undock_robot(bridge)
        return result['ok'], result['message']
    if stype == 'call_service':
        try:
            client, srv_cls = _get_service_client(bridge, step['service_type'], step['service'])
            request = srv_cls.Request()
            set_message_fields(request, step.get('request') or {})
            response = await call_service(client, request, step['service'],
                                          timeout=float(step.get('timeout', 15.0)))
            return True, json.dumps(message_to_ordereddict(response))[:500]
        except Exception as exc:  # noqa: BLE001
            return False, str(exc)
    if stype == 'call_action':
        try:
            client, action_cls = _get_action_client(bridge, step['action_type'], step['action'])
            goal = action_cls.Goal()
            set_message_fields(goal, step.get('goal') or {})
            handle = await send_goal(action_client=client, goal=goal, name=step['action'])
            wrapped = await ros_future(handle.get_result_async(),
                                       timeout=float(step.get('timeout', 300.0)))
            status = getattr(wrapped, 'status', None)
            if status == 4:  # GoalStatus.STATUS_SUCCEEDED
                result_dict = message_to_ordereddict(getattr(wrapped, 'result', None))
                return True, json.dumps(result_dict)[:500]
            return False, f'action ended with status {status}'
        except Exception as exc:  # noqa: BLE001
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


async def _run_mission(bridge, opts, mission: dict) -> None:
    store = opts['store']
    steps = mission['steps']
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
        self.send({'missions': self.opts['store'].list_missions()})

    def post(self) -> None:
        """Create or replace a mission definition. Does not start it — see
        MissionControlHandler's `start` action."""
        data = self.body(('id', 'steps'))
        mission_id = str(data['id']).strip()
        if not mission_id:
            raise ApiError(400, 'invalid_field', 'id must not be empty')
        steps = _validate_steps(data['steps'])
        loop_forever, loop_count = _validate_loop(data)
        mission = {'id': mission_id, 'name': str(data.get('name') or mission_id),
                   'steps': steps, 'loop_forever': loop_forever, 'loop_count': loop_count}
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
