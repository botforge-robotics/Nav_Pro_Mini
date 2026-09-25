#!/usr/bin/env python3
"""Docking and undocking, via dock_manager's two action servers."""

from __future__ import annotations

import asyncio
import tornado.iostream

import json
import math
import os
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
    TRACKER.begin(None, 'docking')
    if hasattr(bridge, 'acquire_video_stream'):
        bridge.acquire_video_stream()
    if hasattr(bridge, 'publish_led_command'):
        bridge.publish_led_command('solid,255,255,255')
    if hasattr(bridge, 'publish_display_state'):
        bridge.publish_display_state('docking')
    bridge.emit_event('dock.started')

    goal = DockRobot.Goal()
    goal.dock_type = 'simple_charging_dock'
    # navigate_to_staging_pose drives to the standoff first. Skipping it only
    # makes sense when the robot is already parked in front of the dock;
    # default to the safe behaviour.
    goal.navigate_to_staging_pose = bool(navigate_to_staging)
    goal.use_dock_id = True     # use the robot's own saved dock pose

    try:
        handle = await send_goal(goal=goal, action_client=bridge.act_dock, name='dock')
        TRACKER.handle = handle
        return handle
    except Exception as exc:
        TRACKER.finish('failed', str(exc))
        if hasattr(bridge, 'release_video_stream'):
            bridge.release_video_stream()
        if hasattr(bridge, 'publish_display_state'):
            bridge.publish_display_state('ready')
        raise


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
    finally:
        if hasattr(bridge, 'release_video_stream'):
            bridge.release_video_stream()
        if hasattr(bridge, 'publish_display_state'):
            bridge.publish_display_state('ready')


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
    TRACKER.begin(None, 'undocking')
    bridge.emit_event('dock.started', {'operation': 'undock'})

    goal = NavigateToPose.Goal()
    goal.pose = PoseStamped()   # empty frame_id + zero quaternion = undock only

    try:
        handle = await send_goal(goal=goal, action_client=bridge.act_navigate, name='undock')
        TRACKER.handle = handle
        return handle
    except Exception as exc:
        TRACKER.finish('failed', str(exc))
        raise


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
        if hasattr(self.bridge, 'release_video_stream'):
            self.bridge.release_video_stream()
        if hasattr(self.bridge, 'publish_display_state'):
            self.bridge.publish_display_state('ready')
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
        is_charging = bool(battery.get('charging')) or battery.get('status') in ('charging', 'full')
        effective_state = dock_status or 'unknown'
        if is_charging and effective_state in ('undocked', 'unknown'):
            effective_state = 'full' if battery.get('status') == 'full' else 'charging'
        self.send({
            'state': effective_state,
            'age_sec': age,
            'operation': TRACKER.state,
            'message': TRACKER.message,
            'charging': is_charging,
            'battery_status': battery.get('status'),
            'tag_visible': bool(tag.get('visible')) if tag else False,
        })


class DockPoseHandler(BaseHandler):
    """Read or set where the robot believes its dock is."""

    def get(self) -> None:
        value, age = self.bridge.get_with_age('dock_pose')
        if value is None:
            dock_file = os.path.expanduser('~/.navpromini_dock_pose.json')
            if os.path.isfile(dock_file):
                try:
                    with open(dock_file, 'r') as f:
                        d = json.load(f)
                    theta = d.get('theta')
                    if theta is None:
                        qz = d.get('qz', 0.0)
                        qw = d.get('qw', 1.0)
                        theta = 2.0 * math.atan2(qz, qw)
                    value = {'x': d['x'], 'y': d['y'], 'theta': theta}
                    age = 0.0
                except Exception:
                    pass
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

        with self.bridge._lock:
            self.bridge._cache['dock_pose'] = (
            {'x': x, 'y': y, 'theta': theta, 'frame_id': 'map'},
            self.bridge.get_clock().now().nanoseconds / 1e9,
        )

        dock_file = os.path.expanduser('~/.navpromini_dock_pose.json')
        dock_dict = {
            'frame_id': 'map',
            'x': x,
            'y': y,
            'z': 0.0,
            'qx': 0.0,
            'qy': 0.0,
            'qz': math.sin(theta / 2.0),
            'qw': math.cos(theta / 2.0),
            'theta': theta,
        }
        try:
            tmp_path = dock_file + '.tmp'
            with open(tmp_path, 'w') as f:
                json.dump(dock_dict, f, indent=2)
            os.replace(tmp_path, dock_file)
        except Exception as exc:
            pass

        self.send({'x': x, 'y': y, 'theta': theta})

    def delete(self) -> None:
        self.bridge.invalidate('dock_pose')
        dock_file = os.path.expanduser('~/.navpromini_dock_pose.json')
        if os.path.isfile(dock_file):
            try:
                os.remove(dock_file)
            except Exception:
                pass
        self.send({'deleted': True})


_TINY_JPEG = (
    b'\xff\xd8\xff\xe0\x00\x10JFIF\x00\x01\x01\x01\x00H\x00H\x00\x00'
    b'\xff\xdb\x00C\x00\x08\x06\x06\x07\x06\x05\x08\x07\x07\x07\t\t'
    b'\x08\n\x0c\x14\r\x0c\x0b\x0b\x0c\x19\x12\x13\x0f\x14\x1d\x1a'
    b'\x1f\x1e\x1d\x1a\x1c\x1c $.\' ",#\x1c\x1c(7),01444\x1f\'9=82<.342'
    b'\xff\xc0\x00\x0b\x08\x00\x01\x00\x01\x01\x01\x11\x00'
    b'\xff\xc4\x00\x1f\x00\x00\x01\x05\x01\x01\x01\x01\x01\x01\x00'
    b'\x00\x00\x00\x00\x00\x00\x00\x01\x02\x03\x04\x05\x06\x07\x08'
    b'\t\n\x0b\xff\xda\x00\x08\x01\x01\x00\x00?\x00\xbf\x00\xff\xd9'
)

_IMAGE_LEASE_TASK: asyncio.Task | None = None

async def _hold_video_lease(bridge, duration: float = 6.0):
    global _IMAGE_LEASE_TASK
    try:
        await asyncio.sleep(duration)
    except asyncio.CancelledError:
        return
    finally:
        if bridge and hasattr(bridge, 'release_video_stream'):
            bridge.release_video_stream()
        _IMAGE_LEASE_TASK = None


class DockDebugImageHandler(BaseHandler):
    """Serve the latest dock alignment debug frame or raw camera frame as JPEG."""

    async def get(self) -> None:
        global _IMAGE_LEASE_TASK
        if self.bridge and hasattr(self.bridge, 'acquire_video_stream'):
            if _IMAGE_LEASE_TASK is None:
                self.bridge.acquire_video_stream()
            else:
                _IMAGE_LEASE_TASK.cancel()
            _IMAGE_LEASE_TASK = asyncio.create_task(_hold_video_lease(self.bridge, 6.0))

        img_data = None
        if self.bridge:
            dbg_data, dbg_age = self.bridge.get_with_age('dock_debug_image')
            if dbg_data is not None and dbg_age < 2.5:
                img_data = dbg_data
            else:
                cam_data, cam_age = self.bridge.get_with_age('camera_image')
                if cam_data is not None and cam_age < 2.5:
                    img_data = cam_data

        if img_data is None and self.bridge:
            for _ in range(12):
                await asyncio.sleep(0.05)
                dbg_data, dbg_age = self.bridge.get_with_age('dock_debug_image')
                if dbg_data is not None and dbg_age < 2.0:
                    img_data = dbg_data
                    break
                cam_data, cam_age = self.bridge.get_with_age('camera_image')
                if cam_data is not None and cam_age < 2.0:
                    img_data = cam_data
                    break

        if img_data is None:
            img_data = _TINY_JPEG

        self.set_header('Content-Type', 'image/jpeg')
        self.set_header('Cache-Control', 'no-cache, no-store, must-revalidate')
        self.set_header('Pragma', 'no-cache')
        self.set_header('Expires', '0')
        self.write(img_data)


class DockDebugStreamHandler(BaseHandler):
    """Multipart MJPEG stream of dock debug / camera feed."""

    async def get(self) -> None:
        if self.bridge and hasattr(self.bridge, 'acquire_video_stream'):
            self.bridge.acquire_video_stream()
        try:
            self.set_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.set_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.set_header('Pragma', 'no-cache')
            self.set_header('Connection', 'close')

            last_sent = None
            while not self._finished:
                img_data = None
                if self.bridge:
                    dbg_data, dbg_age = self.bridge.get_with_age('dock_debug_image')
                    if dbg_data is not None and dbg_age < 2.5:
                        img_data = dbg_data
                    else:
                        cam_data, cam_age = self.bridge.get_with_age('camera_image')
                        if cam_data is not None and cam_age < 2.5:
                            img_data = cam_data

                if img_data and img_data != last_sent:
                    header = (
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n"
                        + f"Content-Length: {len(img_data)}\r\n\r\n".encode()
                    )
                    self.write(header + img_data + b"\r\n")
                    await self.flush()
                    last_sent = img_data
                await asyncio.sleep(0.06)
        except (tornado.iostream.StreamClosedError, asyncio.CancelledError):
            pass
        finally:
            if self.bridge and hasattr(self.bridge, 'release_video_stream'):
                self.bridge.release_video_stream()
