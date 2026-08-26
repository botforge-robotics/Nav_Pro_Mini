#!/usr/bin/env python3
"""WebSocket event stream.

The REST surface answers "what is true now". This answers "tell me when it
changes" — the equivalent of the reference SDK's asynchronous reports, without
the serial framing.

Protocol, deliberately minimal:
    client -> {"action": "subscribe",   "streams": ["pose", "battery"]}
    client -> {"action": "unsubscribe", "streams": ["pose"]}
    client -> {"action": "ping"}
    server -> {"stream": "battery", "data": {...}, "ts": 1699...}

Subscription is opt-in per stream rather than firehose-by-default: pose updates
at 20Hz would swamp a client that only wanted battery, over WiFi that has
already proven unreliable on this robot.

"events" is a different kind of stream from the rest of this list: the others
are telemetry ("here is the current pose"), rate-limited because their source
publishes continuously. "events" carries discrete occurrences
(`navigation.completed`, `battery.low`, ...) written via
`RosBridge.emit_event()` under an `event:`-prefixed cache key — see that
method's docstring. Never rate-limited: an event is emitted once by
definition, so there is nothing to throttle.
    server -> {"stream": "events", "data": {"event": "navigation.completed",
                                            "data": {...}}, "ts": 1699...}
"""

from __future__ import annotations

import json
import time

import tornado.websocket

from ..ros_bridge import RosBridge

# Cache key -> public stream name.
STREAMS = {
    'pose_map': 'pose',
    'pose_odom': 'pose_odom',
    'velocity': 'velocity',
    'battery': 'battery',
    'scan': 'scan',
    'imu': 'imu',
    'dock_status': 'dock_status',
    'dock_tag': 'dock_tag',
    'cpu_temperature': 'cpu_temperature',
    'plan': 'path',
}
EVENTS_STREAM = 'events'
PUBLIC = sorted(set(STREAMS.values()) | {EVENTS_STREAM})

# Per-stream minimum interval (seconds). Without this, pose and scan alone
# would push tens of messages a second per client.
_MIN_INTERVAL = {
    'pose': 0.1, 'pose_odom': 0.1, 'velocity': 0.1,
    'scan': 0.5, 'imu': 0.2, 'path': 0.5,
    'battery': 0.0, 'dock_status': 0.0, 'dock_tag': 0.2, 'cpu_temperature': 0.0,
}


class EventSocket(tornado.websocket.WebSocketHandler):
    def initialize(self, bridge=None, **kw) -> None:
        self.bridge = bridge
        self.opts = kw
        self._subs: set[str] = set()
        self._last: dict[str, float] = {}
        self._listener = None

    def check_origin(self, origin: str) -> bool:
        # Same reasoning as the REST CORS policy: LAN device, no cookie auth,
        # so an origin check would block legitimate dashboards without adding
        # protection. Token auth (when enabled) still applies below.
        return True

    def open(self) -> None:
        token = self.opts.get('auth_token')
        if token and self.get_argument('token', '') != token:
            self.close(code=4401, reason='unauthorized')
            return
        self._listener = self._on_ros
        self.bridge.add_listener(self._listener)
        self.write_message(json.dumps({
            'stream': 'hello',
            'data': {'streams': PUBLIC, 'api_version': 'v1'},
            'ts': time.time(),
        }))

    def on_close(self) -> None:
        if self._listener is not None:
            self.bridge.remove_listener(self._listener)
            self._listener = None

    def on_message(self, message: str) -> None:
        try:
            msg = json.loads(message)
        except ValueError:
            self._error('invalid_json', 'Message is not valid JSON')
            return
        action = msg.get('action')
        if action == 'ping':
            self.write_message(json.dumps({'stream': 'pong', 'ts': time.time()}))
            return
        if action not in ('subscribe', 'unsubscribe'):
            self._error('invalid_action',
                        'action must be subscribe, unsubscribe or ping')
            return
        streams = msg.get('streams') or []
        if not isinstance(streams, list):
            self._error('invalid_streams', 'streams must be a list of names')
            return
        unknown = [s for s in streams if s not in PUBLIC]
        if unknown:
            self._error('unknown_stream',
                        f'Unknown stream(s): {", ".join(map(str, unknown))}',
                        {'valid': PUBLIC})
            return
        if action == 'subscribe':
            self._subs.update(streams)
        else:
            self._subs.difference_update(streams)
        self.write_message(json.dumps({
            'stream': 'subscribed',
            'data': {'streams': sorted(self._subs)},
            'ts': time.time(),
        }))

    def _error(self, code: str, message: str, detail: dict | None = None) -> None:
        self.write_message(json.dumps({
            'stream': 'error',
            'data': {'code': code, 'message': message, 'detail': detail or {}},
            'ts': time.time(),
        }))

    def _on_ros(self, key: str, value) -> None:
        """Called from a ROS executor thread — must not touch the socket here.

        Tornado's WebSocket is not thread-safe, so the write is scheduled onto
        the IO loop instead. Writing directly from the ROS thread corrupts the
        connection under load, which is exactly when it matters.
        """
        if key.startswith(RosBridge.EVENT_KEY_PREFIX):
            if EVENTS_STREAM not in self._subs:
                return
            now = time.time()
            payload = json.dumps({'stream': EVENTS_STREAM, 'data': value, 'ts': now})
            try:
                self.opts['ioloop'].add_callback(self._safe_write, payload)
            except Exception:  # noqa: BLE001
                pass
            return

        name = STREAMS.get(key)
        if name is None or name not in self._subs:
            return
        now = time.time()
        if now - self._last.get(name, 0.0) < _MIN_INTERVAL.get(name, 0.0):
            return
        self._last[name] = now
        payload = json.dumps({'stream': name, 'data': value, 'ts': now})
        try:
            self.opts['ioloop'].add_callback(self._safe_write, payload)
        except Exception:  # noqa: BLE001
            pass

    def _safe_write(self, payload: str) -> None:
        try:
            self.write_message(payload)
        except tornado.websocket.WebSocketClosedError:
            pass
