#!/usr/bin/env python3
"""Keeps rosbridge's client count above zero while the SDK owns a launch.

WHY THIS EXISTS
---------------
launch_manager subscribes to /client_count (published by rosbridge, which
counts connected WebSocket clients) and, when it reaches zero, SIGINTs every
launch it started. For the Flutter app that is a feature: close the UI, the
robot stops.

For an API it is a bug. An SDK client that starts navigation over HTTP and then
disconnects — which is normal, HTTP is not a persistent connection — would have
navigation killed under it the moment the last browser tab also closed.

Rather than duplicate launch_manager's process supervision (which would create
a second owner of mapping/navigation, and two publishers of map->odom), the SDK
simply becomes a rosbridge client for as long as it has a launch running. The
count never reaches zero, launch_manager never fires, and there is still exactly
one owner of the launches.

Reference-counted: several SDK-owned launches can overlap, and the connection
drops only when the last one is released.
"""

from __future__ import annotations

import threading
from typing import Optional

try:
    from tornado.websocket import websocket_connect
    from tornado.ioloop import IOLoop
except ImportError:  # pragma: no cover - tornado is an exec_depend
    websocket_connect = None
    IOLoop = None


class ClientHold:
    def __init__(self, url: str = 'ws://127.0.0.1:9090', logger=None) -> None:
        self.url = url
        self._log = logger
        self._count = 0
        self._conn = None
        self._lock = threading.Lock()

    def _info(self, msg: str) -> None:
        if self._log:
            self._log.info(msg)

    def _warn(self, msg: str) -> None:
        if self._log:
            self._log.warn(msg)

    def acquire(self) -> None:
        with self._lock:
            self._count += 1
            first = self._count == 1
        if first:
            IOLoop.current().add_callback(self._connect)

    def release(self) -> None:
        with self._lock:
            self._count = max(0, self._count - 1)
            last = self._count == 0
        if last:
            IOLoop.current().add_callback(self._disconnect)

    async def _connect(self) -> None:
        if self._conn is not None or websocket_connect is None:
            return
        try:
            self._conn = await websocket_connect(self.url)
            self._info('holding a rosbridge connection so launch_manager does '
                       'not stop SDK-started launches at client_count 0')
        except Exception as exc:  # noqa: BLE001
            # Not fatal: without rosbridge running there is no client_count
            # publisher either, so nothing will kill the launch.
            self._conn = None
            self._warn(f'could not hold a rosbridge connection ({exc}); '
                       'SDK-started launches may stop if a UI disconnects')

    async def _disconnect(self) -> None:
        conn, self._conn = self._conn, None
        if conn is not None:
            try:
                conn.close()
            except Exception:  # noqa: BLE001
                pass


class ModeState:
    """What mode the SDK believes the robot is in, and which launch backs it."""

    def __init__(self) -> None:
        import time
        self._time = time
        self.mode = 'idle'
        self.map_name: Optional[str] = None
        self.launch_id: Optional[str] = None
        self.busy = False
        self._since = time.time()

    def set(self, mode: str, map_name: Optional[str], launch_id: Optional[str]) -> None:
        self.mode = mode
        self.map_name = map_name
        self.launch_id = launch_id
        self._since = self._time.time()

    def age(self) -> float:
        return round(self._time.time() - self._since, 1)
