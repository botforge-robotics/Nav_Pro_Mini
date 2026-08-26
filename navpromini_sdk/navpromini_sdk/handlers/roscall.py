#!/usr/bin/env python3
"""Bridging rclpy futures into tornado coroutines.

rclpy futures are driven by the ROS executor's threads; tornado awaits its own
loop's futures. Awaiting an rclpy future directly from a handler would never
complete, because nothing on tornado's loop advances it. These helpers attach a
done-callback on the ROS side and hand the result back to tornado's loop with
`call_soon_threadsafe`, which is the only safe way to cross that boundary.
"""

from __future__ import annotations

import asyncio
from typing import Any

from .base import ApiError


async def ros_future(rf, timeout: float = 15.0) -> Any:
    """Await an rclpy Future from tornado's event loop."""
    loop = asyncio.get_running_loop()
    fut: asyncio.Future = loop.create_future()

    def _done(done_future) -> None:
        if fut.done():
            return
        try:
            loop.call_soon_threadsafe(fut.set_result, done_future.result())
        except Exception as exc:  # noqa: BLE001
            loop.call_soon_threadsafe(fut.set_exception, exc)

    rf.add_done_callback(_done)
    try:
        return await asyncio.wait_for(fut, timeout=timeout)
    except asyncio.TimeoutError:
        raise ApiError(504, 'ros_timeout',
                       f'ROS call did not complete within {timeout}s')


async def call_service(client, request, name: str, timeout: float = 15.0) -> Any:
    """Call a ROS service without ever blocking tornado's event loop.

    Readiness is checked with the NON-blocking service_is_ready(). The obvious
    alternative, wait_for_service(timeout_sec=...), is synchronous: calling it
    from a coroutine freezes the whole IO loop for its duration, stalling every
    other request and the event stream with it. Observed as an endpoint simply
    never responding. If the service is not up, say so immediately.
    """
    if not client.service_is_ready():
        raise ApiError(503, 'service_unavailable',
                       f'{name} service is not available — is the robot stack '
                       'running, and can this process see its ROS graph?',
                       {'service': name})
    return await ros_future(client.call_async(request), timeout)


async def send_goal(action_client, goal, name: str) -> Any:
    """Send an action goal and return its accepted handle.

    Returns once the goal is ACCEPTED, not once it finishes — a dock or a drive
    across a room takes minutes, far longer than any sane HTTP timeout. Callers
    poll the matching status endpoint or watch the event stream.

    server_is_ready() is non-blocking; wait_for_server() is not, and would
    freeze the IO loop (see call_service).
    """
    if not action_client.server_is_ready():
        raise ApiError(503, 'action_unavailable',
                       f'{name} action server is not available — is the '
                       'navigation stack running?', {'action': name})
    handle = await ros_future(action_client.send_goal_async(goal), timeout=10.0)
    if not handle.accepted:
        raise ApiError(409, 'goal_rejected', f'{name} rejected the goal',
                       {'action': name})
    return handle
