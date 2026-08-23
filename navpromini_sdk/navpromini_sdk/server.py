#!/usr/bin/env python3
"""NavProMini SDK — HTTP + WebSocket API server.

Runs the tornado application on one thread and the rclpy executor on another.
They meet only through RosBridge: its cache (guarded by a lock) and the
future-bridging helpers in handlers/roscall.py. Nothing else crosses.

Start with:
    ros2 launch navpromini_sdk sdk.launch.py
    ros2 run navpromini_sdk sdk_server --ros-args -p port:=8090
"""

from __future__ import annotations

import asyncio
import signal
import threading
from typing import Any

import rclpy
import tornado.ioloop
import tornado.web
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor

from .client_hold import ClientHold, ModeState
from .handlers import docking, maps, mode, motion, navigation, state, system, waypoints
from .handlers.base import BaseHandler
from .handlers.events import EventSocket
from .ros_bridge import LATCHED_QOS, RosBridge
from .store import Store

API = r'/api/v1'


class NotFoundHandler(BaseHandler):
    """Unknown paths get the same error shape as everything else.

    Tornado's stock 404 is HTML; a client parsing JSON would choke on it right
    when it most needs a readable error — during integration.
    """

    def prepare(self) -> None:
        self.fail(404, 'not_found',
                  f'No such endpoint: {self.request.method} {self.request.path}',
                  {'docs': 'https://botforge-robotics.github.io/navpromini_sdk/'})


class NotImplementedHandler(BaseHandler):
    """Reserved namespaces, documented as planned.

    These exist so the URL space for virtual walls, fixed routes and missions
    is claimed and visible now. A client that probes them gets an explicit
    "planned, not built" rather than a 404 that looks like a typo.
    """

    def prepare(self) -> None:
        self.fail(501, 'not_implemented',
                  'This capability is planned but not available on this robot.',
                  {'path': self.request.path,
                   'roadmap': 'https://botforge-robotics.github.io/navpromini_sdk/roadmap/'})


def build_app(bridge: RosBridge, store: Store, opts: dict[str, Any]) -> tornado.web.Application:
    routes = [
        # system
        (rf'{API}/system/info', system.InfoHandler, opts),
        (rf'{API}/system/health', system.HealthHandler, opts),
        # state
        (rf'{API}/state/pose', state.PoseHandler, opts),
        (rf'{API}/state/velocity', state.VelocityHandler, opts),
        (rf'{API}/state/battery', state.BatteryHandler, opts),
        (rf'{API}/state/imu', state.ImuHandler, opts),
        (rf'{API}/state/scan', state.ScanHandler, opts),
        (rf'{API}/state/temperature', state.TemperatureHandler, opts),
        # mode
        (rf'{API}/mode', mode.ModeHandler, opts),
        # maps
        (rf'{API}/maps', maps.MapsHandler, opts),
        (rf'{API}/maps/current', maps.CurrentMapHandler, opts),
        (rf'{API}/maps/([^/]+)/activate', maps.ActivateMapHandler, opts),
        (rf'{API}/maps/([^/]+)', maps.MapHandler, opts),
        # waypoints
        (rf'{API}/waypoints', waypoints.WaypointsHandler, opts),
        (rf'{API}/waypoints/([^/]+)', waypoints.WaypointHandler, opts),
        # navigation
        (rf'{API}/navigation/goto', navigation.GotoHandler, opts),
        (rf'{API}/navigation/status', navigation.StatusHandler, opts),
        (rf'{API}/navigation/goal', navigation.CancelHandler, opts),
        (rf'{API}/navigation/localize', navigation.LocalizeHandler, opts),
        (rf'{API}/navigation/path', navigation.PathHandler, opts),
        # docking
        (rf'{API}/dock', docking.DockHandler, opts),
        (rf'{API}/undock', docking.UndockHandler, opts),
        (rf'{API}/dock/status', docking.DockStatusHandler, opts),
        (rf'{API}/dock/pose', docking.DockPoseHandler, opts),
        # motion
        (rf'{API}/motion/velocity', motion.VelocityHandler, opts),
        (rf'{API}/motion/stop', motion.StopHandler, opts),
        (rf'{API}/motion/move', motion.MoveHandler, opts),
        (rf'{API}/motion/rotate', motion.RotateHandler, opts),
        # events
        (rf'{API}/events', EventSocket, opts),
        # reserved
        (rf'{API}/zones.*', NotImplementedHandler, opts),
        (rf'{API}/routes.*', NotImplementedHandler, opts),
        (rf'{API}/missions.*', NotImplementedHandler, opts),
    ]
    return tornado.web.Application(routes, default_handler_class=NotFoundHandler,
                                   default_handler_args=opts)


def main(args=None) -> None:
    rclpy.init(args=args)
    bridge = RosBridge()

    bridge.declare_parameter('port', 8090)
    bridge.declare_parameter('address', '0.0.0.0')
    # Empty disables auth. A LAN robot with an open control API is a real
    # risk, so the knob is here and documented — but defaulting it ON would
    # lock out the very first request anyone makes.
    bridge.declare_parameter('auth_token', '')
    bridge.declare_parameter('rosbridge_url', 'ws://127.0.0.1:9090')

    port = int(bridge.get_parameter('port').value)
    address = str(bridge.get_parameter('address').value)
    token = str(bridge.get_parameter('auth_token').value) or None
    rosbridge_url = str(bridge.get_parameter('rosbridge_url').value)

    # Spin ROS on its own thread; tornado owns the main thread.
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(bridge)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    store = Store()
    loop = tornado.ioloop.IOLoop.current()
    opts: dict[str, Any] = {
        'bridge': bridge,
        'store': store,
        'mode_state': ModeState(),
        'client_hold': ClientHold(rosbridge_url, logger=bridge.get_logger()),
        'auth_token': token,
        'ioloop': loop,
        'spawn': lambda coro: loop.add_callback(lambda: asyncio.ensure_future(coro)),
        'dock_pose_pub': bridge.create_publisher(PoseStamped, 'dock_pose', LATCHED_QOS),
    }

    # GET /mode is a cache read (see ModeHandler) so it stays cheap under
    # load; this timer is what keeps that cache honest when navigation or
    # mapping is started by something other than this SDK — the Flutter app
    # talks to launch_manager directly. See mode.reconcile_mode().
    bridge.create_timer(2.0, lambda: mode.reconcile_mode(bridge, opts['mode_state']))

    app = build_app(bridge, store, opts)
    app.listen(port, address=address)
    bridge.get_logger().info(
        f'SDK listening on http://{address}:{port}{API}  '
        f'(auth {"enabled" if token else "disabled"}) — '
        f'events at ws://{address}:{port}{API}/events')

    # Shut the HTTP server down together with ROS.
    #
    # rclpy installs its own SIGINT/SIGTERM handling that invalidates the
    # context. Without stopping tornado too, the process survives with a live
    # HTTP server and a dead ROS context: every endpoint then answers, but
    # with "context is invalid" errors. That is worse than being down, because
    # a health check sees an HTTP 200 and concludes the robot is fine.
    # Observed exactly this after a plain SIGTERM.
    def _stop(_signum, _frame) -> None:
        bridge.get_logger().info('signal received — stopping SDK server')
        loop.add_callback_from_signal(loop.stop)

    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)

    try:
        loop.start()
    except KeyboardInterrupt:
        pass
    finally:
        loop.stop()
        executor.shutdown()
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
