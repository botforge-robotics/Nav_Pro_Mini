#!/usr/bin/env python3
"""The single rclpy node behind the SDK's HTTP layer.

Everything ROS lives here: subscriptions, service clients, action clients. HTTP
handlers never touch rclpy directly — they read a snapshot dict or await a
future this class hands back.

WHY A SNAPSHOT CACHE
--------------------
HTTP requests arrive on tornado's event loop; ROS callbacks run on the
executor's threads. A handler that tried to spin ROS to answer a GET would
either block tornado (stalling every other client, including the event stream)
or re-enter the executor. So subscriptions write into a cache under a lock and
handlers read it. A GET is then a dict lookup: bounded, non-blocking, and
impossible to deadlock.

The cost is that reads are as fresh as the publisher, not fresher — each entry
carries its own timestamp so a caller can tell stale data from live data rather
than guessing. Topics that do not publish at all report `null` with an age of
`null`, which is honest about "no data" instead of inventing zeros.
"""

from __future__ import annotations

import json
import math
import threading
import time
from typing import Any, Callable, Optional

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry, Path
from nav2_msgs.action import DockRobot, DriveOnHeading, NavigateToPose, Spin
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import BatteryState, Imu, LaserScan
from std_msgs.msg import Float32, Float32MultiArray, String

from navpromini_launch_manager_interfaces.srv import (
    DeleteMap,
    GetMapList,
    LaunchWithArgs,
    StopLaunch,
)
from rcl_interfaces.srv import GetParameters

# battery.low fires once on crossing below this, not on every reading below
# it — see _on_battery. A pack sitting right at the line would otherwise emit
# on every ~1Hz BatteryState message.
BATTERY_LOW_PERCENT = 15.0
# ...and only after the reading has *held* below the line this long, same
# reasoning as status_display_node.py's CHARGE_DEBOUNCE_SEC: a noisy percentage
# estimate bouncing across 15% must not spam the event stream.
BATTERY_LOW_DEBOUNCE_SEC = 5.0
# Pogo-pin contact bounces on arrival at the dock (status_display_node.py
# observed CHARGING/DISCHARGING 1.2s apart on a real dock) — same debounce
# value reused here for charging.started/charging.completed.
CHARGING_DEBOUNCE_SEC = 0.6

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)
LATCHED_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quat_of(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class RosBridge(Node):
    """Owns every ROS interaction the SDK needs."""

    def __init__(self) -> None:
        super().__init__('navpromini_sdk')
        self._lock = threading.Lock()
        self._cache: dict[str, tuple[Any, float]] = {}
        self._listeners: list[Callable[[str, Any], None]] = []
        cb = ReentrantCallbackGroup()
        self._cb = cb

        # battery.low debounce state — see BATTERY_LOW_DEBOUNCE_SEC.
        self._batt_low_raw = False
        self._batt_low_since = 0.0
        self._batt_low_shown = False
        # charging.started / charging.completed debounce state.
        self._charging_raw: Optional[bool] = None
        self._charging_since = 0.0
        self._charging_shown: Optional[bool] = None

        # --- telemetry subscriptions -------------------------------------
        self.create_subscription(Odometry, 'odom', self._on_odom, 10, callback_group=cb)
        self.create_subscription(BatteryState, 'battery/state', self._on_battery, 10,
                                 callback_group=cb)
        self.create_subscription(String, 'battery/info', self._on_battery_info, 10,
                                 callback_group=cb)
        self.create_subscription(Imu, 'imu', self._on_imu, SENSOR_QOS, callback_group=cb)
        self.create_subscription(LaserScan, 'scan', self._on_scan, SENSOR_QOS,
                                 callback_group=cb)
        self.create_subscription(Float32, 'system/cpu_temperature', self._on_cpu_temp, 10,
                                 callback_group=cb)
        self.create_subscription(String, 'dock_status', self._on_dock_status, 10,
                                 callback_group=cb)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._on_amcl, 10,
                                 callback_group=cb)
        self.create_subscription(Path, 'plan', self._on_plan, 10, callback_group=cb)
        # dock_manager latches dock_pose (TRANSIENT_LOCAL), so subscribing
        # with matching durability gets the current value immediately rather
        # than waiting for the next publish — which may never come, since it
        # is only published when the pose changes.
        self.create_subscription(PoseStamped, 'dock_pose', self._on_dock_pose,
                                 LATCHED_QOS, callback_group=cb)
        self.create_subscription(Float32MultiArray, 'dock_tag', self._on_dock_tag, 10,
                                 callback_group=cb)

        # --- outbound -----------------------------------------------------
        self._pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel_teleop', 10)
        self._pub_initial = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10)

        # --- service clients (launch_manager) ------------------------------
        self.cli_launch = self.create_client(LaunchWithArgs, 'launch_with_args',
                                             callback_group=cb)
        self.cli_stop = self.create_client(StopLaunch, 'stop_launch', callback_group=cb)
        self.cli_maplist = self.create_client(GetMapList, 'get_map_list', callback_group=cb)
        self.cli_delmap = self.create_client(DeleteMap, 'delete_map', callback_group=cb)
        # Reads the real running map name straight off map_server's own
        # yaml_filename parameter — see mode.py's reconcile_mode(), which
        # otherwise has no way to know what map a navigation session it
        # didn't start itself is using.
        self.cli_map_server_param = self.create_client(
            GetParameters, '/map_server/get_parameters', callback_group=cb)

        # --- action clients -------------------------------------------------
        # Navigation goes through dock_manager's `undock`, never bt_navigator
        # directly: it undocks first when docked, so a docked robot cannot be
        # told to drive off while still on the connector.
        self.act_navigate = ActionClient(self, NavigateToPose, 'undock', callback_group=cb)
        self.act_dock = ActionClient(self, DockRobot, 'dock', callback_group=cb)
        self.act_drive = ActionClient(self, DriveOnHeading, 'drive_on_heading',
                                      callback_group=cb)
        self.act_spin = ActionClient(self, Spin, 'spin', callback_group=cb)

        self.get_logger().info('SDK ROS bridge up')

    # -- cache plumbing -----------------------------------------------------

    def _put(self, key: str, value: Any) -> None:
        with self._lock:
            self._cache[key] = (value, time.time())
        for fn in list(self._listeners):
            try:
                fn(key, value)
            except Exception:  # noqa: BLE001 — a bad listener must not kill ROS
                self.get_logger().warn(f'event listener raised on {key}', once=True)

    def get(self, key: str) -> Optional[Any]:
        with self._lock:
            entry = self._cache.get(key)
        return entry[0] if entry else None

    def get_with_age(self, key: str) -> tuple[Optional[Any], Optional[float]]:
        """Value plus seconds since it arrived, so callers can judge staleness."""
        with self._lock:
            entry = self._cache.get(key)
        if entry is None:
            return None, None
        return entry[0], round(time.time() - entry[1], 3)

    def add_listener(self, fn: Callable[[str, Any], None]) -> None:
        self._listeners.append(fn)

    def remove_listener(self, fn: Callable[[str, Any], None]) -> None:
        if fn in self._listeners:
            self._listeners.remove(fn)

    # -- semantic events ------------------------------------------------------
    #
    # The rest of this class answers "what is true now" (telemetry, cached and
    # re-readable). An event answers "something just happened" — discrete, not
    # a value with a shelf life. Modeled on the Mini AMR architecture doc's
    # event taxonomy (navigation.completed, battery.low, ...): reuses the same
    # cache+listener plumbing above under an `event:`-prefixed key rather than
    # adding a second notification path. handlers/events.py is the only reader
    # that treats these specially; everyone else that walks the cache would see
    # a write-only, meaningless "current value" for one, which is expected.

    EVENT_KEY_PREFIX = 'event:'

    def emit_event(self, name: str, data: Optional[dict] = None) -> None:
        self._put(f'{self.EVENT_KEY_PREFIX}{name}', {
            'event': name,
            'timestamp': time.time(),
            'data': data or {},
        })

    # -- subscription callbacks --------------------------------------------

    def _on_odom(self, m: Odometry) -> None:
        p = m.pose.pose.position
        self._put('pose_odom', {
            'x': round(p.x, 4), 'y': round(p.y, 4),
            'theta': round(yaw_of(m.pose.pose.orientation), 4),
            'frame': m.header.frame_id or 'odom',
        })
        self._put('velocity', {
            'linear': round(m.twist.twist.linear.x, 4),
            'angular': round(m.twist.twist.angular.z, 4),
        })

    def _on_amcl(self, m: PoseWithCovarianceStamped) -> None:
        p = m.pose.pose.position
        self._put('pose_map', {
            'x': round(p.x, 4), 'y': round(p.y, 4),
            'theta': round(yaw_of(m.pose.pose.orientation), 4),
            'frame': m.header.frame_id or 'map',
        })

    _CHARGE = {0: 'unknown', 1: 'charging', 2: 'discharging',
               3: 'not_charging', 4: 'full'}

    def _on_battery(self, m: BatteryState) -> None:
        pct = m.percentage
        if pct == pct and pct <= 1.0:      # some stacks report 0-1, others 0-100
            pct *= 100.0
        charging = m.power_supply_status in (1, 4)
        self._put('battery', {
            'percentage': round(float(pct), 1) if pct == pct else None,
            'voltage': round(float(m.voltage), 2) if m.voltage == m.voltage else None,
            'current': round(float(m.current), 2) if m.current == m.current else None,
            'temperature': (round(float(m.temperature), 1)
                            if m.temperature == m.temperature and m.temperature != 0.0
                            else None),
            'status': self._CHARGE.get(m.power_supply_status, 'unknown'),
            'charging': charging,
        })
        self._check_battery_low(pct, charging)
        self._check_charging_edge(charging)

    def _check_charging_edge(self, charging: bool) -> None:
        now = time.time()
        if charging != self._charging_raw:
            self._charging_raw = charging
            self._charging_since = now
            return
        if charging == self._charging_shown or now - self._charging_since < CHARGING_DEBOUNCE_SEC:
            return
        if self._charging_shown is not None:  # skip the very first observation
            self.emit_event('charging.started' if charging else 'charging.completed')
        self._charging_shown = charging

    def _check_battery_low(self, pct: float, charging: bool) -> None:
        # NaN-safe: pct != pct is true only for NaN.
        raw = pct == pct and pct < BATTERY_LOW_PERCENT and not charging
        now = time.time()
        if raw != self._batt_low_raw:
            self._batt_low_raw = raw
            self._batt_low_since = now
            return
        if raw == self._batt_low_shown or now - self._batt_low_since < BATTERY_LOW_DEBOUNCE_SEC:
            return
        self._batt_low_shown = raw
        if raw:
            self.emit_event('battery.low', {'percentage': round(float(pct), 1)})

    def _on_battery_info(self, m: String) -> None:
        try:
            self._put('battery_info', json.loads(m.data))
        except (ValueError, TypeError):
            pass

    def _on_imu(self, m: Imu) -> None:
        self._put('imu', {
            'orientation': {'x': m.orientation.x, 'y': m.orientation.y,
                            'z': m.orientation.z, 'w': m.orientation.w},
            'angular_velocity': {'x': m.angular_velocity.x, 'y': m.angular_velocity.y,
                                 'z': m.angular_velocity.z},
            'linear_acceleration': {'x': m.linear_acceleration.x,
                                    'y': m.linear_acceleration.y,
                                    'z': m.linear_acceleration.z},
        })

    def _on_scan(self, m: LaserScan) -> None:
        # Full 720-beam scans are large and most callers want a shape, not
        # every beam; the raw arrays stay available over rosbridge.
        self._put('scan', {
            'angle_min': round(m.angle_min, 5),
            'angle_max': round(m.angle_max, 5),
            'angle_increment': round(m.angle_increment, 6),
            'range_min': round(m.range_min, 3),
            'range_max': round(m.range_max, 3),
            'count': len(m.ranges),
            'ranges': [None if not math.isfinite(r) else round(float(r), 3)
                       for r in m.ranges],
        })

    def _on_cpu_temp(self, m: Float32) -> None:
        self._put('cpu_temperature', round(float(m.data), 1))

    def _on_dock_status(self, m: String) -> None:
        self._put('dock_status', m.data)

    def _on_dock_pose(self, m: PoseStamped) -> None:
        p = m.pose.position
        self._put('dock_pose', {
            'x': round(p.x, 4), 'y': round(p.y, 4),
            'theta': round(yaw_of(m.pose.orientation), 4),
            'frame': m.header.frame_id or 'map',
        })

    def _on_dock_tag(self, m: Float32MultiArray) -> None:
        d = list(m.data)
        if len(d) < 7:
            return
        self._put('dock_tag', {
            'visible': d[0] > 0.5,
            'id': int(d[1]),
            'offset_px': round(d[2], 1),
            'size_px': round(d[4], 1),
            'bearing_rad': round(d[5], 4),
            'skew': round(d[6], 4),
        })

    def _on_plan(self, m: Path) -> None:
        self._put('plan', [{'x': round(p.pose.position.x, 3),
                            'y': round(p.pose.position.y, 3)} for p in m.poses])

    # -- outbound helpers ---------------------------------------------------

    def publish_cmd_vel(self, linear: float, angular: float) -> None:
        t = Twist()
        t.linear.x = float(linear)
        t.angular.z = float(angular)
        self._pub_cmd_vel.publish(t)

    def publish_initial_pose(self, x: float, y: float, theta: float,
                             frame: str = 'map') -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        qx, qy, qz, qw = quat_of(float(theta))
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        # Same covariance RViz's 2D Pose Estimate uses — AMCL expects a
        # meaningful spread here, and zeros make it over-trust the guess.
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.06853
        self._pub_initial.publish(msg)
