#!/usr/bin/env python3
"""Geometric, TF-aware LaserScan self-filter.

/scan -> [remove only confidently-identified robot self-geometry] -> /scan_filtered

Replaces a broad box/footprint exclusion filter with per-object ray
geometry: for every configured self-object (pillar, future wire, ...),
transform its geometry from base_link into the lidar frame, work out which
beams could possibly hit it, and for each of those beams compare the
MEASURED range to the range that object's own geometry would produce. Only
remove the point if they match within a small, configurable tolerance.

Everything else about a beam — including a real obstacle sitting right
next to a pillar, or a wall directly behind one on a DIFFERENT beam — is
left completely alone. See the module docstrings in geometry.py and
self_objects.py for the shape math and the config/transform pipeline; see
this package's README.md for the full safety rationale, the measurement
procedure for the pillars, and how to add future self-geometry.

SAFETY: if the base_link -> lidar TF cannot be looked up within
max_tf_age, or filtering is disabled, this node republishes /scan
UNCHANGED. It never guesses a transform, and never removes a point it
cannot geometrically justify against a currently-valid TF.
"""

from __future__ import annotations

import math
import time
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Point
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA, String
from tf2_ros import (
    ConnectivityException,
    ExtrapolationException,
    LookupException,
    TransformException,
)
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from . import geometry as geo
from . import self_objects as so

_SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)
# The /scan_filtered PUBLISHER, not the raw /scan subscription above:
# caught live — Nav2's real consumers (costmaps, AMCL) expect RELIABLE
# here, matching the existing scan_to_scan_filter_chain's own convention.
# Publishing BEST_EFFORT (this node's subscription QoS re-used without
# thinking) silently produced a QoS-incompatible match with every
# downstream subscriber — "New subscription discovered... requesting
# incompatible QoS. No messages will be sent to it." — found by actually
# running this against the robot before it ever touched the real filter.
_FILTERED_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


class SelfFilterNode(Node):
    def __init__(self) -> None:
        super().__init__('self_filter_node')

        p = self.declare_parameter
        p('scan_topic', '/scan')
        p('filtered_scan_topic', '/scan_filtered')
        p('base_frame', 'base_link')
        p('lidar_frame', 'lidar_1')
        p('filter_enabled', True)
        # How close a measured range must be to a self-object's expected
        # range to count as that object, not "possibly something else close
        # by". Kept small and explicit rather than folded into a vague
        # "exclusion zone" radius — see README Primary Safety Requirement.
        p('range_tolerance', 0.02)
        # Extra angular margin added to each object's own computed angular
        # extent, to absorb angular quantization and small TF/measurement
        # error without having to widen range_tolerance to compensate.
        p('angular_tolerance', 0.01)
        # Doubles as the TF lookup timeout: see _get_transform's docstring
        # for why staleness is judged by lookup success, not by comparing
        # the returned transform's header stamp to a wall-clock age.
        p('max_tf_age', 0.5)
        p('debug_enabled', False)
        p('debug_publish_removed_points', False)
        p('self_object_names', [''])  # ROS2 requires a typed default; see below

        self._scan_topic = str(self.get_parameter('scan_topic').value)
        self._filtered_topic = str(self.get_parameter('filtered_scan_topic').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._lidar_frame = str(self.get_parameter('lidar_frame').value)
        self._range_tolerance = float(self.get_parameter('range_tolerance').value)
        self._angular_tolerance = float(self.get_parameter('angular_tolerance').value)
        self._max_tf_age = float(self.get_parameter('max_tf_age').value)
        self._debug_enabled = bool(self.get_parameter('debug_enabled').value)
        self._debug_removed_points = bool(
            self.get_parameter('debug_publish_removed_points').value)

        # filter_enabled is re-read live (not cached) so it can be flipped
        # at runtime via `ros2 param set` without a relaunch — TEST 8 in
        # the README exercises exactly this.
        self._filter_enabled_param_name = 'filter_enabled'

        names = [n for n in self.get_parameter('self_object_names').value if n]
        self._object_names = names
        self._declare_object_params(names)
        self._object_configs = so.load_all(self._get_param_value, names)
        enabled_count = sum(1 for c in self._object_configs if c.enabled)
        self.get_logger().info(
            f'self_filter: {len(self._object_configs)} object(s) configured, '
            f'{enabled_count} enabled: '
            f'{[c.name for c in self._object_configs if c.enabled]}')

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Transformed-into-lidar-frame objects, refreshed whenever a scan
        # arrives and a valid TF is available (cheap — see class docstring
        # on why this is fine at scan rate).
        self._live_objects: List[geo.Circle | geo.Capsule | geo.Polygon] = []

        # Diagnostics, reset every scan.
        self._last_tf_ok = False
        self._tf_fail_count = 0

        self._pub = self.create_publisher(LaserScan, self._filtered_topic, _FILTERED_QOS)
        if self._debug_enabled:
            self._marker_pub = self.create_publisher(
                MarkerArray, '~/debug/self_geometry', 10)
            self._points_pub = (
                self.create_publisher(Marker, '~/debug/classified_points', 10)
                if self._debug_removed_points else None)
            self._stats_pub = self.create_publisher(String, '~/debug/stats', 10)
        else:
            self._marker_pub = None
            self._points_pub = None
            self._stats_pub = None

        self._sub = self.create_subscription(
            LaserScan, self._scan_topic, self._on_scan, _SENSOR_QOS)

        self.get_logger().info(
            f'self_filter: {self._scan_topic} -> {self._filtered_topic}, '
            f'base_frame={self._base_frame} lidar_frame={self._lidar_frame}, '
            f'range_tolerance={self._range_tolerance}m')

    # -- parameter plumbing ---------------------------------------------

    def _declare_object_params(self, names: List[str]) -> None:
        """Declare every field every object type might use, for every
        named object. Undeclared-but-unused fields (e.g. a circle's unused
        x1/y1/x2/y2) cost nothing and keep this simple — one declare pass
        covers any geometry_type a name might be configured with.
        """
        for name in names:
            self.declare_parameter(f'{name}.enabled', False)
            self.declare_parameter(f'{name}.geometry_type', '')
            self.declare_parameter(f'{name}.x', 0.0)
            self.declare_parameter(f'{name}.y', 0.0)
            self.declare_parameter(f'{name}.z', 0.0)
            self.declare_parameter(f'{name}.orientation', 0.0)
            self.declare_parameter(f'{name}.radius', 0.0)
            self.declare_parameter(f'{name}.x1', 0.0)
            self.declare_parameter(f'{name}.y1', 0.0)
            self.declare_parameter(f'{name}.z1', 0.0)
            self.declare_parameter(f'{name}.x2', 0.0)
            self.declare_parameter(f'{name}.y2', 0.0)
            self.declare_parameter(f'{name}.z2', 0.0)
            self.declare_parameter(f'{name}.points_x', [0.0])
            self.declare_parameter(f'{name}.points_y', [0.0])

    def _get_param_value(self, key: str, default):
        try:
            return self.get_parameter(key).value
        except Exception:  # noqa: BLE001 — genuinely optional, use default
            return default

    def _filter_enabled(self) -> bool:
        return bool(self.get_parameter(self._filter_enabled_param_name).value)

    # -- TF ---------------------------------------------------------------

    def _get_transform(self) -> Optional[Tuple[so.Vec3, so.Quat]]:
        """base_link -> lidar_frame, or None if it can't be had in time.

        max_tf_age is used as the LOOKUP TIMEOUT, not as a threshold
        compared against the returned transform's header stamp. That is a
        deliberate choice, not an oversight: this mount is a static
        transform (robot_state_publisher, from URDF), and tf2_ros returns
        static transforms carrying their ORIGINAL broadcast stamp — which
        can be hours old on a robot that has been running a while, even
        though the transform is completely valid right now. Comparing that
        stamp to a wall-clock age would make this filter spuriously treat
        a perfectly good, connected TF as "stale" and stop filtering,
        which helps no one. What actually matters for the safety
        requirement — "if TF is unavailable, do not filter" — is whether
        the lookup itself succeeds within a bounded wait, which is exactly
        what lookup_transform's timeout tests.
        """
        try:
            t = self._tf_buffer.lookup_transform(
                self._lidar_frame, self._base_frame, Time(),
                timeout=Duration(seconds=self._max_tf_age))
        except (LookupException, ConnectivityException,
                ExtrapolationException, TransformException) as e:
            self._tf_fail_count += 1
            if self._tf_fail_count == 1 or self._tf_fail_count % 100 == 0:
                self.get_logger().warn(
                    f'TF {self._base_frame} -> {self._lidar_frame} unavailable '
                    f'({e}) — passing /scan through unfiltered '
                    f'(x{self._tf_fail_count})')
            return None
        self._tf_fail_count = 0
        tr = t.transform.translation
        q = t.transform.rotation
        return ((tr.x, tr.y, tr.z), (q.x, q.y, q.z, q.w))

    # -- main callback ------------------------------------------------------

    def _on_scan(self, msg: LaserScan) -> None:
        if not self._filter_enabled():
            self._pub.publish(msg)
            return

        tf = self._get_transform()
        if tf is None:
            # SAFETY: unchanged pass-through. See class docstring.
            self._pub.publish(msg)
            self._last_tf_ok = False
            return
        self._last_tf_ok = True

        translation, rotation = tf
        live: List[geo.Circle | geo.Capsule | geo.Polygon] = []
        for cfg in self._object_configs:
            obj = so.transform_object(cfg, translation, rotation)
            if obj is not None:
                live.append(obj)
        self._live_objects = live

        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = list(msg.ranges)
        out.intensities = list(msg.intensities)

        n = len(out.ranges)
        removed_by: dict[str, int] = {}
        removed_total = 0
        removed_idx: List[int] = []

        margin = self._angular_tolerance + abs(msg.angle_increment)
        for obj in live:
            lo, hi = obj.angular_extent(margin)
            for i in self._beam_indices_in_window(
                    msg.angle_min, msg.angle_increment, n, lo, hi):
                r = out.ranges[i]
                # Deliberately NOT gating on msg.range_min here — caught
                # live: this lidar's driver reports real, consistent
                # numeric ranges below its own nominal range_min (0.15m),
                # exactly where these 8mm pillars physically sit
                # (~0.12m). Gating on the spec's nominal minimum made the
                # filter unable to ever match the very geometry it exists
                # to remove. range_max is still respected — a reading
                # past it is a real "no valid return", not a self-object.
                if r is None or r != r or r <= 0.0 or r > msg.range_max:
                    continue  # already no valid return — nothing to compare
                theta = msg.angle_min + i * msg.angle_increment
                expected = obj.expected_range(theta)
                if expected is None:
                    continue
                if abs(r - expected) <= self._range_tolerance:
                    # CONFIDENT match -> remove. NaN is the LaserScan spec's
                    # own documented value for "erroneous, invalid, or
                    # missing measurement" — costmap layers correctly skip
                    # a NaN beam for BOTH marking and clearing, which is
                    # exactly right here: we know a self-object explains
                    # this return, we do NOT know what (if anything) is
                    # beyond it, so we must not claim it is clear either.
                    out.ranges[i] = float('nan')
                    removed_by[obj.name] = removed_by.get(obj.name, 0) + 1
                    removed_total += 1
                    removed_idx.append(i)

        self._pub.publish(out)

        if self._debug_enabled:
            self._publish_debug(msg, live, removed_idx, removed_by, removed_total)

    @staticmethod
    def _beam_indices_in_window(angle_min: float, angle_increment: float,
                                n: int, lo: float, hi: float) -> List[int]:
        """Which beam indices fall in [lo, hi] — the pruning step that
        keeps this cheap. See geometry.py's module docstring.
        """
        if n == 0 or angle_increment == 0.0:
            return []
        indices = []
        # Try the direct window and both +-2pi wraps, since lo/hi can land
        # outside [angle_min, angle_min + n*angle_increment) near the seam.
        for shift in (0.0, 2 * math.pi, -2 * math.pi):
            lo_s, hi_s = lo + shift, hi + shift
            i_lo = math.floor((lo_s - angle_min) / angle_increment)
            i_hi = math.ceil((hi_s - angle_min) / angle_increment)
            for i in range(max(0, i_lo), min(n - 1, i_hi) + 1):
                indices.append(i)
        return indices

    # -- debug ----------------------------------------------------------

    def _publish_debug(self, msg: LaserScan,
                       live: List[geo.Circle | geo.Capsule | geo.Polygon],
                       removed_idx: List[int], removed_by: dict,
                       removed_total: int) -> None:
        stamp = msg.header.stamp
        markers = MarkerArray()
        mid = 0
        for obj in live:
            m = Marker()
            m.header.frame_id = self._lidar_frame
            m.header.stamp = stamp
            m.ns = 'self_geometry'
            m.id = mid
            mid += 1
            m.action = Marker.ADD
            m.color = ColorRGBA(r=0.2, g=0.8, b=1.0, a=0.6)
            m.pose.orientation.w = 1.0
            if isinstance(obj, geo.Circle):
                m.type = Marker.CYLINDER
                m.pose.position.x = obj.cx
                m.pose.position.y = obj.cy
                m.scale.x = m.scale.y = obj.radius * 2.0
                m.scale.z = 0.2
            elif isinstance(obj, geo.Capsule):
                m.type = Marker.LINE_STRIP
                m.scale.x = max(obj.radius * 2.0, 0.005)
                m.points = [Point(x=obj.ax, y=obj.ay, z=0.0),
                           Point(x=obj.bx, y=obj.by, z=0.0)]
            elif isinstance(obj, geo.Polygon):
                m.type = Marker.LINE_STRIP
                m.scale.x = 0.005
                m.points = [Point(x=x, y=y, z=0.0) for (x, y) in obj.points]
                if obj.points:
                    m.points.append(Point(x=obj.points[0][0], y=obj.points[0][1], z=0.0))
            markers.markers.append(m)
        if self._marker_pub is not None:
            self._marker_pub.publish(markers)

        if self._points_pub is not None:
            pm = Marker()
            pm.header.frame_id = self._lidar_frame
            pm.header.stamp = stamp
            pm.ns = 'classified_points'
            pm.id = 0
            pm.type = Marker.POINTS
            pm.action = Marker.ADD
            pm.scale.x = pm.scale.y = 0.03
            pm.pose.orientation.w = 1.0
            removed_set = set(removed_idx)
            for i, r in enumerate(msg.ranges):
                # Same validity rule as the real filter above — see its
                # comment on why msg.range_min is not part of this check.
                if r is None or r != r or r <= 0.0 or r > msg.range_max:
                    continue
                theta = msg.angle_min + i * msg.angle_increment
                x, y = r * math.cos(theta), r * math.sin(theta)
                pm.points.append(Point(x=x, y=y, z=0.0))
                removed = i in removed_set
                pm.colors.append(ColorRGBA(
                    r=1.0 if removed else 0.0,
                    g=0.0 if removed else 1.0,
                    b=0.0, a=0.9))
            self._points_pub.publish(pm)

        if self._stats_pub is not None:
            raw_valid = sum(
                1 for r in msg.ranges
                if r is not None and r == r and 0.0 < r <= msg.range_max)
            parts = [f'raw_points={raw_valid}',
                    f'removed_points={removed_total}',
                    f'filtered_points={raw_valid - removed_total}']
            for obj in live:
                parts.append(f'removed_by_{obj.name}={removed_by.get(obj.name, 0)}')
            self._stats_pub.publish(String(data=', '.join(parts)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SelfFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
