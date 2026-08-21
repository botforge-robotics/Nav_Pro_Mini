#!/usr/bin/env python3
"""Lidar-based dock pose detection for opennav_docking's SimpleChargingDock.

Hardware: RPLidar A1M8 — 360deg, 720 beams (0.499deg/beam), range_min 0.15m.
Mounted at base_link + (0.0325, 0.0013, 0.189), no rotation (verified live
from /tf_static).

The dock is a rounded rectangle: ~21.5cm side-to-side, ~8cm front-to-back,
with a flat front (connector) face. The robot backs INTO it, so the lidar
sees that flat front face throughout the approach. At 0.30m that face spans
~39deg (~78 beams); at 0.15m, ~71deg (~142 beams) — plenty of points to fit
a line to, which is what this node does.

Why a line fit rather than a cluster centroid (the previous approach):

  * Orientation is MEASURED, not inherited. The old version copied yaw
    straight from the user-placed bookmark prior, so any error in how the
    bookmark was rotated fed directly into docking_server's approach
    controller and the robot arrived skewed ("moved back cross"). A line fit
    over the flat face recovers the dock's true normal from the sensor.
  * The centroid of a partial/asymmetric return is biased toward whichever
    part of the face happens to reflect; a fitted line's midpoint is not.
  * Fit width is a much stronger validity check than max-pairwise-extent,
    which happily accepted two unrelated clusters ~35cm apart (observed live
    as detected_dock_pose oscillating between two points every other scan).

Scan topic: RAW /scan by design, NOT /scan_filtered. The filtered topic runs
a LaserScanFootprintFilter with inscribed_radius 0.13 plus a robot-body box
filter, which deletes returns close to the chassis — exactly the ones that
matter as the dock closes to within ~15-20cm. Measured live: /scan_filtered
drops roughly half of all sub-0.35m returns vs /scan.

Blind zone: nothing closer than range_min (0.15m) is visible at all, so the
final ~15cm is necessarily open-loop — mechanical funnel guides plus the
battery-state check own that segment, not this node.

Publishes geometry_msgs/PoseStamped on `detected_dock_pose` (the fixed,
unremapped topic SimpleChargingDock subscribes to) whenever a valid dock face
is found; stays silent otherwise, letting docking_server's own
external_detection_timeout handle gaps.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
import tf2_geometry_msgs  # noqa: F401 — registers PoseStamped transform support
import tf2_ros
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformException

# Dock footprint (meters).
_DOCK_WIDTH = 0.215
_DOCK_DEPTH = 0.08


@dataclass
class _FaceFit:
    """A line fitted to a candidate dock front face, in scan frame."""

    cx: float          # midpoint of the fitted segment
    cy: float
    normal: float      # yaw of the face normal, pointing back toward the lidar
    width: float       # extent of the points along the face
    rms: float         # perpendicular fit residual — flatness measure
    n_points: int


class DockDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__('navpromini_dock_detector')

        # Raw /scan by default — see module docstring on why not /scan_filtered.
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('search_radius', 0.60)
        self.declare_parameter('cluster_break_dist', 0.06)
        self.declare_parameter('min_cluster_points', 6)
        self.declare_parameter('max_range_consider', 1.5)
        # Accept a partially-observed face (rounded sides may not return) but
        # reject anything clearly not dock-sized. The floor also acts as a
        # backstop against the wrap-around split described in _cluster():
        # measured live, the whole face fits 0.200m and a single half fits
        # 0.103m, so 0.14 separates them cleanly even if a merge is missed.
        self.declare_parameter('min_face_width', 0.14)
        self.declare_parameter('max_face_width', 0.30)
        # A real flat face fits a line tightly; reject curved/ragged clutter.
        self.declare_parameter('max_fit_rms', 0.015)
        # Tracking gate: once locked on, prefer candidates near the previous
        # detection over anything else (the robot creeps ~3mm/scan at docking
        # speed, so this is generous).
        self.declare_parameter('tracking_gate', 0.15)
        # The robot docks backwards, so the dock is necessarily BEHIND it
        # (bearing ~180deg) from staging through contact. Rejecting anything
        # outside a rear cone is the one check that reliably separates the
        # dock from other flat, dock-width furniture nearby — measured live,
        # a bad search prior made the detector lock onto a surface at
        # bearing -97deg (off to the side) and the tracking gate then held it
        # there for the whole approach. Set to 180 to disable if this robot
        # is ever configured to dock forwards.
        self.declare_parameter('max_rear_angle_deg', 60.0)
        # The bookmark pose is taken to sit at the dock BODY centre, while the
        # lidar only ever sees the front face — shift the measurement back by
        # half the dock depth so both refer to the same point. Set to 0.0 if
        # the bookmark is instead placed on the face itself.
        self.declare_parameter('face_to_center_offset', _DOCK_DEPTH / 2.0)

        scan_topic = str(self.get_parameter('scan_topic').value)
        self._search_radius = float(self.get_parameter('search_radius').value)
        self._cluster_break_dist = float(self.get_parameter('cluster_break_dist').value)
        self._min_cluster_points = int(self.get_parameter('min_cluster_points').value)
        self._max_range_consider = float(self.get_parameter('max_range_consider').value)
        self._min_face_width = float(self.get_parameter('min_face_width').value)
        self._max_face_width = float(self.get_parameter('max_face_width').value)
        self._max_fit_rms = float(self.get_parameter('max_fit_rms').value)
        self._tracking_gate = float(self.get_parameter('tracking_gate').value)
        self._max_rear_angle = math.radians(
            float(self.get_parameter('max_rear_angle_deg').value))
        self._face_to_center = float(self.get_parameter('face_to_center_offset').value)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._expected_pose: Optional[PoseStamped] = None
        self._last_detected: Optional[Tuple[float, float]] = None
        self.create_subscription(PoseStamped, 'dock_expected_pose', self._on_expected_pose, 10)

        sensor_qos = QoSProfile(
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.create_subscription(LaserScan, scan_topic, self._on_scan, sensor_qos)
        self._detected_pub = self.create_publisher(PoseStamped, 'detected_dock_pose', 10)

        self.get_logger().info(
            f'dock_detector: scan={scan_topic} search_radius={self._search_radius}m '
            f'face_width={_DOCK_WIDTH}m (accept {self._min_face_width}-{self._max_face_width}m)'
        )

    def _on_expected_pose(self, msg: PoseStamped) -> None:
        self._expected_pose = msg
        self._last_detected = None  # new attempt — drop any prior lock-on
        self.get_logger().info(
            f'dock_expected_pose received: frame={msg.header.frame_id} '
            f'x={msg.pose.position.x:.2f} y={msg.pose.position.y:.2f}'
        )

    # -- main pipeline ---------------------------------------------------

    def _on_scan(self, scan: LaserScan) -> None:
        if self._expected_pose is None:
            return

        expected = self._expected_pose_in_scan_frame(scan.header.frame_id)
        if expected is None:
            return
        ex, ey = expected

        # Search around the tracked detection once locked on, else the prior.
        if self._last_detected is not None:
            sx, sy = self._last_detected
            window = self._tracking_gate
        else:
            sx, sy = ex, ey
            window = self._search_radius

        # Only close the cluster loop for a genuinely 360deg scan.
        wrap = (scan.angle_max - scan.angle_min) >= (2.0 * math.pi - 0.2)

        points = self._scan_points_near(scan, sx, sy, window)
        fit = self._best_face(points, sx, sy, wrap)

        if fit is None and self._last_detected is not None:
            # Lost the tracked face — fall back to a full re-acquire against
            # the prior rather than going silent until it reappears.
            points = self._scan_points_near(scan, ex, ey, self._search_radius)
            fit = self._best_face(points, ex, ey, wrap)

        if fit is None:
            return

        self._last_detected = (fit.cx, fit.cy)

        # Shift from the observed face back to the dock body centre.
        cx = fit.cx - self._face_to_center * math.cos(fit.normal)
        cy = fit.cy - self._face_to_center * math.sin(fit.normal)

        pose = PoseStamped()
        pose.header.frame_id = scan.header.frame_id
        pose.header.stamp = scan.header.stamp
        pose.pose.position.x = cx
        pose.pose.position.y = cy
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(fit.normal / 2.0)
        pose.pose.orientation.w = math.cos(fit.normal / 2.0)
        self._detected_pub.publish(pose)

    def _expected_pose_in_scan_frame(self, scan_frame: str) -> Optional[Tuple[float, float]]:
        msg = self._expected_pose
        try:
            if msg.header.frame_id == scan_frame:
                pose = msg.pose
            else:
                tf = self._tf_buffer.lookup_transform(
                    scan_frame, msg.header.frame_id, rclpy.time.Time(),
                )
                pose = tf2_geometry_msgs.do_transform_pose(msg.pose, tf)
        except TransformException as exc:
            self.get_logger().warn(f'TF lookup failed: {exc}', throttle_duration_sec=5.0)
            return None
        return pose.position.x, pose.position.y

    def _scan_points_near(self, scan: LaserScan, cx: float, cy: float,
                          window: float) -> List[Tuple[float, float]]:
        pts: List[Tuple[float, float]] = []
        max_r = min(self._max_range_consider, scan.range_max)
        angle = scan.angle_min
        for r in scan.ranges:
            if scan.range_min <= r <= max_r:
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                if math.hypot(x - cx, y - cy) <= window:
                    pts.append((x, y))
            angle += scan.angle_increment
        return pts

    def _cluster(self, points: List[Tuple[float, float]],
                 wrap: bool) -> List[List[Tuple[float, float]]]:
        """Split by gaps. Points arrive in scan (angular) order, so adjacent
        entries are angularly adjacent and a distance jump means a new object.

        [wrap] closes the loop between the last and first entries. This is
        essential here and not a nicety: the robot backs in, so the dock sits
        at bearing ~180deg, which is exactly where this lidar's array
        starts/ends (angle_min -179deg, angle_max +180deg). Without it the
        dock's returns land at opposite ends of the array and can never join,
        so the fit locks onto one HALF of the face.

        Measured live with the robot staged square-on 0.48m from the dock:
          without wrap  -> width 0.103m, centre y = +0.051m  (half the dock)
          with wrap     -> width 0.200m, centre y = +0.001m  (the whole face)
        That 5cm lateral bias — flipping sign as the fit jumped between
        halves — is what sent the robot in crooked past the funnel guides.
        """
        clusters: List[List[Tuple[float, float]]] = []
        current: List[Tuple[float, float]] = []
        for pt in points:
            if current and math.dist(pt, current[-1]) > self._cluster_break_dist:
                clusters.append(current)
                current = []
            current.append(pt)
        if current:
            clusters.append(current)

        if wrap and len(clusters) > 1:
            first, last = clusters[0], clusters[-1]
            if math.dist(first[0], last[-1]) <= self._cluster_break_dist:
                clusters[0] = last + first
                clusters.pop()

        return [c for c in clusters if len(c) >= self._min_cluster_points]

    def _fit_face(self, cluster: List[Tuple[float, float]]) -> Optional[_FaceFit]:
        """Total-least-squares line fit (PCA) over one cluster."""
        n = len(cluster)
        mx = sum(p[0] for p in cluster) / n
        my = sum(p[1] for p in cluster) / n
        sxx = sum((p[0] - mx) ** 2 for p in cluster)
        syy = sum((p[1] - my) ** 2 for p in cluster)
        sxy = sum((p[0] - mx) * (p[1] - my) for p in cluster)

        # Principal axis = direction along the face.
        along = 0.5 * math.atan2(2.0 * sxy, sxx - syy)
        ux, uy = math.cos(along), math.sin(along)
        # Normal is perpendicular to it.
        nx, ny = -uy, ux

        # Orient the normal back toward the lidar (which sits at the origin):
        # the face we can see must point at us.
        if nx * (0.0 - mx) + ny * (0.0 - my) < 0.0:
            nx, ny = -nx, -ny

        # Extent along the face, and flatness across it.
        ts = [(p[0] - mx) * ux + (p[1] - my) * uy for p in cluster]
        ds = [(p[0] - mx) * nx + (p[1] - my) * ny for p in cluster]
        t_min, t_max = min(ts), max(ts)
        width = t_max - t_min
        rms = math.sqrt(sum(d * d for d in ds) / n)

        # Midpoint of the observed segment (not the raw centroid, which skews
        # toward whichever end returned more points).
        t_mid = 0.5 * (t_min + t_max)
        cx = mx + t_mid * ux
        cy = my + t_mid * uy

        return _FaceFit(cx=cx, cy=cy, normal=math.atan2(ny, nx),
                        width=width, rms=rms, n_points=n)

    def _best_face(self, points: List[Tuple[float, float]],
                   ref_x: float, ref_y: float, wrap: bool) -> Optional[_FaceFit]:
        best: Optional[_FaceFit] = None
        best_score = float('inf')
        for cluster in self._cluster(points, wrap):
            fit = self._fit_face(cluster)
            if fit is None:
                continue
            if not (self._min_face_width <= fit.width <= self._max_face_width):
                continue
            if fit.rms > self._max_fit_rms:
                continue
            # Must lie behind the robot — see max_rear_angle_deg.
            bearing = math.atan2(fit.cy, fit.cx)
            rear_err = abs(math.atan2(math.sin(bearing - math.pi),
                                      math.cos(bearing - math.pi)))
            if rear_err > self._max_rear_angle:
                continue
            width_err = abs(fit.width - _DOCK_WIDTH)
            dist_err = math.hypot(fit.cx - ref_x, fit.cy - ref_y)
            # Flatness and correct width matter more than raw proximity, which
            # the search window already bounds.
            score = 3.0 * width_err + dist_err + 10.0 * fit.rms
            if score < best_score:
                best_score = score
                best = fit
        return best


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = DockDetectorNode()
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
