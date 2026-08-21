#!/usr/bin/env python3
"""AprilTag 36h11 detection for the rear dock camera.

Publishes `dock_tag` (std_msgs/Float32MultiArray):

    [0] found          1.0 / 0.0
    [1] id
    [2] dx_px          tag centre minus image centre, +right
    [3] dy_px          tag centre minus image centre, +down
    [4] side_px        mean edge length — a distance proxy
    [5] bearing_rad    horizontal angle to the tag, + = tag is to the right
    [6] skew           tag's own yaw indicator, + = we view it from its right
    [7] age_ms         0 (present sample)
    [8] image_width
    [9] image_height

Why these and not a full 6-DoF pose: pose estimation needs real intrinsics,
and this camera is running a *guessed* pinhole model (see camera_node's
_make_camera_info). Range from uncalibrated intrinsics can be tens of percent
out. But `dx_px` crosses zero exactly when the camera points at the tag,
whatever the focal length is — so a controller that servos dx to zero is
correct today, and simply gets better if the camera is calibrated later.

`skew` is the piece neither the IR pair nor the lidar gave us cheaply: the
robot's heading relative to the dock FACE. The two vertical edges of a square
tag project to different lengths unless viewed square-on, and the difference
is signed. IR yields a purely lateral error (spec section 4a: heading is not
observable), so a crooked approach was invisible — this makes it measurable.
"""

from __future__ import annotations

import time
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_msgs.msg import Float32MultiArray

_SENSOR_QOS = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=1)


def _detector_params():
    """Detector settings, kept CHEAP on purpose.

    An earlier version widened the adaptive-threshold sweep to
    min=3/max=53/step=4 and raised perspectiveRemovePixelPerCell to 8. That is
    13 full threshold passes over a 1280x720 image every frame, and on this Pi
    — already running Nav2 — it drove load average to 9 and collapsed this
    topic from 15Hz to 1.8Hz. tag_dock_node then saw only stale data and
    aborted every goal with "dock tag not visible".

    It bought nothing: measured side by side, default and widened parameters
    both detected 100% of frames. So the sweep is back to a narrow range and
    the expensive per-cell sampling is back to default. Only the genuinely
    cheap, genuinely useful settings are kept.

    NOT using CORNER_REFINE_APRILTAG either — it segfaults OpenCV 4.6 on some
    frames (reproduced: a live run dumped core).
    """
    p = cv2.aruco.DetectorParameters_create()
    p.adaptiveThreshWinSizeMin = 3
    p.adaptiveThreshWinSizeMax = 23
    p.adaptiveThreshWinSizeStep = 10        # 3 passes, the OpenCV default
    p.minMarkerPerimeterRate = 0.02         # cheap: see the tag further away
    p.polygonalApproxAccuracyRate = 0.05    # cheap: tolerate soft corners
    p.errorCorrectionRate = 0.8             # cheap: 36h11 has distance to spare
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return p


class DockTagNode(Node):
    def __init__(self) -> None:
        super().__init__('navpromini_dock_tag')

        p = self.declare_parameter
        # The COMPRESSED stream, not raw. Raw 720p bgr8 is 2.76MB per
        # message; at 15Hz that is 41MB/s through DDS, and with depth=1
        # best-effort most frames are dropped before they arrive — measured,
        # this topic ran at ~1Hz while this node sat at 4% CPU, i.e. starved
        # of images rather than short of compute. The JPEG is ~100kB, 30x
        # less, and decoding it costs a few ms. Detection accuracy is
        # unaffected: 100% detection was measured off this same compressed
        # stream.
        p('image_topic', 'camera/image_raw/compressed')
        p('camera_info_topic', 'camera/camera_info')
        p('tag_id', 0)
        p('tag_size_m', 0.08)
        # Fall back only until camera_info arrives. 673px is the Lenovo 300
        # FHD's 95-degree diagonal FOV at 1280x720, not the image width — the
        # old 1280 placeholder was ~2x too large and halved every bearing.
        p('default_fx', 673.0)
        # Detect at this rate, not at camera frame rate. The controller
        # re-measures every few seconds, so 5Hz is ample, and it leaves the
        # CPU for Nav2 — detection at full 15fps was starving the whole robot.
        p('detect_rate_hz', 5.0)
        # Search only a window around the last detection instead of the whole
        # 1280x720 frame. AprilTag cost scales with pixel count, and the tag
        # moves a few pixels between frames at these speeds, so a padded ROI
        # finds it just as reliably for a fraction of the work. Falls back to
        # a full-frame search the moment the tag is lost, so nothing is
        # permanently missed — this is a speed optimisation, not a tracker
        # that can get stuck.
        p('roi_enabled', True)
        p('roi_pad_factor', 1.6)       # window = tag side x this, each way
        p('roi_min_px', 160)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self._want_id = int(g('tag_id'))
        self._tag_size = float(g('tag_size_m'))
        self._fx = float(g('default_fx'))
        self._min_period = 1.0 / max(float(g('detect_rate_hz')), 0.1)
        self._last_detect = 0.0
        self._roi_enabled = bool(g('roi_enabled'))
        self._roi_pad = float(g('roi_pad_factor'))
        self._roi_min = int(g('roi_min_px'))
        self._roi: Optional[tuple] = None      # (x0, y0, x1, y1)
        self._roi_hits = 0
        self._full_scans = 0

        self._dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        self._params = _detector_params()

        self.create_subscription(CompressedImage, str(g('image_topic')),
                                 self._on_image, _SENSOR_QOS)
        self.create_subscription(CameraInfo, str(g('camera_info_topic')),
                                 self._on_info, 10)
        self._pub = self.create_publisher(Float32MultiArray, 'dock_tag', 10)

        self._seen = 0
        self._frames = 0
        self.create_timer(5.0, self._report)
        self.get_logger().info(
            f'dock_tag up: looking for AprilTag 36h11 id={self._want_id}, '
            f'{self._tag_size * 1000:.0f}mm — publishing dock_tag'
        )

    def _on_info(self, msg: CameraInfo) -> None:
        # msg.k is a numpy array — truth-testing it raises.
        if len(msg.k) >= 1 and float(msg.k[0]) > 1.0:
            self._fx = float(msg.k[0])

    def _report(self) -> None:
        if self._frames:
            self.get_logger().info(
                f'dock_tag: {self._seen}/{self._frames} frames '
                f'({100.0 * self._seen / self._frames:.0f}%), '
                f'{self._roi_hits} via ROI / {self._full_scans} full scans'
            )
        self._seen = self._frames = 0
        self._roi_hits = self._full_scans = 0

    def _on_image(self, msg: CompressedImage) -> None:
        now = time.monotonic()
        if now - self._last_detect < self._min_period:
            return
        self._last_detect = now
        self._frames += 1
        # Decode straight to greyscale — the detector never needs colour, and
        # this skips a full BGR->GRAY conversion of a 720p frame.
        gray = cv2.imdecode(np.frombuffer(msg.data, np.uint8),
                            cv2.IMREAD_GRAYSCALE)
        if gray is None:
            return
        h, w = gray.shape[:2]

        # ROI first, full frame only if that misses.
        off_x = off_y = 0
        corners = ids = None
        if self._roi_enabled and self._roi is not None:
            x0, y0, x1, y1 = self._roi
            sub = gray[y0:y1, x0:x1]
            if sub.size:
                corners, ids, _ = cv2.aruco.detectMarkers(
                    sub, self._dict, parameters=self._params)
                if ids is not None and len(ids):
                    off_x, off_y = x0, y0
                    self._roi_hits += 1
                else:
                    corners = ids = None
        if ids is None:
            self._full_scans += 1
            self._roi = None
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self._dict, parameters=self._params)

        out = [0.0] * 10
        out[8], out[9] = float(w), float(h)

        if ids is not None and len(ids):
            pick = None
            for c, i in zip(corners, ids.flatten()):
                if int(i) == self._want_id:
                    pick = c
                    break
            if pick is not None:
                self._seen += 1
                q = pick.reshape(4, 2).copy()
                q[:, 0] += off_x
                q[:, 1] += off_y
                tx, ty = float(q[:, 0].mean()), float(q[:, 1].mean())
                dx, dy = tx - w / 2.0, ty - h / 2.0
                # Corner order from detectMarkers is clockwise from top-left.
                left = float(np.linalg.norm(q[0] - q[3]))
                right = float(np.linalg.norm(q[1] - q[2]))
                top = float(np.linalg.norm(q[0] - q[1]))
                bottom = float(np.linalg.norm(q[2] - q[3]))
                side = 0.25 * (left + right + top + bottom)
                denom = left + right
                skew = (left - right) / denom if denom > 1e-6 else 0.0
                out[0] = 1.0
                out[1] = float(self._want_id)
                out[2] = dx
                out[3] = dy
                out[4] = side
                out[5] = float(np.arctan2(dx, self._fx))
                out[6] = skew

                if self._roi_enabled:
                    half = max(self._roi_min, int(side * self._roi_pad))
                    self._roi = (max(0, int(tx) - half), max(0, int(ty) - half),
                                 min(w, int(tx) + half), min(h, int(ty) + half))
            else:
                self._roi = None
        else:
            self._roi = None

        m = Float32MultiArray()
        m.data = out
        self._pub.publish(m)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = DockTagNode()
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
