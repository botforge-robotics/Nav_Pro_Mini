#!/usr/bin/env python3
"""Rear USB camera publisher.

Feeds the rear-facing webcam into ROS so the dock's AprilTag can be seen (and
later detected) from the robot. Publishes:

    /camera/image_raw              sensor_msgs/Image        (bgr8)
    /camera/image_raw/compressed   sensor_msgs/CompressedImage (jpeg)
    /camera/camera_info            sensor_msgs/CameraInfo

Raw and compressed are both published on purpose. DDS only puts a topic on the
wire when something subscribes, so the raw stream stays local to the Pi (shared
memory) for on-robot processing, while a laptop viewing over WiFi subscribes to
the compressed topic and costs a fraction of the bandwidth. That distinction
matters here: this robot has already dropped off WiFi twice mid-run, and a raw
640x480 bgr8 stream at 15fps is ~74 Mbit/s — enough to cause exactly that.
Always view `/camera/image_raw/compressed` from the PC, never `/camera/image_raw`.

The video device itself is only opened while docking actually needs it — see
`camera/set_active` below. Nothing else in this stack consumes this camera
(there's no live-view feature; the app's own docking status is served from
the SDK, not the raw feed), so there is no reason to keep a USB webcam capture
running, drawing power and CPU, for the ~99% of a robot's life it spends idle,
navigating, or already docked. It also sidesteps a real, previously
unverifiable suspicion: some UVC webcams' auto-exposure can drift after many
hours of continuous capture (no exposure/gain control is set here — it's
whatever the driver's auto mode decides), which reads exactly like "docking
worked fine earlier in the day, then stopped finding the tag." Re-opening the
device fresh on every dock attempt gives auto-exposure a clean start every
time instead of letting it run for hours unattended.
"""

from __future__ import annotations

import math
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_srvs.srv import SetBool

# Video is loss-tolerant and latency-sensitive: a dropped frame is better than
# a retransmit stalling the stream, especially on a link this flaky.
_VIDEO_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


class CameraNode(Node):
    def __init__(self) -> None:
        super().__init__('navpromini_camera')

        p = self.declare_parameter
        p('device', 0)
        # 1280x720, not 640x480. Detection measured 100% at 133px of tag
        # side, but tag side scales with 1/distance: the same 80mm tag at 1.5m
        # is only ~45px at VGA, and AprilTag 36h11 needs roughly 5 pixels per
        # bit cell (8 cells + border) to decode reliably. 720p doubles the
        # working distance for the same reliability, which is what decides
        # whether the marker is usable during an approach rather than only at
        # the dock.
        p('width', 1280)
        p('height', 720)
        # 15, which this camera advertises. Do NOT set an arbitrary rate here:
        # dropping it to 8 to save CPU made the stream collapse to ~0.4 fps —
        # UVC devices negotiate from a fixed mode list, and asking for a rate
        # outside it degrades rather than rounds. CPU is saved by throttling
        # DETECTION (dock_tag_node's detect_rate_hz) instead, which is free of
        # that constraint.
        p('fps', 15.0)
        # Raised from 70: JPEG ringing lands hardest on exactly the
        # high-contrast bit edges the decoder samples.
        p('jpeg_quality', 80)
        # The camera faces backward and may be mounted inverted; flip here
        # rather than in every consumer, so the AprilTag detector and the
        # human looking at rqt agree on which way is up.
        p('rotate_180', False)
        p('flip_horizontal', False)
        p('frame_id', 'camera_link')
        # Off by default: dock_tag_node consumes the compressed stream, and
        # publishing raw 720p bgr8 pushed 2.76MB per frame through DDS for no
        # subscriber. Enable only for a consumer that genuinely needs
        # uncompressed pixels on this machine.
        p('publish_raw', False)
        # Lenovo 300 FHD (GXC1B34793): 95 degrees, quoted as the wide-angle
        # figure and treated here as DIAGONAL, which is the webcam
        # convention. Used to derive focal length instead of the old
        # placeholder fx = image width, which overestimated it by ~2x.
        p('fov_diagonal_deg', 95.0)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self._w = int(g('width'))
        self._h = int(g('height'))
        self._fps = float(g('fps'))
        self._quality = int(g('jpeg_quality'))
        self._rot180 = bool(g('rotate_180'))
        self._flip_h = bool(g('flip_horizontal'))
        self._frame_id = str(g('frame_id'))
        self._publish_raw = bool(g('publish_raw'))
        self._fov_diag = float(g('fov_diagonal_deg'))

        self._bridge = CvBridge()
        self._device = int(g('device'))
        self._cap: Optional[cv2.VideoCapture] = None
        self._active = False

        self._pub_raw = (self.create_publisher(Image, 'camera/image_raw', _VIDEO_QOS)
                         if self._publish_raw else None)
        self._pub_jpg = self.create_publisher(
            CompressedImage, 'camera/image_raw/compressed', _VIDEO_QOS)
        self._pub_info = self.create_publisher(CameraInfo, 'camera/camera_info', 10)

        self._info = self._make_camera_info()
        self._fail_streak = 0
        self.create_service(SetBool, 'camera/set_active', self._on_set_active)
        # One persistent timer regardless of active state — cheap to leave
        # running (an inactive tick is a single bool check), and avoids the
        # per-call timer create/destroy churn a previous tick_dock_node fix
        # elsewhere in this stack found to be a real CPU cost on this Pi.
        self.create_timer(1.0 / max(self._fps, 1.0), self._tick)
        self.get_logger().info(
            f'camera ready: {self._w}x{self._h} @{self._fps:g}fps, jpeg q={self._quality}, '
            f'fx~{(math.hypot(self._w, self._h) / 2.0) / math.tan(math.radians(self._fov_diag) / 2.0):.0f}px '
            '— inactive until a dock attempt calls camera/set_active(true)'
        )

    def _on_set_active(self, request: SetBool.Request,
                       response: SetBool.Response) -> SetBool.Response:
        if request.data:
            response.success = self._activate()
            response.message = '' if response.success else (
                f'cannot open video device {self._device}')
        else:
            self._deactivate()
            response.success = True
        return response

    def _activate(self) -> bool:
        if self._active:
            return True
        cap = cv2.VideoCapture(self._device, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            self.get_logger().error(f'cannot open video device {self._device}')
            return False
        # MJPG so the camera does the JPEG work instead of the Pi's CPU.
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._h)
        cap.set(cv2.CAP_PROP_FPS, self._fps)
        # Keep the grab queue short: a backlog shows the operator where the
        # robot *was*, which is worse than useless while aligning a marker.
        try:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:  # noqa: BLE001 — not supported on every backend
            pass
        self._cap = cap
        self._fail_streak = 0
        self._active = True
        self.get_logger().info('camera activated')
        return True

    def _deactivate(self) -> None:
        if not self._active:
            return
        self._active = False
        cap, self._cap = self._cap, None
        if cap is not None:
            try:
                cap.release()
            except Exception:  # noqa: BLE001
                pass
        self.get_logger().info('camera deactivated')

    def _make_camera_info(self) -> CameraInfo:
        """Intrinsics ESTIMATED from the camera's published field of view.

        Focal length comes from the datasheet FOV and the image diagonal, not
        from a calibration: f = (diag/2) / tan(fov/2). For this camera that
        gives fx ~= 673 at 1280x720, where the previous placeholder used
        fx = image width = 1280 — nearly double, which halved every reported
        bearing.

        Still NOT a calibration. Distortion is assumed zero, and a 95 degree
        lens has real barrel distortion toward the edges. It happens to matter
        least where this system uses it: the controller servos the tag to the
        image CENTRE, where distortion is smallest and where the zero-crossing
        is exact regardless of focal length. Absolute range from side_px, and
        any 6-DoF pose, still need a checkerboard calibration.
        """
        info = CameraInfo()
        info.header.frame_id = self._frame_id
        info.width = self._w
        info.height = self._h
        diag = math.hypot(float(self._w), float(self._h))
        fx = fy = (diag / 2.0) / math.tan(math.radians(self._fov_diag) / 2.0)
        cx, cy = self._w / 2.0, self._h / 2.0
        info.distortion_model = 'plumb_bob'
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def _tick(self) -> None:
        if not self._active or self._cap is None:
            return
        ok, frame = self._cap.read()
        if not ok or frame is None:
            self._fail_streak += 1
            if self._fail_streak in (5, 50, 500):
                self.get_logger().warn(
                    f'{self._fail_streak} consecutive capture failures — '
                    'camera unplugged or claimed by another process?'
                )
            return
        self._fail_streak = 0

        if self._rot180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        elif self._flip_h:
            frame = cv2.flip(frame, 1)

        stamp = self.get_clock().now().to_msg()

        jpg = CompressedImage()
        jpg.header.stamp = stamp
        jpg.header.frame_id = self._frame_id
        jpg.format = 'jpeg'
        ok_enc, buf = cv2.imencode(
            '.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), self._quality])
        if ok_enc:
            jpg.data = np.asarray(buf).tobytes()
            self._pub_jpg.publish(jpg)

        # cv_bridge conversion is not free at 720p, and DDS drops the message
        # anyway if nothing is listening — so skip the work, not just the send.
        if self._pub_raw is not None and self._pub_raw.get_subscription_count() > 0:
            msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = stamp
            msg.header.frame_id = self._frame_id
            self._pub_raw.publish(msg)

        self._info.header.stamp = stamp
        self._pub_info.publish(self._info)

    def destroy_node(self) -> bool:
        self._deactivate()
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = CameraNode()
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
