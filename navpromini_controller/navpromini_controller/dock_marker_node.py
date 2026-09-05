#!/usr/bin/env python3

from __future__ import annotations

import math
from typing import Optional

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import Float32MultiArray

_SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


def _make_detector(dict_id: int):
    dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
    if hasattr(cv2.aruco, 'DetectorParameters_create'):
        params = cv2.aruco.DetectorParameters_create()
    else:
        params = cv2.aruco.DetectorParameters()
    params.adaptiveThreshWinSizeMin = 3
    params.adaptiveThreshWinSizeMax = 23
    params.adaptiveThreshWinSizeStep = 10
    params.minMarkerPerimeterRate = 0.02
    params.polygonalApproxAccuracyRate = 0.05
    params.errorCorrectionRate = 0.8
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    if hasattr(cv2.aruco, 'ArucoDetector'):
        detector = cv2.aruco.ArucoDetector(dictionary, params)
        return lambda img: detector.detectMarkers(img)
    return lambda img: cv2.aruco.detectMarkers(img, dictionary, parameters=params)


class DockMarkerNode(Node):
    def __init__(self) -> None:
        super().__init__('navpromini_dock_marker')

        p = self.declare_parameter
        p('dictionary', int(cv2.aruco.DICT_APRILTAG_36h11))
        p('marker_id', 0)
        p('marker_size_m', 0.08)
        p('image_topic', 'camera/image_raw/compressed')
        p('camera_info_topic', 'camera/camera_info')
        p('max_rate_hz', 30.0)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self._marker_id = int(g('marker_id'))
        self._size = float(g('marker_size_m'))
        self._min_period = 1.0 / max(float(g('max_rate_hz')), 1e-3)

        h = self._size / 2.0
        self._obj_points = np.array([
            [-h,  h, 0.0],
            [h,  h, 0.0],
            [h, -h, 0.0],
            [-h, -h, 0.0],
        ], dtype=np.float32)

        self._detect = _make_detector(int(g('dictionary')))
        self._k: Optional[np.ndarray] = None
        self._d: Optional[np.ndarray] = None
        self._last_stamp = 0.0
        self._last_yaw: Optional[float] = None
        self._last_yaw_stamp = 0.0
        self._yaw_continuity_sec = 1.0

        self._pub_pose = self.create_publisher(PoseStamped, 'dock_marker', 10)
        self._pub_tag = self.create_publisher(Float32MultiArray, 'dock_tag', 10)
        self._pub_debug_img = self.create_publisher(CompressedImage, 'dock_debug/compressed', 10)
        self._pub_debug_raw = self.create_publisher(Image, 'dock_debug', 10)
        self.create_subscription(CameraInfo, str(g('camera_info_topic')),
                                 self._on_info, 10)
        self.create_subscription(CompressedImage, str(g('image_topic')),
                                 self._on_image, _SENSOR_QOS)

        # Load calibration fallback immediately so camera frames can be processed even before camera_info arrives
        import yaml, os
        calib_path = '/home/navpromini/.navpromini_camera_calibration.yaml'
        if os.path.exists(calib_path):
            try:
                with open(calib_path) as f:
                    cal = yaml.safe_load(f)
                k = np.array(cal['camera_matrix']['data'], dtype=np.float64).reshape(3, 3)
                d = np.array(cal['distortion_coefficients']['data'], dtype=np.float64).reshape(1, -1)
                # Scale if running at 1280x720 (calibration file is 640x360)
                k[0, 0] *= 2.0
                k[0, 2] *= 2.0
                k[1, 1] *= 2.0
                k[1, 2] *= 2.0
                self._k = k
                self._d = d
            except Exception as e:
                self.get_logger().warn(f'could not load calibration fallback: {e}')

        self.get_logger().info(
            f'dock_marker ready: id={self._marker_id}, '
            f'{self._size * 1000:.0f}mm' + (', calibrated (fallback loaded)' if self._k is not None else ', waiting for camera_info'))

    def _on_info(self, msg: CameraInfo) -> None:
        if len(msg.k) >= 9 and float(msg.k[0]) > 1.0:
            self._k = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self._d = np.array(msg.d, dtype=np.float64).reshape(1, -1)

    def _on_image(self, msg: CompressedImage) -> None:
        if self._k is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_stamp < self._min_period:
            return
        self._last_stamp = now

        raw = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_UNCHANGED)
        if raw is None:
            return
        if len(raw.shape) == 2:
            gray = raw
            bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        else:
            bgr = raw
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = self._detect(gray)
        if ids is None or not len(ids):
            # publish crosshair on empty frame
            h_img, w_img = bgr.shape[:2]
            cv2.line(bgr, (w_img // 2, 0), (w_img // 2, h_img), (80, 80, 80), 1)
            cv2.putText(bgr, 'NO DOCK TAG', (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            _, enc = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 70])
            dbg_msg = CompressedImage()
            dbg_msg.header = msg.header
            dbg_msg.format = 'jpeg'
            dbg_msg.data = bytes(enc)
            self._pub_debug_img.publish(dbg_msg)
            if self._pub_debug_raw.get_subscription_count() > 0:
                raw_msg = Image()
                raw_msg.header = msg.header
                raw_msg.height, raw_msg.width = bgr.shape[:2]
                raw_msg.encoding = 'bgr8'
                raw_msg.is_bigendian = False
                raw_msg.step = raw_msg.width * 3
                raw_msg.data = bgr.tobytes()
                self._pub_debug_raw.publish(raw_msg)
            return
        pick = None
        for c, i in zip(corners, ids.flatten()):
            if int(i) == self._marker_id:
                pick = c
                break
        if pick is None:
            return

        img_points = pick.reshape(4, 2).astype(np.float32)
        flags = getattr(cv2, 'SOLVEPNP_IPPE_SQUARE', cv2.SOLVEPNP_ITERATIVE)
        ok, rvecs, tvecs, errs = cv2.solvePnPGeneric(
            self._obj_points, img_points, self._k, self._d, flags=flags)
        if not ok or not rvecs:
            return

        candidates = []
        for i in range(len(rvecs)):
            rot_i, _ = cv2.Rodrigues(rvecs[i])
            yaw_i = float(math.atan2(-rot_i[2][0],
                                     math.hypot(rot_i[2][1], rot_i[2][2])))
            candidates.append((rvecs[i], tvecs[i], yaw_i))

        yaw_fresh = (self._last_yaw is not None
                    and now - self._last_yaw_stamp < self._yaw_continuity_sec)
        if len(candidates) > 1 and yaw_fresh:
            def _ang_dist(a: float, b: float) -> float:
                return abs((a - b + math.pi) % (2.0 * math.pi) - math.pi)
            best = min(range(len(candidates)),
                      key=lambda i: _ang_dist(candidates[i][2], self._last_yaw))
        else:
            best = min(range(len(candidates)), key=lambda i: float(errs[i][0]))
        rvec, tvec, yaw = candidates[best]
        self._last_yaw = yaw
        self._last_yaw_stamp = now

        x = float(tvec[0][0])
        y = float(tvec[1][0])
        z = float(tvec[2][0])

        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = msg.header.frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        self._pub_pose.publish(pose)

        theta = math.atan2(x, z)
        r = math.hypot(x, z)
        lat = r * math.sin(2.0 * theta - (theta + yaw))
        
        out = Float32MultiArray()
        out.data = [1.0, float(self._marker_id), x, y, z,
                    r, theta, yaw]
        self._pub_tag.publish(out)

        # Draw visual debug overlays
        h_img, w_img = bgr.shape[:2]
        cx, cy = w_img // 2, h_img // 2
        # Center reference crosshairs
        cv2.line(bgr, (cx, 0), (cx, h_img), (255, 255, 0), 1)
        cv2.line(bgr, (0, cy), (w_img, cy), (255, 255, 0), 1)
        
        # Draw detected marker box & axis
        try:
            cv2.aruco.drawDetectedMarkers(bgr, corners, ids)
            cv2.drawFrameAxes(bgr, self._k, self._d, rvec, tvec, self._size * 0.75)
        except Exception:
            pass
            
        # HUD Text Overlay
        cv2.putText(bgr, f'DIST: {r*100:.1f}cm | LAT: {lat*100:+.1f}cm | YAW: {math.degrees(yaw):+.1f}deg',
                    (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(bgr, f'X: {x*100:+.1f}cm  Z: {z*100:.1f}cm',
                    (15, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)

        _, enc = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 70])
        dbg_msg = CompressedImage()
        dbg_msg.header = msg.header
        dbg_msg.format = 'jpeg'
        dbg_msg.data = bytes(enc)
        self._pub_debug_img.publish(dbg_msg)
        if self._pub_debug_raw.get_subscription_count() > 0:
            raw_msg = Image()
            raw_msg.header = msg.header
            raw_msg.height, raw_msg.width = bgr.shape[:2]
            raw_msg.encoding = 'bgr8'
            raw_msg.is_bigendian = False
            raw_msg.step = raw_msg.width * 3
            raw_msg.data = bgr.tobytes()
            self._pub_debug_raw.publish(raw_msg)


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = DockMarkerNode()
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
