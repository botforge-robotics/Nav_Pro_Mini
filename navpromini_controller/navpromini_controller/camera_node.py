#!/usr/bin/env python3

from __future__ import annotations

from typing import Optional

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_srvs.srv import SetBool

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
        p('width', 1280)
        p('height', 720)
        p('fps', 30.0)
        p('jpeg_quality', 80)
        p('rotate_180', False)
        p('flip_horizontal', False)
        p('auto_exposure_value', 3)
        p('start_active', True)
        p('manual_exposure', False)
        p('exposure', 150)
        p('gain', 60)
        p('grayscale', True)
        p('frame_id', 'camera_link')
        p('camera_info_url', '/home/navpromini/.navpromini_camera_calibration.yaml')

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self._device = int(g('device'))
        self._w = int(g('width'))
        self._h = int(g('height'))
        self._fps = float(g('fps'))
        self._quality = int(g('jpeg_quality'))
        self._rot180 = bool(g('rotate_180'))
        self._flip_h = bool(g('flip_horizontal'))
        self._auto_exposure_value = int(g('auto_exposure_value'))
        self._start_active = bool(g('start_active'))
        self._manual_exposure = bool(g('manual_exposure'))
        self._exposure = int(g('exposure'))
        self._gain = int(g('gain'))
        self._grayscale = bool(g('grayscale'))
        self._frame_id = str(g('frame_id'))
        self._camera_info_url = str(g('camera_info_url'))

        self._cap: Optional[cv2.VideoCapture] = None
        self._active = False
        self._fail_streak = 0

        self._pub_jpg = self.create_publisher(
            CompressedImage, 'camera/image_raw/compressed', _VIDEO_QOS)
        self._pub_info = self.create_publisher(CameraInfo, 'camera/camera_info', 10)

        self._info = self._load_calibration(self._camera_info_url)
        self.create_service(SetBool, 'camera/set_active', self._on_set_active)
        self.create_timer(1.0 / max(self._fps, 1.0), self._tick)
        if self._start_active:
            self._activate()

        if self._info is None:
            self.get_logger().error(
                f'no usable calibration at {self._camera_info_url} — '
                'camera_info will not be published')
        else:
            self.get_logger().info(
                f'camera ready: {self._w}x{self._h} @{self._fps:g}fps, '
                f'jpeg q={self._quality}, fx~{self._info.k[0]:.0f}px [CALIBRATED]')

    def _load_calibration(self, path: str) -> Optional[CameraInfo]:
        try:
            with open(path, encoding='utf-8') as f:
                raw = yaml.safe_load(f)
        except (OSError, yaml.YAMLError) as exc:
            self.get_logger().error(f'calibration unreadable ({exc})')
            return None
        try:
            info = CameraInfo()
            info.header.frame_id = self._frame_id
            info.width = int(raw['image_width'])
            info.height = int(raw['image_height'])
            info.distortion_model = str(raw.get('distortion_model', 'plumb_bob'))
            info.d = [float(v) for v in raw['distortion_coefficients']['data']]
            info.k = [float(v) for v in raw['camera_matrix']['data']]
            info.r = [float(v) for v in raw['rectification_matrix']['data']]
            info.p = [float(v) for v in raw['projection_matrix']['data']]
        except (KeyError, TypeError, ValueError) as exc:
            self.get_logger().error(f'calibration malformed ({exc})')
            return None
        if info.width != self._w or info.height != self._h:
            sx = self._w / info.width
            sy = self._h / info.height
            self.get_logger().info(
                f'scaling camera calibration from {info.width}x{info.height} to '
                f'{self._w}x{self._h} (scale={sx:.2f}x, {sy:.2f}y)')
            info.width = self._w
            info.height = self._h
            k = list(info.k)
            k[0] *= sx
            k[2] *= sx
            k[4] *= sy
            k[5] *= sy
            info.k = k
            p_mat = list(info.p)
            p_mat[0] *= sx
            p_mat[2] *= sx
            p_mat[3] *= sx
            p_mat[5] *= sy
            p_mat[6] *= sy
            p_mat[7] *= sy
            info.p = p_mat
        return info

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
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._h)
        cap.set(cv2.CAP_PROP_FPS, self._fps)
        if self._manual_exposure:
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1.0)
            cap.set(cv2.CAP_PROP_EXPOSURE, float(self._exposure))
            cap.set(cv2.CAP_PROP_GAIN, float(self._gain))
            self.get_logger().info(
                f'camera manual shutter locked: exposure={self._exposure} ({(self._exposure*0.1):.1f}ms), gain={self._gain}')
        else:
            if not cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, self._auto_exposure_value):
                self.get_logger().warn(
                    f'could not set auto-exposure to {self._auto_exposure_value}')
        try:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:  # noqa: BLE001
            pass
        self._cap = cap
        self._fail_streak = 0
        self._active = True
        self.get_logger().info(
            f'camera activated (auto_exposure={cap.get(cv2.CAP_PROP_AUTO_EXPOSURE)})')
        return True

    def _deactivate(self) -> None:
        if self._start_active:
            # Keep camera hardware open and streaming so debug topics stay continuously alive
            return
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

    def _tick(self) -> None:
        if not self._active or self._cap is None:
            return
        ok, frame = self._cap.read()
        if not ok or frame is None:
            self._fail_streak += 1
            if self._fail_streak in (5, 50, 500):
                self.get_logger().warn(
                    f'{self._fail_streak} consecutive capture failures')
            return
        self._fail_streak = 0

        if self._rot180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        elif self._flip_h:
            frame = cv2.flip(frame, 1)

        if self._grayscale and len(frame.shape) == 3:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

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

        if self._info is not None:
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
