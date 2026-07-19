#!/usr/bin/env python3
"""Differential-drive odometry from ESP32 /wheel_ticks.

Uses the same geometry model as ros_control diff_drive_controller:
  https://wiki.ros.org/diff_drive_controller

    meters_per_tick = (2 * pi * wheel_radius) / ticks_per_revolution
    # or equivalently:  1 / encoder_ticks_per_meter

    d_left  = left_tick_diff  * meters_per_tick
    d_right = right_tick_diff * meters_per_tick
    d       = 0.5 * (d_left + d_right)
    d_theta = (d_right - d_left) / wheel_separation

    x     += d * cos(theta + 0.5 * d_theta)
    y     += d * sin(theta + 0.5 * d_theta)
    theta += d_theta
"""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32MultiArray
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def signed_delta_i32(current: int, previous: int) -> int:
    return (current - previous + 2**31) % 2**32 - 2**31


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class WheelOdomNode(Node):
    def __init__(self) -> None:
        super().__init__('navpromini_odom')

        # Geometry (diff_drive_controller style).
        self.declare_parameter('wheel_radius', 0.0325)  # 65 mm diameter
        self.declare_parameter('wheel_separation', 0.225)  # 22.5 cm
        self.declare_parameter('ticks_per_revolution', 1470.0)
        # If > 0, overrides radius/ticks_per_rev (use your measured ticks/m).
        self.declare_parameter('encoder_ticks_per_meter', 7800.0)

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('wheel_ticks_topic', 'wheel_ticks')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('left_wheel_sign', 1.0)
        self.declare_parameter('right_wheel_sign', 1.0)
        # Ignore a single-frame jump larger than this (glitch / reorder).
        self.declare_parameter('max_tick_jump', 2000)

        wheel_radius = float(self.get_parameter('wheel_radius').value)
        ticks_per_rev = float(self.get_parameter('ticks_per_revolution').value)
        ticks_per_m_param = float(self.get_parameter(
            'encoder_ticks_per_meter').value)
        self._wheel_separation = float(
            self.get_parameter('wheel_separation').value)

        if ticks_per_m_param > 0.0:
            self._ticks_per_m = ticks_per_m_param
            self._meters_per_tick = 1.0 / self._ticks_per_m
        else:
            if wheel_radius <= 0.0 or ticks_per_rev <= 0.0:
                raise ValueError(
                    'wheel_radius and ticks_per_revolution must be > 0')
            self._meters_per_tick = (
                2.0 * math.pi * wheel_radius) / ticks_per_rev
            self._ticks_per_m = 1.0 / self._meters_per_tick

        if self._wheel_separation <= 0.0:
            raise ValueError('wheel_separation must be > 0')

        self._odom_frame = str(self.get_parameter('odom_frame').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._publish_tf = bool(self.get_parameter('publish_tf').value)
        self._left_sign = float(self.get_parameter('left_wheel_sign').value)
        self._right_sign = float(self.get_parameter('right_wheel_sign').value)
        self._max_tick_jump = int(self.get_parameter('max_tick_jump').value)

        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._v_x = 0.0
        self._v_y = 0.0
        self._v_theta = 0.0
        self._prev_left: Optional[int] = None
        self._prev_right: Optional[int] = None
        self._prev_time_ns: Optional[int] = None
        self._got_ticks = False
        self._trip_left_ticks = 0
        self._trip_right_ticks = 0
        self._path_length = 0.0

        odom_topic = str(self.get_parameter('odom_topic').value)
        ticks_topic = str(self.get_parameter('wheel_ticks_topic').value)

        # Match micro-ROS best-effort publisher; keep last sample.
        ticks_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self._tf_broadcaster = TransformBroadcaster(
            self) if self._publish_tf else None
        self.create_subscription(
            Int32MultiArray, ticks_topic, self._on_wheel_ticks, ticks_qos
        )
        self.create_timer(2.0, self._watchdog)

        self.get_logger().info(
            f'diff-drive odom: ticks/m={self._ticks_per_m:.1f} '
            f'(m/tick={self._meters_per_tick:.8f}), '
            f'separation={self._wheel_separation:.4f} m → /{odom_topic}. '
            f'Restart this node before a calibration run so pose starts at 0.'
        )

    def _watchdog(self) -> None:
        if not self._got_ticks:
            self.get_logger().warn(
                'No /wheel_ticks yet — check micro-ROS agent + firmware, then: '
                'ros2 topic echo /wheel_ticks --once'
            )
            return

        avg_ticks = 0.5 * (
            abs(self._trip_left_ticks) + abs(self._trip_right_ticks)
        )
        tick_path = avg_ticks * self._meters_per_tick
        pose_r = math.hypot(self._x, self._y)
        self.get_logger().info(
            f'odom check: pose=({self._x:.3f},{self._y:.3f}) r={pose_r:.3f} m | '
            f'path={self._path_length:.3f} m | '
            f'from ticks≈{tick_path:.3f} m '
            f'(ΔL={self._trip_left_ticks}, ΔR={self._trip_right_ticks}, '
            f'ticks/m={self._ticks_per_m:.1f})'
        )
        # Pose distance cannot exceed path length for valid differential drive.
        if self._path_length > 0.05 and pose_r > self._path_length * 1.05:
            self.get_logger().error(
                'INCONSISTENT ODOM: pose distance > integrated path — '
                'restart navpromini_odom and retest (stale pose / bad scale).'
            )

    def _on_wheel_ticks(self, msg: Int32MultiArray) -> None:
        if len(msg.data) < 2:
            return

        self._got_ticks = True
        left = int(round(self._left_sign * float(msg.data[0])))
        right = int(round(self._right_sign * float(msg.data[1])))
        now_ns = self.get_clock().now().nanoseconds
        stamp = self.get_clock().now().to_msg()

        if self._prev_left is None or self._prev_right is None:
            self._prev_left = left
            self._prev_right = right
            self._prev_time_ns = now_ns
            self._publish(stamp)
            return

        left_diff = signed_delta_i32(left, self._prev_left)
        right_diff = signed_delta_i32(right, self._prev_right)

        # Reject impossible jumps (DDS reorder / counter glitch).
        if (
            abs(left_diff) > self._max_tick_jump
            or abs(right_diff) > self._max_tick_jump
        ):
            self.get_logger().warn(
                f'Ignoring tick jump L={left_diff} R={right_diff} '
                f'(max={self._max_tick_jump}); resyncing.'
            )
            self._prev_left = left
            self._prev_right = right
            self._prev_time_ns = now_ns
            return

        dt = 0.0
        if self._prev_time_ns is not None:
            dt = max((now_ns - self._prev_time_ns) * 1e-9, 0.0)
        self._prev_left = left
        self._prev_right = right
        self._prev_time_ns = now_ns

        self._trip_left_ticks += left_diff
        self._trip_right_ticks += right_diff

        # diff_drive / Rio: convert ticks → meters, then fuse.
        d_left = float(left_diff) * self._meters_per_tick
        d_right = float(right_diff) * self._meters_per_tick
        d = 0.5 * (d_left + d_right)
        d_theta = (d_right - d_left) / self._wheel_separation

        if dt > 1e-4:
            self._v_x = d / dt
            self._v_y = 0.0
            self._v_theta = d_theta / dt
        else:
            self._v_x = 0.0
            self._v_y = 0.0
            self._v_theta = 0.0

        if abs(d) > 1e-9 or abs(d_theta) > 1e-9:
            mid_heading = self._theta + 0.5 * d_theta
            self._x += d * math.cos(mid_heading)
            self._y += d * math.sin(mid_heading)
            self._theta = normalize_angle(self._theta + d_theta)
            self._path_length += abs(d)

        self._publish(stamp)

    def _publish(self, stamp) -> None:
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = yaw_to_quaternion(self._theta)
        odom.twist.twist.linear.x = self._v_x
        odom.twist.twist.linear.y = self._v_y
        odom.twist.twist.angular.z = self._v_theta

        odom.pose.covariance[0] = 0.05
        odom.pose.covariance[7] = 0.05
        odom.pose.covariance[35] = 0.1
        odom.twist.covariance[0] = 0.05
        odom.twist.covariance[35] = 0.1

        self._odom_pub.publish(odom)

        if self._tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self._odom_frame
            t.child_frame_id = self._base_frame
            t.transform.translation.x = self._x
            t.transform.translation.y = self._y
            t.transform.translation.z = 0.0
            t.transform.rotation = odom.pose.pose.orientation
            self._tf_broadcaster.sendTransform(t)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WheelOdomNode()
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
