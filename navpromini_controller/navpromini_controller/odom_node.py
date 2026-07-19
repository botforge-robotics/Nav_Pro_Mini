#!/usr/bin/env python3
"""Differential-drive odometry from ESP32 /wheel_ticks.

Odometry math mirrors Botforge Rio firmware (update_odometry):

    d_left  = left_tick_diff  / encoder_ticks_per_meter
    d_right = right_tick_diff / encoder_ticks_per_meter
    d       = (d_left + d_right) / 2
    d_theta = (d_right - d_left) / wheel_separation

Calibrate ``encoder_ticks_per_meter`` by driving 1 m and reading tick delta.
"""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
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
    """Tick delta with int32 wrap handling."""
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

        # Primary calibration (Rio: encoder_ticks_per_mtr).
        # Default ≈ 420 ticks/rev / (2*pi*0.034 m) ≈ 1966 ticks/m — replace after 1 m drive test.
        self.declare_parameter('encoder_ticks_per_meter', 1966.0)
        self.declare_parameter('wheel_separation', 0.187)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('wheel_ticks_topic', 'wheel_ticks')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('left_wheel_sign', 1.0)
        self.declare_parameter('right_wheel_sign', 1.0)

        self._ticks_per_m = float(self.get_parameter('encoder_ticks_per_meter').value)
        self._wheel_separation = float(self.get_parameter('wheel_separation').value)
        self._odom_frame = str(self.get_parameter('odom_frame').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._publish_tf = bool(self.get_parameter('publish_tf').value)
        self._left_sign = float(self.get_parameter('left_wheel_sign').value)
        self._right_sign = float(self.get_parameter('right_wheel_sign').value)

        if self._ticks_per_m <= 0.0:
            raise ValueError('encoder_ticks_per_meter must be > 0')
        if self._wheel_separation <= 0.0:
            raise ValueError('wheel_separation must be > 0')

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

        odom_topic = str(self.get_parameter('odom_topic').value)
        ticks_topic = str(self.get_parameter('wheel_ticks_topic').value)

        self._odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self._tf_broadcaster = TransformBroadcaster(self) if self._publish_tf else None
        self.create_subscription(
            Int32MultiArray,
            ticks_topic,
            self._on_wheel_ticks,
            qos_profile_sensor_data,
        )
        self.create_timer(2.0, self._watchdog)

        self.get_logger().info(
            f'Odom from {ticks_topic}: ticks/m={self._ticks_per_m:.1f}, '
            f'track={self._wheel_separation:.4f} m → /{odom_topic} '
            f'+ TF {self._odom_frame}→{self._base_frame}'
        )

    def _watchdog(self) -> None:
        if not self._got_ticks:
            self.get_logger().warn(
                'No /wheel_ticks yet — flash firmware that publishes '
                'std_msgs/Int32MultiArray [left, right], then check: '
                'ros2 topic echo /wheel_ticks --once'
            )

    def _on_wheel_ticks(self, msg: Int32MultiArray) -> None:
        if len(msg.data) < 2:
            return

        self._got_ticks = True
        left = int(round(self._left_sign * float(msg.data[0])))
        right = int(round(self._right_sign * float(msg.data[1])))
        now_ns = self.get_clock().now().nanoseconds
        stamp = self.get_clock().now().to_msg()

        if self._prev_left is None or self._prev_right is None or self._prev_time_ns is None:
            self._prev_left = left
            self._prev_right = right
            self._prev_time_ns = now_ns
            self._publish(stamp)
            return

        dt = (now_ns - self._prev_time_ns) * 1e-9
        self._prev_time_ns = now_ns
        if dt < 1e-4:
            return

        left_diff = signed_delta_i32(left, self._prev_left)
        right_diff = signed_delta_i32(right, self._prev_right)
        self._prev_left = left
        self._prev_right = right

        # Rio update_odometry(): meters from calibrated ticks-per-meter.
        d_left = float(left_diff) / self._ticks_per_m
        d_right = float(right_diff) / self._ticks_per_m
        d = 0.5 * (d_left + d_right)
        d_theta = (d_right - d_left) / self._wheel_separation

        linear_vel = d / dt
        angular_vel = d_theta / dt

        if abs(d) > 1e-5:
            mid_heading = self._theta + 0.5 * d_theta
            self._x += d * math.cos(mid_heading)
            self._y += d * math.sin(mid_heading)

        self._theta = normalize_angle(self._theta + d_theta)

        # Body-frame twist (Nav2 / ROS convention). Rio stores world-frame
        # components; we keep linear.x as forward speed in base_link.
        self._v_x = linear_vel
        self._v_y = 0.0
        self._v_theta = angular_vel

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
