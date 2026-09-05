#!/usr/bin/env python3
"""Firmware Odometry Bridge & Broadcaster for NavPro Mini.

Disables redundant host-side slip gating calculations.
Uses onboard firmware fused odometry (/firmware_odom) when published by the ESP32.
Falls back to Runge-Kutta odometry from /wheel_ticks without slip freeze.
Publishes /odom (nav_msgs/Odometry) and broadcasts TF odom -> base_link for Nav2 and UI.
"""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray, Int32MultiArray
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


class OdomBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("navpromini_odom")

        self.declare_parameter("wheel_radius", 0.0325)
        self.declare_parameter("wheel_separation", 0.225)
        self.declare_parameter("ticks_per_revolution", 1470.0)
        self.declare_parameter("encoder_ticks_per_meter", 7800.0)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("firmware_odom_topic", "firmware_odom")
        self.declare_parameter("wheel_ticks_topic", "wheel_ticks")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("left_wheel_sign", 1.0)
        self.declare_parameter("right_wheel_sign", 1.0)
        self.declare_parameter("max_tick_jump", 2000)

        wheel_radius = float(self.get_parameter("wheel_radius").value)
        ticks_per_rev = float(self.get_parameter("ticks_per_revolution").value)
        ticks_per_m_param = float(self.get_parameter("encoder_ticks_per_meter").value)
        self._wheel_separation = float(self.get_parameter("wheel_separation").value)

        if ticks_per_m_param > 0.0:
            self._ticks_per_m = ticks_per_m_param
            self._meters_per_tick = 1.0 / self._ticks_per_m
        else:
            self._meters_per_tick = (2.0 * math.pi * wheel_radius) / ticks_per_rev
            self._ticks_per_m = 1.0 / self._meters_per_tick

        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._publish_tf = bool(self.get_parameter("publish_tf").value)
        self._left_sign = float(self.get_parameter("left_wheel_sign").value)
        self._right_sign = float(self.get_parameter("right_wheel_sign").value)
        self._max_tick_jump = int(self.get_parameter("max_tick_jump").value)

        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._v_x = 0.0
        self._v_y = 0.0
        self._v_theta = 0.0
        self._prev_left: Optional[int] = None
        self._prev_right: Optional[int] = None
        self._prev_time_ns: Optional[int] = None
        self._using_firmware_odom = False

        odom_topic = str(self.get_parameter("odom_topic").value)
        firmware_odom_topic = str(self.get_parameter("firmware_odom_topic").value)
        ticks_topic = str(self.get_parameter("wheel_ticks_topic").value)

        be_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self._odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self._tf_broadcaster = TransformBroadcaster(self) if self._publish_tf else None

        # 1. Onboard firmware odometry subscriber (ESP32 Runge-Kutta + IMU gyro)
        self.create_subscription(
            Float32MultiArray, firmware_odom_topic, self._on_firmware_odom, be_qos
        )

        # 2. Wheel ticks subscriber (fallback / live tick bridge)
        self.create_subscription(
            Int32MultiArray, ticks_topic, self._on_wheel_ticks, be_qos
        )

        self.get_logger().info(
            f"Odom Bridge active: /{firmware_odom_topic} (firmware fused) / /{ticks_topic} -> /{odom_topic} + TF {self._odom_frame}->{self._base_frame}"
        )

    def _on_firmware_odom(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 5:
            return
        self._using_firmware_odom = True
        self._x = float(msg.data[0])
        self._y = float(msg.data[1])
        self._theta = normalize_angle(float(msg.data[2]))
        self._v_x = float(msg.data[3])
        self._v_theta = float(msg.data[4])
        self._publish(self.get_clock().now().to_msg())

    def _on_wheel_ticks(self, msg: Int32MultiArray) -> None:
        if self._using_firmware_odom:
            return
        if len(msg.data) < 2:
            return

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

        if abs(left_diff) > self._max_tick_jump or abs(right_diff) > self._max_tick_jump:
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

        d_left = float(left_diff) * self._meters_per_tick
        d_right = float(right_diff) * self._meters_per_tick
        d = 0.5 * (d_left + d_right)
        d_theta = (d_right - d_left) / self._wheel_separation

        if dt > 1e-4:
            self._v_x = d / dt
            self._v_theta = d_theta / dt
        else:
            self._v_x = 0.0
            self._v_theta = 0.0

        if abs(d) > 1e-9 or abs(d_theta) > 1e-9:
            mid_heading = self._theta + 0.5 * d_theta
            self._x += d * math.cos(mid_heading)
            self._y += d * math.sin(mid_heading)
            self._theta = normalize_angle(self._theta + d_theta)

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

        odom.pose.covariance[0] = 0.001
        odom.pose.covariance[7] = 0.001
        odom.pose.covariance[35] = 0.003
        odom.twist.covariance[0] = 0.001
        odom.twist.covariance[35] = 0.003

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
    node = OdomBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
