#!/usr/bin/env python3
"""Differential-drive odometry from ESP32 /wheel_ticks + IMU slip gate.

Geometry model matches ros_control diff_drive_controller.
IMU (body yaw rate) gates encoder integration when wheels spin but the
chassis does not — same idea used on industrial AMRs for free-spin / lift.
Absolute map pose is still AMCL + lidar, not this node.
"""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
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

        self.declare_parameter('wheel_radius', 0.0325)
        self.declare_parameter('wheel_separation', 0.225)
        self.declare_parameter('ticks_per_revolution', 1470.0)
        self.declare_parameter('encoder_ticks_per_meter', 7800.0)

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('wheel_ticks_topic', 'wheel_ticks')
        self.declare_parameter('imu_topic', 'imu')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('left_wheel_sign', 1.0)
        self.declare_parameter('right_wheel_sign', 1.0)
        self.declare_parameter('max_tick_jump', 2000)

        # Industrial-style slip gate (encoder vs body IMU).
        self.declare_parameter('slip_gate_enable', True)
        # |ω_wheel − ω_imu| above this → treat yaw as free-spin / slip
        self.declare_parameter('slip_yaw_err_thresh', 0.45)
        # Wheel |v| above this with body nearly static → lift / free-roll
        self.declare_parameter('slip_v_thresh', 0.06)
        self.declare_parameter('slip_gyro_still_thresh', 0.08)
        self.declare_parameter('slip_accel_still_thresh', 0.55)
        self.declare_parameter('slip_hold_samples', 4)

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

        self._slip_enable = bool(self.get_parameter('slip_gate_enable').value)
        self._slip_yaw_err = float(self.get_parameter('slip_yaw_err_thresh').value)
        self._slip_v = float(self.get_parameter('slip_v_thresh').value)
        self._slip_gyro_still = float(
            self.get_parameter('slip_gyro_still_thresh').value)
        self._slip_accel_still = float(
            self.get_parameter('slip_accel_still_thresh').value)
        self._slip_hold = max(1, int(self.get_parameter('slip_hold_samples').value))

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
        self._warned_no_ticks = False

        self._imu_gz = 0.0
        self._imu_ax = 0.0
        self._imu_ay = 0.0
        self._imu_az = 9.81
        self._got_imu = False
        self._slip_count = 0
        self._slip_active = False

        odom_topic = str(self.get_parameter('odom_topic').value)
        ticks_topic = str(self.get_parameter('wheel_ticks_topic').value)
        imu_topic = str(self.get_parameter('imu_topic').value)

        be_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self._tf_broadcaster = TransformBroadcaster(self) if self._publish_tf else None
        self.create_subscription(
            Int32MultiArray, ticks_topic, self._on_wheel_ticks, be_qos
        )
        self.create_subscription(Imu, imu_topic, self._on_imu, be_qos)
        self.create_timer(5.0, self._watchdog)

        self.get_logger().info(
            f'diff-drive odom: ticks/m={self._ticks_per_m:.1f}, '
            f'separation={self._wheel_separation:.4f} m → /{odom_topic} '
            f'(slip_gate={"on" if self._slip_enable else "off"})'
        )

    def _watchdog(self) -> None:
        if not self._got_ticks and not self._warned_no_ticks:
            self._warned_no_ticks = True
            self.get_logger().warn(
                'No /wheel_ticks yet — check micro-ROS agent + firmware'
            )

    def _on_imu(self, msg: Imu) -> None:
        self._got_imu = True
        self._imu_gz = float(msg.angular_velocity.z)
        self._imu_ax = float(msg.linear_acceleration.x)
        self._imu_ay = float(msg.linear_acceleration.y)
        self._imu_az = float(msg.linear_acceleration.z)

    def _detect_slip(self, v_x: float, v_theta: float) -> bool:
        """Two independent gates, both IMU-referenced against the wheels.

        1) Yaw disagreement: wheels claim a turn the chassis is not doing
           (lift spin / yaw slip).
        2) Forward claim while the body is IMU-still: wheels claim linear
           motion while gyro and horizontal accel both say "not moving"
           (stuck / high-centered / free-rolling wheel, no yaw component
           so gate (1) alone misses it).

        Straight cruising has gyro≈0 and horiz accel≈0 too, but v_x is also
        near 0 in that steady state — gate (2) only fires when the wheels
        report *both* a still body and real forward speed at once.
        """
        if not self._slip_enable or not self._got_imu:
            return False

        yaw_err = abs(v_theta - self._imu_gz)
        if yaw_err > self._slip_yaw_err and abs(v_theta) > 0.20:
            return True

        body_still = (
            abs(self._imu_gz) < self._slip_gyro_still
            and math.hypot(self._imu_ax, self._imu_ay) < self._slip_accel_still
        )
        if body_still and abs(v_x) > self._slip_v:
            return True

        return False

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
            self._publish(stamp, slipping=False)
            return

        left_diff = signed_delta_i32(left, self._prev_left)
        right_diff = signed_delta_i32(right, self._prev_right)

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

        d_left = float(left_diff) * self._meters_per_tick
        d_right = float(right_diff) * self._meters_per_tick
        d = 0.5 * (d_left + d_right)
        d_theta = (d_right - d_left) / self._wheel_separation

        if dt > 1e-4:
            v_x = d / dt
            v_theta = d_theta / dt
        else:
            v_x = 0.0
            v_theta = 0.0

        slipping = self._detect_slip(v_x, v_theta)
        if slipping:
            self._slip_count += 1
        else:
            self._slip_count = 0

        self._slip_active = self._slip_count >= self._slip_hold

        if self._slip_active:
            # Freeze dead-reckoning — chassis not doing what wheels claim.
            d = 0.0
            d_theta = 0.0
            self._v_x = 0.0
            self._v_y = 0.0
            self._v_theta = 0.0
        else:
            self._v_x = v_x
            self._v_y = 0.0
            self._v_theta = v_theta
            if abs(d) > 1e-9 or abs(d_theta) > 1e-9:
                mid_heading = self._theta + 0.5 * d_theta
                self._x += d * math.cos(mid_heading)
                self._y += d * math.sin(mid_heading)
                self._theta = normalize_angle(self._theta + d_theta)

        self._publish(stamp, slipping=self._slip_active)

    def _publish(self, stamp, slipping: bool) -> None:
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

        # Inflate yaw vs IMU gyro so EKF can correct wheel-slip spin smoothly
        odom.pose.covariance[0] = 0.02
        odom.pose.covariance[7] = 0.02
        odom.pose.covariance[35] = 0.5
        odom.twist.covariance[0] = 0.02
        odom.twist.covariance[35] = 0.08

        if slipping:
            odom.pose.covariance[0] = 0.5
            odom.pose.covariance[7] = 0.5
            odom.pose.covariance[35] = 1.0
            odom.twist.covariance[0] = 0.5
            odom.twist.covariance[35] = 0.5

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
