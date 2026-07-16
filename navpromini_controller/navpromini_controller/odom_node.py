#!/usr/bin/env python3
"""Differential-drive odometry from wheel joint_states.

Subscribes to /joint_states (LeftWheelJoint, RightWheelJoint positions in rad),
publishes /odom and TF odom -> base_link.
"""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class WheelOdomNode(Node):
    def __init__(self) -> None:
        super().__init__('navpromini_odom')

        self.declare_parameter('wheel_radius', 0.034)
        self.declare_parameter('wheel_separation', 0.187)
        self.declare_parameter('left_joint_name', 'LeftWheelJoint')
        self.declare_parameter('right_joint_name', 'RightWheelJoint')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('joint_states_topic', 'joint_states')
        self.declare_parameter('publish_tf', True)
        # Flip if a wheel angle increases when driving backward.
        self.declare_parameter('left_wheel_sign', 1.0)
        self.declare_parameter('right_wheel_sign', 1.0)

        self._wheel_radius = float(self.get_parameter('wheel_radius').value)
        self._wheel_separation = float(self.get_parameter('wheel_separation').value)
        self._left_joint = str(self.get_parameter('left_joint_name').value)
        self._right_joint = str(self.get_parameter('right_joint_name').value)
        self._odom_frame = str(self.get_parameter('odom_frame').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._publish_tf = bool(self.get_parameter('publish_tf').value)
        self._left_sign = float(self.get_parameter('left_wheel_sign').value)
        self._right_sign = float(self.get_parameter('right_wheel_sign').value)

        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._prev_left: Optional[float] = None
        self._prev_right: Optional[float] = None

        odom_topic = str(self.get_parameter('odom_topic').value)
        js_topic = str(self.get_parameter('joint_states_topic').value)

        self._odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self._tf_broadcaster = TransformBroadcaster(self) if self._publish_tf else None
        self._js_sub = self.create_subscription(
            JointState, js_topic, self._on_joint_states, 10
        )

        self.get_logger().info(
            f'Odom from {js_topic}: r={self._wheel_radius:.4f} m, '
            f'track={self._wheel_separation:.4f} m → /{odom_topic} '
            f'+ TF {self._odom_frame}→{self._base_frame}'
        )

    def _on_joint_states(self, msg: JointState) -> None:
        try:
            left_i = msg.name.index(self._left_joint)
            right_i = msg.name.index(self._right_joint)
        except ValueError:
            return

        if left_i >= len(msg.position) or right_i >= len(msg.position):
            return

        left = self._left_sign * float(msg.position[left_i])
        right = self._right_sign * float(msg.position[right_i])

        if self._prev_left is None or self._prev_right is None:
            self._prev_left = left
            self._prev_right = right
            return

        d_left = (left - self._prev_left) * self._wheel_radius
        d_right = (right - self._prev_right) * self._wheel_radius
        self._prev_left = left
        self._prev_right = right

        ds = 0.5 * (d_left + d_right)
        d_yaw = (d_right - d_left) / self._wheel_separation

        # Mid-point integration
        self._x += ds * math.cos(self._yaw + 0.5 * d_yaw)
        self._y += ds * math.sin(self._yaw + 0.5 * d_yaw)
        self._yaw = math.atan2(math.sin(self._yaw + d_yaw), math.cos(self._yaw + d_yaw))

        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = self.get_clock().now().to_msg()

        v_left = 0.0
        v_right = 0.0
        if left_i < len(msg.velocity) and right_i < len(msg.velocity):
            v_left = self._left_sign * float(msg.velocity[left_i]) * self._wheel_radius
            v_right = self._right_sign * float(msg.velocity[right_i]) * self._wheel_radius
        linear = 0.5 * (v_left + v_right)
        angular = (v_right - v_left) / self._wheel_separation

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = yaw_to_quaternion(self._yaw)
        odom.twist.twist.linear.x = linear
        odom.twist.twist.angular.z = angular

        # Simple diagonal covariances (tune later)
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
