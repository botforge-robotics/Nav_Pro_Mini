#!/usr/bin/env python3
"""Relay geometry_msgs/Twist fleet_teleop (std_msgs/String JSON or Twist) → Twist.

mapping-bridge publishes String JSON on /fleet_teleop in some setups and Twist
in others. This node normalizes to geometry_msgs/Twist on `fleet_teleop` for
twist_mux, and optionally republishes stamped locks.
"""

from __future__ import annotations

import json
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String


class FleetTeleopBridge(Node):
    def __init__(self) -> None:
        super().__init__('navpro_fleet_teleop_bridge')
        self.declare_parameter('input_twist_topic', 'fleet_teleop_raw')
        self.declare_parameter('input_string_topic', 'fleet_teleop_json')
        self.declare_parameter('output_topic', 'fleet_teleop')
        self.declare_parameter('also_subscribe_fleet_teleop_string', True)

        out = str(self.get_parameter('output_topic').value)
        self._pub = self.create_publisher(Twist, out, 10)
        self._lock_pub = self.create_publisher(Bool, 'navpro/cmd_vel_lock', 10)

        self.create_subscription(
            Twist,
            str(self.get_parameter('input_twist_topic').value),
            self._on_twist,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('input_string_topic').value),
            self._on_string,
            10,
        )
        # mapping-bridge uses /{name}/fleet_teleop as String in fleet server;
        # with zenoh namespace stripped locally it often arrives as fleet_teleop String.
        if bool(self.get_parameter('also_subscribe_fleet_teleop_string').value):
            self.create_subscription(String, 'fleet_teleop_cmd', self._on_string, 10)

        # Unlock by default
        unlock = Bool()
        unlock.data = False
        self._lock_pub.publish(unlock)
        self.get_logger().info(f'fleet_teleop_bridge → {out}')

    def _on_twist(self, msg: Twist) -> None:
        self._pub.publish(msg)

    def _on_string(self, msg: String) -> None:
        try:
            data = json.loads(msg.data or '{}')
        except json.JSONDecodeError:
            self.get_logger().warn('fleet_teleop JSON parse failed', throttle_duration_sec=5.0)
            return
        tw = Twist()
        tw.linear.x = float(data.get('linear', data.get('lin', 0.0)) or 0.0)
        tw.angular.z = float(data.get('angular', data.get('ang', 0.0)) or 0.0)
        # Allow nested {linear:{x}, angular:{z}}
        if isinstance(data.get('linear'), dict):
            tw.linear.x = float(data['linear'].get('x', 0.0) or 0.0)
        if isinstance(data.get('angular'), dict):
            tw.angular.z = float(data['angular'].get('z', 0.0) or 0.0)
        self._pub.publish(tw)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = FleetTeleopBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
