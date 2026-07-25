#!/usr/bin/env python3
"""QoS adapter: zenoh fleet_teleop (RELIABLE) → twist_mux (BEST_EFFORT).

Zenoh bridge publishes Twist as RELIABLE; twist_mux subscribes BEST_EFFORT.
Without this relay, joystick commands are silently dropped (incompatible QoS).
"""

from __future__ import annotations

from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

# Match zenoh-bridge publisher.
RELIABLE_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
)


class FleetTeleopBridge(Node):
    def __init__(self) -> None:
        super().__init__('navpro_fleet_teleop_bridge')
        self.declare_parameter('input_topic', 'fleet_teleop')
        self.declare_parameter('output_topic', 'fleet_drive')

        inp = str(self.get_parameter('input_topic').value)
        out = str(self.get_parameter('output_topic').value)
        self._pub = self.create_publisher(Twist, out, qos_profile_sensor_data)
        self.create_subscription(Twist, inp, self._on_twist, RELIABLE_QOS)
        self.get_logger().info(f'fleet teleop QoS bridge: {inp} (reliable) → {out} (best_effort)')

    def _on_twist(self, msg: Twist) -> None:
        self._pub.publish(msg)


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
