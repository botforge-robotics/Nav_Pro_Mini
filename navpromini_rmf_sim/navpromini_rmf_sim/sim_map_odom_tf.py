#!/usr/bin/env python3
"""Publish identity map→robotN/odom for Gazebo cafe sim.

Cafe Gazebo odom is world/map-aligned. AMCL tf_broadcast is off in sim so it
cannot fight this transform after doors. Fleet-server navpromini_ff_tf may
publish the same identity later — duplicates are harmless if identical.

Without this node, Nav2 global_costmap logs:
  Invalid frame ID "map" ... frame does not exist
whenever gui_up / ff_tf is not running yet.
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


class SimMapOdomTf(Node):
    def __init__(self) -> None:
        super().__init__('sim_map_odom_tf')
        self.declare_parameter('robot_names', ['robot1', 'robot2'])
        names = self.get_parameter('robot_names').value
        if isinstance(names, str):
            names = [n.strip() for n in names.split(',') if n.strip()]
        elif isinstance(names, (list, tuple)):
            names = [str(n).strip() for n in names if str(n).strip()]
        else:
            names = ['robot1', 'robot2']
        self._robots = list(names)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._pub_global = self.create_publisher(TFMessage, '/tf', qos)
        self._pubs = {
            ns: self.create_publisher(TFMessage, f'/{ns}/tf', qos)
            for ns in self._robots
        }
        self.create_timer(0.05, self._tick)
        self.get_logger().info(
            'identity map→odom for: ' + ', '.join(self._robots)
        )

    def _tick(self) -> None:
        stamp = self.get_clock().now().to_msg()
        global_msg = TFMessage()
        for ns in self._robots:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = 'map'
            t.child_frame_id = f'{ns}/odom'
            t.transform.rotation.w = 1.0
            msg = TFMessage(transforms=[t])
            self._pubs[ns].publish(msg)
            global_msg.transforms.append(t)
        if global_msg.transforms:
            self._pub_global.publish(global_msg)


def main() -> None:
    rclpy.init()
    node = SimMapOdomTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
