#!/usr/bin/env python3
"""Publish AMCL initial poses so Nav2 will accept navigate_to_pose goals.

AMCL's global_frame_id is \"map\". Poses published with frame_id \"robotN/map\"
are ignored (\"initial poses must be in the global frame, map\"), and without
localization bt_navigator often never accepts goals.

Usage:
  ros2 run navpromini_rmf_sim publish_amcl_initial_poses
  # or:
  python3 scripts/publish_amcl_initial_poses.py [--spawn-yaml PATH]
"""

from __future__ import annotations

import argparse
import math
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from navpromini_rmf_sim.launch_utils import default_spawn_pairs, parse_robot_list


def _main(spawn_yaml: str, robot_names: str, repeats: int) -> None:
    robots = parse_robot_list(robot_names)
    poses = {
        name: (x, y, yaw)
        for name, x, y, _z, yaw in default_spawn_pairs(robots, spawn_yaml)
    }

    rclpy.init()
    node = Node('publish_amcl_initial_poses')
    node.set_parameters([
        rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
    ])
    # Match typical RViz / AMCL subscription QoS.
    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    pubs = {
        name: node.create_publisher(
            PoseWithCovarianceStamped, f'/{name}/initialpose', qos
        )
        for name in poses
    }

    # Wait for subscribers (AMCL) when possible.
    deadline = time.time() + 10.0
    while time.time() < deadline and rclpy.ok():
        if all(p.get_subscription_count() > 0 for p in pubs.values()):
            break
        rclpy.spin_once(node, timeout_sec=0.2)

    for _ in range(max(1, repeats)):
        stamp = node.get_clock().now().to_msg()
        for name, (x, y, yaw) in poses.items():
            msg = PoseWithCovarianceStamped()
            # CRITICAL: AMCL requires global_frame_id == "map"
            msg.header.frame_id = 'map'
            msg.header.stamp = stamp
            msg.pose.pose.position.x = float(x)
            msg.pose.pose.position.y = float(y)
            msg.pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
            msg.pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
            msg.pose.covariance[0] = 0.25
            msg.pose.covariance[7] = 0.25
            msg.pose.covariance[35] = 0.068
            pubs[name].publish(msg)
            node.get_logger().info(
                f'published /{name}/initialpose frame=map '
                f'({x:.3f}, {y:.3f}, yaw={yaw:.3f})'
            )
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.15)

    node.destroy_node()
    rclpy.shutdown()


def main(argv: list[str] | None = None) -> None:
    pkg = get_package_share_directory('navpromini_rmf_sim')
    default_spawn = str(Path(pkg) / 'site' / 'spawn_poses.yaml')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--spawn-yaml', default=default_spawn)
    parser.add_argument('--robot-names', default='robot1,robot2')
    parser.add_argument('--repeats', type=int, default=8)
    args = parser.parse_args(argv)
    _main(args.spawn_yaml, args.robot_names, args.repeats)


if __name__ == '__main__':
    main()
