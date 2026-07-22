#!/usr/bin/env python3
"""Publish rmf_fleet_msgs/RobotState for NavProMini robots.

Sources pose from /robotN/odom when available, otherwise from spawn_poses.yaml
so robots still appear in rmf-web without Gazebo.
"""

from __future__ import annotations

import math
import os
from typing import Dict, Optional, Tuple

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rmf_fleet_msgs.msg import Location, RobotMode, RobotState


def _yaw_from_quat(z: float, w: float) -> float:
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


class PublishRobotStates(Node):
    def __init__(self):
        super().__init__('navpromini_publish_robot_states')
        self.declare_parameter('robot_names', 'robot1,robot2')
        self.declare_parameter('level_name', 'L1')
        self.declare_parameter('fleet_name', 'navpromini')
        self.declare_parameter('model', 'NavProMini')
        self.declare_parameter('publish_hz', 10.0)
        self.declare_parameter('spawn_poses', '')
        # Do not declare use_sim_time — rclpy already provides it.

        names = [
            n.strip()
            for n in self.get_parameter('robot_names').value.split(',')
            if n.strip()
        ]
        self._level = self.get_parameter('level_name').value
        self._fleet = self.get_parameter('fleet_name').value
        self._model = self.get_parameter('model').value
        self._seq: Dict[str, int] = {n: 0 for n in names}
        # name -> (x, y, yaw, have_odom)
        self._pose: Dict[str, Tuple[float, float, float, bool]] = {}

        spawn_path = self.get_parameter('spawn_poses').value
        if not spawn_path:
            pkg = get_package_share_directory('navpromini_rmf_sim')
            spawn_path = os.path.join(pkg, 'site', 'spawn_poses.yaml')
        self._load_spawns(spawn_path, names)

        for name in names:
            self.create_subscription(
                Odometry,
                f'/{name}/odom',
                lambda msg, n=name: self._on_odom(n, msg),
                qos_profile_sensor_data,
            )

        self._pub = self.create_publisher(RobotState, 'robot_state', 100)
        hz = float(self.get_parameter('publish_hz').value)
        self.create_timer(1.0 / max(hz, 1.0), self._tick)
        self.get_logger().info(
            f'Publishing RobotState for {names} on /robot_state '
            f'(odom preferred, spawn fallback)'
        )

    def _load_spawns(self, path: str, names):
        data = {}
        if os.path.isfile(path):
            with open(path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
        robots = data.get('robots') or {}
        for name in names:
            r = robots.get(name) or {}
            self._pose[name] = (
                float(r.get('x', 0.0)),
                float(r.get('y', 0.0)),
                float(r.get('yaw', 0.0)),
                False,
            )

    def _on_odom(self, name: str, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._pose[name] = (p.x, p.y, _yaw_from_quat(q.z, q.w), True)

    def _tick(self):
        now = self.get_clock().now().to_msg()
        for name, (x, y, yaw, _live) in self._pose.items():
            self._seq[name] += 1
            msg = RobotState()
            msg.name = name
            msg.model = self._model
            msg.task_id = ''
            msg.seq = self._seq[name]
            msg.mode.mode = RobotMode.MODE_IDLE
            msg.battery_percent = 100.0
            msg.location.t = now
            msg.location.x = float(x)
            msg.location.y = float(y)
            msg.location.yaw = float(yaw)
            msg.location.level_name = self._level
            msg.path = []
            self._pub.publish(msg)


def main():
    rclpy.init()
    node = PublishRobotStates()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
