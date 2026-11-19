#!/usr/bin/env python3
"""
CLI Client for NavProMini Object Manipulation.

Usage:
  ros2 run navpromini_manipulation manipulation_client --action pick_and_place --target red_cube --destination red_tray
  ros2 run navpromini_manipulation manipulation_client --action pick --target green_cylinder
  ros2 run navpromini_manipulation manipulation_client --action place --destination blue_tray
  ros2 run navpromini_manipulation manipulation_client --action home
  ros2 run navpromini_manipulation manipulation_client --action ready
"""

import argparse
import sys
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main(args=None):
    parser = argparse.ArgumentParser(description='Send commands to Object Manipulation Node')
    parser.add_argument('--action', type=str, default='pick_and_place',
                        choices=['pick', 'place', 'pick_and_place', 'home', 'ready', 'stow', 'inspect'],
                        help='Manipulation action to execute')
    parser.add_argument('--target', type=str, default='red_cube',
                        help='Target object name (e.g. red_cube, green_cylinder, blue_box, yellow_cylinder)')
    parser.add_argument('--destination', type=str, default='red_tray',
                        help='Destination tray name (e.g. red_tray, blue_tray)')

    parsed, unknown = parser.parse_known_args()

    rclpy.init(args=args)
    node = Node('manipulation_client_cli')
    pub = node.create_publisher(String, '/manipulation/command', 10)

    # Allow publisher to establish connection
    time.sleep(0.5)

    if parsed.action in ('home', 'ready', 'stow', 'inspect'):
        cmd_str = parsed.action
    elif parsed.action == 'pick':
        cmd_str = f"pick {parsed.target}"
    elif parsed.action == 'place':
        cmd_str = f"place {parsed.destination}"
    else:  # pick_and_place
        cmd_str = f"pick_and_place {parsed.target} {parsed.destination}"

    msg = String()
    msg.data = cmd_str
    pub.publish(msg)
    node.get_logger().info(f'Sent command to /manipulation/command: "{cmd_str}"')

    # Allow message to flush
    time.sleep(0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
