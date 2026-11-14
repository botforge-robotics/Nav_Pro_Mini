#!/usr/bin/env python3
"""
SO101 Arm Commander & Teleop Node for NavProMini Mobile Manipulator.

Provides smooth trajectory interpolation and named poses for the 6-DOF SO101 arm:
- Subscribes to:
    /arm/named_pose (std_msgs/String): 'home', 'reach', 'stow', 'zero', 'open', 'close'
    /arm/joint_goals (std_msgs/Float64MultiArray): [pan, lift, elbow, wrist_flex, wrist_roll, gripper]
    /joint_states (sensor_msgs/JointState): for current position tracking
- Publishes to:
    /arm/<joint>/cmd_pos (std_msgs/Float64): Individual joint position targets
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray, String

JOINT_NAMES = [
    'shoulder_pan',
    'shoulder_lift',
    'elbow_flex',
    'wrist_flex',
    'wrist_roll',
    'gripper',
]

NAMED_POSES = {
    'home': [0.0, -1.5707963, 1.5707963, 0.0, 0.0, 0.0],
    'reach': [0.0, -0.6, 0.9, -0.3, 0.0, 0.5],
    'forward': [0.0, -0.4, 0.7, -0.3, 0.0, 0.5],
    'stow': [0.0, -1.74, 1.69, -1.5, 0.0, 0.0],
    'zero': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'inspect': [0.0, -1.1, 1.2, 0.5, 0.0, 0.0],
    'open': None,   # only modifies gripper
    'close': None,  # only modifies gripper
}


class ArmTeleopNode(Node):

    def __init__(self):
        super().__init__('arm_teleop_node')

        self.declare_parameter('interpolation_time', 2.0)
        self.declare_parameter('control_rate', 50.0)

        self.interpolation_time = self.get_parameter('interpolation_time').value
        self.control_rate = self.get_parameter('control_rate').value

        # Current and target positions
        self.current_positions = {j: 0.0 for j in JOINT_NAMES}
        self.current_positions['shoulder_lift'] = -1.5707963
        self.current_positions['elbow_flex'] = 1.5707963

        self.start_positions = self.current_positions.copy()
        self.target_positions = self.current_positions.copy()

        self.trajectory_start_time = None
        self.is_moving = False

        # Publishers for each joint
        self.joint_pubs = {}
        for joint in JOINT_NAMES:
            self.joint_pubs[joint] = self.create_publisher(
                Float64, f'/arm/{joint}/cmd_pos', 10
            )

        # Subscribers
        self.sub_named = self.create_subscription(
            String, '/arm/named_pose', self.named_pose_callback, 10
        )
        self.sub_goals = self.create_subscription(
            Float64MultiArray, '/arm/joint_goals', self.joint_goals_callback, 10
        )
        self.sub_joint_states = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        # Control loop timer
        timer_period = 1.0 / self.control_rate
        self.timer = self.create_timer(timer_period, self.control_loop)

        self.get_logger().info(
            'SO101 Arm Commander active. Ready for /arm/named_pose or /arm/joint_goals.'
        )

    def joint_state_callback(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if name in self.current_positions:
                self.current_positions[name] = pos

    def named_pose_callback(self, msg: String):
        name = msg.data.strip().lower()
        if name == 'open':
            self.move_to_joint_target('gripper', 1.4)
            return
        if name == 'close':
            self.move_to_joint_target('gripper', 0.0)
            return

        if name not in NAMED_POSES:
            self.get_logger().warn(
                f"Unknown named pose '{name}'. Valid poses: {list(NAMED_POSES.keys())}"
            )
            return

        target_angles = NAMED_POSES[name]
        self.start_trajectory(target_angles)
        self.get_logger().info(f"Moving to named pose '{name}'...")

    def joint_goals_callback(self, msg: Float64MultiArray):
        if len(msg.data) != len(JOINT_NAMES):
            self.get_logger().error(
                f'Expected {len(JOINT_NAMES)} joint targets, received {len(msg.data)}'
            )
            return
        self.start_trajectory(list(msg.data))

    def move_to_joint_target(self, joint_name: str, target_val: float):
        targets = [self.target_positions[j] for j in JOINT_NAMES]
        idx = JOINT_NAMES.index(joint_name)
        targets[idx] = target_val
        self.start_trajectory(targets)

    def start_trajectory(self, target_angles: list):
        self.start_positions = self.current_positions.copy()
        for idx, joint in enumerate(JOINT_NAMES):
            self.target_positions[joint] = target_angles[idx]

        self.trajectory_start_time = self.get_clock().now()
        self.is_moving = True

    def control_loop(self):
        if not self.is_moving:
            return

        now = self.get_clock().now()
        elapsed = (now - self.trajectory_start_time).nanoseconds / 1e9
        alpha = min(1.0, elapsed / self.interpolation_time)

        # Smooth S-curve (cosine) interpolation: alpha_smooth in [0, 1]
        alpha_smooth = 0.5 * (1.0 - math.cos(math.pi * alpha))

        for joint in JOINT_NAMES:
            q_start = self.start_positions[joint]
            q_goal = self.target_positions[joint]
            q_cmd = q_start + alpha_smooth * (q_goal - q_start)

            msg = Float64()
            msg.data = float(q_cmd)
            self.joint_pubs[joint].publish(msg)

        if alpha >= 1.0:
            self.is_moving = False
            self.get_logger().info('Target arm pose reached.')


def main(args=None):
    rclpy.init(args=args)
    node = ArmTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
