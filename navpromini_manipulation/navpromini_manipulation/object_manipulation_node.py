#!/usr/bin/env python3
"""
NavProMini Mobile Manipulation Executive.

Coordinates autonomous navigation and robotic arm manipulation:
- Navigates the mobile base between Home Dock, Doorway, Pick Workcell, and Sorting Trays.
- Supports both Nav2 action client (/navigate_to_pose) and direct closed-loop Odometry Waypoint Tracking.
- Controls the SO101 6-DOF arm for precision picking, lifting, carrying, and placing.
- Real-time Wrist Camera perception integration (verifies target color before grasping).
- Status & telemetry reporting on /manipulation/status.
"""

import json
import math
import time
from threading import Thread
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger

JOINT_NAMES = [
    'shoulder_pan',
    'shoulder_lift',
    'elbow_flex',
    'wrist_flex',
    'wrist_roll',
    'gripper',
]

# Arm geometry offsets relative to base_link (meters)
ARM_BASE_OFFSET = (0.057764, -0.012279, 0.220655)
L_SHOULDER_Z = 0.065
L1_UPPER_ARM = 0.115
L2_LOWER_ARM = 0.110
L3_WRIST_GRIPPER = 0.100

# Predefined Arm Poses [pan, lift, elbow, wrist_flex, wrist_roll, gripper]
NAMED_POSES = {
    'home': [0.0, -1.5708, 1.5708, 0.0, 0.0, 0.0],
    'stow': [0.0, -1.74, 1.69, -1.5, 0.0, 0.0],
    'ready': [0.0, -0.55, 0.85, -0.30, 0.0, 0.7],
    'reach_forward': [0.0, -0.30, 0.76, -0.46, 0.0, 0.7],
    'grasp_forward': [0.0, -0.30, 0.76, -0.46, 0.0, 0.0],
    'lift_high': [0.0, -0.75, 1.05, -0.30, 0.0, 0.0],
    'carry': [0.0, -1.25, 1.45, -0.20, 0.0, 0.0],
    'inspect_table': [0.0, -0.95, 1.15, 0.45, 0.0, 0.0],
    'place_tray_red': [0.0, -0.36, 0.82, -0.46, 0.0, 0.7],
    'place_tray_blue': [0.0, -0.36, 0.82, -0.46, 0.0, 0.7],
}

# Mobile Station Waypoints in Playground Arena (x, y, yaw in radians)
STATION_WAYPOINTS = {
    'dock_staging': (-9.85, -5.00, 0.0),
    'doorway_south': (-6.00, -1.80, 1.5708),
    'doorway_north': (-6.00, 1.80, 1.5708),
    'pick_table': (-7.80, 4.40, 1.5708),       # Facing North staged 30cm in front of Pick Table
    'sort_red_tray': (-5.50, 4.40, 1.5708),    # Facing North staged 30cm in front of Red Tray
    'sort_blue_tray': (-4.90, 4.40, 1.5708),   # Facing North staged 30cm in front of Blue Tray
}

TARGET_OBJECTS = {
    'red_cube': {
        'coords': (0.33, 0.0, 0.28),
        'color': 'red',
        'default_dest': 'red_tray',
        'approach_station': 'pick_table',
        'pose_ready': [0.0, -0.55, 0.85, -0.30, 0.0, 0.7],
        'pose_pick': [0.0, -0.30, 0.76, -0.46, 0.0, 0.7],
        'pose_grasp': [0.0, -0.30, 0.76, -0.46, 0.0, 0.0],
    },
    'green_cylinder': {
        'coords': (0.32, 0.08, 0.28),
        'color': 'green',
        'default_dest': 'blue_tray',
        'approach_station': 'pick_table',
        'pose_ready': [0.18, -0.55, 0.85, -0.30, 0.0, 0.7],
        'pose_pick': [0.18, -0.30, 0.76, -0.46, 0.0, 0.7],
        'pose_grasp': [0.18, -0.30, 0.76, -0.46, 0.0, 0.0],
    },
    'blue_box': {
        'coords': (0.33, -0.10, 0.28),
        'color': 'blue',
        'default_dest': 'blue_tray',
        'approach_station': 'pick_table',
        'pose_ready': [-0.22, -0.55, 0.85, -0.30, 0.0, 0.7],
        'pose_pick': [-0.22, -0.30, 0.76, -0.46, 0.0, 0.7],
        'pose_grasp': [-0.22, -0.30, 0.76, -0.46, 0.0, 0.0],
    },
    'yellow_cylinder': {
        'coords': (0.35, 0.08, 0.28),
        'color': 'yellow',
        'default_dest': 'red_tray',
        'approach_station': 'pick_table',
        'pose_ready': [0.24, -0.55, 0.85, -0.30, 0.0, 0.7],
        'pose_pick': [0.24, -0.30, 0.76, -0.46, 0.0, 0.7],
        'pose_grasp': [0.24, -0.30, 0.76, -0.46, 0.0, 0.0],
    },
}

DESTINATION_TRAYS = {
    'red_tray': {
        'approach_station': 'sort_red_tray',
        'pose_approach': [0.0, -0.55, 0.85, -0.30, 0.0, 0.0],
        'pose_place': [0.0, -0.36, 0.82, -0.46, 0.0, 0.7],
    },
    'blue_tray': {
        'approach_station': 'sort_blue_tray',
        'pose_approach': [0.0, -0.55, 0.85, -0.30, 0.0, 0.0],
        'pose_place': [0.0, -0.36, 0.82, -0.46, 0.0, 0.7],
    },
}


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class ObjectManipulationNode(Node):

    def __init__(self):
        super().__init__('object_manipulation_node')

        self.declare_parameter('step_delay_sec', 1.4)
        self.declare_parameter('enable_vision_verification', True)
        self.declare_parameter('enable_mobile_navigation', True)
        self.declare_parameter('max_linear_speed', 0.35)
        self.declare_parameter('max_angular_speed', 1.0)

        self.step_delay = self.get_parameter('step_delay_sec').value
        self.enable_vision = self.get_parameter('enable_vision_verification').value
        self.enable_nav = self.get_parameter('enable_mobile_navigation').value
        self.max_lin = self.get_parameter('max_linear_speed').value
        self.max_ang = self.get_parameter('max_angular_speed').value

        self.latest_wrist_image: Optional[np.ndarray] = None
        self.current_joint_positions = {j: 0.0 for j in JOINT_NAMES}
        self.current_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.odom_received = False
        self.current_state = 'IDLE'
        self.is_task_running = False

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_pub = self.create_publisher(Float64MultiArray, '/arm/joint_goals', 10)
        self.named_pub = self.create_publisher(String, '/arm/named_pose', 10)
        self.status_pub = self.create_publisher(String, '/manipulation/status', 10)

        # Action Client for Nav2 NavigateToPose
        self.nav2_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Subscribers
        self.cmd_sub = self.create_subscription(String, '/manipulation/command', self.on_command, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.on_joint_states, 10)
        self.cam_sub = self.create_subscription(Image, '/arm/wrist_camera/image_raw', self.on_wrist_camera, 10)

        # Service
        self.srv_demo = self.create_service(Trigger, '/manipulation/execute_task', self.on_execute_task_service)

        self.status_timer = self.create_timer(1.0, self.publish_status)

        self.get_logger().info('Mobile Manipulation Executive initialized.')
        self.get_logger().info('Ready for commands on /manipulation/command (e.g. "pick_and_place red_cube red_tray")')

    def publish_status(self):
        msg = String()
        msg.data = json.dumps({
            'state': self.current_state,
            'is_running': self.is_task_running,
            'robot_pose': {k: round(v, 3) for k, v in self.current_pose.items()},
            'wrist_camera_active': self.latest_wrist_image is not None,
            'timestamp': self.get_clock().now().nanoseconds / 1e9,
        })
        self.status_pub.publish(msg)

    def on_odom(self, msg: Odometry):
        self.current_pose['x'] = msg.pose.pose.position.x
        self.current_pose['y'] = msg.pose.pose.position.y
        self.current_pose['yaw'] = yaw_from_quaternion(msg.pose.pose.orientation)
        self.odom_received = True

    def on_joint_states(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if name in self.current_joint_positions:
                self.current_joint_positions[name] = pos

    def on_wrist_camera(self, msg: Image):
        try:
            if msg.encoding in ('bgr8', 'rgb8'):
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
                if msg.encoding == 'rgb8':
                    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                self.latest_wrist_image = img
            elif msg.encoding == 'mono8':
                self.latest_wrist_image = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width))
        except Exception:
            pass

    def on_command(self, msg: String):
        raw = msg.data.strip()
        self.get_logger().info(f'Received manipulation command: "{raw}"')
        if self.is_task_running:
            self.get_logger().warn('Another manipulation task is already in progress. Ignoring command.')
            return

        Thread(target=self._dispatch_command, args=(raw,), daemon=True).start()

    def _dispatch_command(self, raw: str):
        self.is_task_running = True
        try:
            parts = raw.lower().split()
            action = parts[0] if parts else 'home'

            if action in ('home', 'stow', 'ready', 'inspect'):
                self.move_to_named(action)
            elif action == 'pick':
                target = parts[1] if len(parts) > 1 else 'red_cube'
                self.execute_mobile_pick(target)
            elif action == 'place':
                dest = parts[1] if len(parts) > 1 else 'red_tray'
                self.execute_mobile_place(dest)
            elif action in ('pick_and_place', 'pick_place', 'sort'):
                target = parts[1] if len(parts) > 1 else 'red_cube'
                dest = parts[2] if len(parts) > 2 else 'red_tray'
                self.execute_full_mobile_pick_and_place(target, dest)
            else:
                self.get_logger().warn(f'Unknown command: {raw}')
        finally:
            self.is_task_running = False
            self.current_state = 'IDLE'

    def on_execute_task_service(self, request, response):
        if self.is_task_running:
            response.success = False
            response.message = 'Task already running'
            return response

        self.get_logger().info('Executing demo via service...')
        Thread(target=self._dispatch_command, args=('pick_and_place red_cube red_tray',), daemon=True).start()
        response.success = True
        response.message = 'Dispatched mobile pick_and_place red_cube red_tray'
        return response

    # ==========================================
    # NAVIGATION METHODS
    # ==========================================
    def navigate_to_station(self, station_name: str) -> bool:
        """Drive mobile base to designated station with obstacle transit."""
        if not self.enable_nav:
            self.get_logger().info('Mobile navigation disabled by parameter. Skipping drive.')
            return True

        # Ensure odometry has been received before planning waypoints
        wait_start = time.time()
        while not self.odom_received and (time.time() - wait_start) < 4.0:
            time.sleep(0.05)

        if station_name not in STATION_WAYPOINTS:
            self.get_logger().error(f'Unknown station: {station_name}')
            return False

        target_wp = STATION_WAYPOINTS[station_name]
        self.get_logger().info(f'Navigating to [{station_name}] at ({target_wp[0]:.2f}, {target_wp[1]:.2f})...')
        self.current_state = f'NAVIGATING_TO_{station_name.upper()}'

        # 1. Undock pulse: if robot is inside dock bay (x < -9.5 and y < -3.5),
        # drive forward to clear the dock completely into open space before turning or calling Nav2.
        if self.current_pose['x'] < -9.5 and self.current_pose['y'] < -3.5:
            self.get_logger().info('Undocking forward from charging station...')
            t_twist = Twist()
            t_twist.linear.x = 0.20
            t_start = time.time()
            while (time.time() - t_start) < 2.5:
                self.cmd_vel_pub.publish(t_twist)
                time.sleep(0.05)
            self.cmd_vel_pub.publish(Twist())
            time.sleep(0.5)

        # 2. When Nav2 is available, send goal DIRECTLY to target station!
        # Nav2's global planner calculates the single optimal obstacle-free path through the doorway.
        if self.nav2_client.server_is_ready():
            self.get_logger().info(f'Nav2 active: planning global path directly to [{station_name}]...')
            ok = self._send_nav2_goal(target_wp[0], target_wp[1], target_wp[2])
            if ok:
                self.get_logger().info(f'[ARRIVED] Nav2 successfully reached [{station_name}].')
                return True
            else:
                self.get_logger().warn('Nav2 navigation unsuccessful — attempting closed-loop odometry fallback.')

        # 3. Odometry-Only Fallback (used when Nav2 is not running)
        cur_y = self.current_pose['y']
        tgt_y = target_wp[1]
        waypoints = []

        # Cross dividing wall doorway if moving between South (y<0) and North (y>0)
        if cur_y < -0.5 and tgt_y > 1.0:
            waypoints.extend([
                STATION_WAYPOINTS['doorway_south'],
                STATION_WAYPOINTS['doorway_north'],
            ])
        elif cur_y > 1.0 and tgt_y < -0.5:
            waypoints.extend([
                STATION_WAYPOINTS['doorway_north'],
                STATION_WAYPOINTS['doorway_south'],
            ])

        waypoints.append(target_wp)

        # Execute waypoints
        for i, wp in enumerate(waypoints):
            is_final = (i == len(waypoints) - 1)
            ok = self._drive_to_pose(wp[0], wp[1], wp[2] if is_final else None)
            if not ok:
                self.get_logger().error(f'Failed to reach waypoint {wp}')
                return False

        self.get_logger().info(f'[ARRIVED] Successfully reached [{station_name}].')
        return True

    def _drive_to_pose(self, target_x: float, target_y: float, target_yaw: Optional[float] = None) -> bool:
        """Drive to target coordinates using smooth closed-loop odometry control."""
        # Fallback: Autonomous Odometry Feedback Controller
        rate_hz = 20.0
        dt = 1.0 / rate_hz
        timeout_sec = 60.0
        start_time = time.time()

        while (time.time() - start_time) < timeout_sec:
            dx = target_x - self.current_pose['x']
            dy = target_y - self.current_pose['y']
            dist = math.sqrt(dx * dx + dy * dy)

            # Check position arrival
            if dist < 0.10:
                break

            target_heading = math.atan2(dy, dx)
            heading_err = normalize_angle(target_heading - self.current_pose['yaw'])

            twist = Twist()
            # If large heading error, pivot in place first
            if abs(heading_err) > 0.45:
                twist.linear.x = 0.0
                ang = float(np.clip(2.0 * heading_err, -self.max_ang, self.max_ang))
                if abs(ang) < 0.18:
                    ang = math.copysign(0.18, ang)
                twist.angular.z = ang
            else:
                # Drive forward with proportional steering
                twist.linear.x = float(np.clip(0.7 * dist, 0.08, self.max_lin))
                twist.angular.z = float(np.clip(1.8 * heading_err, -self.max_ang, self.max_ang))

            self.cmd_vel_pub.publish(twist)
            time.sleep(dt)

        # Final Yaw Alignment (if specified)
        if target_yaw is not None:
            align_start = time.time()
            while (time.time() - align_start) < 12.0:
                yaw_err = normalize_angle(target_yaw - self.current_pose['yaw'])
                if abs(yaw_err) < 0.04:  # ~2.3 degrees
                    break
                twist = Twist()
                twist.angular.z = float(np.clip(2.0 * yaw_err, -self.max_ang, self.max_ang))
                if abs(twist.angular.z) < 0.15:
                    twist.angular.z = math.copysign(0.15, twist.angular.z)
                self.cmd_vel_pub.publish(twist)
                time.sleep(dt)

        # Stop robot
        self.cmd_vel_pub.publish(Twist())
        time.sleep(0.3)
        return True

    def _send_nav2_goal(self, target_x: float, target_y: float, target_yaw: Optional[float] = None) -> bool:
        """Send goal to Nav2 NavigateToPose action server with generous simulation timeout."""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(target_x)
        goal_msg.pose.pose.position.y = float(target_y)
        goal_msg.pose.pose.position.z = 0.0

        yaw = target_yaw if target_yaw is not None else 0.0
        goal_msg.pose.pose.orientation = Quaternion(
            x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0)
        )

        send_future = self.nav2_client.send_goal_async(goal_msg)
        start_t = time.time()
        while not send_future.done():
            if time.time() - start_t > 15.0:
                self.get_logger().warn('Nav2 goal submission timed out.')
                return False
            time.sleep(0.05)

        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().warn('Nav2 goal rejected by server.')
            return False

        res_future = goal_handle.get_result_async()
        start_t = time.time()
        # Generous timeout (300s) for multi-meter journeys in Gazebo simulation
        while not res_future.done():
            if time.time() - start_t > 300.0:
                self.get_logger().error('Nav2 navigation timed out after 300s.')
                goal_handle.cancel_goal_async()
                return False
            time.sleep(0.2)

        result_obj = res_future.result()
        if result_obj.status == GoalStatus.STATUS_SUCCEEDED:
            return True
        else:
            self.get_logger().warn(f'Nav2 goal ended with status {result_obj.status}')
            return False

    # ==========================================
    # MANIPULATION METHODS
    # ==========================================
    def send_joint_goal(self, joints: List[float], wait_sec: Optional[float] = None) -> None:
        msg = Float64MultiArray()
        msg.data = [float(v) for v in joints]
        self.goal_pub.publish(msg)
        time.sleep(wait_sec if wait_sec is not None else self.step_delay)

    def move_to_named(self, name: str) -> None:
        if name in NAMED_POSES:
            self.current_state = f'ARM_{name.upper()}'
            self.get_logger().info(f'Moving arm to: {name}')
            self.send_joint_goal(NAMED_POSES[name])

    def verify_target_vision(self, expected_color: str) -> bool:
        """Inspect wrist camera image for expected target color signature."""
        if not self.enable_vision or self.latest_wrist_image is None:
            return True

        hsv = cv2.cvtColor(self.latest_wrist_image, cv2.COLOR_BGR2HSV)
        mask = None
        if expected_color == 'red':
            m1 = cv2.inRange(hsv, np.array([0, 90, 80]), np.array([10, 255, 255]))
            m2 = cv2.inRange(hsv, np.array([170, 90, 80]), np.array([180, 255, 255]))
            mask = cv2.bitwise_or(m1, m2)
        elif expected_color == 'green':
            mask = cv2.inRange(hsv, np.array([35, 70, 70]), np.array([85, 255, 255]))
        elif expected_color == 'blue':
            mask = cv2.inRange(hsv, np.array([100, 70, 70]), np.array([135, 255, 255]))
        elif expected_color == 'yellow':
            mask = cv2.inRange(hsv, np.array([18, 90, 90]), np.array([35, 255, 255]))

        if mask is not None:
            pixels = cv2.countNonZero(mask)
            self.get_logger().info(f'Wrist Camera vision check [{expected_color}]: {pixels} pixels detected')
            return pixels > 80
        return True

    def execute_mobile_pick(self, target_name: str) -> bool:
        """Drive to table, precision approach, and execute pick routine for target object."""
        target = TARGET_OBJECTS.get(target_name.lower())
        if not target:
            self.get_logger().error(f'Unknown target object: {target_name}')
            return False

        # 1. Drive to Pick Station (staging point in front of table)
        if not self.navigate_to_station(target['approach_station']):
            return False

        # 2. Precision Final Dock to Table Edge (close final 12cm gap)
        self.get_logger().info('Executing precision creep towards table...')
        creep_twist = Twist()
        creep_twist.linear.x = 0.10
        t_creep = time.time()
        while (time.time() - t_creep) < 1.1:
            self.cmd_vel_pub.publish(creep_twist)
            time.sleep(0.05)
        self.cmd_vel_pub.publish(Twist())
        time.sleep(0.4)

        # 3. Arm Hover Approach
        self.current_state = f'PICK_APPROACH_{target_name.upper()}'
        self.get_logger().info(f'Positioning arm over [{target_name}]...')
        self.send_joint_goal(target['pose_ready'])

        # 4. Wrist Camera Verification
        if self.enable_vision:
            time.sleep(0.5)
            confirmed = self.verify_target_vision(target['color'])
            self.get_logger().info(f'Wrist camera confirmation: {"VERIFIED" if confirmed else "UNCONFIRMED"}')

        # 5. Descend to Pick Height
        self.current_state = 'PICK_DESCEND'
        self.send_joint_goal(target['pose_pick'])

        # 6. Grasp
        self.current_state = 'PICK_GRASP'
        self.send_joint_goal(target['pose_grasp'], wait_sec=1.5)

        # 7. Lift Object
        self.current_state = 'PICK_LIFT'
        self.send_joint_goal(NAMED_POSES['lift_high'])

        # 8. Secure in Carry Pose for Mobile Transit
        self.current_state = 'CARRY'
        self.send_joint_goal(NAMED_POSES['carry'])

        # 9. Back off cleanly from table so robot can rotate freely
        self.get_logger().info('Backing off from pick table...')
        back_twist = Twist()
        back_twist.linear.x = -0.15
        t_back = time.time()
        while (time.time() - t_back) < 1.4:
            self.cmd_vel_pub.publish(back_twist)
            time.sleep(0.05)
        self.cmd_vel_pub.publish(Twist())
        time.sleep(0.4)

        self.get_logger().info(f'[SUCCESS] Picked up [{target_name}]. Safe in carry pose.')
        return True

    def execute_mobile_place(self, destination: str) -> bool:
        """Drive to sorting station, precision approach, and place object into designated tray."""
        tray = DESTINATION_TRAYS.get(destination.lower())
        if not tray:
            self.get_logger().error(f'Unknown destination: {destination}')
            return False

        # 1. Drive to Sorting Tray Station
        if not self.navigate_to_station(tray['approach_station']):
            return False

        # 2. Precision Final Dock to Tray Edge
        self.get_logger().info(f'Executing precision creep towards [{destination}]...')
        creep_twist = Twist()
        creep_twist.linear.x = 0.10
        t_creep = time.time()
        while (time.time() - t_creep) < 1.1:
            self.cmd_vel_pub.publish(creep_twist)
            time.sleep(0.05)
        self.cmd_vel_pub.publish(Twist())
        time.sleep(0.4)

        # 3. Extend Arm over Tray
        self.current_state = f'PLACE_APPROACH_{destination.upper()}'
        self.send_joint_goal(tray['pose_approach'])

        # 4. Lower and Release
        self.current_state = 'PLACE_RELEASE'
        self.send_joint_goal(tray['pose_place'], wait_sec=1.5)

        # 5. Retract Arm
        self.current_state = 'PLACE_RETRACT'
        self.send_joint_goal(tray['pose_approach'], wait_sec=1.0)

        # 6. Return Arm to Home
        self.current_state = 'HOME'
        self.send_joint_goal(NAMED_POSES['home'])

        # 7. Back off cleanly from sorting table
        self.get_logger().info('Backing off from sorting table...')
        back_twist = Twist()
        back_twist.linear.x = -0.15
        t_back = time.time()
        while (time.time() - t_back) < 1.4:
            self.cmd_vel_pub.publish(back_twist)
            time.sleep(0.05)
        self.cmd_vel_pub.publish(Twist())
        time.sleep(0.4)

        self.get_logger().info(f'[SUCCESS] Deposited object into [{destination}].')
        return True

    def execute_full_mobile_pick_and_place(self, target_name: str, destination: str, return_dock: bool = True) -> bool:
        """Full end-to-end mission: Undock -> Drive to Table -> Pick -> Drive to Tray -> Place -> Return to Dock."""
        self.get_logger().info(f'=== Starting Mobile Manipulation: Pick [{target_name}] -> Place [{destination}] ===')

        # Step 1: Pick
        ok_pick = self.execute_mobile_pick(target_name)
        if not ok_pick:
            self.get_logger().error('Mobile pick routine failed.')
            return False

        time.sleep(0.5)

        # Step 2: Place
        ok_place = self.execute_mobile_place(destination)
        if not ok_place:
            self.get_logger().error('Mobile place routine failed.')
            return False

        # Step 3: Return Home / Dock
        if return_dock:
            self.get_logger().info('Mission complete. Returning to Home Dock Bay...')
            # Navigate to dock staging point
            self.navigate_to_station('dock_staging')

            # Final alignment facing East (yaw=0.0 rad, rear to dock)
            self._drive_to_pose(-9.85, -5.00, 0.0)

            # Reversing gently into dock contacts (-10.57m)
            self.get_logger().info('Reversing into dock contacts...')
            rev_twist = Twist()
            rev_twist.linear.x = -0.12
            rev_start = time.time()
            while (time.time() - rev_start) < 6.0 and self.current_pose['x'] > -10.55:
                self.cmd_vel_pub.publish(rev_twist)
                time.sleep(0.05)
            self.cmd_vel_pub.publish(Twist())
            self.get_logger().info('[DOCKED] Robot safely docked at charging bay.')

        self.get_logger().info('=== Mobile Manipulation Mission Completed Successfully! ===')
        return True


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = ObjectManipulationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_vel_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
