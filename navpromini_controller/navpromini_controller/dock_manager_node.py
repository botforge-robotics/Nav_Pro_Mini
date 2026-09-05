#!/usr/bin/env python3

from __future__ import annotations

import json
import math
import os
import time
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Twist
from nav2_msgs.action import DockRobot, NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.task import Future
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32MultiArray, String
from std_srvs.srv import SetBool

DOCK_POSE_FILE = os.path.expanduser('~/.navpromini_dock_pose.json')
_TICK = 0.1

_LATCHED_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)
_SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)
_VOLATILE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


class DockManagerNode(Node):
    def __init__(self) -> None:
        super().__init__('navpromini_dock_manager')

        p = self.declare_parameter
        p('angular_rate', 0.035)
        p('wheel_separation_m', 0.225)
        p('wheel_breakaway_mps', 0.02)
        p('wheel_floor_max_scale', 1.8)
        p('linear_rate', 0.045)
        p('turn_radians', 0.3491)
        p('min_turn_period', 0.18)
        p('sign', -1)
        p('k_lat', 1.0)
        p('k_yaw', 0.40)
        p('k_rho', 0.015)
        p('max_linear_speed', 0.0125)
        p('min_servo_speed', 0.008)
        p('max_omega', 0.08)
        p('omega_slew', 1.0)
        p('servo_filter_weight', 0.65)
        p('blind_min_r', 0.28)
        p('blind_fallback_r', 0.55)
        p('blind_creep_m', 0.25)
        p('blind_creep_speed', 0.018)
        p('blind_push_max_scale', 3.5)
        p('stall_speed_mps', 0.003)
        p('stall_confirm_sec', 1.2)
        p('stall_charge_wait_sec', 10.0)
        p('stall_min_travel_m', 0.06)
        p('straight_kp', 1.5)
        p('straight_max_omega', 0.1)
        p('retreat_on_fail_m', 0.25)
        p('undock_distance', 0.20)
        p('undock_speed', 0.05)
        p('dock_origin_offset_m', 0.13)
        p('standoff_m', 0.90)
        p('staging_timeout_sec', 300.0)
        p('marker_wait_timeout', 0.4)
        p('max_run_timeout', 600.0)
        p('settle_sec', 0.4)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self._w_rate = float(g('angular_rate'))
        self._wheel_sep = float(g('wheel_separation_m'))
        self._wheel_break_mps = float(g('wheel_breakaway_mps'))
        self._wheel_floor_max_scale = float(g('wheel_floor_max_scale'))
        self._v_rate = float(g('linear_rate'))
        self._turn_radians = float(g('turn_radians'))
        self._min_turn_period = float(g('min_turn_period'))
        self._sign = 1.0 if int(g('sign')) >= 0 else -1.0
        self._k_lat = float(g('k_lat'))
        self._k_yaw = float(g('k_yaw'))
        self._k_rho = float(g('k_rho'))
        self._max_linear_speed = float(g('max_linear_speed'))
        self._min_servo_speed = float(g('min_servo_speed'))
        self._max_omega = float(g('max_omega'))
        self._omega_slew = float(g('omega_slew'))
        self._servo_filter_weight = float(g('servo_filter_weight'))
        self._blind_min_r = float(g('blind_min_r'))
        self._blind_fallback_r = float(g('blind_fallback_r'))
        self._blind_creep_m = float(g('blind_creep_m'))
        self._blind_creep_speed = float(g('blind_creep_speed'))
        self._blind_push_max_scale = float(g('blind_push_max_scale'))
        self._stall_speed = float(g('stall_speed_mps'))
        self._stall_confirm = float(g('stall_confirm_sec'))
        self._stall_charge_wait = float(g('stall_charge_wait_sec'))
        self._stall_min_travel = float(g('stall_min_travel_m'))
        self._straight_kp = float(g('straight_kp'))
        self._straight_max_omega = float(g('straight_max_omega'))
        self._retreat_on_fail = float(g('retreat_on_fail_m'))
        self._undock_distance = float(g('undock_distance'))
        self._undock_speed = float(g('undock_speed'))
        self._dock_origin_offset = float(g('dock_origin_offset_m'))
        self._standoff = float(g('standoff_m'))
        self._staging_timeout = float(g('staging_timeout_sec'))
        self._marker_timeout = float(g('marker_wait_timeout'))
        self._max_run = float(g('max_run_timeout'))
        self._settle = float(g('settle_sec'))

        cb = ReentrantCallbackGroup()
        self._busy = False
        self._docked = False
        self._docked_state_known = False
        self._status = 'undocked'
        self._tag: Optional[list] = None
        self._tag_stamp = 0.0
        self._power: Optional[int] = None
        self._odom: Optional[Odometry] = None

        self._cmd = self.create_publisher(Twist, 'cmd_vel_dock', 10)
        self._status_pub = self.create_publisher(String, 'dock_status', 10)
        self._dock_pose_pub = self.create_publisher(
            PoseStamped, 'dock_pose', _LATCHED_QOS)
        self.create_subscription(PoseStamped, 'dock_pose',
                                 self._on_dock_pose_set, _VOLATILE_QOS,
                                 callback_group=cb)
        self.create_subscription(Float32MultiArray, 'dock_tag',
                                 self._on_tag, 10, callback_group=cb)
        self.create_subscription(BatteryState, 'battery/state',
                                 self._on_batt, 10, callback_group=cb)
        self.create_subscription(Odometry, 'odom',
                                 self._on_odom, _SENSOR_QOS, callback_group=cb)

        self._camera_client = self.create_client(
            SetBool, 'camera/set_active', callback_group=cb)
        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose', callback_group=cb)

        self._dock_pose: Optional[PoseStamped] = self._load_dock_pose()
        if self._dock_pose is None:
            self._dock_pose = self._dock_pose_at_map_origin()
            self.get_logger().info(
                f'No saved dock pose — assuming dock at map origin ({-self._dock_origin_offset:+.2f}m x, yaw 0)')
        if self._dock_pose is not None:
            self._dock_pose_pub.publish(self._dock_pose)

        ActionServer(self, DockRobot, 'dock',
                     execute_callback=self._execute_dock,
                     goal_callback=self._on_goal,
                     cancel_callback=lambda _g: CancelResponse.ACCEPT,
                     callback_group=cb)
        ActionServer(self, NavigateToPose, 'undock',
                     execute_callback=self._execute_undock,
                     goal_callback=self._on_goal,
                     cancel_callback=lambda _g: CancelResponse.ACCEPT,
                     callback_group=cb)

        self._tick_waiters: list = []
        self.create_timer(_TICK, self._on_tick, callback_group=cb)
        self.create_timer(1.0, self._publish_status, callback_group=cb)
        self.get_logger().info('dock_manager ready — actions dock / undock')

    def _on_goal(self, _request) -> GoalResponse:
        return GoalResponse.ACCEPT

    def _on_tick(self) -> None:
        waiters, self._tick_waiters = self._tick_waiters, []
        for f in waiters:
            if not f.done():
                f.set_result(None)

    async def _tick(self) -> None:
        fut = Future()
        self._tick_waiters.append(fut)
        await fut

    async def _sleep(self, sec: float) -> None:
        end = time.monotonic() + sec
        while time.monotonic() < end:
            await self._tick()

    def _on_tag(self, msg: Float32MultiArray) -> None:
        if len(msg.data) >= 8 and msg.data[0] > 0.5:
            self._tag = list(msg.data)
            self._tag_stamp = time.monotonic()

    def _on_batt(self, msg: BatteryState) -> None:
        self._power = msg.power_supply_status
        if self._charging() and self._busy:
            # Immediate wheel stop the millisecond electrodes touch charger
            self._stop()

        is_charging = self._charging()
        if is_charging:
            if not self._docked:
                self._docked = True
                self.get_logger().info(
                    'Battery charging detected — inferring robot is parked at dock.')
                if self._dock_pose is not None:
                    self._dock_pose_pub.publish(self._dock_pose)
            self._set_status(
                'full' if msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_FULL
                else 'charging')
        elif self._docked and not self._busy:
            if msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_DISCHARGING:
                self._docked = False
                self._set_status('undocked')
                self.get_logger().info('Battery discharging — robot removed from dock.')
        self._docked_state_known = True

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg

    def _charging(self) -> bool:
        return self._power in (BatteryState.POWER_SUPPLY_STATUS_CHARGING,
                               BatteryState.POWER_SUPPLY_STATUS_FULL)

    def _in_view(self) -> bool:
        return (self._tag is not None
                and time.monotonic() - self._tag_stamp < self._marker_timeout)

    def _fid2pos(self):
        x = float(self._tag[2])
        z = float(self._tag[4])
        yaw = float(self._tag[7])
        theta = math.atan2(x, z)
        r = math.hypot(x, z)
        beta = theta + yaw
        return theta, beta, r

    def _set_status(self, status: str) -> None:
        if self._status != status:
            self.get_logger().info(f'dock state -> {status}')
        self._status = status
        self._publish_status()

    def _publish_status(self) -> None:
        self._status_pub.publish(String(data=self._status))

    def _drive(self, lin: float, ang: float, effort_boost: float = 1.0) -> None:
        lin = float(lin)
        ang = float(ang)
        if lin or ang:
            half = self._wheel_sep / 2.0
            slower = min(abs(lin - ang * half), abs(lin + ang * half))
            if 0.0 < slower < self._wheel_break_mps:
                max_scale = self._wheel_floor_max_scale * max(1.0, effort_boost)
                scale = min(max_scale, (self._wheel_break_mps / slower) * effort_boost)
                lin *= scale
                ang *= scale
            elif effort_boost > 1.0:
                lin *= effort_boost
                ang *= effort_boost
        t = Twist()
        t.linear.x = lin
        t.angular.z = ang
        self._cmd.publish(t)

    def _stop(self) -> None:
        for _ in range(3):
            self._cmd.publish(Twist())

    async def _turn(self, radians: float, stop_when=None) -> bool:
        """Closed-loop encoder angle turn: uses odometry to rotate precisely
        by the requested angle with P-control. Stops immediately if stop_when() is True."""
        start_yaw = self._odom_yaw()
        if start_yaw is None:
            # Fallback to timed if odometry is unavailable
            period = max(abs(radians) / self._w_rate, self._min_turn_period)
            rate = -self._w_rate if radians > 0 else self._w_rate
            end = time.monotonic() + period
            while time.monotonic() < end:
                if stop_when is not None and stop_when():
                    break
                self._drive(0.0, rate)
                await self._tick()
            self._stop()
            await self._sleep(self._settle)
            return True

        # In robot frame: turning left/CCW is positive radians, turning right/CW is negative radians
        target_yaw = (start_yaw + radians + math.pi) % (2.0 * math.pi) - math.pi
        deadline = time.monotonic() + max(3.0, (abs(radians) / max(0.05, self._w_rate)) * 2.5)

        while time.monotonic() < deadline:
            if stop_when is not None and stop_when():
                self._stop()
                await self._sleep(self._settle)
                return False

            current_yaw = self._odom_yaw()
            if current_yaw is None:
                break

            err = (target_yaw - current_yaw + math.pi) % (2.0 * math.pi) - math.pi
            if abs(err) < 0.025:  # ~1.4 degrees deadband
                break

            # Closed-loop P-controller on heading error
            kp = 1.0
            omega = max(-self._max_omega, min(self._max_omega, kp * err))
            if abs(omega) < 0.04:
                omega = 0.04 if err > 0 else -0.04

            self._drive(0.0, omega)
            await self._tick()

        self._stop()
        await self._sleep(self._settle)
        return True

    async def _settle_and_check_charging(self) -> bool:
        """Give the battery controller's charging status time to catch up
        with real physical contact before reporting failure — contact can
        precede the status update by several seconds."""
        end = time.monotonic() + self._settle + self._stall_charge_wait
        while time.monotonic() < end:
            if self._charging():
                return True
            await self._tick()
        return self._charging()

    async def _jog(self, distance: float, speed: Optional[float] = None,
                    angular: float = 0.0, push_effort: bool = False) -> bool:
        rate = speed if speed is not None else self._v_rate
        period = abs(distance) / rate
        lin = -rate if distance > 0 else rate
        # When pushing against friction in a narrow funnel, allow extra time margin
        end = time.monotonic() + period * (1.8 if push_effort else 1.0)
        origin = None
        if self._odom is not None:
            p = self._odom.pose.pose.position
            origin = (p.x, p.y)
        stalled_since = None
        effort_boost = 1.0
        while time.monotonic() < end:
            if self._charging():
                self._stop()
                return True
            if self._odom is not None and origin is not None:
                p = self._odom.pose.pose.position
                travelled = math.hypot(p.x - origin[0], p.y - origin[1])
                speed_now = abs(self._odom.twist.twist.linear.x)

                if push_effort:
                    # If sidewall friction in the narrow funnel slows down the robot,
                    # incrementally increase effort/torque without raising nominal target speed
                    if speed_now < rate * 0.75:
                        effort_boost = min(self._blind_push_max_scale, effort_boost + 0.1)
                    elif speed_now >= rate * 0.95:
                        effort_boost = max(1.0, effort_boost - 0.05)

                if travelled > self._stall_min_travel and speed_now < self._stall_speed:
                    now = time.monotonic()
                    stalled_since = stalled_since or now
                    if now - stalled_since > self._stall_confirm:
                        self._stop()
                        self.get_logger().info(
                            'jog: stalled (odom speed near zero) — '
                            'stopping instead of pushing further')
                        return await self._settle_and_check_charging()
                else:
                    stalled_since = None
            self._drive(lin, angular, effort_boost=effort_boost if push_effort else 1.0)
            await self._tick()
        self._stop()
        return await self._settle_and_check_charging()

    def _odom_yaw(self) -> Optional[float]:
        if self._odom is None:
            return None
        q = self._odom.pose.pose.orientation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    async def _drive_straight(self, distance: float,
                              speed: Optional[float] = None) -> bool:
        """Closed-loop straight line: distance comes from odometry (not a
        timer) and heading is actively held to the starting yaw, so drift
        is corrected as it happens. Positive distance drives forward.
        Returns False only on a genuine physical stall."""
        rate = speed if speed is not None else self._v_rate
        lin = rate if distance > 0 else -rate
        target = abs(distance)
        start_yaw = self._odom_yaw()
        origin = None
        if self._odom is not None:
            p = self._odom.pose.pose.position
            origin = (p.x, p.y)
        if origin is None or start_yaw is None:
            self.get_logger().warn(
                'straight drive: no odometry — falling back to timed jog')
            return await self._jog_plain(distance, speed=speed)
        deadline = time.monotonic() + (target / rate) * 3.0 + 5.0
        stalled_since = None
        while time.monotonic() < deadline:
            p = self._odom.pose.pose.position
            travelled = math.hypot(p.x - origin[0], p.y - origin[1])
            if travelled >= target:
                break
            speed_now = abs(self._odom.twist.twist.linear.x)
            if travelled > self._stall_min_travel and speed_now < self._stall_speed:
                now = time.monotonic()
                stalled_since = stalled_since or now
                if now - stalled_since > self._stall_confirm:
                    self._stop()
                    self.get_logger().info(
                        'straight drive: stalled (odom speed near zero) — stopping')
                    await self._sleep(self._settle)
                    return False
            else:
                stalled_since = None
            err = (self._odom_yaw() - start_yaw + math.pi) % (2.0 * math.pi) - math.pi
            omega = max(-self._straight_max_omega,
                        min(self._straight_max_omega, -self._straight_kp * err))
            self._drive(lin, omega)
            await self._tick()
        self._stop()
        await self._sleep(self._settle)
        return True

    async def _clear_dock(self) -> None:
        """Drive forward off the dock before reporting failure. The dock is
        behind the robot (rear camera), so forward is away from it — this
        keeps the next attempt's staging rotation from grinding against the
        dock while the robot is still pressed into it."""
        if self._retreat_on_fail <= 0.0:
            return
        self.get_logger().info(
            f'clearing dock: driving forward {self._retreat_on_fail:.2f}m '
            'before reporting failure')
        await self._drive_straight(self._retreat_on_fail, speed=self._undock_speed)

    async def _jog_plain(self, distance: float, speed: Optional[float] = None) -> bool:
        """Drive straight for  (positive = forward). Returns False
        only on a genuine physical stall (odometry shows no real motion) —
        no charging check, used outside the dock-approach context."""
        rate = speed if speed is not None else self._v_rate
        period = abs(distance) / rate
        lin = rate if distance > 0 else -rate
        end = time.monotonic() + period
        origin = None
        if self._odom is not None:
            p = self._odom.pose.pose.position
            origin = (p.x, p.y)
        stalled_since = None
        while time.monotonic() < end:
            if self._odom is not None and origin is not None:
                p = self._odom.pose.pose.position
                travelled = math.hypot(p.x - origin[0], p.y - origin[1])
                speed_now = abs(self._odom.twist.twist.linear.x)
                if travelled > self._stall_min_travel and speed_now < self._stall_speed:
                    now = time.monotonic()
                    stalled_since = stalled_since or now
                    if now - stalled_since > self._stall_confirm:
                        self._stop()
                        self.get_logger().info(
                            'undock jog: stalled (odom speed near zero) — stopping')
                        await self._sleep(self._settle)
                        return False
                else:
                    stalled_since = None
            self._drive(lin, 0.0)
            await self._tick()
        self._stop()
        await self._sleep(self._settle)
        return True

    async def _set_camera(self, active: bool) -> None:
        if not self._camera_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn('camera/set_active unavailable')
            return
        req = SetBool.Request()
        req.data = active
        try:
            await self._camera_client.call_async(req)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'camera/set_active({active}) failed: {exc}')

    def _dock_pose_at_map_origin(self) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position = Point(x=-self._dock_origin_offset, y=0.0, z=0.0)
        pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        return pose

    def _staging_pose_for(self, dock_pose: PoseStamped) -> PoseStamped:
        q = dock_pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        staging = PoseStamped()
        staging.header.frame_id = dock_pose.header.frame_id or 'map'
        staging.pose.position = Point(
            x=dock_pose.pose.position.x + math.cos(yaw) * self._standoff,
            y=dock_pose.pose.position.y + math.sin(yaw) * self._standoff,
            z=0.0,
        )
        staging.pose.orientation = Quaternion(
            x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))
        return staging

    async def _navigate_to_staging(self, dock_pose: PoseStamped) -> bool:
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('navigate_to_pose action server not available')
            return False
        staging = self._staging_pose_for(dock_pose)
        self.get_logger().info(
            f'navigating to staging pose ({staging.pose.position.x:.2f}, '
            f'{staging.pose.position.y:.2f}) — {self._standoff:.2f}m from dock')
        deadline = time.monotonic() + self._staging_timeout
        goal_handle = None
        while time.monotonic() < deadline:
            staging.header.stamp = self.get_clock().now().to_msg()
            nav_goal = NavigateToPose.Goal()
            nav_goal.pose = staging
            send_fut = self._nav_client.send_goal_async(nav_goal)
            while not send_fut.done():
                if time.monotonic() > deadline:
                    return False
                await self._tick()
            candidate = send_fut.result()
            if candidate.accepted:
                goal_handle = candidate
                break
            self.get_logger().warn(
                'navigate_to_pose rejected the staging goal (server not '
                'active yet?) — retrying')
            await self._sleep(1.0)
        if goal_handle is None:
            return False
        result_fut = goal_handle.get_result_async()
        while not result_fut.done():
            if time.monotonic() > deadline:
                self.get_logger().warn('staging navigation exceeded staging_timeout — canceling')
                await goal_handle.cancel_goal_async()
                return False
            if self._in_view():
                self.get_logger().info('dock tag spotted in camera during staging approach — canceling Nav2 early to begin visual servoing')
                await goal_handle.cancel_goal_async()
                return True
            await self._tick()
        res = result_fut.result()
        return res.status == GoalStatus.STATUS_SUCCEEDED

    async def _execute_dock(self, goal_handle):
        self._busy = True
        try:
            goal: DockRobot.Goal = goal_handle.request
            if not goal.use_dock_id:
                self._on_dock_pose_set(goal.dock_pose)
            dock_pose = self._dock_pose

            # Only undock if the robot is physically on the charger
            if self._charging():
                self.get_logger().info('dock: currently charging on dock — undocking gently before redocking')
                self._set_status('undocking')
                await self._drive_straight(self._undock_distance, speed=self._undock_speed)
                self._docked = False
                self._set_status('undocked')
            else:
                self._docked = False

            # Navigate to the staging standoff pose in front of the dock via Nav2
            if dock_pose is not None:
                self._set_status('staging')
                if not await self._navigate_to_staging(dock_pose):
                    self.get_logger().error('failed to reach staging pose via Nav2')
                    self._stop()
                    self._set_status('docking_failed')
                    goal_handle.abort()
                    return DockRobot.Result(
                        success=False, error_msg='failed to reach staging pose')
            else:
                self.get_logger().warn(
                    'no dock pose configured — attempting visual servoing directly')

            deadline = time.monotonic() + self._max_run
            await self._set_camera(True)

            # Check if tag is already visible in rear camera (e.g. manually placed or after undock)
            # If so, proceed directly to servo without any in-place spin!
            for _ in range(15):
                if self._in_view():
                    break
                await self._sleep(0.1)

            if self._in_view():
                self.get_logger().info('dock: tag already in camera view — skipping search sweep, starting visual servoing')
                self._set_status('servo')
            else:
                self.get_logger().info('dock: tag not yet in view — entering search sweep')
                self._set_status('searching')
            omega_cmd = 0.0
            blind_yaw = 0.0
            approach = 1
            last_log = 0.0
            last_seen_tag_x: Optional[float] = None
            last_seen_z: Optional[float] = None
            filt_alpha = filt_beta = filt_r = None

            while True:
                if goal_handle.is_cancel_requested:
                    self._stop()
                    goal_handle.canceled()
                    self._set_status('undocked')
                    return DockRobot.Result(success=False, error_msg='cancelled')
                if self._charging():
                    self._stop()
                    self._docked = True
                    self._set_status(
                        'full' if self._power == BatteryState.POWER_SUPPLY_STATUS_FULL
                        else 'charging')
                    goal_handle.succeed()
                    return DockRobot.Result(success=True)
                if time.monotonic() > deadline:
                    self._stop()
                    await self._clear_dock()
                    self._set_status('docking_failed')
                    goal_handle.abort()
                    return DockRobot.Result(
                        success=False,
                        error_msg=f'docking timed out after {approach} '
                                  f'approach(es)')

                if self._status == 'searching':
                    if self._in_view():
                        self._stop()
                        omega_cmd = 0.0
                        filt_alpha = filt_beta = filt_r = None
                        self._set_status('servo')
                    else:
                        # Close to dock: never execute in-place rotational sweep; creep straight into funnel
                        if last_seen_z is not None and last_seen_z < self._blind_fallback_r:
                            self._stop()
                            self.get_logger().info(
                                f'searching: close to dock (z={last_seen_z*100:.1f}cm < {self._blind_fallback_r*100:.0f}cm) — creeping straight without sweeping')
                            self._set_status('blind_creep')
                            continue
                        turn_angle = -self._turn_radians if (last_seen_tag_x is None or last_seen_tag_x < 0) else self._turn_radians
                        await self._turn(turn_angle, stop_when=self._in_view)
                    continue

                if self._status == 'servo':
                    if not self._in_view():
                        # If we were already close to the dock / entering the funnel,
                        # do NOT rotate in place (sweep)! Creep straight back instead.
                        if filt_r is not None and filt_r < self._blind_fallback_r:
                            self._stop()
                            blind_yaw = filt_beta - filt_alpha
                            self.get_logger().info(
                                f'servo: tag lost at close range (r={filt_r:.3f}m < {self._blind_fallback_r:.2f}m) '
                                'inside funnel — creeping blind directly without in-place sweep')
                            self._set_status('blind_creep')
                            continue
                        self._stop()
                        self._set_status('searching')
                        continue

                    raw_alpha, raw_beta, raw_r = self._fid2pos()
                    if filt_alpha is None:
                        filt_alpha, filt_beta, filt_r = raw_alpha, raw_beta, raw_r
                    else:
                        w = self._servo_filter_weight
                        filt_alpha += w * (raw_alpha - filt_alpha)
                        filt_beta += w * (raw_beta - filt_beta)
                        filt_r += w * (raw_r - filt_r)
                    alpha, beta, r = filt_alpha, filt_beta, filt_r

                    tag_x = float(self._tag[2])
                    tag_z = float(self._tag[4])
                    tag_yaw = float(self._tag[7])
                    last_seen_tag_x = tag_x
                    last_seen_z = tag_z

                    # Continuously visual servo as long as tag is visible in frame!
                    # Only transition to blind_creep if the tag exceeds the frame boundaries and is lost.

                    # Pure Bearing Centering with Close-Range Rate Tapering (> 28cm distance):
                    # Bearing theta = atan2(x, z) points directly at the dock tag center
                    theta = math.atan2(tag_x, tag_z)

                    # Tight deadband of 0.008 rad (~0.45 deg, ~3mm at 40cm) ensures continuous centering
                    if abs(theta) > 0.008:
                        # Taper max turn rate close to dock to prevent edge-trimming and blur:
                        # Far (>45cm): up to max_omega (0.08 rad/s = 4.6 deg/s) for quick capture
                        # Near (<=45cm): capped at 0.035 rad/s (2.0 deg/s) for ultra-gentle micro-alignment
                        if tag_z > 0.45:
                            max_ang = self._max_omega
                        elif tag_z > 0.25:
                            max_ang = 0.035
                        else:
                            max_ang = 0.020
                        cmd = -0.65 * theta
                        target = max(-max_ang, min(max_ang, cmd))
                    else:
                        target = 0.0

                    step = self._omega_slew * _TICK
                    omega_cmd += max(-step, min(step, target - omega_cmd))

                    # Staged Approach Velocity:
                    # Far (>0.60m): 0.040 m/s
                    # Medium (0.35m - 0.60m): 0.022 m/s
                    # Close (0.20m - 0.35m): 0.012 m/s
                    # Final (<=0.20m): 0.008 m/s (slow micro-approach to contacts)
                    if tag_z > 0.60:
                        v_stage = 0.040
                    elif tag_z > 0.35:
                        v_stage = 0.022
                    elif tag_z > 0.20:
                        v_stage = 0.012
                    else:
                        v_stage = 0.008

                    # Slow down to gentle crawl during active angular turns to avoid compound motion blur
                    turn_fraction = abs(target) / max(1e-3, self._max_omega)
                    v = v_stage * max(0.25, 1.0 - 0.75 * turn_fraction)

                    now_log = time.monotonic()
                    if now_log - last_log > 1.0:
                        last_log = now_log
                        self.get_logger().info(
                            f'servo: x={tag_x * 100:+.1f}cm '
                            f'z={tag_z * 100:.1f}cm '
                            f'yaw={math.degrees(tag_yaw):+.1f}deg '
                            f'v={v:+.3f} omega={omega_cmd:+.3f}')

                    self._drive(-v, omega_cmd)
                    await self._tick()
                    continue

                if self._status == 'blind_creep':
                    if self._charging():
                        self._stop()
                        continue
                    # Remaining distance to contacts: tag was lost at dock mouth (e.g. 8-10cm),
                    # so remaining travel to contacts is only ~4-7cm, NOT 25cm!
                    creep_dist = min(0.08, max(0.03, (last_seen_z - 0.02) if last_seen_z is not None else 0.05))
                    self.get_logger().info(
                        f'blind_creep: last seen at {last_seen_z*100 if last_seen_z else 6:.1f}cm '
                        f'— creeping {creep_dist*100:.1f}cm gently at 10mm/s into contacts')
                    if await self._jog(creep_dist,
                                       speed=0.010,
                                       angular=0.0,
                                       push_effort=False):
                        continue
                    self._stop()
                    approach += 1
                    self.get_logger().info(
                        f'contact without charge — backing off for '
                        f'approach {approach}')
                    await self._clear_dock()
                    self._set_status('searching')
                    continue

                await self._tick()
        finally:
            self._stop()
            await self._set_camera(False)
            self._busy = False

    @staticmethod
    def _has_nav_goal(goal: NavigateToPose.Goal) -> bool:
        """Does this request actually ask to go somewhere?
         is NavigateToPose so one action can mean both 'undock' and
        'undock, then drive there'. But the UI's Undock button sends no pose at all,
        and an unset pose is (0,0) with a zero quaternion. Real goal carries a frame_id
        and a unit quaternion.
        """
        if not goal.pose.header.frame_id:
            return False
        q = goal.pose.pose.orientation
        return (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w) > 0.5

    async def _execute_undock(self, goal_handle):
        self._busy = True
        try:
            goal: NavigateToPose.Goal = goal_handle.request
            # Always drive forward off the dock when undock action is called
            if True:
                self._set_status('undocking')
                ok = False
                for attempt in range(3):
                    ok = await self._drive_straight(self._undock_distance, speed=self._undock_speed)
                    if ok:
                        break
                    self.get_logger().info(
                        f'undock: stalled on attempt {attempt + 1} — pushing '
                        'forward again (no in-place rotation)')
                if not ok:
                    self._set_status('undock_failed')
                    goal_handle.abort()
                    return NavigateToPose.Result()
                self._docked = False
                self._set_status('undocked')

            # If no destination pose was provided (pure 'Undock' button press), finish here
            if not self._has_nav_goal(goal):
                self.get_logger().info('undock: no navigation goal supplied — staying put')
                goal_handle.succeed()
                return NavigateToPose.Result()

            # Otherwise, forward the destination pose to Nav2 bt_navigator's navigate_to_pose action
            self._set_status('navigating')
            # Wait up to 90s for bt_navigator to be ACTIVE (not just present).
            # On boot, Nav2 lifecycle startup takes ~60-90s after AMCL publishes
            # the initial pose — goals sent before that are rejected with ACTION_SERVER_INACTIVE.
            nav_activate_deadline = time.monotonic() + 90.0
            if not self._nav_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error('navigate_to_pose action server not available')
                goal_handle.abort()
                self._set_status('error')
                return NavigateToPose.Result()

            self.get_logger().info(
                f'forwarding nav goal ({goal.pose.pose.position.x:.2f}, '
                f'{goal.pose.pose.position.y:.2f}) to Nav2 navigate_to_pose '
                f'(will retry up to 90s if bt_navigator still inactive)')

            def on_feedback(fb_msg):
                goal_handle.publish_feedback(fb_msg.feedback)

            child_goal_handle = None
            while time.monotonic() < nav_activate_deadline:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self._set_status('undocked')
                    return NavigateToPose.Result()
                child_nav_fut = self._nav_client.send_goal_async(goal, feedback_callback=on_feedback)
                while not child_nav_fut.done():
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                        self._set_status('undocked')
                        return NavigateToPose.Result()
                    await self._tick()
                candidate = child_nav_fut.result()
                if candidate.accepted:
                    child_goal_handle = candidate
                    break
                remaining = nav_activate_deadline - time.monotonic()
                self.get_logger().warn(
                    f'bt_navigator rejected goal (still inactive?) — retrying in 3s '
                    f'({remaining:.0f}s remaining)')
                await self._sleep(3.0)

            if child_goal_handle is None:
                self.get_logger().error('bt_navigator remained inactive — aborting nav goal')
                goal_handle.abort()
                self._set_status('undocked')
                return NavigateToPose.Result()

            result_fut = child_goal_handle.get_result_async()
            while not result_fut.done():
                if goal_handle.is_cancel_requested:
                    await child_goal_handle.cancel_goal_async()
                    goal_handle.canceled()
                    self._set_status('undocked')
                    return NavigateToPose.Result()
                await self._tick()

            nav_result = result_fut.result()
            if nav_result.status == GoalStatus.STATUS_SUCCEEDED:
                self._set_status('undocked')
                goal_handle.succeed()
                return nav_result.result
            elif nav_result.status == GoalStatus.STATUS_CANCELED:
                self._set_status('undocked')
                goal_handle.canceled()
                return nav_result.result
            else:
                self._set_status('undocked')
                goal_handle.abort()
                return nav_result.result

        finally:
            self._stop()
            self._busy = False

    def _load_dock_pose(self) -> Optional[PoseStamped]:
        if not os.path.exists(DOCK_POSE_FILE):
            return None
        try:
            with open(DOCK_POSE_FILE, 'r') as f:
                d = json.load(f)
            pose = PoseStamped()
            pose.header.frame_id = d['frame_id']
            pose.pose.position = Point(x=d['x'], y=d['y'], z=d['z'])
            pose.pose.orientation = Quaternion(
                x=d['qx'], y=d['qy'], z=d['qz'], w=d['qw'])
            self.get_logger().info(f'Loaded saved dock pose from {DOCK_POSE_FILE}')
            return pose
        except (OSError, ValueError, KeyError) as exc:
            self.get_logger().warn(f'Failed to load {DOCK_POSE_FILE}: {exc}')
            return None

    def _persist_dock_pose(self, pose: PoseStamped) -> None:
        d = {
            'frame_id': pose.header.frame_id,
            'x': pose.pose.position.x,
            'y': pose.pose.position.y,
            'z': pose.pose.position.z,
            'qx': pose.pose.orientation.x,
            'qy': pose.pose.orientation.y,
            'qz': pose.pose.orientation.z,
            'qw': pose.pose.orientation.w,
        }
        try:
            tmp_path = DOCK_POSE_FILE + '.tmp'
            with open(tmp_path, 'w') as f:
                json.dump(d, f)
            os.replace(tmp_path, DOCK_POSE_FILE)
        except OSError as exc:
            self.get_logger().warn(
                f'Failed to persist dock pose to {DOCK_POSE_FILE}: {exc}')

    def _on_dock_pose_set(self, msg: PoseStamped) -> None:
        self._dock_pose = msg
        self._persist_dock_pose(msg)


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = DockManagerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
