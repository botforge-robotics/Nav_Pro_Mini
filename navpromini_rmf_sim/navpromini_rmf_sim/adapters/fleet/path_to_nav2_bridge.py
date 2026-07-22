#!/usr/bin/env python3
"""Bridge RMF PathRequest → Nav2 NavigateToPose + RobotState feedback.

Production contract with rmf_demos EasyFullControl / fleet_manager:

  - fleet_manager navigate() publishes PathRequest = [current, destination]
    for **one** graph waypoint at a time (doors/events are separate RMF phases).
  - Command is complete only when RobotState has MODE_IDLE, path=[], and
    task_id == cmd_id. Completing early skips DoorOpen timing and makes the
    robot appear to jump to the final schedule waypoint.

This node therefore:
  1. Seeds AMCL /robotN/initialpose (frame_id=map) at startup
  2. Drives Nav2 to PathRequest.path[-1] (goal yaw = bearing to waypoint)
  3. Keeps MODE_MOVING + non-empty path until xy arrival within a tight tol
"""

from __future__ import annotations

import math
import threading
from typing import Dict, Optional, Tuple

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rmf_fleet_msgs.msg import PathRequest, RobotMode, RobotState


def _yaw_from_quat(z: float, w: float) -> float:
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


def _quat_from_yaw(yaw: float) -> Tuple[float, float]:
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


# Must stay below typical RMF lane spacing near doors (~0.4 m). Larger values
# false-complete approach waypoints → DoorOpen fires before the robot arrives.
_GOAL_TOL_M = 0.25
_GOAL_HOLD_SEC = 0.3
_SAME_CMD_TOL_M = 0.35


class _Robot:
    def __init__(self, node: 'PathToNav2Bridge', name: str, x: float, y: float, yaw: float):
        self.node = node
        self.name = name
        self.x = x
        self.y = y
        self.yaw = yaw
        self.have_odom = False
        self.level = 'L1'
        self.task_id = ''
        self.path: list = []
        self.mode = RobotMode.MODE_IDLE
        self.seq = 0
        self._lock = threading.Lock()
        self._goal_handle = None
        self._goal_seq = 0
        self._active_xy: Optional[Tuple[float, float]] = None
        self._last_nav_status = GoalStatus.STATUS_UNKNOWN
        self._inside_tol_since: Optional[float] = None
        self.client = ActionClient(
            node, NavigateToPose, f'/{name}/navigate_to_pose'
        )
        self.init_pub = node.create_publisher(
            PoseWithCovarianceStamped, f'/{name}/initialpose', 10
        )
        node.create_subscription(
            Odometry,
            f'/{name}/odom',
            self._on_odom,
            qos_profile_sensor_data,
        )

    def _now_sec(self) -> float:
        return self.node.get_clock().now().nanoseconds * 1e-9

    def _dist(self, x: float, y: float, ref: Optional[Tuple[float, float]]) -> float:
        if ref is None:
            return float('inf')
        dx = x - ref[0]
        dy = y - ref[1]
        return math.hypot(dx, dy)

    def _same_xy(
        self, x: float, y: float, ref: Optional[Tuple[float, float]], tol: float
    ) -> bool:
        return self._dist(x, y, ref) < tol

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.x = float(p.x)
        self.y = float(p.y)
        self.yaw = _yaw_from_quat(float(q.z), float(q.w))
        self.have_odom = True
        self._maybe_finish_arrival()

    def publish_initial_pose(self) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        z, w = _quat_from_yaw(self.yaw)
        msg.pose.pose.orientation.z = z
        msg.pose.pose.orientation.w = w
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.068
        self.init_pub.publish(msg)

    def _cancel_nav(self) -> None:
        with self._lock:
            handle = self._goal_handle
            self._goal_handle = None
            self._goal_seq += 1
        if handle is not None:
            handle.cancel_goal_async()

    def _complete_command(self, *, cancel_nav: bool, log: bool) -> None:
        """Signal fleet_manager: MODE_IDLE + empty path for current task_id."""
        if cancel_nav:
            self._cancel_nav()
        with self._lock:
            self._last_nav_status = GoalStatus.STATUS_SUCCEEDED
            # Keep _active_xy so identical republished PathRequests do not
            # restart Nav2 after success.
        self.path = []
        self.mode = RobotMode.MODE_IDLE
        self._inside_tol_since = None
        if log:
            xy = self._active_xy or (self.x, self.y)
            self.node.get_logger().info(
                f'[{self.name}] cmd complete task={self.task_id} '
                f'xy=({xy[0]:.2f}, {xy[1]:.2f})'
            )

    def _maybe_finish_arrival(self) -> None:
        with self._lock:
            target = self._active_xy
            navigating = (
                self.mode == RobotMode.MODE_MOVING
                and target is not None
                and (
                    self._goal_handle is not None
                    or self._last_nav_status
                    in (
                        GoalStatus.STATUS_ACCEPTED,
                        GoalStatus.STATUS_EXECUTING,
                    )
                )
            )
        if not navigating or target is None:
            self._inside_tol_since = None
            return

        if self._dist(self.x, self.y, target) >= _GOAL_TOL_M:
            self._inside_tol_since = None
            return

        now = self._now_sec()
        if self._inside_tol_since is None:
            self._inside_tol_since = now
            return
        if (now - self._inside_tol_since) < _GOAL_HOLD_SEC:
            return

        # Stop yaw hunting; report arrival only after a short dwell in tol.
        self._complete_command(cancel_nav=True, log=True)

    def handle_path_request(self, req: PathRequest) -> None:
        # fleet_manager stop_robot(): empty path OR identical start==end pose
        # (same x/y/yaw). Do NOT treat yaw-only navigate (same x/y, different
        # yaw) as stop — that breaks door approach / lane-end holds.
        if not req.path:
            self._stop_clear(req.task_id)
            return
        if len(req.path) >= 2:
            a, b = req.path[0], req.path[-1]
            same_xy = (
                abs(a.x - b.x) < 1e-3 and abs(a.y - b.y) < 1e-3
            )
            same_yaw = abs(a.yaw - b.yaw) < 1e-3
            if same_xy and same_yaw:
                self._stop_clear(req.task_id)
                return

        target = req.path[-1]
        tx, ty = float(target.x), float(target.y)
        self.task_id = req.task_id
        self.level = target.level_name or self.level

        with self._lock:
            same_cmd = self._same_xy(tx, ty, self._active_xy, _SAME_CMD_TOL_M)
            in_flight = self._goal_handle is not None or (
                self.mode == RobotMode.MODE_MOVING
                and self._last_nav_status
                in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)
            )
            already_done = (
                same_cmd
                and self._last_nav_status == GoalStatus.STATUS_SUCCEEDED
                and self.mode == RobotMode.MODE_IDLE
            )

        if already_done:
            # Keep completion signal for fleet_manager (IDLE + empty path).
            self.path = []
            self.mode = RobotMode.MODE_IDLE
            return

        if in_flight and same_cmd:
            # High-rate PathRequest republish — do not preempt Nav2.
            self.mode = RobotMode.MODE_MOVING
            if not self.path:
                self.path = [target]
            return

        # Already at destination (incl. yaw-only hold near door).
        if self._dist(self.x, self.y, (tx, ty)) < _GOAL_TOL_M and not in_flight:
            with self._lock:
                self._active_xy = (tx, ty)
            self._complete_command(cancel_nav=False, log=True)
            return

        self.mode = RobotMode.MODE_MOVING
        # Remaining path for fleet_manager: destination only (one WP command).
        self.path = [target]
        self._inside_tol_since = None
        # Goal yaw = bearing to waypoint (not frozen current yaw). Using the
        # yaw at request-time made DWB RotateToGoal spin at the end when the
        # approach heading differed from the heading when the goal was sent.
        if self.have_odom and self._dist(self.x, self.y, (tx, ty)) > 0.05:
            goal_yaw = math.atan2(ty - self.y, tx - self.x)
        else:
            goal_yaw = float(target.yaw)
        self._navigate(tx, ty, goal_yaw)

    def _stop_clear(self, task_id: str) -> None:
        self._cancel_nav()
        self.task_id = task_id or self.task_id
        self.path = []
        self.mode = RobotMode.MODE_IDLE
        with self._lock:
            self._active_xy = None
            self._last_nav_status = GoalStatus.STATUS_UNKNOWN
        self._inside_tol_since = None
        self.node.get_logger().info(
            f'[{self.name}] stop/clear task_id={self.task_id}'
        )
    def _navigate(self, x: float, y: float, yaw: float) -> None:
        if not self.client.server_is_ready():
            self.node.get_logger().warn(
                f'[{self.name}] Nav2 navigate_to_pose not ready '
                '(is AMCL localized? retrying...)'
            )
            if not self.client.wait_for_server(timeout_sec=0.0):
                return

        with self._lock:
            if (
                self._same_xy(x, y, self._active_xy, _SAME_CMD_TOL_M)
                and self._goal_handle is not None
            ):
                return
            old = self._goal_handle
            self._goal_handle = None
            self._goal_seq += 1
            seq = self._goal_seq
            self._active_xy = (x, y)
            self._last_nav_status = GoalStatus.STATUS_ACCEPTED
        if old is not None:
            old.cancel_goal_async()

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        z, w = _quat_from_yaw(yaw)
        pose.pose.orientation.z = z
        pose.pose.orientation.w = w
        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.node.get_logger().info(
            f'[{self.name}] Nav2 goal -> ({x:.2f}, {y:.2f}) task={self.task_id}'
        )
        fut = self.client.send_goal_async(goal)

        def _resp(f):
            handle = f.result()
            if handle is None or not handle.accepted:
                self.node.get_logger().error(
                    f'[{self.name}] Nav2 rejected goal — publish AMCL '
                    'initialpose (frame_id=map) and ensure scan/map TF'
                )
                with self._lock:
                    if seq == self._goal_seq:
                        self._active_xy = None
                        self._last_nav_status = GoalStatus.STATUS_ABORTED
                self.mode = RobotMode.MODE_ADAPTER_ERROR
                self.path = []
                return
            with self._lock:
                if seq != self._goal_seq:
                    handle.cancel_goal_async()
                    return
                self._goal_handle = handle
                self._last_nav_status = GoalStatus.STATUS_EXECUTING
            self.mode = RobotMode.MODE_MOVING
            rf = handle.get_result_async()

            def _done(rfut):
                with self._lock:
                    if seq != self._goal_seq:
                        return
                    self._goal_handle = None
                result = rfut.result()
                status = (
                    result.status
                    if result is not None
                    else GoalStatus.STATUS_ABORTED
                )
                if status == GoalStatus.STATUS_SUCCEEDED or self._dist(
                    self.x, self.y, (x, y)
                ) < _GOAL_TOL_M:
                    self._complete_command(cancel_nav=False, log=True)
                elif status in (
                    GoalStatus.STATUS_CANCELED,
                    GoalStatus.STATUS_CANCELING,
                ):
                    self.node.get_logger().debug(
                        f'[{self.name}] Nav2 canceled (superseded?)'
                    )
                else:
                    # Never report IDLE+empty on abort — fleet_manager treats
                    # that as command complete and skips door / waypoints.
                    with self._lock:
                        self._last_nav_status = status
                        still_target = self._active_xy
                    self.mode = RobotMode.MODE_MOVING
                    self.node.get_logger().warn(
                        f'[{self.name}] Nav2 finished status={status} — retry'
                    )
                    if still_target is not None:
                        self._navigate(
                            still_target[0],
                            still_target[1],
                            self.yaw if self.have_odom else 0.0,
                        )

            rf.add_done_callback(_done)

        fut.add_done_callback(_resp)

    def robot_state_msg(self) -> RobotState:
        self.seq += 1
        msg = RobotState()
        msg.name = self.name
        msg.model = 'NavProMini'
        msg.task_id = self.task_id
        msg.seq = self.seq
        msg.mode.mode = self.mode
        msg.battery_percent = 100.0
        msg.location.t = self.node.get_clock().now().to_msg()
        msg.location.x = float(self.x)
        msg.location.y = float(self.y)
        msg.location.yaw = float(self.yaw)
        msg.location.level_name = self.level
        msg.path = list(self.path)
        return msg


class PathToNav2Bridge(Node):
    def __init__(self):
        super().__init__('navpromini_path_to_nav2_bridge')
        self.declare_parameter('robot_names', 'robot1,robot2')
        self.declare_parameter('spawn_poses', '')
        self.declare_parameter('level_name', 'L1')
        self.declare_parameter('publish_hz', 10.0)
        self.declare_parameter('initialpose_repeats', 15)

        names = [
            n.strip()
            for n in self.get_parameter('robot_names').value.split(',')
            if n.strip()
        ]
        spawn_path = self.get_parameter('spawn_poses').value
        if not spawn_path:
            import os
            pkg = get_package_share_directory('navpromini_rmf_sim')
            spawn_path = os.path.join(pkg, 'site', 'spawn_poses.yaml')
        poses = self._load_spawns(spawn_path, names)
        level = self.get_parameter('level_name').value

        self._robots: Dict[str, _Robot] = {}
        for name in names:
            x, y, yaw = poses.get(name, (0.0, 0.0, 0.0))
            robot = _Robot(self, name, x, y, yaw)
            robot.level = level
            self._robots[name] = robot

        self._state_pub = self.create_publisher(RobotState, 'robot_state', 100)
        self.create_subscription(
            PathRequest, 'robot_path_requests', self._on_path, 100
        )

        hz = float(self.get_parameter('publish_hz').value)
        self.create_timer(1.0 / max(hz, 1.0), self._tick)

        self._init_left = int(self.get_parameter('initialpose_repeats').value)
        self.create_timer(0.2, self._init_tick)

        self.get_logger().info(
            f'PathRequest→Nav2 bridge ready for {names} '
            f'(tol={_GOAL_TOL_M}m hold={_GOAL_HOLD_SEC}s)'
        )

    def _load_spawns(self, path: str, names):
        import os
        out = {}
        data = {}
        if os.path.isfile(path):
            with open(path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
        robots = data.get('robots') or {}
        for name in names:
            r = robots.get(name) or {}
            out[name] = (
                float(r.get('x', 0.0)),
                float(r.get('y', 0.0)),
                float(r.get('yaw', 0.0)),
            )
        return out

    def _init_tick(self) -> None:
        if self._init_left <= 0:
            return
        self._init_left -= 1
        for robot in self._robots.values():
            robot.publish_initial_pose()
        if self._init_left == 0:
            self.get_logger().info('Finished publishing AMCL initial poses')

    def _on_path(self, msg: PathRequest) -> None:
        robot = self._robots.get(msg.robot_name)
        if robot is None:
            return
        robot.handle_path_request(msg)

    def _tick(self) -> None:
        for robot in self._robots.values():
            robot._maybe_finish_arrival()
            self._state_pub.publish(robot.robot_state_msg())


def main():
    rclpy.init()
    node = PathToNav2Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
