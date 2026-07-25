"""Relay fleet PoseStamped goals → local Nav2 NavigateToPose.

RMF free_fleet talks to Nav2 over zenoh *action* keys whose CDR does not match
zenoh-bridge-ros2dds (goals are dropped as invalid). Topics do match, so the
fleet adapter publishes PoseStamped on ``fleet_nav_goal`` and we call Nav2
locally with a normal rclpy ActionClient.
"""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Empty, Int32


class NavGoalRelay(Node):
    def __init__(self) -> None:
        super().__init__('navpro_nav_goal_relay')
        self._cb = ReentrantCallbackGroup()
        self._client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose', callback_group=self._cb
        )
        self._goal_handle = None
        self._active_xy: Optional[tuple[float, float]] = None
        self._status_pub = self.create_publisher(Int32, 'fleet_nav_status', 10)
        self.create_subscription(
            PoseStamped, 'fleet_nav_goal', self._on_goal, 10, callback_group=self._cb
        )
        self.create_subscription(
            Empty, 'fleet_nav_cancel', self._on_cancel, 10, callback_group=self._cb
        )
        self._publish_status(GoalStatus.STATUS_UNKNOWN)
        self.get_logger().info(
            'nav_goal_relay: fleet_nav_goal → /navigate_to_pose '
            '(status on fleet_nav_status)'
        )

    def _publish_status(self, status: int) -> None:
        msg = Int32()
        msg.data = int(status)
        self._status_pub.publish(msg)

    def _on_cancel(self, _msg: Empty) -> None:
        handle = self._goal_handle
        if handle is None:
            return
        self.get_logger().info('cancelling Nav2 goal')
        self._publish_status(GoalStatus.STATUS_CANCELING)
        fut = handle.cancel_goal_async()
        fut.add_done_callback(lambda _f: None)
        self._goal_handle = None
        self._active_xy = None

    def _on_goal(self, msg: PoseStamped) -> None:
        x = msg.pose.position.x
        y = msg.pose.position.y
        q = msg.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        # Same goal already executing — ignore (avoids cancel→backup jerks).
        if (
            self._goal_handle is not None
            and self._active_xy is not None
            and math.hypot(x - self._active_xy[0], y - self._active_xy[1]) < 0.30
        ):
            self.get_logger().info(
                f'ignore duplicate fleet_nav_goal ({x:.2f}, {y:.2f})'
            )
            return

        self.get_logger().info(
            f'fleet_nav_goal → Nav2 ({x:.2f}, {y:.2f}, yaw={yaw:.2f}) '
            f'frame={msg.header.frame_id or "map"}'
        )
        if not self._client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('navigate_to_pose action server not ready')
            self._publish_status(GoalStatus.STATUS_ABORTED)
            return

        # Cancel any in-flight goal first.
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:  # noqa: BLE001
                pass
            self._goal_handle = None

        goal = NavigateToPose.Goal()
        goal.pose = msg
        if not goal.pose.header.frame_id:
            goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        self._active_xy = (float(x), float(y))
        self._publish_status(GoalStatus.STATUS_ACCEPTED)
        send_fut = self._client.send_goal_async(goal)
        send_fut.add_done_callback(self._goal_response)

    def _goal_response(self, fut) -> None:
        try:
            handle = fut.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'send_goal failed: {exc}')
            self._publish_status(GoalStatus.STATUS_ABORTED)
            return
        if handle is None or not handle.accepted:
            self.get_logger().error('Nav2 rejected goal')
            self._publish_status(GoalStatus.STATUS_ABORTED)
            self._goal_handle = None
            return
        self._goal_handle = handle
        self._publish_status(GoalStatus.STATUS_EXECUTING)
        result_fut = handle.get_result_async()
        result_fut.add_done_callback(self._goal_result)

    def _goal_result(self, fut) -> None:
        try:
            wrapped = fut.result()
            status = int(wrapped.status)
            result = getattr(wrapped, 'result', None)
            err = getattr(result, 'error_code', None) if result is not None else None
            msg = getattr(result, 'error_msg', None) if result is not None else None
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'get_result failed: {exc}')
            status = GoalStatus.STATUS_ABORTED
            err = None
            msg = None
        self._publish_status(status)
        self._goal_handle = None
        self._active_xy = None
        if err is not None or msg:
            self.get_logger().info(
                f'Nav2 finished status={status} error_code={err} msg={msg!r}'
            )
        else:
            self.get_logger().info(f'Nav2 finished status={status}')


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = NavGoalRelay()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
