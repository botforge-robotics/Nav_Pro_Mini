#!/usr/bin/env python3
"""Single robot-side contract point for docking/undocking/navigation.

Every client (this Flutter app today, a future web API later) should call
these two actions instead of touching docking_server / bt_navigator directly,
so cross-cutting behavior lives in exactly one place on the robot:

  dock     (nav2_msgs/action/DockRobot)
      Navigates to a staging pose in front of the dock with the robot's rear
      facing it, then hands the final approach to tag_dock_node's `tag_dock`
      action, which servos the dock's AprilTag to the centre of the rear
      camera and reverses straight in until the battery reports charging.

      Replaces earlier IR-beacon and lidar back ends, both now removed.
      Measured on this robot the tag gives 100% detection repeating to 0.3px,
      where the IR zone flapped several times a second and the lidar dock fit
      was unusable beyond ~45cm.

  undock   (nav2_msgs/action/NavigateToPose)
      The entry point any client should use to send a nav goal on this
      robot. If currently docked, undocks first (awaited) before relaying to
      bt_navigator's `navigate_to_pose` with the given goal pose — implements
      "any new goal undocks first" for every client uniformly, not just one
      app. If not currently docked, it's just a normal navigate-to-pose.

Dock pose persistence — so a web API client or a bare `ros2 action
send_goal`/`ros2 topic pub` from a terminal can dock without needing to
already know the dock's pose (today only the Flutter app's bookmark does):

  - Any client can publish geometry_msgs/PoseStamped on `dock_pose`
    (reliable + transient_local — a late subscriber gets the last value
    automatically, no polling needed) to set/update the saved dock pose.
    Persisted to DOCK_POSE_FILE so it survives a node/robot restart.
  - `dock` action goals with `use_dock_id: true` use this saved pose
    instead of requiring the caller to supply one.
  - `dock` action goals with `use_dock_id: false` (an explicit dock_pose
    given) still work as before, and also refresh the saved pose — so
    Flutter's existing per-call pose sending keeps the robot's stored copy
    in sync automatically, no separate publish required from it either.

Publishes:
  /dock_status (std_msgs/String) — one of:
      undocked | staging | detecting | docking | waiting_for_charge |
      charging | full | undocking | error
  /dock_pose (geometry_msgs/PoseStamped, transient_local) — the currently
      saved dock pose, if any.
"""

from __future__ import annotations

import json
import math
import os
import traceback
from typing import Optional

# Node output wasn't reliably reaching journald through the nested
# ros2-launch chain this node is started from (buffering/capture gap,
# separate from any real bug) — write crashes straight to a file too so a
# failing execute callback is never silently invisible.
CRASH_LOG_FILE = '/tmp/dock_manager_crash.log'

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from nav2_msgs.action import BackUp, DockRobot, DriveOnHeading, NavigateToPose
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.task import Future
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from std_srvs.srv import SetBool

DOCK_POSE_FILE = os.path.expanduser('~/.navpromini_dock_pose.json')

# Publish side: transient_local so a client that (re)subscribes after the
# pose was set still gets it without polling (same pattern as /map).
_DOCK_POSE_PUB_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)
# Subscribe side: plain volatile. DDS QoS compatibility requires a
# transient_local *subscriber* to only match a transient_local publisher —
# rosbridge-side publishers (Flutter, or `ros2 topic pub` from a terminal)
# are ordinary volatile publishers, so staying volatile here is what
# actually lets their updates reach us. The two ends don't need matching
# QoS on the same topic name.
_DOCK_POSE_SUB_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

# nav2_msgs/action/DockRobot Feedback.state values
_DOCK_STATE_TO_STATUS = {
    DockRobot.Feedback.NONE: 'staging',
    DockRobot.Feedback.NAV_TO_STAGING_POSE: 'staging',
    DockRobot.Feedback.INITIAL_PERCEPTION: 'detecting',
    DockRobot.Feedback.CONTROLLING: 'docking',
    DockRobot.Feedback.WAIT_FOR_CHARGE: 'waiting_for_charge',
    DockRobot.Feedback.RETRY: 'docking',
}

# Final ~1cm nudge backward to seat firmly against the dock connector, once
# docking_server itself reports success. Sign convention (target.x negative =
# backward with positive speed) matches nav2_behaviors::BackUp — bench-test
# on the real robot before trusting the direction unattended.
_SEAT_BACKUP_TARGET = Point(x=-0.01, y=0.0, z=0.0)
_SEAT_BACKUP_SPEED = 0.02
_SEAT_BACKUP_TIMEOUT_SEC = 5.0

# Odometry-only fallback for the final approach, used when docking_server's
# own lidar-detection-driven controller aborts after we'd already gotten
# close (reached CONTROLLING/RETRY/WAIT_FOR_CHARGE — past initial
# perception). Confirmed live: that controller can abort unpredictably even
# once aligned and approaching, with no usable error surfaced. Plain
# odometry drift over this short a distance is small enough to trust
# directly rather than keep depending on a lidar re-detection loop that's
# proven flaky. Slow speed per explicit request — this is a blind backup
# into a fixed object, err on the side of "too slow to hurt anything".
#
# 0.30 undershot in the first live test — compared /dock_pose against
# /amcl_pose right after a failed attempt and the robot was still ~0.385m
# from the dock's bookmark pose, meaning the controller handed off to this
# fallback from farther out than assumed. Bumped with margin; now that
# _confirm_charging actually verifies contact instead of assuming success,
# overshoot just means pressing gently against the dock rather than a false
# "docked" report, so erring longer here is the safer direction.
# Where navigation parks before handing the final approach to the tag
# controller. 60cm from the dock face, measured to the robot's rear.
#
# Was 75cm, chosen back when the IR beacon drove the approach and a
# differential base needed maximum "arc room" to fix lateral error before
# closing in. The AprilTag controller does not have that constraint: it steers
# continuously while reversing, so it corrects on the way in rather than
# needing the error gone before it starts. Closer staging means a shorter,
# straighter, faster approach with less accumulated odometry drift.
#
# Was 0.60. The ~0.5m floor this comment used to warn about was reasoned
# from pixel size alone (tag subtends more px, not fewer, as this shrinks —
# 60cm gives ~90px, 40cm gives ~135px, both comfortably above the ~40px
# marginal-decoding point) — closer was never a size problem. The real risk
# at close range is the camera's own minimum focus distance / the tag
# drifting toward frame-edge distortion, neither measured here — by request,
# tightened anyway. If detection gets flaky specifically at this distance
# (not the "search sweep" symptom, a genuine blur/framing one), that is the
# mechanism to revisit, not pixel size.
_STAGING_STANDOFF_M = 0.40

# Undocking is just "drive forward far enough to break the pogo-pin contact".
# 40cm clears the connector and the funnel with margin.
#
# Done with nav2_behaviors' DriveOnHeading rather than opennav_docking's
# UndockRobot, deliberately. docking_server never docked this robot — the
# AprilTag controller did — so its internal state says "not docked" and its
# undock did the bare minimum then reported failure: observed as the robot
# moving ~2cm until the electrodes parted, then "undock unsuccessful". Worse,
# `undock` is the entry point for every navigation goal, so a failed undock
# could poison ordinary driving.
_UNDOCK_DISTANCE_M = 0.40
_UNDOCK_SPEED = 0.05
_UNDOCK_TIMEOUT_SEC = 25.0

_ODOM_FALLBACK_DISTANCE_M = 0.45
_ODOM_FALLBACK_SPEED = 0.03
# 0.45m / 0.03m/s = 15s nominal — timeout gives real margin on top of that.
_ODOM_FALLBACK_TIMEOUT_SEC = 25.0


class DockManagerNode(Node):
    def __init__(self) -> None:
        super().__init__('navpromini_dock_manager')

        self.declare_parameter('dock_type', 'simple_charging_dock')
        self._dock_type = str(self.get_parameter('dock_type').value)

        self.declare_parameter('use_tag_docking', True)
        self._use_tag_docking = bool(self.get_parameter('use_tag_docking').value)
        # Mapping is started with the robot DOCKED, so slam_toolbox's origin
        # is the docked pose and the dock's location in map frame is known
        # without anyone placing a bookmark. That is what makes the dock pose
        # survive a re-map: re-mapping produces a new origin, but if the origin
        # is always the dock, the dock pose is always the same numbers.
        #
        # Used only as a FALLBACK — an explicitly saved/placed dock pose always
        # wins, because a robot that was not docked when mapping started would
        # otherwise silently drive at the map origin.
        self.declare_parameter('assume_dock_at_map_origin', True)
        self._assume_origin = bool(
            self.get_parameter('assume_dock_at_map_origin').value)
        # base_link to the dock face when docked: the robot's rear surface,
        # i.e. its radius (260mm diameter). The dock sits behind the robot,
        # hence negative x, and its outward normal points the way the robot
        # faces when docked, hence yaw 0.
        self.declare_parameter('dock_origin_offset_m', 0.13)
        self._dock_origin_offset = float(
            self.get_parameter('dock_origin_offset_m').value)

        self.declare_parameter('staging_standoff_m', _STAGING_STANDOFF_M)
        self._standoff = float(self.get_parameter('staging_standoff_m').value)
        # The saved dock pose's yaw points OUTWARD from the dock face, so a
        # robot sitting at the staging pose with that same yaw has its rear
        # toward the dock — which is the orientation it docks in. Confirmed
        # live against the previous opennav_docking staging behaviour (a
        # positive staging_x_offset moved the robot away from the dock, and
        # the robot ended up back-to-dock). Flip this to pi if a future dock
        # pose convention stores the inward direction instead.
        self.declare_parameter('staging_yaw_offset_rad', 0.0)
        self._staging_yaw_offset = float(
            self.get_parameter('staging_yaw_offset_rad').value)

        cb = ReentrantCallbackGroup()

        # Downstream clients (docking_server / behavior_server / bt_navigator).
        self._dock_client = ActionClient(self, DockRobot, 'dock_robot', callback_group=cb)
        self._undock_client = ActionClient(self, DriveOnHeading, 'drive_on_heading', callback_group=cb)
        self._backup_client = ActionClient(self, BackUp, 'backup', callback_group=cb)
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=cb)
        self._tag_dock_client = ActionClient(self, DockRobot, 'tag_dock', callback_group=cb)
        # Rear camera + AprilTag detector — only needed for the tag_dock
        # portion of a dock attempt below (_execute_dock_tag), so they sit
        # inactive (no USB capture open, no detection CPU) the rest of a
        # robot's life: idle, navigating, undocking, already docked.
        self._camera_active_client = self.create_client(
            SetBool, 'camera/set_active', callback_group=cb)
        self._tag_active_client = self.create_client(
            SetBool, 'dock_tag/set_active', callback_group=cb)

        # Our own action servers — the only two things any client should call.
        self._dock_server = ActionServer(
            self, DockRobot, 'dock',
            execute_callback=self._execute_dock,
            goal_callback=self._accept_goal,
            cancel_callback=self._accept_cancel,
            callback_group=cb,
        )
        self._undock_server = ActionServer(
            self, NavigateToPose, 'undock',
            execute_callback=self._execute_undock_then_navigate,
            goal_callback=self._accept_goal,
            cancel_callback=self._accept_cancel,
            callback_group=cb,
        )

        self._status_pub = self.create_publisher(String, 'dock_status', 10)
        # Tells dock_detector_node roughly where to look — published once per
        # dock attempt, not a continuous stream.
        self._expected_pose_pub = self.create_publisher(PoseStamped, 'dock_expected_pose', 10)
        self.create_subscription(BatteryState, 'battery/state', self._on_battery, 10)

        # Saved dock pose — see module docstring. Publisher is transient_local
        # so a client that connects after the pose was set still gets it.
        self._dock_pose_pub = self.create_publisher(PoseStamped, 'dock_pose', _DOCK_POSE_PUB_QOS)
        self.create_subscription(PoseStamped, 'dock_pose', self._on_dock_pose_set, _DOCK_POSE_SUB_QOS)
        self._dock_pose: Optional[PoseStamped] = self._load_dock_pose()
        if self._dock_pose is None and self._assume_origin:
            self._dock_pose = self._dock_pose_at_map_origin()
            self.get_logger().info(
                'No saved dock pose — assuming the dock is at the map origin '
                f'({-self._dock_origin_offset:+.2f}m x, yaw 0), which holds when '
                'mapping was started with the robot docked. Publish on '
                '`dock_pose` or place a dock bookmark to override.')
        if self._dock_pose is not None:
            self._dock_pose_pub.publish(self._dock_pose)

        self._docked = False
        self._status = 'undocked'
        self._last_power_supply_status: Optional[int] = None
        # Tracks whichever child goal (dock_robot / drive_on_heading /
        # navigate_to_pose) is currently in flight, so _accept_cancel can
        # propagate a cancel on our own goal without polling — see
        # _await_with_cancel.
        self._active_child_handle = None

        self.create_timer(1.0, self._publish_status)
        backend = 'AprilTag' if self._use_tag_docking else 'lidar/opennav_docking'
        self.get_logger().info(
            f'dock_manager ready: dock_type={self._dock_type!r}, '
            f'approach={backend} — actions dock / undock'
        )

    # -- goal/cancel bookkeeping -------------------------------------------------

    def _accept_goal(self, _goal_request) -> GoalResponse:
        return GoalResponse.ACCEPT

    def _accept_cancel(self, _goal_handle) -> CancelResponse:
        child = self._active_child_handle
        if child is not None:
            # Fire-and-forget — this callback is synchronous (not a
            # coroutine), so we can't await the cancel confirmation here.
            # The child's own result future (awaited in _await_with_cancel)
            # completes with CANCELED once the cancel actually goes through,
            # which is what unblocks the execute callback.
            child.cancel_goal_async()
        return CancelResponse.ACCEPT

    def _set_status(self, status: str) -> None:
        self._status = status
        self._publish_status()

    def _publish_status(self) -> None:
        self._status_pub.publish(String(data=self._status))

    # -- saved dock pose (persisted robot-side; see module docstring) ----------

    def _load_dock_pose(self) -> Optional[PoseStamped]:
        if not os.path.exists(DOCK_POSE_FILE):
            return None
        try:
            with open(DOCK_POSE_FILE, 'r') as f:
                d = json.load(f)
            pose = PoseStamped()
            pose.header.frame_id = d['frame_id']
            pose.pose.position = Point(x=d['x'], y=d['y'], z=d['z'])
            pose.pose.orientation = Quaternion(x=d['qx'], y=d['qy'], z=d['qz'], w=d['qw'])
            self.get_logger().info(f'Loaded saved dock pose from {DOCK_POSE_FILE}')
            return pose
        except (OSError, ValueError, KeyError) as exc:
            self.get_logger().warn(f'Failed to load {DOCK_POSE_FILE}: {exc}')
            return None

    def _dock_pose_at_map_origin(self) -> PoseStamped:
        """The dock's pose assuming the map origin is the docked pose.

        With the robot docked at the origin it faces away from the dock
        (identity orientation), so the dock face lies `dock_origin_offset_m`
        behind it along -x, and the face's outward normal points along +x —
        i.e. yaw 0, matching the convention _staging_pose_for expects.

        Deliberately NOT persisted: it is a derived default, and writing it to
        disk would make it indistinguishable from a pose someone actually
        placed, so a later correction could not tell them apart.
        """
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position = Point(x=-self._dock_origin_offset, y=0.0, z=0.0)
        pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        return pose

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
            self.get_logger().warn(f'Failed to persist dock pose to {DOCK_POSE_FILE}: {exc}')

    def _set_dock_pose(self, pose: PoseStamped, *, republish: bool) -> None:
        self._dock_pose = pose
        self._persist_dock_pose(pose)
        if republish:
            self._dock_pose_pub.publish(pose)

    def _on_dock_pose_set(self, msg: PoseStamped) -> None:
        # A client (Flutter bookmark edit, web API, terminal) told us the
        # dock's pose — no need to republish, we're just relaying our own
        # topic's transient_local cache back to whoever set it.
        self._set_dock_pose(msg, republish=False)

    def _on_battery(self, msg: BatteryState) -> None:
        self._last_power_supply_status = msg.power_supply_status
        if not self._docked:
            return
        if msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_FULL:
            self._set_status('full')
        elif msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING:
            self._set_status('charging')
        # DISCHARGING/NOT_CHARGING/UNKNOWN while docked: leave last dock-phase
        # status alone (still connecting) rather than guessing.

    async def _await_with_cancel(self, _goal_handle, child_goal_handle, result_future):
        """Await result_future; propagate a cancel on our own goal to the
        child goal via _accept_cancel (which fires independently through
        self._active_child_handle — see there).

        Returns the child's wrapped result once it settles, including the
        CANCELED case (never returns None; a cancel just makes result_future
        resolve with status=CANCELED like any other terminal state).

        Previously polled with `while not result_future.done(): ...
        asyncio.sleep(0.1)`, which crashed instantly with "no running event
        loop" — rclpy's MultiThreadedExecutor drives coroutines with its own
        mechanism, not a real asyncio loop, so raw asyncio.sleep() doesn't
        work here even though awaiting an rclpy Future directly (as done
        below) does. Confirmed live via dock_manager's own crash log: this
        was silently aborting every dock/undock goal right after
        successfully forwarding it, while the forwarded goal kept running to
        completion on its own — explains "robot works, UI says failed".
        """
        self._active_child_handle = child_goal_handle
        try:
            return await result_future
        finally:
            self._active_child_handle = None

    # -- dock -------------------------------------------------

    def _log_crash(self, where: str, exc: Exception) -> None:
        tb = traceback.format_exc()
        self.get_logger().error(f'{where} raised {exc!r}\n{tb}')
        try:
            with open(CRASH_LOG_FILE, 'a') as f:
                f.write(f'--- {where} ---\n{tb}\n')
        except OSError:
            pass

    async def _execute_dock(self, goal_handle):
        try:
            return await self._execute_dock_inner(goal_handle)
        except Exception as e:  # noqa: BLE001 — see _log_crash: make failures visible
            self._log_crash('_execute_dock', e)
            self._set_status('error')
            try:
                goal_handle.abort()
            except Exception:
                pass
            return DockRobot.Result(success=False, error_msg=f'{type(e).__name__}: {e}')

    async def _execute_dock_inner(self, goal_handle):
        goal: DockRobot.Goal = goal_handle.request
        self._set_status(_DOCK_STATE_TO_STATUS[DockRobot.Feedback.NAV_TO_STAGING_POSE])

        # use_dock_id repurposed as "use the pose we've saved" — any client
        # (web API, terminal) can dock without knowing the pose, as long as
        # someone (typically the Flutter dock bookmark) set it at some point.
        # An explicit dock_pose (use_dock_id: false) still works as before,
        # and also refreshes the saved copy so it stays current.
        if goal.use_dock_id:
            if self._dock_pose is None:
                goal_handle.abort()
                self._set_status('error')
                return DockRobot.Result(
                    success=False,
                    error_msg='use_dock_id requested but no dock pose has been saved yet '
                              '(publish one on the dock_pose topic first)',
                )
            goal.use_dock_id = False
            goal.dock_pose = self._dock_pose
        else:
            self._set_dock_pose(goal.dock_pose, republish=True)

        self._expected_pose_pub.publish(goal.dock_pose)

        if self._use_tag_docking:
            return await self._execute_dock_tag(goal_handle, goal)

        if not self._dock_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('dock_robot action server not available')
            goal_handle.abort()
            self._set_status('error')
            return DockRobot.Result(success=False, error_msg='dock_robot server unavailable')

        # Tracks whether we ever got past initial perception into the actual
        # approach (CONTROLLING/RETRY/WAIT_FOR_CHARGE) — gates the odometry
        # fallback below: only trust a blind backup once we know we were
        # actually aligned and closing in, not from a cold/unknown position.
        reached_controlling = False

        def on_feedback(fb_msg) -> None:
            nonlocal reached_controlling
            fb: DockRobot.Feedback = fb_msg.feedback
            if fb.state in (
                DockRobot.Feedback.CONTROLLING,
                DockRobot.Feedback.WAIT_FOR_CHARGE,
                DockRobot.Feedback.RETRY,
            ):
                reached_controlling = True
            goal_handle.publish_feedback(fb)
            self._set_status(_DOCK_STATE_TO_STATUS.get(fb.state, 'docking'))

        child_goal_handle = await self._dock_client.send_goal_async(goal, feedback_callback=on_feedback)
        if not child_goal_handle.accepted:
            goal_handle.abort()
            self._set_status('error')
            return DockRobot.Result(success=False, error_msg='dock_robot goal rejected')

        wrapped = await self._await_with_cancel(goal_handle, child_goal_handle, child_goal_handle.get_result_async())
        if wrapped is None:
            self._set_status('error')
            return DockRobot.Result(success=False, error_msg='cancelled')

        dock_result = wrapped.result
        if not dock_result.success:
            if reached_controlling:
                self.get_logger().warn(
                    'dock_robot aborted after reaching the approach phase — '
                    'trying an odometry-only fallback for the final stretch '
                    'instead of giving up (see _ODOM_FALLBACK_* in config)'
                )
                if await self._odometry_finish_dock():
                    return await self._finish_successful_dock(
                        goal_handle, DockRobot.Result(success=True))
                self.get_logger().warn('Odometry fallback also failed/timed out')
            goal_handle.abort()
            self._set_status('error')
            return dock_result

        return await self._finish_successful_dock(goal_handle, dock_result)

    # -- IR docking path ------------------------------------------------------

    def _staging_pose_for(self, dock_pose: PoseStamped) -> PoseStamped:
        """Staging pose: robot rear `staging_standoff_m` from the dock face.

        The dock pose's yaw points away from the dock face, so stepping
        `staging_standoff_m` along it lands in front of the dock, and keeping the
        same yaw leaves the robot's rear — where the charging contacts and the
        IR receivers are — pointing at the dock. See staging_yaw_offset_rad.
        """
        q = dock_pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        staging_yaw = yaw + self._staging_yaw_offset

        staging = PoseStamped()
        staging.header.frame_id = dock_pose.header.frame_id or 'map'
        staging.header.stamp = self.get_clock().now().to_msg()
        # standoff is measured from the robot's REAR to the dock face, which
        # is what "stop 60cm before the dock" means physically — so the pose
        # for base_link sits one rear-offset further out again. Without this
        # the robot stopped a robot-radius closer than asked.
        reach = self._standoff + self._dock_origin_offset
        staging.pose.position = Point(
            x=dock_pose.pose.position.x + math.cos(yaw) * reach,
            y=dock_pose.pose.position.y + math.sin(yaw) * reach,
            z=0.0,
        )
        staging.pose.orientation = Quaternion(
            x=0.0, y=0.0,
            z=math.sin(staging_yaw / 2.0),
            w=math.cos(staging_yaw / 2.0),
        )
        return staging

    async def _set_tag_sensing(self, active: bool) -> bool:
        """Turns the rear camera and AprilTag detector on/off — see the
        comment by the two SetBool clients above. Best-effort but honest
        about failure: if activation genuinely fails, the caller aborts
        rather than sweeping for a tag that structurally cannot be seen.
        Deactivation failures are only logged — leaving the camera on a
        little longer than needed is harmless, unlike proceeding blind.
        """
        ok = True
        for client, name in ((self._camera_active_client, 'camera'),
                             (self._tag_active_client, 'dock_tag')):
            if not client.wait_for_service(timeout_sec=3.0):
                self.get_logger().warn(
                    f'{name}/set_active unavailable — is {name}_node running?')
                ok = False
                continue
            req = SetBool.Request()
            req.data = active
            try:
                resp = await client.call_async(req)
                if not resp.success:
                    self.get_logger().warn(
                        f'{name}/set_active({active}) reported failure: {resp.message}')
                    ok = False
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f'{name}/set_active({active}) call failed: {exc}')
                ok = False
        return ok or not active

    async def _execute_dock_tag(self, goal_handle, goal: DockRobot.Goal):
        """Nav to the standoff, then hand the last 75cm to the IR beacon.

        Split deliberately: getting to the staging pose is ordinary
        navigation and AMCL is good enough for it, while the final approach
        needs centimetre accuracy that map-frame localisation cannot give —
        which is exactly the accuracy the beacon measures directly, in the
        robot's own frame, with no dependence on how precisely the dock
        bookmark was placed.
        """
        if not self._tag_dock_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('tag_dock action server not available')
            goal_handle.abort()
            self._set_status('error')
            return DockRobot.Result(
                success=False,
                error_msg='tag_dock server unavailable (is tag_dock_node running?)',
            )

        if goal.navigate_to_staging_pose:
            staging = self._staging_pose_for(goal.dock_pose)
            self.get_logger().info(
                f'Tag dock: navigating to staging pose '
                f'({staging.pose.position.x:.2f}, {staging.pose.position.y:.2f}) '
                f'— rear {self._standoff:.2f}m from the dock face'
            )
            self._set_status('staging')
            if not await self._navigate_to(goal_handle, staging):
                goal_handle.abort()
                self._set_status('error')
                return DockRobot.Result(
                    success=False,
                    error_msg='failed to reach the docking staging pose',
                )

        self._set_status('detecting')

        def on_feedback(fb_msg) -> None:
            fb: DockRobot.Feedback = fb_msg.feedback
            goal_handle.publish_feedback(fb)
            self._set_status(_DOCK_STATE_TO_STATUS.get(fb.state, 'docking'))

        ir_goal = DockRobot.Goal()
        ir_goal.dock_pose = goal.dock_pose
        ir_goal.dock_type = self._dock_type
        ir_goal.navigate_to_staging_pose = False

        if not await self._set_tag_sensing(True):
            goal_handle.abort()
            self._set_status('error')
            return DockRobot.Result(
                success=False,
                error_msg='could not activate the dock camera/tag detector '
                          '(is camera_node/dock_tag_node running?)',
            )
        try:
            child = await self._tag_dock_client.send_goal_async(
                ir_goal, feedback_callback=on_feedback)
            if not child.accepted:
                goal_handle.abort()
                self._set_status('error')
                return DockRobot.Result(success=False, error_msg='tag_dock goal rejected')

            wrapped = await self._await_with_cancel(
                goal_handle, child, child.get_result_async())
            if wrapped is None:
                self._set_status('error')
                return DockRobot.Result(success=False, error_msg='cancelled')

            result = wrapped.result
            if not result.success:
                goal_handle.abort()
                self._set_status('error')
                return result
        finally:
            # Always, regardless of which path above returned — success,
            # rejection, cancellation, or failure all end the tag_dock
            # portion and should turn the camera back off.
            await self._set_tag_sensing(False)

        # tag_dock_node already drove in until it stalled against the connector
        # and confirmed charging itself, so no seat nudge here — a BackUp at
        # this point would just push harder against a dock we are already
        # seated in. _confirm_charging still runs as the single success gate.
        return await self._finish_successful_dock(goal_handle, result, seat=False)

    async def _navigate_to(self, goal_handle, pose: PoseStamped) -> bool:
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('navigate_to_pose action server not available')
            return False
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = pose
        child = await self._nav_client.send_goal_async(nav_goal)
        if not child.accepted:
            return False
        wrapped = await self._await_with_cancel(
            goal_handle, child, child.get_result_async())
        return wrapped is not None and wrapped.status == GoalStatus.STATUS_SUCCEEDED

    async def _sleep(self, seconds: float) -> None:
        """rclpy-compatible sleep. Plain asyncio.sleep() does NOT work here
        — confirmed via crash log: MultiThreadedExecutor drives these
        coroutines with its own mechanism, not a real running asyncio
        event loop, so asyncio.sleep() raises "no running event loop"
        (same root cause as the earlier _await_with_cancel crash). A
        one-shot rclpy Timer resolving an rclpy.task.Future is awaitable
        the same way action-client futures already are here."""
        future = Future()
        timer = self.create_timer(seconds, lambda: future.set_result(None))
        try:
            await future
        finally:
            timer.cancel()
            timer.destroy()

    async def _confirm_charging(self, *, timeout_sec: float = 5.0,
                                 poll_interval_sec: float = 0.5) -> bool:
        """Poll actual /battery/state.power_supply_status for up to
        timeout_sec, True once it reports CHARGING or FULL.

        This is the one place that decides "are we actually docked" —
        both the normal docking_server-success path and the odometry
        fallback path go through it, so neither can declare success (and
        flip the LED to the charging pattern) without a real battery-state
        confirmation. Confirmed live: the odometry fallback previously
        declared success (and changed the LED) purely because its blind
        backup movement completed, with no check that contact was
        actually made — battery never started charging, robot ended up
        sitting ~30cm short. docking_server's own use_battery_status
        already gates its reported success on this internally, so this is
        mostly a no-op double-check on that path; it's essential on the
        fallback path.
        """
        elapsed = 0.0
        while elapsed < timeout_sec:
            if self._last_power_supply_status in (
                BatteryState.POWER_SUPPLY_STATUS_CHARGING,
                BatteryState.POWER_SUPPLY_STATUS_FULL,
            ):
                return True
            await self._sleep(poll_interval_sec)
            elapsed += poll_interval_sec
        return False

    async def _finish_successful_dock(self, goal_handle, dock_result, *, seat: bool = True):
        # Seat firmly first (small backward nudge), then the one real
        # confirmation gate — see _confirm_charging's docstring for why
        # this can't just be assumed from either path completing.
        if seat:
            seat_ok = await self._seat_backup()
            if not seat_ok:
                self.get_logger().warn('Seat backup after docking failed/timed out — checking charging anyway')

        if not await self._confirm_charging():
            self.get_logger().warn(
                'Approach completed but battery never started charging — '
                'not actually docked, reporting failure instead of a false success'
            )
            self._docked = False
            goal_handle.abort()
            self._set_status('error')
            return DockRobot.Result(
                success=False,
                error_msg='approach completed but battery never started charging (not seated)',
            )

        self._docked = True
        if self._last_power_supply_status == BatteryState.POWER_SUPPLY_STATUS_FULL:
            self._set_status('full')
        else:
            self._set_status('charging')

        goal_handle.succeed()
        return dock_result

    async def _odometry_finish_dock(self) -> bool:
        """Blind, odometry-only backup covering the final approach when
        docking_server's own controller aborted after we were already
        aligned and closing in. See _ODOM_FALLBACK_* constants."""
        if not self._backup_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn(
                'backup action server not available — cannot run odometry docking fallback'
            )
            return False
        goal = BackUp.Goal()
        goal.target = Point(x=-_ODOM_FALLBACK_DISTANCE_M, y=0.0, z=0.0)
        goal.speed = _ODOM_FALLBACK_SPEED
        sec = int(_ODOM_FALLBACK_TIMEOUT_SEC)
        goal.time_allowance.sec = sec
        goal.time_allowance.nanosec = int((_ODOM_FALLBACK_TIMEOUT_SEC - sec) * 1e9)

        child_goal_handle = await self._backup_client.send_goal_async(goal)
        if not child_goal_handle.accepted:
            return False
        result = await child_goal_handle.get_result_async()
        return result.status == GoalStatus.STATUS_SUCCEEDED

    async def _seat_backup(self) -> bool:
        if not self._backup_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn('backup action server not available — skipping seat nudge')
            return False
        goal = BackUp.Goal()
        goal.target = _SEAT_BACKUP_TARGET
        goal.speed = _SEAT_BACKUP_SPEED
        sec = int(_SEAT_BACKUP_TIMEOUT_SEC)
        goal.time_allowance.sec = sec
        goal.time_allowance.nanosec = int((_SEAT_BACKUP_TIMEOUT_SEC - sec) * 1e9)

        child_goal_handle = await self._backup_client.send_goal_async(goal)
        if not child_goal_handle.accepted:
            return False
        result = await child_goal_handle.get_result_async()
        return result.status == GoalStatus.STATUS_SUCCEEDED

    # -- undock (always the entry point for "go to this pose" too) -------------

    async def _execute_undock_then_navigate(self, goal_handle):
        try:
            return await self._execute_undock_then_navigate_inner(goal_handle)
        except Exception as e:  # noqa: BLE001 — see _log_crash: make failures visible
            self._log_crash('_execute_undock_then_navigate', e)
            self._set_status('error')
            try:
                goal_handle.abort()
            except Exception:
                pass
            return NavigateToPose.Result()

    @staticmethod
    def _has_nav_goal(goal: NavigateToPose.Goal) -> bool:
        """Does this request actually ask to go somewhere?

        `undock` is deliberately NavigateToPose so one action can mean both
        "undock" and "undock, then drive there". But the UI's Undock button
        sends no pose at all, and an unset pose is (0,0) with a zero
        quaternion — which, now that the map origin is the docked pose, is the
        DOCK. The robot therefore drove out and then circled trying to reach
        the very dock it had just left.

        A real goal always carries a frame_id and a unit quaternion, so
        either being absent means "undock only".
        """
        if not goal.pose.header.frame_id:
            return False
        q = goal.pose.pose.orientation
        return (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w) > 0.5

    async def _execute_undock_then_navigate_inner(self, goal_handle):
        goal: NavigateToPose.Goal = goal_handle.request

        if self._docked:
            self.get_logger().info('undock: currently docked — undocking before navigating')
            if not self._undock_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error('drive_on_heading action server not available')
                goal_handle.abort()
                self._set_status('error')
                return NavigateToPose.Result()

            self._set_status('undocking')
            undock_goal = DriveOnHeading.Goal()
            undock_goal.target = Point(x=_UNDOCK_DISTANCE_M, y=0.0, z=0.0)
            undock_goal.speed = _UNDOCK_SPEED
            sec = int(_UNDOCK_TIMEOUT_SEC)
            undock_goal.time_allowance.sec = sec
            undock_goal.time_allowance.nanosec = int(
                (_UNDOCK_TIMEOUT_SEC - sec) * 1e9)
            child_undock_handle = await self._undock_client.send_goal_async(undock_goal)
            if not child_undock_handle.accepted:
                goal_handle.abort()
                self._set_status('error')
                return NavigateToPose.Result()

            wrapped = await self._await_with_cancel(
                goal_handle, child_undock_handle, child_undock_handle.get_result_async()
            )
            if wrapped is None:
                return NavigateToPose.Result()
            if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().error('drive_on_heading failed while undocking')
                goal_handle.abort()
                self._set_status('error')
                return NavigateToPose.Result()

            self._docked = False
            self._set_status('undocked')

        if not self._has_nav_goal(goal):
            self.get_logger().info(
                'undock: no navigation goal supplied — undock complete, staying put')
            goal_handle.succeed()
            return NavigateToPose.Result()

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('navigate_to_pose action server not available')
            goal_handle.abort()
            return NavigateToPose.Result()

        def on_feedback(fb_msg) -> None:
            goal_handle.publish_feedback(fb_msg.feedback)

        child_nav_handle = await self._nav_client.send_goal_async(goal, feedback_callback=on_feedback)
        if not child_nav_handle.accepted:
            goal_handle.abort()
            return NavigateToPose.Result()

        wrapped = await self._await_with_cancel(
            goal_handle, child_nav_handle, child_nav_handle.get_result_async()
        )
        if wrapped is None:
            return NavigateToPose.Result()

        if wrapped.status == GoalStatus.STATUS_SUCCEEDED:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return wrapped.result


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = DockManagerNode()
    executor = MultiThreadedExecutor(num_threads=8)
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
