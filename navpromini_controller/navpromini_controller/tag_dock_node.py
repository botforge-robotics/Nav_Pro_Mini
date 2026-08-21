#!/usr/bin/env python3
"""AprilTag-guided docking approach.

Exposes `tag_dock` (nav2_msgs/action/DockRobot) so dock_manager_node can relay
to it exactly as it did to the IR and lidar controllers this replaces.

Sensor: `dock_tag` from dock_tag_node — an AprilTag 36h11 on the dock face,
seen by the rear camera. Measured on this robot: 100% detection with the
horizontal offset repeating to 0.3px and bearing steady to 0.1deg. For
comparison, the IR beam zone it replaces flapped between left/right/overlap
several times a second, and the lidar dock fit was unusable beyond ~45cm.

The control law servos the tag's horizontal offset `dx` to zero and then
reverses straight in. Two properties make that sound:

  * It is CALIBRATION-INDEPENDENT. dx crosses zero exactly when the camera
    points at the tag, whatever the focal length is. camera_node publishes a
    *guessed* pinhole model, so anything scale-dependent (range, 6-DoF pose)
    would be wrong today — this is exact, and merely gets more precise if the
    camera is calibrated later.
  * It is CLOSED LOOP ON THE IMAGE, not on odometry. In-place rotation is
    where wheel odometry lies worst: the wheels skid, odometry reports the
    commanded angle, and the robot turns far less. That is what made the
    earlier IR corrections useless — nine consecutive corrections moved the
    robot essentially nowhere. Here each rotation's actual effect is
    re-measured, so skid costs an extra iteration instead of breaking the loop.

Charging is the only success condition. An approach that completes, or a
stall, is not evidence of contact — an earlier revision declared success on
movement alone and flipped the LED while sitting 30cm short of the dock.
"""

from __future__ import annotations

import math
import time
import traceback
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav2_msgs.action import DockRobot
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32MultiArray, String

CRASH_LOG_FILE = '/tmp/tag_dock_crash.log'
_TICK = 0.05


class TagDockNode(Node):
    def __init__(self) -> None:
        super().__init__('navpromini_tag_dock')
        p = self.declare_parameter

        p('tag_topic', 'dock_tag')
        p('cmd_vel_topic', 'cmd_vel_dock')
        p('odom_topic', 'odom')
        p('battery_topic', 'battery/state')

        # Generous: the Pi also runs Nav2, and a scheduling hiccup there must
        # not read as "the tag is gone". Detection itself runs at 5Hz.
        p('tag_timeout_sec', 2.5)
        # Continuous visual servo, not stop-turn-stop. Reversing and steering
        # at the same time removes the discrete rotation entirely — and the
        # discrete rotation is precisely what this base executes badly:
        # measured, a commanded 3.9deg moved the tag 227px where ~46px was
        # expected, because odometry feedback lags and the firmware slews on
        # stop. A continuous law never asks for a small precise turn; it just
        # reduces the error it can see, so lag costs a little tracking delay
        # instead of a 5x overshoot.
        #
        # It also retires the tolerance problem: a pixel tolerance tightens as
        # the robot closes (30px is 2.2cm at 50cm but 0.4cm at 9cm), so the
        # old law demanded its finest precision exactly where the actuator was
        # weakest. A proportional law has no threshold to tighten.
        p('k_alpha', 1.2)              # rad/s per rad of bearing error
        # Second control term: the dock FACE's own yaw, from dock_tag's `skew`.
        #
        # k_alpha alone only points the camera AT the tag. That is pure
        # pursuit: it converges in position but arrives at an angle whenever
        # the approach started off-axis, which is exactly the observed
        # "reaches the funnel turned the wrong way and inclined to the dock".
        # Squaring up needs the face orientation as well — the standard
        # differential-drive parking law is w = k_alpha*alpha + k_beta*beta.
        #
        # beta is derived from skew, the signed difference between the tag's
        # two vertical edge lengths: a square-on view gives 0, and the nearer
        # edge projects longer as the view angle grows. Gain is deliberately
        # well below k_alpha — beta corrects the final heading, and letting it
        # dominate makes the robot swing wide instead of closing in.
        p('k_beta', 0.9)
        # skew is a ratio, not an angle. This converts it to something the
        # same order as alpha so the two gains are comparable.
        p('skew_to_rad', 1.5)
        # Simulated both signs through the closed-loop kinematics: -1
        # converges to centred and within +/-2.6deg from every start tested,
        # while +1 DIVERGES to 30-35deg. Still auto-checked on hardware rather
        # than trusted, because the sim assumes a skew->angle mapping that
        # depends on how the tag is mounted, and getting it wrong steers the
        # robot into the dock at an angle.
        p('beta_sign', -1)
        p('beta_autocalibrate', True)
        p('beta_deadband', 0.02)       # ignore noise around square-on
        p('max_omega', 0.35)
        p('omega_slew', 1.2)           # rad/s^2, keeps the arc smooth
        # Ease off the throttle while badly off-axis so the turn leads the
        # approach rather than the robot committing to a crooked line.
        p('alpha_slow_rad', 0.35)
        p('min_speed_frac', 0.35)
        # Which way to turn for a given dx depends on how the rear camera is
        # mounted, and guessing wrong drives the robot away from the dock.
        # The first correction measures it instead: if |dx| grows, the sign
        # flips and is remembered. That removes a class of error that cost
        # several runs when it had to be assumed.
        p('rotate_sign', -1)

        # Slower on request. Not lower than this: the firmware's creep/
        # breakaway floor means very low commands stop producing motion at all,
        # and "commanded but not moving" reads identically to "pressed against
        # the dock" to the stall test.
        p('approach_speed', 0.02)
        p('max_travel_m', 1.0)
        # The last stretch is necessarily unmeasured: the 80mm tag overfills
        # the 720p frame as the robot closes, so it stops being detectable
        # while still short of contact. Observed on this robot: when the tag
        # is lost the robot is already centred with only ~3-4cm to go, so the
        # blind allowance is sized for that plus margin — NOT for a long push.
        # Oversizing it is actively harmful: past contact the wheels keep
        # slipping and the robot shoves the DOCK out of position, which ruins
        # alignment for every later attempt too.
        p('blind_final_m', 0.08)
        # Slower than the main approach — this is the segment that actually
        # mates the pogo pins, and it is running unmeasured.
        p('blind_speed', 0.012)
        # Losing the tag only means "close" if it was BIG just before it went.
        # Losing a small tag means it went out of frame sideways, or the
        # camera was occluded — pushing blind on that would drive the robot
        # into the dock off-centre.
        p('blind_min_side_px', 350.0)
        # ...and it must also have been CENTRED when it went. Size alone is
        # not enough: measured, an attempt lost the tag at 456px while 10deg
        # off-axis — it left the frame sideways, not by overfilling it — and
        # the blind creep then drove 8cm at an angle and missed the contacts.
        # A tag that vanishes because it filled the frame is centred by
        # definition; one that slides out of view is not, and creeping blind
        # while crooked is also how the dock gets pushed out of position.
        p('blind_max_dx_px', 60.0)

        p('stall_speed_mps', 0.008)
        p('stall_confirm_sec', 1.0)
        p('stall_grace_sec', 2.5)
        p('stall_min_travel_m', 0.03)

        p('backoff_m', 0.18)
        p('charge_confirm_sec', 8.0)
        p('max_retries', 3)

        # Rotate in place to look for the tag when it is not already in view.
        # Deliberately TIME-based rather than odometry-based: a search only
        # needs to sweep the camera past the dock, and this robot's odometry
        # under-reports in-place rotation badly enough that an odometry sweep
        # would be the wrong size. Coverage is what matters here, not angle.
        p('search_omega', 0.25)
        p('search_leg_sec', 4.0)       # ~57deg per leg at 0.25 rad/s
        p('search_max_sec', 40.0)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self._tag_timeout = float(g('tag_timeout_sec'))
        self._k_alpha = float(g('k_alpha'))
        self._k_beta = float(g('k_beta'))
        self._skew_to_rad = float(g('skew_to_rad'))
        self._beta_sign = 1.0 if int(g('beta_sign')) >= 0 else -1.0
        self._beta_autocal = bool(g('beta_autocalibrate'))
        self._beta_checked = False
        self._beta_deadband = float(g('beta_deadband'))
        self._max_omega = float(g('max_omega'))
        self._omega_slew = float(g('omega_slew'))
        self._alpha_slow = float(g('alpha_slow_rad'))
        self._min_speed_frac = float(g('min_speed_frac'))
        self._sign = 1.0 if int(g('rotate_sign')) >= 0 else -1.0
        self._sign_checked = False
        self._speed = float(g('approach_speed'))
        self._max_travel = float(g('max_travel_m'))
        self._blind_final = float(g('blind_final_m'))
        self._blind_speed = float(g('blind_speed'))
        self._blind_min_side = float(g('blind_min_side_px'))
        self._blind_max_dx = float(g('blind_max_dx_px'))
        self._last_dx_px = 0.0
        self._last_side_px = 0.0
        self._stall_speed = float(g('stall_speed_mps'))
        self._stall_confirm = float(g('stall_confirm_sec'))
        self._stall_grace = float(g('stall_grace_sec'))
        self._stall_min_travel = float(g('stall_min_travel_m'))
        self._backoff = float(g('backoff_m'))
        self._charge_confirm = float(g('charge_confirm_sec'))
        self._max_retries = int(g('max_retries'))
        self._search_omega = float(g('search_omega'))
        self._search_leg = float(g('search_leg_sec'))
        self._search_max = float(g('search_max_sec'))

        cb = ReentrantCallbackGroup()
        self._tag: Optional[list] = None
        self._tag_stamp = 0.0
        self._odom: Optional[Odometry] = None
        self._power: Optional[int] = None
        self._busy = False
        self._state = 'idle'

        self.create_subscription(Float32MultiArray, str(g('tag_topic')),
                                 self._on_tag, 10, callback_group=cb)
        self.create_subscription(Odometry, str(g('odom_topic')),
                                 self._on_odom, 10, callback_group=cb)
        self.create_subscription(BatteryState, str(g('battery_topic')),
                                 self._on_batt, 10, callback_group=cb)
        self._cmd = self.create_publisher(Twist, str(g('cmd_vel_topic')), 10)
        self._state_pub = self.create_publisher(String, 'tag_dock/state', 10)

        self._server = ActionServer(
            self, DockRobot, 'tag_dock',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=lambda _g: CancelResponse.ACCEPT,
            callback_group=cb,
        )
        self.create_timer(0.5, lambda: self._state_pub.publish(String(data=self._state)),
                          callback_group=cb)
        # One persistent tick timer for the control loop. _sleep() creates and
        # destroys a timer per call, which at 20Hz made this the single
        # largest CPU consumer on the robot (21.8%) purely in rcl allocation
        # churn — and that load is what starved the odometry feedback the
        # controller depends on. One timer, many awaiters, no churn.
        self._tick_waiters: list = []
        self.create_timer(_TICK, self._on_tick, callback_group=cb)
        self.get_logger().info('tag_dock ready — action `tag_dock`')

    # -- inputs --------------------------------------------------------------

    def _on_tag(self, msg: Float32MultiArray) -> None:
        if len(msg.data) >= 7 and msg.data[0] > 0.5:
            self._tag = list(msg.data)
            self._tag_stamp = time.monotonic()
            self._last_side_px = float(msg.data[4])
            self._last_dx_px = float(msg.data[2])

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg

    def _on_batt(self, msg: BatteryState) -> None:
        self._power = msg.power_supply_status

    def _tag_now(self) -> Optional[list]:
        if self._tag is None or time.monotonic() - self._tag_stamp > self._tag_timeout:
            return None
        return self._tag

    def _charging(self) -> bool:
        return self._power in (BatteryState.POWER_SUPPLY_STATUS_CHARGING,
                               BatteryState.POWER_SUPPLY_STATUS_FULL)

    def _set_state(self, s: str) -> None:
        self._state = s
        self.get_logger().info(f'tag_dock state -> {s}')
        self._state_pub.publish(String(data=s))

    # -- motion --------------------------------------------------------------

    def _drive(self, lin: float, ang: float) -> None:
        t = Twist()
        t.linear.x = float(lin)
        t.angular.z = float(ang)
        self._cmd.publish(t)

    def _stop(self) -> None:
        for _ in range(3):
            self._cmd.publish(Twist())

    def _on_tick(self) -> None:
        waiters, self._tick_waiters = self._tick_waiters, []
        for f in waiters:
            if not f.done():
                f.set_result(None)

    async def _tick(self) -> None:
        """Wait one control tick on the shared timer — no per-call allocation."""
        fut = Future()
        self._tick_waiters.append(fut)
        await fut

    async def _sleep(self, sec: float) -> None:
        """rclpy-compatible sleep. asyncio.sleep() raises "no running event
        loop" inside these coroutines — the MultiThreadedExecutor drives them
        with its own mechanism, not a real asyncio loop."""
        fut = Future()
        timer = self.create_timer(sec, lambda: fut.set_result(None))
        try:
            await fut
        finally:
            timer.cancel()
            timer.destroy()

    def _xy(self) -> Optional[tuple[float, float]]:
        if self._odom is None:
            return None
        p = self._odom.pose.pose.position
        return (p.x, p.y)

    def _travelled(self, origin) -> float:
        now = self._xy()
        return 0.0 if now is None else math.dist(now, origin)

    # -- tag centring --------------------------------------------------------

    # -- action --------------------------------------------------------------

    def _on_goal(self, _r) -> GoalResponse:
        return GoalResponse.REJECT if self._busy else GoalResponse.ACCEPT

    def _fb(self, gh, state: int, t0: float, retries: int) -> None:
        fb = DockRobot.Feedback()
        fb.state = state
        fb.docking_time.sec = int(time.monotonic() - t0)
        fb.num_retries = retries
        gh.publish_feedback(fb)

    async def _execute(self, gh):
        self._busy = True
        try:
            return await self._inner(gh)
        except Exception as e:  # noqa: BLE001 — never let a crash look like a hang
            tb = traceback.format_exc()
            self.get_logger().error(f'_execute raised {e!r}\n{tb}')
            try:
                with open(CRASH_LOG_FILE, 'a') as f:
                    f.write(tb + '\n')
            except OSError:
                pass
            self._stop()
            self._set_state('error')
            try:
                gh.abort()
            except Exception:
                pass
            return DockRobot.Result(success=False, error_msg=f'{type(e).__name__}: {e}')
        finally:
            self._stop()
            self._busy = False

    async def _inner(self, gh):
        t0 = time.monotonic()
        if self._odom is None:
            gh.abort()
            self._set_state('error')
            return DockRobot.Result(success=False, error_msg='no odometry yet')

        for attempt in range(self._max_retries + 1):
            if gh.is_cancel_requested:
                self._stop()
                gh.canceled()
                self._set_state('idle')
                return DockRobot.Result(success=False, error_msg='cancelled')
            if self._charging():
                self._stop()
                self._set_state('charging')
                gh.succeed()
                return DockRobot.Result(success=True, num_retries=attempt)

            self._fb(gh, DockRobot.Feedback.INITIAL_PERCEPTION, t0, attempt)
            if not await self._search_for_tag(gh):
                self._stop()
                gh.abort()
                self._set_state('error')
                return DockRobot.Result(
                    success=False,
                    error_msg='dock tag not found after sweeping — is the camera '
                              'running and the dock within view?')
            if await self._approach(gh, t0, attempt):
                self._fb(gh, DockRobot.Feedback.WAIT_FOR_CHARGE, t0, attempt)
                self._set_state('waiting_for_charge')
                if await self._confirm_charging():
                    self._set_state('charging')
                    gh.succeed()
                    return DockRobot.Result(success=True, num_retries=attempt)
                self.get_logger().warn(
                    f'Contact but no charging (attempt {attempt + 1}) — not seated')

            self._fb(gh, DockRobot.Feedback.RETRY, t0, attempt)
            if attempt >= self._max_retries:
                break
            self._set_state('backing_off')
            await self._drive_back_off()

        self._stop()
        gh.abort()
        self._set_state('error')
        return DockRobot.Result(success=False, num_retries=self._max_retries,
                                error_msg='never confirmed charging')

    async def _search_for_tag(self, gh) -> bool:
        """Sweep in place until the tag comes into view.

        Legs alternate and grow — right, then left past centre, then right
        again — so the sweep widens around the starting heading instead of
        walking away from it in one direction.
        """
        if self._tag_now() is not None:
            return True
        self._set_state('search')
        self.get_logger().info('SEARCH: tag not in view — sweeping to find it')
        deadline = time.monotonic() + self._search_max

        for leg, direction in enumerate((1.0, -1.0, 1.0, -1.0)):
            span = self._search_leg * (1.0 if leg == 0 else 2.0)
            end = time.monotonic() + span
            while time.monotonic() < end:
                if gh.is_cancel_requested or time.monotonic() > deadline:
                    self._stop()
                    return self._tag_now() is not None
                if self._tag_now() is not None:
                    self._stop()
                    self.get_logger().info('SEARCH: tag acquired')
                    # Let the base settle so the first bearing is not measured
                    # mid-rotation.
                    await self._sleep(0.6)
                    return True
                self._drive(0.0, direction * self._search_omega)
                await self._tick()
            self._stop()
            await self._sleep(0.4)

        self._stop()
        found = self._tag_now() is not None
        if not found:
            self.get_logger().warn('SEARCH: swept without finding the tag')
        return found

    async def _drive_back_off(self) -> None:
        origin = self._xy()
        if origin is None:
            return
        deadline = time.monotonic() + 20.0
        while self._travelled(origin) < self._backoff:
            if time.monotonic() > deadline:
                break
            self._drive(self._speed, 0.0)
            await self._tick()
        self._stop()

    async def _approach(self, gh, t0: float, attempt: int) -> bool:
        """Reverse and steer simultaneously, servoing the tag to image centre.

        omega = k_alpha * bearing, slew-limited; linear speed eases off while
        the bearing error is large so the turn leads the approach. See
        k_alpha for why this replaced discrete centre-then-advance.

        Sign is still verified rather than assumed: over the first couple of
        seconds, if |dx| grows the steering direction flips. Rear-camera
        handedness is easy to get backwards and it has cost runs before.
        """
        self._set_state('approach')
        origin = self._xy()
        if origin is None:
            return False

        started = time.monotonic()
        omega = 0.0
        stalled_since = None
        blind_from = None
        sign_t0 = None
        sign_dx0 = None
        beta_t0 = None
        beta_skew0 = None
        last_log = 0.0

        while True:
            if gh.is_cancel_requested:
                self._stop()
                return False
            if self._charging():
                self._stop()
                self.get_logger().info('CONTACT: charging — seated')
                return True

            now = time.monotonic()
            travelled = self._travelled(origin)
            if travelled >= self._max_travel:
                self._stop()
                self.get_logger().warn(
                    f'APPROACH: {travelled:.2f}m without contact — travel backstop')
                return False

            tag = self._tag_now()
            if tag is not None:
                blind_from = None
                dx, alpha = float(tag[2]), float(tag[5])
                skew = float(tag[6])

                # One-shot sign check over the first ~2.5s of motion.
                if not self._sign_checked:
                    if sign_t0 is None:
                        sign_t0, sign_dx0 = now, dx
                    elif now - sign_t0 > 2.5:
                        self._sign_checked = True
                        if abs(dx) > abs(sign_dx0) + 15.0:
                            self._sign = -self._sign
                            self.get_logger().warn(
                                f'TAG: |dx| grew {abs(sign_dx0):.0f}->{abs(dx):.0f}px — '
                                f'flipping steering sign to {self._sign:+.0f}')
                        else:
                            self.get_logger().info(
                                f'TAG: sign confirmed ({abs(sign_dx0):.0f}->'
                                f'{abs(dx):.0f}px)')

                # beta: how far the dock face is from square-on. Deadbanded so
                # sensor noise near square does not add a permanent bias.
                beta = 0.0
                if abs(skew) > self._beta_deadband:
                    beta = (skew * self._skew_to_rad * self._beta_sign
                            * self._sign)
                # Same one-shot check as for alpha: if |skew| is growing, the
                # heading term is fighting the geometry rather than correcting
                # it, so flip and carry on.
                if self._beta_autocal and not self._beta_checked:
                    if beta_t0 is None:
                        if abs(skew) > self._beta_deadband:
                            beta_t0, beta_skew0 = now, skew
                    elif now - beta_t0 > 3.0:
                        self._beta_checked = True
                        if abs(skew) > abs(beta_skew0) + 0.03:
                            self._beta_sign = -self._beta_sign
                            self.get_logger().warn(
                                f'TAG: |skew| grew {abs(beta_skew0):.3f}->'
                                f'{abs(skew):.3f} — flipping beta sign to '
                                f'{self._beta_sign:+.0f}')
                        else:
                            self.get_logger().info(
                                f'TAG: beta sign confirmed '
                                f'({abs(beta_skew0):.3f}->{abs(skew):.3f})')

                target = self._sign * self._k_alpha * alpha + self._k_beta * beta
                target = max(-self._max_omega, min(self._max_omega, target))
                step = self._omega_slew * _TICK
                omega += max(-step, min(step, target - omega))

                frac = max(self._min_speed_frac,
                           1.0 - abs(alpha) / max(self._alpha_slow, 1e-3))
                v = self._speed * min(1.0, frac)
                self._drive(-v, omega)

                if now - last_log > 2.0:
                    last_log = now
                    self.get_logger().info(
                        f'SERVO: dx={dx:+6.1f}px ({math.degrees(alpha):+5.1f}deg) '
                        f'side={tag[4]:.0f}px skew={skew:+.3f} '
                        f'(a={self._sign * self._k_alpha * alpha:+.3f} '
                        f'b={self._k_beta * beta:+.3f}) -> '
                        f'v={-v:+.3f} w={omega:+.3f}')
            else:
                here = self._xy()
                if blind_from is None:
                    if (self._last_side_px < self._blind_min_side
                            or abs(self._last_dx_px) > self._blind_max_dx):
                        self._stop()
                        self.get_logger().warn(
                            f'APPROACH: tag lost at {self._last_side_px:.0f}px, '
                            f'dx={self._last_dx_px:+.0f}px — '
                            + ('too small' if self._last_side_px < self._blind_min_side
                               else 'off-centre, so it slid out of frame rather '
                                    'than filled it')
                            + '. Backing off instead of creeping in crooked')
                        return False
                    blind_from = here
                    self.get_logger().info(
                        f'APPROACH: tag filled the frame at '
                        f'{self._last_side_px:.0f}px and dropped out — creeping '
                        f'the last {self._blind_final * 100:.0f}cm at '
                        f'{self._blind_speed * 100:.1f}cm/s to find the contacts')
                elif math.dist(here, blind_from) > self._blind_final:
                    self._stop()
                    self.get_logger().warn(
                        'APPROACH: blind allowance used up without charging')
                    return False
                # Straight and slow: no steering without a measurement.
                omega = 0.0
                self._drive(-self._blind_speed, 0.0)

            spd = abs(self._odom.twist.twist.linear.x) if self._odom else 1.0
            if (now - started > self._stall_grace
                    and travelled > self._stall_min_travel
                    and spd < self._stall_speed):
                stalled_since = stalled_since or now
                if now - stalled_since > self._stall_confirm:
                    self._stop()
                    self.get_logger().info(
                        f'CONTACT: stalled at {travelled * 100:.0f}cm — '
                        'charging check decides')
                    return True
            else:
                stalled_since = None

            await self._tick()

    async def _confirm_charging(self) -> bool:
        """The single success gate. Nothing else may report a dock."""
        deadline = time.monotonic() + self._charge_confirm
        while time.monotonic() < deadline:
            if self._charging():
                return True
            await self._sleep(0.25)
        return False


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = TagDockNode()
    ex = MultiThreadedExecutor(num_threads=6)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
