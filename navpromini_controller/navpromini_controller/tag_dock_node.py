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
        # differential-drive parking law is w = k_alpha*alpha - k_beta*beta
        # (k_beta a positive magnitude, subtracted — see _approach's own
        # comment on the sign), where beta is NOT the raw heading signal on
        # its own (an earlier version of this file used it that way — see
        # _approach's own comment on why that's wrong) but
        # beta = alpha + theta: coupled to alpha, per the closed-loop
        # unicycle-parking result this is actually named after (Aicardi,
        # Casalino, Bicchi, Balestrino 1995). Squaring up and centring are
        # not independent goals for a
        # nonholonomic vehicle — correcting one changes the other — and
        # decoupling them is what let a large-but-real skew fight alpha's
        # own convergence instead of complementing it.
        #
        # theta (the raw heading signal beta is built from) comes from
        # skew, the signed difference between the tag's two vertical edge
        # lengths: a square-on view gives 0, and the nearer edge projects
        # longer as the view angle grows. Gain is deliberately well below
        # k_alpha — heading is the secondary correction here, and letting
        # it dominate makes the robot swing wide instead of closing in.
        #
        # "Well below" matters quantitatively now that beta is properly
        # coupled (beta = alpha + theta, see _approach): the *net* reaction
        # to alpha works out to (k_alpha - k_beta)*alpha - k_beta*theta, so
        # if k_beta sits anywhere close to k_alpha, that net alpha term
        # collapses toward zero. Confirmed live at the previous value
        # (0.9, only 25% below k_alpha's 1.2): alpha and theta grow
        # together as the robot drifts off-centre (the same drift skews
        # the tag's face too), so their contributions nearly cancelled —
        # logged sitting at a=-0.402 b=+0.388 -> w=-0.013 for a dozen
        # consecutive samples with dx frozen at 234px, essentially no net
        # correction while badly off-centre. Dropping k_beta here keeps
        # (k_alpha - k_beta) comfortably dominant so that cancellation
        # can't happen regardless of how alpha and theta happen to align.
        #
        # 0.3 swung too far the other way: confirmed live, dx stayed well
        # controlled (good — (k_alpha-k_beta)=0.9 is plenty dominant) but
        # skew climbed steadily anyway — 0.094 to 0.181 over ~16s while w
        # never exceeded 0.06 — because whenever alpha is already small,
        # (k_alpha-k_beta)*alpha is tiny too and k_beta*theta is nearly the
        # *only* thing reacting to heading drift; 0.3*theta wasn't enough
        # authority to keep up with it, ending "jammed against the guide
        # funnel, not seated". Splitting the difference: still leaves
        # (k_alpha - k_beta)=0.65 comfortably dominant, well short of the
        # 0.9 value that let alpha and beta cancel, while roughly doubling
        # theta's own correction authority against real drift.
        #
        # 0.55 -> 0.62: a live run still showed theta climbing to +0.069
        # rad on one approach (contact 3x without charging before the
        # retry limit forced a full restart from staging) — same shape as
        # the 0.3 failure above, just less severe. A second attempt in the
        # same session stayed under blind_max_skew and mated fine, so this
        # is marginal rather than broken. Small nudge for more authority;
        # (k_alpha - k_beta) drops to 0.58, still comfortably clear of the
        # 0.9 cancellation zone. Re-tune in small steps from here — do not
        # jump straight back toward 0.9.
        #
        # 0.62 -> 0.68: still not enough on the very next run — skew hit
        # the blind_max_skew (0.05) gate three separate times (0.060,
        # 0.065, 0.053), every time backing off before ever reaching
        # CONTACT, and the whole action errored out after 4 approach
        # sub-attempts. Same direction of fix, another small step;
        # (k_alpha - k_beta) drops to 0.52 — still positive and dominant,
        # but getting closer to the range worth watching for the
        # alpha/beta cancellation this file warns about. If 0.68 is still
        # not enough, look at why the initial bearing entering approach is
        # so large first (this run started at +448px/33.7deg and
        # -282px/22.8deg — much worse than earlier runs) rather than
        # keep raising k_beta past this.
        p('k_beta', 0.68)
        # Live data at 0.68 showed WHY raising k_beta further is the wrong
        # lever: during the close-in growth phase the logged a/b components
        # (k_alpha*alpha and -k_beta*beta) were nearly opposite —
        # a=-0.173 b=+0.143 -> w=-0.030 — the same cancellation this file's
        # own history already warned about at k_beta=0.9, just not fully
        # there yet. Raising k_beta more only pushes closer to that.
        #
        # By request: correct heading EARLY, while there's still standoff
        # distance/arc room to do it with, rather than only fighting it out
        # close-in where there's little room left and cancellation eats
        # the correction. This boosts k_beta's effective value while far
        # (side_px small, near tag acquisition) and linearly tapers it back
        # to the plain k_beta above by close_slow_side_px — so the
        # already-tested close-in behavior is unchanged; this only adds
        # authority earlier, where there's actual room to use it. 0.5 = up
        # to 50% more at side_px~0, clamped in _approach so the effective
        # k_beta can never approach k_alpha (the actual cancellation
        # danger zone).
        p('k_beta_far_boost', 0.5)
        # skew is a ratio, not an angle. This converts it to something the
        # same order as alpha so the two gains are comparable.
        p('skew_to_rad', 1.5)
        # That old simulation (-1 converges, +1 diverges) predates beta's
        # coupling to alpha (beta = alpha + theta, see _approach's own
        # comment) and doesn't apply to this formula. Flipped to +1 on a
        # direct hardware report, watching the robot: with -1, a skew of
        # e.g. -15deg was being driven toward +15deg before coming back
        # rather than toward 0 — corrected in the wrong direction first.
        # Plausibly related to this robot's specific kinematics (driven
        # wheels at the front, a passive trailing caster at the back where
        # the dock/camera are — not a centred differential pair), which the
        # unicycle-style derivation beta = alpha + theta doesn't model; a
        # geometry-driven sign flip specific to the coupled formula is
        # consistent with -1 having worked fine for the old, uncoupled one.
        #
        # Autocalibrate is now OFF, deliberately: its "does |skew| grow or
        # shrink" check can't distinguish "beta's own sign is wrong" from
        # "alpha's own correction is dominating and masking it" now that
        # the two are coupled — exactly the ambiguity that let a wrong
        # value read as "confirmed" before. A direct report of which way it
        # actually drove is better evidence than that heuristic; trust it
        # over letting the runtime check silently re-flip this back.
        p('beta_sign', 1)
        p('beta_autocalibrate', False)
        p('beta_deadband', 0.02)       # ignore noise around square-on
        # 0.35 -> 0.25 by request. A live run showed SEARCH acquiring the
        # tag at ~37deg off-axis (twice — SEARCH stops sweeping as soon as
        # it first sees the tag, not once it's centred), and the servo
        # commanding w up to 0.338 rad/s — right at the old 0.35 cap — to
        # correct it, a fast, hard rotation right at the start of approach.
        # Softer cap trades correction speed for a gentler turn; it will
        # take longer to null a big initial bearing, not fail to null it
        # (alpha's own gain still drives it, just slower at the extreme).
        p('max_omega', 0.25)
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
        # mates the pogo pins, and it is running unmeasured. Was 0.012 —
        # live-observed the funnel's own physical friction stopping the
        # robot short of seating at that speed, and 0.012 left only 0.004m/s
        # of margin above stall_speed_mps (0.008) before a genuine "still
        # creeping but slowly" got misread as stalled. Raised to widen that
        # margin and give it enough push through the funnel; still far
        # slower than approach_speed (0.02).
        p('blind_speed', 0.018)
        # By request: the blind-creep heading nudge (added after 3/4
        # attempts reached CONTACT but none seated) should be a very minor
        # correction, not an attempt to fully null the residual error —
        # there is no live measurement to center against here, only a
        # dead-reckoned guess from the last real reading, and a confident
        # guess is worse than none. Small dedicated gain (well under
        # k_beta) and a tight cap, both independent of the live approach
        # gains so tuning k_beta later doesn't silently change this too.
        p('blind_nudge_gain', 0.20)
        p('blind_nudge_max_frac', 0.10)  # of max_omega
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
        # ...and it must have been roughly SQUARE to the dock face, not just
        # centred and close. dx and skew measure different things: dx is
        # lateral offset, skew is heading. A robot can be dead-centred
        # (dx~0) while still facing the dock several degrees off — measured
        # live: skew grew to 0.085-0.117 (~7-10deg via skew_to_rad) across
        # four separate close-in stalls that all reached blind_min_side_px
        # yet never charged. Confirmed by watching it happen: the robot
        # entered the dock's guide funnel on an incline and jammed against
        # it, never reaching the pins. Neither dx nor side_px catches that.
        # 0.05 -> 0.07: small increase by request. Recent live runs (with
        # k_beta already raised to 0.68 and staging standoff back to 0.75m)
        # were still backing off on skew that only just missed 0.05 —
        # 0.050, 0.053, 0.060, 0.065 — costing a full retry each time even
        # though those are close, not the badly-crooked 0.085-0.117 case
        # this gate was originally sized to catch. 0.07 lets the near-miss
        # cases creep in blind while still rejecting anything in that
        # original bad range. If jamming against the guide funnel shows up
        # again at values in the new 0.05-0.07 band, that's this gate
        # having been opened too far — tighten back toward 0.05 rather
        # than reaching for k_beta again.
        p('blind_max_skew', 0.07)
        # Eases approach speed down further as the tag fills the frame, on
        # top of alpha's own bearing-based slowdown below — that one only
        # reacts to angular error, not proximity, so a well-centred
        # approach kept closing at full speed even in the final stretch,
        # giving the servo loop proportionally less time to correct a
        # residual drift before pixel-sensitivity to a given real-world
        # offset ramps up (the closer the camera, the more px a fixed cm of
        # lateral error produces). Confirmed live: dx converging to
        # ~30-40px mid-approach, then growing back past blind_max_dx_px
        # specifically as side_px closed in past ~400-500px — losing an
        # otherwise-good approach right at the finish line, not from a bad
        # sign or a genuinely crooked line. Ramps from full speed at this
        # value down to min_speed_frac at blind_min_side_px, so by the time
        # it's close enough to actually go blind it's already moving at the
        # same floor alpha's own slowdown uses — more correction time
        # exactly where the log showed it was being lost, without loosening
        # blind_max_dx_px itself (that one's tightened for a real,
        # measured reason — see its own comment).
        p('close_slow_side_px', 250.0)

        # Only trusted once the tag was last seen at blind_min_side_px or
        # bigger, i.e. plausibly close enough to touch. Without that gate, a
        # brief friction hiccup at approach_speed (0.02 m/s, barely above
        # stall_speed_mps) reads as "stalled against the dock" from anywhere
        # along the approach — measured live: declared contact at ~55cm out,
        # centred and moving fine, then burned charge_confirm_sec doing
        # nothing and wasted a full retry backing off from empty space.
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
        # 0.25 -> 0.15 by request: _search_for_tag stops the instant the tag
        # is first decodable (checked every tick) and then sleeps 0.6s "to
        # let the base settle" before the first bearing is read — but a
        # faster sweep has more angular momentum to bleed off in that
        # window, so the base keeps drifting past the point it was actually
        # acquired at. Live logs showed SEARCH landing on a first bearing of
        # ~37deg off-axis twice — plausibly this settle-drift, not the tag
        # only becoming decodable that far off-axis. Slower sweep, less to
        # bleed off, tighter first-bearing reading.
        #
        # search_leg_sec scaled up to match (4.0 -> 6.7): keeps the same
        # ~57deg-per-leg angular coverage this was originally sized for
        # (span = search_omega * search_leg_sec) — slowing the sweep alone
        # without this would shrink coverage per leg and could cost search
        # reliability within search_max_sec, which is not what was asked.
        p('search_omega', 0.15)
        p('search_leg_sec', 6.7)       # ~57deg per leg at 0.15 rad/s
        p('search_max_sec', 40.0)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self._tag_timeout = float(g('tag_timeout_sec'))
        self._k_alpha = float(g('k_alpha'))
        self._k_beta = float(g('k_beta'))
        self._k_beta_far_boost = float(g('k_beta_far_boost'))
        self._skew_to_rad = float(g('skew_to_rad'))
        self._beta_sign = 1.0 if int(g('beta_sign')) >= 0 else -1.0
        self._beta_autocal = bool(g('beta_autocalibrate'))
        self._beta_checked = False
        self._beta_deadband = float(g('beta_deadband'))
        # Minimum |skew| before the one-shot beta-sign check arms its 3s
        # confirmation window — see the check itself, in _approach. Well
        # above beta_deadband (0.02): that deadband only screens out pure
        # noise, not a borderline-but-real skew too small to say with any
        # confidence which way squaring up should turn.
        self._beta_check_min_skew0 = 0.08
        self._max_omega = float(g('max_omega'))
        self._omega_slew = float(g('omega_slew'))
        self._alpha_slow = float(g('alpha_slow_rad'))
        self._min_speed_frac = float(g('min_speed_frac'))
        self._sign = 1.0 if int(g('rotate_sign')) >= 0 else -1.0
        self._sign_checked = False
        # Minimum |dx| (px) before the one-shot sign check even starts
        # timing its 2.5s window — see the check itself, in _approach, for
        # why a near-zero starting bearing must not be trusted to decide
        # the sign. Comfortably above sensor/frame noise, well below a
        # deliberately off-centre approach.
        self._sign_check_min_dx0 = 40.0
        self._speed = float(g('approach_speed'))
        self._max_travel = float(g('max_travel_m'))
        self._blind_final = float(g('blind_final_m'))
        self._blind_speed = float(g('blind_speed'))
        self._blind_nudge_gain = float(g('blind_nudge_gain'))
        self._blind_nudge_max_frac = float(g('blind_nudge_max_frac'))
        self._blind_min_side = float(g('blind_min_side_px'))
        self._blind_max_dx = float(g('blind_max_dx_px'))
        self._blind_max_skew = float(g('blind_max_skew'))
        self._close_slow_side_px = float(g('close_slow_side_px'))
        self._last_dx_px = 0.0
        self._last_side_px = 0.0
        self._last_skew = 0.0
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
            self._last_skew = float(msg.data[6])

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
        """Back off _backoff metres — steering square to the dock face along
        the way if the tag is visible, not just driving straight back.

        A large starting skew is exactly the case this used to fail on: the
        approach begins from a genuinely crooked angle (bad staging, AMCL
        drift), stays crooked as it closes in, and gets caught by the
        skew/stall guards in _approach() near contact rather than seating
        wrong — good, that part was already working. What wasn't: backing
        off in a straight line preserves that same crooked heading exactly,
        so the next SEARCH+APPROACH attempt starts from the same bad angle
        and fails the same way again. Applying the same beta (skew)
        correction term _approach() uses — steering, not centring, since
        dx-centring is what the next approach's own servo loop will handle
        — straightens the heading while backing off, so each retry starts
        closer to square than the last instead of repeating the failure.
        """
        origin = self._xy()
        if origin is None:
            return
        # Extend the backoff distance when starting from a large skew.
        # _backoff is a fixed distance, so it hands every entry the same
        # limited room to straighten out regardless of how much correction
        # is actually needed — but a badly-skewed entry is exactly the case
        # this steering exists for (see the class doc above). Live-observed:
        # successive retries starting progressively worse (13.5deg -> 22deg
        # -> 37deg) rather than converging, consistent with the fixed
        # distance running out before heading actually improved much, not
        # with the steering being wrong. Up to 2x distance for a decisively
        # bad starting skew; unchanged when skew is small or the tag isn't
        # visible yet to judge by (same _beta_check_min_skew0 threshold the
        # sign-flip check below already uses to mean "a real signal, not
        # noise").
        backoff_m = self._backoff
        start_tag = self._tag_now()
        if start_tag is not None:
            start_skew = abs(float(start_tag[6]))
            if start_skew > self._beta_check_min_skew0:
                extra_frac = min(1.0, (start_skew - self._beta_check_min_skew0) / 0.15)
                backoff_m = self._backoff * (1.0 + extra_frac)
                self.get_logger().info(
                    f'BACKOFF: starting skew {start_skew:+.3f} — extending '
                    f'backoff {self._backoff:.2f}m -> {backoff_m:.2f}m for more '
                    'room to straighten out')
        deadline = time.monotonic() + max(20.0, backoff_m / max(self._speed, 1e-3) + 5.0)
        omega = 0.0
        # _approach() drives in REVERSE (negative speed, camera end leading)
        # while this drives FORWARD (positive speed, camera end trailing) —
        # the opposite longitudinal direction. _beta_sign was one-shot
        # verified by _approach() for its own direction only (see the sign
        # check above); nothing here re-verified it holds in reverse. A real
        # run showed exactly that: skew climbing instead of shrinking while
        # backing off, even though beta_sign had verified fine during
        # approach moments earlier. So backoff gets its own one-shot check,
        # local to this call — it must NOT overwrite self._beta_sign, which
        # is correct for _approach() and would otherwise get corrupted for
        # the next attempt.
        backoff_beta_sign = self._beta_sign
        beta_checked = False
        beta_t0 = None
        beta_skew0 = None
        while self._travelled(origin) < backoff_m:
            if time.monotonic() > deadline:
                break
            tag = self._tag_now()
            target = 0.0
            if tag is not None:
                skew = float(tag[6])
                if abs(skew) > self._beta_deadband:
                    if not beta_checked:
                        now = time.monotonic()
                        # Same fragility as _approach()'s own beta check,
                        # same fix: arm the confirmation window only off a
                        # clearly-decisive starting skew, not anything past
                        # the plain noise deadband — see
                        # _beta_check_min_skew0's own doc.
                        if beta_t0 is None:
                            if abs(skew) > self._beta_check_min_skew0:
                                beta_t0, beta_skew0 = now, skew
                        elif now - beta_t0 > 2.0:
                            beta_checked = True
                            if abs(skew) > abs(beta_skew0) + 0.03:
                                backoff_beta_sign = -backoff_beta_sign
                                self.get_logger().warn(
                                    f'BACKOFF: |skew| grew {abs(beta_skew0):.3f}->'
                                    f'{abs(skew):.3f} while backing off — flipping '
                                    f'backoff beta sign to {backoff_beta_sign:+.0f} '
                                    '(approach direction only verifies the sign for '
                                    'reverse, not forward)')
                    beta = skew * self._skew_to_rad * backoff_beta_sign * self._sign
                    target = self._k_beta * beta
                    target = max(-self._max_omega, min(self._max_omega, target))
            step = self._omega_slew * _TICK
            omega += max(-step, min(step, target - omega))
            self._drive(self._speed, omega)
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
        blind_omega = 0.0
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
                side_px = float(tag[4])
                skew = float(tag[6])

                # One-shot sign check over the first ~2.5s of motion — but
                # only once the starting bearing is a real signal, not
                # noise. Confirmed live: this locked in a flip off a
                # dx0 of -15.2px (~1.3deg, right at the noise floor), and
                # every approach afterwards diverged instead of converging
                # for the rest of the process's life — the near-zero
                # starting offset barely constrained which way was
                # actually correct, so it shouldn't have been trusted to
                # decide anything. Waiting for a clearly-off-centre
                # starting dx before even arming the window is what should
                # have stopped that: a later attempt (this one, or the
                # next) still gets to calibrate once a real signal shows
                # up, same as before, just not off a coin-flip.
                if not self._sign_checked:
                    if abs(dx) < self._sign_check_min_dx0:
                        pass
                    elif sign_t0 is None:
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

                # theta: the dock face's own heading error relative to the
                # robot, straight from skew (how far from square-on the
                # view is) — deadbanded so sensor noise near square doesn't
                # add a permanent bias. beta_sign is the same class of
                # hardware fact _sign is for alpha — which way skew maps to
                # a real angle depends on how the tag/camera are physically
                # mounted and can't be derived from the image alone, so
                # it's verified empirically below, same as _sign is.
                theta = 0.0
                if abs(skew) > self._beta_deadband:
                    theta = skew * self._skew_to_rad * self._beta_sign
                # Same one-shot check as for alpha, and the same fragility
                # fixed there: arming this off anything above the plain
                # noise deadband (0.02) let it "confirm" the sign off a
                # borderline, barely-there skew — then a later approach
                # with a genuinely large starting skew would show it was
                # wrong all along (confirmed live: skew climbing instead of
                # shrinking, well after this had already locked in
                # "confirmed"). Requiring a clearly-decisive starting skew
                # before even arming the window is what should have caught
                # that, same reasoning as _sign_check_min_dx0.
                if self._beta_autocal and not self._beta_checked:
                    if beta_t0 is None:
                        if abs(skew) > self._beta_check_min_skew0:
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

                # beta: the standard differential-drive parking law's own
                # heading term (see k_beta's own doc, above, for "w =
                # k_alpha*alpha + k_beta*beta") — NOT theta directly, which
                # this used to use in its place. beta is the heading error
                # the robot would arrive with if it nulled alpha by
                # rotating in place and then drove straight in without
                # turning again: driving straight preserves heading, so
                # that arrival heading is just the current bearing to the
                # tag (alpha) plus the current heading error (theta) —
                # beta = alpha + theta. Verified by direct construction,
                # not just recalled: see the standalone geometry check this
                # was worked through against before touching this file.
                # For a nonholonomic vehicle, centring and squaring up
                # aren't independent goals — correcting one changes the
                # other — which is exactly what an uncoupled `beta = theta`
                # missed. Confirmed live why that matters: a small starting
                # skew converged fine because alpha was doing all the real
                # work and theta rode along for it, but once skew was large
                # enough for its own term to have real authority, it fought
                # alpha instead of complementing it and the approach
                # diverged, worse the longer it ran.
                #
                # The classical result (Aicardi, Casalino, Bicchi,
                # Balestrino 1995) requires beta's gain to act with the
                # opposite sign from alpha's for closed-loop stability —
                # k_beta is kept as a positive *magnitude* (matching its
                # existing tuning/doc above) and subtracted here, rather
                # than folding a sign into k_beta itself, so both gains
                # read as plain positive weights.
                beta = alpha + theta
                # Boost heading-correction authority while far (side_px
                # small — plenty of standoff distance/arc room left),
                # tapering back to the plain, already-tested k_beta by
                # close_slow_side_px so nothing changes in the close-in
                # stretch this was tuned against. See k_beta_far_boost's
                # own doc. Clamped so effective k_beta can never reach
                # k_alpha — that cancellation is the actual failure mode
                # being avoided here, not something to risk reintroducing
                # while trying to fix it.
                k_beta_now = self._k_beta
                if side_px < self._close_slow_side_px:
                    far_frac = 1.0 - (side_px / max(self._close_slow_side_px, 1e-3))
                    k_beta_now = self._k_beta * (1.0 + far_frac * self._k_beta_far_boost)
                    k_beta_now = min(k_beta_now, self._k_alpha - 0.2)
                target = self._sign * (self._k_alpha * alpha
                                        - k_beta_now * beta)
                target = max(-self._max_omega, min(self._max_omega, target))
                step = self._omega_slew * _TICK
                omega += max(-step, min(step, target - omega))

                frac = max(self._min_speed_frac,
                           1.0 - abs(alpha) / max(self._alpha_slow, 1e-3))
                # See close_slow_side_px's own doc: ramps down further, on
                # top of alpha's own slowdown above, as the tag fills the
                # frame — reaching min_speed_frac by blind_min_side_px, the
                # same point the approach goes blind anyway.
                proximity_frac = 1.0
                if side_px > self._close_slow_side_px:
                    proximity_frac = max(
                        self._min_speed_frac,
                        1.0 - (side_px - self._close_slow_side_px)
                        / max(self._blind_min_side - self._close_slow_side_px,
                              1e-3),
                    )
                v = self._speed * min(1.0, frac, proximity_frac)
                self._drive(-v, omega)

                if now - last_log > 2.0:
                    last_log = now
                    self.get_logger().info(
                        f'SERVO: dx={dx:+6.1f}px ({math.degrees(alpha):+5.1f}deg) '
                        f'side={tag[4]:.0f}px skew={skew:+.3f} theta={theta:+.3f} '
                        f'(a={self._sign * self._k_alpha * alpha:+.3f} '
                        f'b={self._sign * -k_beta_now * beta:+.3f}'
                        f'{" *" if k_beta_now != self._k_beta else ""}) -> '
                        f'v={-v:+.3f} w={omega:+.3f}')
            else:
                here = self._xy()
                if blind_from is None:
                    if (self._last_side_px < self._blind_min_side
                            or abs(self._last_dx_px) > self._blind_max_dx
                            or abs(self._last_skew) > self._blind_max_skew):
                        self._stop()
                        if self._last_side_px < self._blind_min_side:
                            why = 'too small'
                        elif abs(self._last_dx_px) > self._blind_max_dx:
                            why = ('off-centre, so it slid out of frame rather '
                                  'than filled it')
                        else:
                            why = (f'squared to only {self._last_skew:+.3f} — '
                                  'blind at this heading drives into the '
                                  'guide funnel at an angle instead of '
                                  'through it')
                        self.get_logger().warn(
                            f'APPROACH: tag lost at {self._last_side_px:.0f}px, '
                            f'dx={self._last_dx_px:+.0f}px, skew='
                            f'{self._last_skew:+.3f} — {why}. Backing off '
                            'instead of creeping in crooked')
                        return False
                    blind_from = here
                    # Small, capped, dead-reckoned heading correction from
                    # the last real skew reading — not live feedback (there
                    # is none here), so this is a one-shot nudge held for
                    # the whole blind stretch, not a controller. Contact was
                    # being reached reliably (see the far-boost/backoff
                    # fixes above) but consistently failing "not seated"
                    # afterward — the last 5-10cm run dead straight with
                    # zero correction on a residual heading error that was
                    # small on screen (2-5deg) but apparently still enough
                    # to miss seating the connector. Own small gain and cap
                    # (blind_nudge_gain/_max_frac) — deliberately NOT
                    # k_beta/max_omega: this must stay a minor nudge, not an
                    # attempt to fully null the error the way live approach
                    # steering does, and must not silently change if those
                    # live gains get retuned later.
                    if abs(self._last_skew) > self._beta_deadband:
                        last_theta = (self._last_skew * self._skew_to_rad
                                      * self._beta_sign)
                        blind_omega = self._sign * (
                            -self._blind_nudge_gain * last_theta)
                        blind_cap = self._max_omega * self._blind_nudge_max_frac
                        blind_omega = max(-blind_cap, min(blind_cap, blind_omega))
                    nudge_note = (
                        f' (holding {blind_omega:+.3f} rad/s from last known '
                        f'skew {self._last_skew:+.3f})' if blind_omega else '')
                    self.get_logger().info(
                        f'APPROACH: tag filled the frame at '
                        f'{self._last_side_px:.0f}px and dropped out — creeping '
                        f'the last {self._blind_final * 100:.0f}cm at '
                        f'{self._blind_speed * 100:.1f}cm/s to find the '
                        f'contacts{nudge_note}')
                elif math.dist(here, blind_from) > self._blind_final:
                    self._stop()
                    self.get_logger().warn(
                        'APPROACH: blind allowance used up without charging')
                    return False
                # Slow, and only the one dead-reckoned nudge above (if any)
                # — no live steering without a measurement.
                omega = blind_omega
                self._drive(-self._blind_speed, blind_omega)

            spd = abs(self._odom.twist.twist.linear.x) if self._odom else 1.0
            close_enough = self._last_side_px >= self._blind_min_side
            if (close_enough
                    and now - started > self._stall_grace
                    and travelled > self._stall_min_travel
                    and spd < self._stall_speed):
                stalled_since = stalled_since or now
                if now - stalled_since > self._stall_confirm:
                    self._stop()
                    if abs(self._last_skew) > self._blind_max_skew:
                        # Close and genuinely stopped, but not square — this
                        # is a robot wedged into the guide funnel at an
                        # angle, not one seated on the pins. Treat it like
                        # any other failed approach (see the blind-creep
                        # gate above for the same reasoning) rather than
                        # spending charge_confirm_sec watching current that
                        # structurally cannot arrive.
                        self.get_logger().warn(
                            f'APPROACH: stalled crooked at {travelled * 100:.0f}cm '
                            f'(tag last {self._last_side_px:.0f}px, skew='
                            f'{self._last_skew:+.3f}) — jammed against the '
                            'guide funnel, not seated. Backing off')
                        return False
                    self.get_logger().info(
                        f'CONTACT: stalled at {travelled * 100:.0f}cm '
                        f'(tag last {self._last_side_px:.0f}px) — '
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
