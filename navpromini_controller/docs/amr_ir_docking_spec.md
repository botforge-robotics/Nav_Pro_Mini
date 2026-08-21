# AMR IR Docking System — Implementation Spec

Close-range docking behavior for a circular AMR backing into a rectangular
charging dock, using a two-LED infrared beacon on the dock and two IR
receivers on the robot, mounted at matching offsets on either side of the
robot's centerline. This spec covers everything from "robot is near the
dock" to "robot is confirmed charging." It assumes normal SLAM/nav gets
the robot to a staging pose near the dock beforehand — this module takes
over from there.

---

## 1. Physical layout

**AMR**: circular, 260mm diameter, **differential drive**. Charging
contacts and the IR receiver are mounted at the rear. The robot backs into
the dock (does not drive in forward).

The drive type is load-bearing for this whole document, so it is stated up
front. A differential base has no sideways primitive: every lateral
correction is an arc, and an arc changes **heading and position together**.
It cannot translate without rotating, and it cannot rotate usefully either
— see §4a. A holonomic (mecanum) base can strafe, decoupling the two, and
on such a base the naive "steer continuously toward the overlap" policy
works fine. On this base it does not, and §4 is written accordingly.

**Dock**: rectangular, 21.5cm × 8cm. A mechanical funnel guide starts
15cm out from the dock face and narrows down to center the robot exactly
on the connector — this funnel does the final few centimeters of
mechanical correction; the IR system only needs to get the robot straight
and roughly centered before it enters the funnel.

**Emitters (on the dock)**:
- 2× generic 3mm IR LEDs, no datasheet available
- 8cm apart (4cm either side of dock centerline)
- Toed outward 15–20° each (~30–40° total spread) as a starting point —
  must be verified against the LED's actual measured beam half-angle
  (see §5, no datasheet available)
- Each LED fitted with an opaque shroud: sides covered, front open,
  matte black interior, length tuned so it doesn't cut into the intended
  beam cone
- Both driven at the same carrier frequency (whatever the on-hand TSOP
  receiver is tuned to — treat as a fixed constant, e.g. 38kHz)

**Receivers (on the AMR)**:
- 2× TSOP-style demodulating IR receivers, rear-facing, tuned to the same
  carrier frequency as the dock emitters
- Mounted at ±4cm from the robot's centerline — matching the dock's
  emitter spacing exactly, not spread arbitrarily
- Output is active-low on each: pin idles HIGH, pulls LOW while a valid
  carrier burst is present
- Identity (left vs right) comes entirely from burst duration (§2),
  decoded identically on *both* receivers — physical position is never
  used to distinguish left from right. Position is used only for range:
  each beam narrows toward its own emitter's location as distance to the
  dock decreases, so a receiver riding at that same offset keeps
  detecting its beam much closer to the dock face than a centerline
  receiver would, directly shrinking the IR dead zone.
  **Measured outcome: it does better than shrink it — it removes it.**
  Driving straight in on the centreline, OVERLAP holds continuously from
  75cm all the way to dock contact. This was the one choice in this
  document justified purely on theory, and it paid off hardest: COMMIT no
  longer drives blind (§4), and loss of signal during the approach becomes
  a usable fault signal instead of an expected event. Do not "simplify"
  this to a single centreline receiver.
- Both receivers' outputs OR-combine into the same shared
  `last_seen_left` / `last_seen_right` variables — either one detecting
  a burst updates the relevant timestamp (see §4 for the actual
  two-instance wiring)

---

## 2. Signal design

Since both LEDs share one carrier frequency, left/right identity is
encoded in **burst duration**, not frequency:

| LED   | Burst ON time | Meaning to firmware |
|-------|---------------|----------------------|
| Left  | ~1ms          | robot is left of centerline |
| Right | ~2ms          | robot is right of centerline |

**Transmit schedule** (dock side) — the two LEDs must never fire
simultaneously, or the receiver can't tell them apart as separate events:

```
Left ON 1ms → OFF 18ms → Right ON 2ms → OFF 18ms → repeat
(one full cycle = 39ms, so each LED gets a fresh burst ~26×/sec)
```

**The 18ms gap is a hard receiver constraint, not a preference.** The
original 3ms gap gives a 33% duty cycle, and a TSOP-style AGC receiver
treats sustained carrier as interference: it drops gain mid-burst and
truncates the output pulse. Measured on the TSOP1838, at a fixed 30cm:

| gap | duty | 1ms burst reads as | 2ms burst reads as |
|-----|------|--------------------|--------------------|
| 3ms  | 33%  | ~786µs  | ~396µs  |
| 8ms  | 16%  | ~840µs  | ~392µs  |
| 12ms | 11%  | ~1093µs | ~1910µs |
| 18ms | 7.7% | ~1050µs | ~1961µs |

The cliff sits between 16% and 11% duty. Truncation hits the **2ms burst
hardest**, which is fatal precisely because burst length *is* the
left/right encoding — the right burst collapses below the 500µs floor and
is discarded as noise while the left burst survives, so the robot sees a
permanent "LEFT" and never reaches OVERLAP.

12ms (11% duty) clears the cliff but proved too thin over a long run: the
receiver seeing the 2ms burst decayed from ~1985µs to ~640µs over about a
second, recovered, and repeated on a ~3s cycle. That oscillation is
invisible in captures shorter than a few seconds. 18ms gives ~2× margin
and held flat over a full minute. **Any retune of the burst widths must
keep duty below ~8% and be verified over 60s+, not a short sample.**

---

## 3. Dock firmware (transmitter — simple, no logic)

Runs forever once powered. No sensing, no state, no communication with
the robot.

```
loop forever:
    drive Left_LED at carrier_freq for 1ms, then off
    wait 18ms
    drive Right_LED at carrier_freq for 2ms, then off
    wait 18ms
```

Two AVR traps, both of which silently produce a *wrong timing* rather than
an error, and both of which were hit during bring-up (see §6):

- Do **not** use `tone(pin, freq, duration)`'s 3-argument form. It computes
  `2 * frequency * duration / 1000` with `frequency` as a 16-bit `unsigned
  int`, so at 38kHz `2 * 38000` wraps to 10464 and a requested 1ms burst is
  emitted as 131µs. Use continuous `tone(pin, freq)`, time the burst with
  `micros()`, then `noTone(pin)`.
- Do **not** use `delayMicroseconds()` for the 18ms gap. It takes a 16-bit
  `unsigned int` and does `us <<= 2` immediately, so anything above 16383
  wraps: `delayMicroseconds(18000)` waits 1615µs. Express the gap in
  milliseconds and use `delay()`.

---

## 4. AMR firmware (receiver + decision logic)

Two independent jobs. Job 1 must be interrupt-driven — a 1ms pulse can be
missed entirely if it's only checked by polling inside a busy main loop.

### Job 1 — burst edge classification (interrupt, runs instantly on pin change)

Runs as **two independent, identical instances** — one per receiver pin
(RX_A at −4cm, RX_B at +4cm). Both instances write into the same shared
`last_seen_left` / `last_seen_right` variables:

```
# attach this same handler twice: once for RX_A_PIN, once for RX_B_PIN
on RX_x_PIN_change:                    # x = A or B
    if pin just went LOW:              # burst starting
        burst_start[x] = micros()      # per-receiver, NOT shared

    if pin just went HIGH:             # burst ending
        width = micros() - burst_start[x]
        if width in range 0.5ms-1.5ms:
            last_seen_left = millis()      # shared across both receivers
        else if width in range 1.5ms-2.5ms:
            last_seen_right = millis()     # shared across both receivers
        else:
            # ignore — noise, reflection glitch, or collision artifact
```

`burst_start` is per-receiver state (each pin times its own edges
independently) — but `last_seen_left`/`last_seen_right` are single,
shared variables, since they answer "was the left/right signal seen by
*anything*," not "seen by which receiver." Job 2 below never needs to
know there are two receivers at all.

### Job 2 — zone classification + steering (periodic, every 30-50ms)

```
loop every 30-50ms:
    left_recent  = (millis() - last_seen_left)  < 90ms
    right_recent = (millis() - last_seen_right) < 90ms

    if left_recent and right_recent:   zone = OVERLAP
    elif left_recent:                  zone = LEFT
    elif right_recent:                 zone = RIGHT
    else:                              zone = NONE
```

The recency window is **90ms, and it is tied to the TX cycle** — it must
span 2+ full cycles so one missed burst does not drop the zone. The 18ms
gap makes a cycle 39ms, so 39 × 2 = 78ms, rounded to 90ms. If the gap
changes, this changes with it. (The original 25ms paired with the original
9ms cycle; carrying 25ms forward against a 39ms cycle drops OVERLAP on
roughly every cycle, which looks exactly like a signal problem and is not.)

Zone is recomputed from scratch every cycle and is never latched. What the
robot *does* with it is where the drive type matters — see §4a.

**Worked example of overlap detection.** Say the clock currently reads
1000ms, and `last_seen_left` = 940ms, `last_seen_right` = 975ms. Both are
within the 90ms recency window (60ms and 25ms ago respectively), so both
`left_recent` and `right_recent` are true this cycle → `zone = OVERLAP`.
"Overlap" is never a state the robot commits to — it's just what the last
90ms of listening happened to show, re-checked from scratch every cycle.

### §4a / Job 2a — turning a zone into motion on a differential base

Two facts constrain this, and together they rule out the obvious policy.

**Zone is a function of position, not heading.** Which beam you are
standing in depends on where you are in space. Rotating in place does not
change it. Rotate far enough and the receivers simply stop facing the dock,
so the zone goes LEFT → NONE without ever passing through OVERLAP. *Spin-
in-place is never a useful correction in ALIGN* — it is only a search
behaviour, for when there is no signal at all to lose.

**Heading is not directly observable.** By §1, left/right identity comes
purely from burst duration and both receivers OR into the same variables.
The zone is a one-dimensional lateral error signal and nothing more.
"Centred but rotated 20°" and "square but offset 5cm" produce identical
readings in a single sample. The controller cannot tell them apart, and so
cannot *measure* heading.

It can, however, **test** heading — indirectly, and only while moving.
Because OVERLAP holds continuously from 75cm to contact when the approach is
square (no dead zone, see §5), a crooked approach walks the receivers off
their matched beams and OVERLAP drops. So *holding* OVERLAP through the
whole reverse is evidence the approach stayed square, and losing it is
evidence it did not. That is a pass/fail check integrated over the approach,
not an angle you can steer on — but it is a sound abort criterion, and it is
the only heading information this design yields.

On a holonomic base neither bites: strafing fixes lateral error without
touching heading, so heading never drifts and never needs measuring. On a
differential base, every correction is an arc that changes heading too —
and since heading is invisible, nothing ever corrects it back. Steering
continuously all the way in therefore converges to the middle of the beam
while pointing off-axis, and the robot enters the funnel crooked.

**So: align at a standoff, then stop steering.**

```
if zone == LEFT:    arc_right(small_step)     # ALIGN, at standoff only
if zone == RIGHT:   arc_left(small_step)      # ALIGN, at standoff only
if zone == OVERLAP: drive_straight_back(small_step)
if zone == NONE:    back_off_then_search()    # NOT spin-in-place
```

**The capture band is a constant-width corridor, not a wedge.** Measured at
±7cm at 30cm and again at ±7cm (±2cm) at 75cm — it does not widen with
distance. The angular overlap does grow with range, but off-axis intensity
falls off fast enough that the detection contour stays roughly parallel.
Two earlier models of this document were wrong: it is neither a wedge that
broadens with distance nor a lens that closes again far out. Over 30–75cm
it is simply a ~14cm-wide lane down the dock centreline.

That has a strong consequence in the robot's favour: **a robot that is
centred and square at 75cm can reverse straight in and stay in the band the
whole way.** No steering, no re-acquisition. Getting into the lane is the
entire problem; staying in it is free.

So the standoff exists for a different reason than band width — it buys
**arc room**. A differential base pays for lateral correction in heading,
and the exchange rate is roughly `alpha ~= 2 * dy / L` (peak heading swing,
lateral error `dy`, travel available `L`):

| travel | fix 8cm | fix 15cm | fix 25cm |
|--------|---------|----------|----------|
| 30cm   | 31°     | 57°      | 95°      |
| 50cm   | 18°     | 34°      | 57°      |
| 75cm   | 12°     | 23°      | 38°      |
| 100cm  | 9°      | 17°      | 29°      |

Correcting 15cm with 30cm of runway costs 57° — the receivers stop facing
the dock long before the manoeuvre finishes, which is exactly why close-in
correction feels impossible. The same correction at 75cm costs 23°.

This also explains the recovery move: on losing the zone, **back off first,
then search.** Reversing out does not widen the band, but it buys runway,
so the correction that was geometrically impossible becomes a gentle arc.

If a lateral trim really is needed closer in, do it open-loop on odometry
as an S-curve — arc out and back by equal amounts, netting zero heading
change — then re-measure. That turns steering into discrete
measure-correct-measure steps and sidesteps the missing heading feedback
rather than pretending it exists.

### Full docking state machine

```
STATE: SEARCH
    trigger: battery low OR task queue empty
    behavior: rotate / spiral scan
             (rotation is legitimate HERE - with no signal at all there is
              no lock to lose, and sweeping the receivers' facing is the
              point. It is not a correction, and must not be used as one
              in ALIGN; see §4a)
    exit: any zone != NONE  →  go to ALIGN

STATE: ALIGN
    behavior: run Job 2 loop above, every cycle, BUT only steer while
              further out than ~75cm (the standoff). Inside that, hold
              heading and reverse straight - see §4a.
    note: the target is the ~14cm-wide capture band (+/-7cm). Steer with
          gentle arcs; if the offset is large enough that the arc would
          exceed ~25deg of heading swing, back off for more runway rather
          than cranking the correction (§4a table).
    if zone == NONE for > [some short timeout]:
              back off ~15-20cm FIRST, then → back to SEARCH
              (reversing puts the same lateral offset into a wider part of
               the beam wedge and often re-acquires immediately; dropping
               straight to a spin will not)
    if zone == OVERLAP continuously for ~0.5-1s:   → go to APPROACH
    (this short confirm window — not the old 5s — is enough to know the
    robot is genuinely centered and moving, not reacting to a stray
    reflection)

STATE: APPROACH                          [differential-drive specific]
    why this exists: on a non-holonomic base, steering all the way in is
    what causes crooked docking (§4a). Alignment is established once, at a
    standoff where the overlap wedge is wide, and then held.
    behavior: reverse straight back from ~75cm, NO lateral correction, while
              the IR signal is still available as a sanity check only.
              The capture band is a constant-width lane (§4a), so staying in
              it costs nothing once entered - do not "help" it.
    abort: if zone becomes LEFT or RIGHT and stays there, the approach was
           not straight enough → back off and return to ALIGN rather than
           steering mid-approach. OVERLAP is available the whole way in, so
           losing it is genuine evidence of a crooked approach - it is the
           closest thing this design has to heading feedback (§4a).
    exit: near contact → go to COMMIT (which is now also sighted, not blind)

STATE: COMMIT
    NOTE: this state no longer drives blind. Earlier revisions assumed an
    IR dead zone in front of the dock, on the reasoning that the toed-out
    emitters leave the centreline uncovered near the dock face. Measurement
    says otherwise: with receivers at +-4cm matching the emitter spacing,
    OVERLAP holds continuously from 75cm all the way to contact. There is
    no dead zone to cross, so there is nothing to cross blind.
    why it still exists: the final push needs contact detection, which is
    a different job from steering.
    behavior: continue straight back with IR STILL MONITORED. Do not steer
              (the band is a constant-width lane, §4a) but do not ignore
              the signal either.
    bound: until contact - drive-motor current spike / stall - or a short
           travel-and-time backstop as a fallback if contact never trips
    LOSS OF OVERLAP IS NOW A FAULT, NOT AN EXPECTED EVENT. Because the
    signal is available the whole way in, losing it means the approach has
    gone crooked or something is occluding a receiver. Abort to the
    back-off/retry path in CONFIRM rather than pushing on.
    exit: contact detected, or backstop reached → go to CONFIRM

STATE: CONFIRM
    action: read BMS charging status
    if charging == true:   → go to DOCKED (success, stop)
    if charging == false:
        back off ~15-20cm, small heading nudge
        retry_count += 1
        if retry_count <= 3:  → back to SEARCH
        else:                 → go to FAULT

STATE: DOCKED
    stop all motion, exit docking behavior

STATE: FAULT
    stop, raise alert to operator / higher-level system
```

---

## 5. Parameters / tunables

| Parameter | Starting value | Notes |
|---|---|---|
| Emitter spacing | 8cm | not sensitive to real-range performance, mounting-driven only |
| Emitter toe-out angle | **14.0° left / 14.9° right** (measured) | Must not exceed the beam half-angle, or a dead zone opens in front of the dock. Too little and the beams overlap everywhere, losing left/right discrimination entirely. Measure toe-out from **the LED's own position**, not the dock centreline — the emitter sits 4cm off centre and ignoring that inflates the angle (14° would read as 21°). The two emitters are aimed within 1° of each other, which is better than hand-aiming suggests |
| Emitter beam half-angle | **36.9° left / 31.0° right** @ 30cm | Single-LED sweeps, single receiver (RX_A), other LED covered, RX_A started at −4cm. Left: edges −34/+11cm, width 45cm. Right: edges −6/+30cm, width 36cm. Both PASS toe-out ≤ half-angle (23° and 16° margin), so **emitter geometry is not a limiting factor.** The ~6° cone-width asymmetry is real and shifts the capture zone right of the dock centreline |
| Capture band (green) | **±7cm at 30cm AND at 75cm** (±2cm) | Measured in production config (both LEDs, both receivers). **Constant width, not a wedge** — it does not broaden with distance. Symmetric and centred on the dock centreline despite the emitters' 6° cone-width asymmetry, so no aiming correction is needed. Wings (LEFT/RIGHT only) run ±7cm out to ±34cm; beyond ±34cm, nothing |
| Standoff distance (align here) | **75cm** | Not chosen for band width — the band is the same there as at 30cm. Chosen for **arc room**: correcting a 15cm offset costs ~23° of heading swing at 75cm versus ~57° at 30cm (§4a). Align here, then reverse straight in; the band is a constant-width lane, so a robot that is centred and square at 75cm stays in it all the way |
| Dead-zone onset (COMMIT bound) | **none — no dead zone exists** | Measured: driving straight in on the centreline, OVERLAP holds continuously from 75cm all the way to dock contact. The ±4cm matched-offset receiver placement (§1) did not merely shrink the dead zone as predicted, it removed it. **COMMIT no longer needs to be blind** — see the state machine |
| — prediction that failed | predicted 25cm wide, centred +2.5cm | Derived from the single-LED sweeps and wrong on both counts. Two causes, both worth remembering when extrapolating from single-LED data: (1) a single LED runs 2.6% duty vs 7.7% production, so the AGC sits at higher gain and every beam edge reads further out than it really is; (2) the right LED's inward edge was a noisy 2cm reading, the least reliable number in the set, and it alone produced the predicted asymmetry. **Measure the capture band directly; do not compute it from single-LED sweeps** |
| Carrier frequency | 38kHz (TSOP1838) | fixed by hardware in stock |
| Left burst width | ~1ms | classification window: 0.5–1.5ms |
| Right burst width | ~2ms | classification window: 1.5–2.5ms |
| Inter-burst gap | **18ms** | AGC constraint, not a preference — see §2. Do NOT shrink; duty must stay under ~8% |
| TX cycle length | 39ms | = 1 + 18 + 2 + 18. Derived; changes if the gap does |
| Recency window (last_seen) | **90ms** | covers 2+ full TX cycles, tolerates one missed pulse. Scales with cycle length — retune together |
| Steering loop rate | 30–50ms | |
| Standoff distance (align here) | **unmeasured** | where the overlap wedge is comfortably wider than the nav stack's stopping accuracy. Measure green-band width vs distance first |
| Overlap-confirm-before-approach | ~0.5-1s | short window proving genuine centering before committing to the un-steered straight reverse. Less critical than it was when COMMIT ran blind, since OVERLAP is now monitored the whole way in and a bad approach aborts itself |
| Commit bound | contact detection (stall / current spike), with a short travel+time backstop | **No longer a blind odometry reverse** — there is no dead zone to cover, so COMMIT keeps watching IR the whole way and treats loss of OVERLAP as a fault. The odometry bound survives only as a backstop for the case where contact never trips |
| Back-off distance | 15–20cm | on failed charge confirmation |
| Max retries | 3 | before raising FAULT |

---

## 6. Implementation notes for whoever writes the code

- Job 1 (edge classification) **must** be a hardware interrupt on the
  TSOP output pin, not a polled `digitalRead()` in the main loop — a 1ms
  pulse is easy to miss otherwise.
- **Every failure during bring-up looked like a hardware fault and was
  not.** In order: `tone()`'s duration argument overflowing (bursts emitted
  at 131µs/263µs instead of 1ms/2ms), AGC suppression from the 33% duty
  cycle (bursts truncated to ~786µs/~396µs), and
  `delayMicroseconds(18000)` wrapping to 1615µs (duty driven to 48%, worse
  than the original). All three presented as "the receiver isn't detecting
  anything." Before suspecting wiring, **measure what is actually being
  emitted** — the receiver's reported burst widths are the ground truth,
  and a width that is frozen across many samples means no bursts are
  arriving at all, while a width that is *wrong but varying* means the
  transmitter is running and mistimed.
- Diagnose duty-cycle/AGC problems over **60s+**. The failure oscillates on
  a multi-second period; a 4-second sample can land entirely inside a good
  phase and read as clean. A useful single metric is the fraction of
  samples where *neither* receiver's width changed — "dead air." Healthy is
  under 1%; the broken configurations ran 72–92%.
- Simultaneous dead air on both receivers is a **common-mode** fault
  (transmitter, power, or geometry). AGC suppression acts per-receiver with
  its own time constant and truncates rather than silences, so if both
  channels go dark together, stop looking at the receivers.
- The two receivers give the robot **no heading information** (§4a). If a
  future revision needs it, the raw material is already there — which
  receiver saw which burst is currently discarded by the OR-combine. A test
  rig can log it without changing the robot's logic, and it is the
  difference between diagnosing "offset" and "rotated."
- The overlap region is a **wedge widening with distance**, not a corridor.
  Most approach-strategy questions on a differential base resolve to
  "align further out, where the wedge is wide."
- The emitter beam angle is unverified (generic LEDs, no datasheet) —
  before finalizing toe-out angle, sweep a receiver in an arc at real
  docking distance (~0.5–1m) to measure the actual half-angle, then set
  toe-out at or below that measured value.
- This module owns everything from "near the dock" through "confirmed
  charging." Getting the robot to the staging pose in front of the dock
  is the job of the normal navigation stack (SLAM/AMCL) and is out of
  scope here.
- Treat `zone` (LEFT / RIGHT / OVERLAP / NONE) as the single interface
  between Job 1/2 and the state machine — keep the burst-timing details
  contained in Job 1 so the state machine logic doesn't need to know
  anything about milliseconds or carrier frequencies.
- Going beyond two receivers (coverage, not identity): if additional
  same-frequency TSOPs are ever added purely to widen search-phase
  detection arc, they follow the exact same pattern as RX_A/RX_B in
  §4 — an independent Job 1 instance per pin, all OR-combining into the
  same shared variables. Job 2 and the state machine need zero changes
  regardless of receiver count. Note the Arduino Nano hardware-interrupt
  constraint below before assuming a third receiver is a simple addition.
- Dead zone: **measured, and there isn't one.** With the pair at ±4cm
  matching the emitters, walking straight in on the centreline holds
  OVERLAP from 75cm through to dock contact. The expectation above — that
  the placement would shorten the dead zone relative to a centreline
  receiver — was too modest; it closes it entirely. Re-measure this if the
  receiver spacing, emitter spacing, or toe-out is ever changed, because
  the result depends on all three matching.
- Platform note (Arduino Nano, no transistor on hand): direct-drive works
  fine here — Nano's 5V logic plus the low duty cycle of this burst
  pattern keeps current safely within the ATmega328P's 40mA/pin absolute
  rating. Wire each LED as GPIO pin → ~150-220Ω resistor → LED anode →
  LED cathode → GND, no transistor stage needed. Generate the carrier
  with the built-in `tone(pin, 38000, duration_ms)` — its single-timer
  limitation (one pin at a time) is a non-issue since Left/Right bursts
  are already scheduled to never overlap.
- Pin reservation (AMR board, Arduino Nano): only D2/D3 support true
  hardware interrupts — with two receivers, that's both of them already
  spoken for: RX_A on D2, RX_B on D3. If a third receiver is ever added
  for coverage, there's no hardware interrupt pin left for it on a Nano
  — it would need pin-change interrupts (PCINT) instead, which behave
  differently in code and aren't a drop-in copy of Job 1.
- Dock transmitter pin choice (separate board from the AMR receivers,
  dedicated solely to these 2 LEDs — no other devices on it): `tone()`
  works on any digital pin, not just the PWM-marked ones. With nothing
  else sharing the board, the only pins actually worth avoiding are
  D0/D1 (the board's own USB-serial link, used for uploading code and
  any debug prints) — every other pin, D2 through D13, is equally fine.
  The earlier caution about D3/D11 (Timer2, shared with `analogWrite()`)
  and D10–D13/A4–A5 (SPI/I2C) only matters if something else on the
  board might use those peripherals — moot here since it's LED-only.
  D5/D6 remain a fine, simple pick if a default is wanted; nothing
  technical requires those specific two.
- COMMIT's stall detection is **no longer optional** — it is now the
  primary exit condition. With no dead zone to bound by odometry, contact
  detection is what tells the robot it has arrived; the travel-and-time
  bound is only a backstop for when contact never trips. If drive current
  is already read for anything else on the platform, reuse it here.
