# navpromini_self_filter

A geometric, TF-aware LaserScan self-filter.

```
/scan  →  remove ONLY confidently-identified robot self-geometry  →  /scan_filtered
```

Built to replace the coarse box/footprint filter this robot shipped with
(`navpromini_controller/config/scan_filter.yaml`, `laser_filters` package),
after live-diagnosing a real safety failure caused by that approach: it
excludes *any* return within a fixed zone around the robot, with no way to
tell "this is my own chassis" from "something is now touching my chassis."
Once a pushed obstacle entered that zone, it vanished from `/scan_filtered`
entirely — Nav2 saw clear space and kept driving into it. This package
exists specifically so that failure mode is structurally impossible: it
only removes a point when that point's *range* matches what a *specific,
known* piece of geometry (an 8mm pillar) would produce, within 2cm. A real
object — even touching the robot — is a different shape at a different
range and simply never matches.

## 1. Architecture

One node (`self_filter_node.py`), two supporting modules:

- **`geometry.py`** — pure-Python ray/shape intersection math (circle,
  capsule, polygon) plus the `Circle`/`Capsule`/`Polygon` self-object
  types built on it. No ROS imports; unit-testable standalone.
- **`self_objects.py`** — loads configured objects from ROS parameters and
  transforms them from `base_link` into the lidar frame via a TF
  (translation + quaternion). Also no ROS imports at its core (the TF
  itself is passed in as plain numbers) — the node is what actually talks
  to `tf2_ros`.
- **`self_filter_node.py`** — the ROS node: subscribes to `/scan`, looks
  up `base_link -> lidar_1`, transforms every enabled self-object, and for
  each object only tests the handful of beams that could possibly hit it
  (see "Geometry algorithm" below), publishing the result to
  `/scan_filtered`.

## 2. Geometry algorithm

For every LaserScan beam at angle θ, the ray is `(cos θ, sin θ)` from the
origin (the sensor always sees along a ray from itself). For every
enabled self-object:

1. Transform its configured `base_link`-frame geometry into the lidar
   frame using the current TF (once per scan, not per beam).
2. Compute that object's **angular extent** — the narrow bearing window
   it could possibly appear in (an 8mm pillar 12cm away spans about
   ±2°). This is the whole performance story: instead of testing all 720
   beams against every object, only the handful inside this window get
   tested at all.
3. For each beam inside that window, compute the range the object's own
   geometry would produce at that exact beam angle (`expected_range`) —
   real ray-vs-circle/capsule/polygon intersection, not a lookup table.
4. Compare the **measured** range to the **expected** range. Remove the
   point only if `abs(measured - expected) <= range_tolerance` (default
   2cm). Otherwise leave it completely alone.

Removed points are set to `NaN` — the value `sensor_msgs/LaserScan`
itself documents as "erroneous, invalid, or missing measurement," and
which Nav2's costmap layers correctly skip for *both* marking and
clearing. That is deliberate: removing a pillar return means "I don't
know what, if anything, is beyond this point" — not "this beam is clear
all the way out."

### The pillar-occludes-wall case

If a pillar physically sits between the lidar and a wall on the *same*
beam, the scanner reports the pillar (nearest surface wins, always) — the
wall return for that one beam genuinely does not exist in the data. This
filter removes the pillar point (correct: it matches known geometry) and
does **not** invent a wall behind it. Any *other* beam that independently
sees the wall is untouched, because it's a different beam being tested
against completely different range/geometry.

## 3–9. Source, package files

- `navpromini_self_filter/geometry.py`
- `navpromini_self_filter/self_objects.py`
- `navpromini_self_filter/self_filter_node.py`
- `package.xml`, `setup.py`, `setup.cfg`, `resource/navpromini_self_filter`
- `config/self_filter.yaml`
- `launch/self_filter.launch.py`
- `test/test_geometry.py`

## Build commands

```bash
cd ~/NavProMini_ws
colcon build --symlink-install --packages-select navpromini_self_filter
source install/setup.bash
```

## Run commands

```bash
# standalone, default config:
ros2 launch navpromini_self_filter self_filter.launch.py

# with a different config file:
ros2 launch navpromini_self_filter self_filter.launch.py \
  config_file:=/path/to/your.yaml

# debug topics/markers on:
ros2 launch navpromini_self_filter self_filter.launch.py \
  config_file:=<(cat config/self_filter.yaml; echo "      debug_enabled: true")
# (simplest in practice: edit debug_enabled in the yaml directly, or
#  ros2 param set /self_filter_node debug_enabled true at runtime)
```

Run the tests (pure Python, no ROS graph needed):

```bash
cd ~/NavProMini_ws/src/navpromini_self_filter
python3 -m pytest test/test_geometry.py -v
```

## 10. RViz2 debugging procedure

1. Set `debug_enabled: true` in `config/self_filter.yaml` (or
   `ros2 param set /self_filter_node debug_enabled true` on a running
   node — it's read live, no restart needed).
2. In RViz2, add:
   - **LaserScan**, topic `/scan` (raw) — Fixed Frame `lidar_1` or `map`.
   - **LaserScan**, topic `/scan_filtered` (filtered) — same frame, a
     different color, so removed points are visibly missing from this
     one but present in the raw one.
   - **MarkerArray**, topic `/self_filter_node/debug/self_geometry` —
     draws every enabled object's geometry, transformed into the lidar
     frame right now (cylinders for circles, line strips for
     capsules/polygons). This is what the filter currently *thinks* is
     robot structure.
   - **Marker** (POINTS), topic `/self_filter_node/debug/classified_points`
     — every valid raw point, colored **red if removed, green if kept**.
     Set `debug_publish_removed_points: true` to enable this one
     specifically (it's the more expensive of the two debug outputs).
3. Watch `/self_filter_node/debug/stats` (`std_msgs/String`) for
   per-scan counts: `raw_points`, `filtered_points`, `removed_points`,
   `removed_by_pillar_1`, etc.

To verify a real obstacle beside a pillar is *not* being removed: put
something next to a configured pillar, confirm its points still show
green in the classified-points marker and still appear in
`/scan_filtered`.

## 11. Example configuration (the four pillars)

See `config/self_filter.yaml` — it's live data, not a template:

| | bearing (from lidar) | range | base_link (x, y) | status |
|---|---|---|---|---|
| pillar_1 (rear-left) | −130.4° | 0.120m | (−0.046, +0.094) | **measured, enabled** |
| pillar_2 (rear-right) | +130.1° | 0.121m | (−0.046, −0.090) | **measured, enabled** |
| pillar_3 (front-left) | — | — | (0.100, 0.06) placeholder | **disabled — not measured** |
| pillar_4 (front-right) | — | — | (0.100, −0.06) placeholder | **disabled — not measured** |

The front two are disabled, not guessed-and-enabled: their ~100mm design
distance is *below this RPLidar's own `range_min` (0.15m, confirmed live
from a real `LaserScan` message)* — the driver discards a return that
close before any software sees it. They are physically invisible to this
sensor right now, not a filtering problem. If that ever changes
(relocated pillars, a different lidar), measure them for real before
flipping `enabled: true` — the current x/y are placeholders, not data.

## 12. Measuring exact pillar X/Y/orientation

What was actually done for pillar_1/pillar_2 above, repeatable for any
future object:

1. Confirm the `base_link -> lidar_1` TF once:
   ```bash
   ros2 run tf2_ros tf2_echo base_link lidar_1
   ```
   (On this robot: translation `(0.032, 0.001, 0.189)`, identity
   rotation — a purely forward-and-up offset, no tilt.)

2. Subscribe to raw `/scan` and, over several seconds, log every beam
   that reads under some threshold (e.g. 0.25m) more than once —
   `scripts/survey_scan.py`-style: a real static pillar shows up as a
   small, *contiguous* cluster of beams at a stable range, repeatedly,
   not a one-off. (This is exactly how pillar_1/pillar_2 above were
   found — a raw-scan survey over 60 samples / 8s, clustering every beam
   index that read close in at least 3 of them.)

3. For the cluster's median range `r` and its bearing `θ` (midpoint of
   the cluster's angular span), compute the point in the **lidar frame**:
   ```
   x_lidar = r * cos(θ)
   y_lidar = r * sin(θ)
   ```
   then into **base_link** using the TF from step 1 (for this robot,
   identity rotation, so it's just adding the translation):
   ```
   x_base_link = x_lidar + 0.032
   y_base_link = y_lidar + 0.001
   z_base_link = 0.189   # the lidar's own height — reasonable for a
                          # vertical post's cross-section at any z
   ```
   If your robot's lidar mount has real roll/pitch (not just yaw), do
   the full rotation, not just a translation add — see
   `self_objects.transform_point` for the general quaternion form.

4. `radius` is the pillar's actual physical radius (half its diameter),
   measured directly with calipers if precision matters — the scan
   itself won't resolve an 8mm feature's true diameter accurately at
   this range, only confirm it's there and where its center is.

## 13. Adding future wires/cables

Two steps, nothing else:

1. Add the name to `self_object_names`.
2. Define it as a `capsule` (`x1,y1,z1,x2,y2,z2,radius`) — the two
   endpoints, plus its actual radius.

A capsule with `radius` set to your wire's real thickness works for
"thin wire/cable" directly; there is no separate "line segment" type —
use a capsule with a very small radius (the spec's own example already
does exactly this, `radius: 0.003`). A "rectangle" is a 4-point
`polygon`. Nothing is ever filtered unless it's both named in
`self_object_names` **and** has `enabled: true` — an unconfigured wire is
never assumed to be self-geometry.

## 14. CPU optimization

- No PointCloud2 conversion anywhere — scans are worked with directly as
  ranges + angles.
- No numpy — these are O(1) scalar tests, and for the pruning pattern
  below, plain floats beat numpy's per-call overhead.
- **The real optimization**: per object, only the beams inside its
  precomputed angular extent are ever tested — for an 8mm pillar at
  12cm, that's roughly 4-8 beams out of 720, not all 720. Cost scales
  with (number of enabled objects × beams-near-each-object), not
  (beams × objects).
- The TF lookup happens once per scan (not per beam, not per object).

## 15. Known LaserScan limitations

- **Occlusion is real and not fixable from one scan.** If a pillar sits
  exactly between the lidar and a wall on one beam, that beam's wall
  return does not exist in the data — see "the pillar-occludes-wall
  case" above. This filter does not and must not invent it.
- **A feature smaller than the LaserScan's own resolution won't be
  perfectly localized.** At the RPLidar's angular resolution and this
  distance, an 8mm pillar is only a few beams wide — the measured
  cluster's angular *center* is a reasonable estimate of the pillar's
  true bearing, but don't expect sub-beam precision from the raw scan.
- **Below `range_min`, there is no data at all** — see pillar_3/pillar_4
  above. No self-filter, this one or any other, can remove or preserve a
  return that the sensor never produced in the first place.
- **This filter assumes a purely vertical self-object** (a cylinder
  whose axis is parallel to the lidar's own z-axis) when it drops z and
  uses only the transformed x/y as a circle's center. True for a
  well-mounted rigid AMR; would need real 3D ray-vs-cylinder math for a
  meaningfully tilted mount.

## 16. Safety considerations

- **Fail-safe is unconditional.** If the TF lookup does not succeed
  within `max_tf_age`, filtering is disabled and `/scan` is republished
  completely unchanged for that scan — no object is ever removed against
  a transform the node isn't confident in right now. See
  `self_filter_node._get_transform`'s docstring for exactly why this is
  judged by lookup success, not by comparing the transform's header
  stamp to a wall-clock age (a static TF's header stamp is not a
  reliable freshness signal, and treating it as one would make this
  filter spuriously stop working on a perfectly good, connected TF).
- **The removal tolerance is tight and explicit** (`range_tolerance`,
  default 2cm), not a spatial exclusion zone. This is the entire
  difference from the filter it's replacing, and the entire reason a
  pushed-in obstacle is safe here: it's a different object at a
  different range, and simply fails the match test.
- **This node's only job is `/scan -> /scan_filtered`.** It has no
  opinion on collision avoidance, costmaps, or navigation behavior —
  those all continue to run entirely off whatever this node publishes,
  same as before.

## Switching from the old filter — do this deliberately, not automatically

This package is **not wired into the robot's launch files by this
commit**. The existing `scan_to_scan_filter_chain` (in
`navpromini_controller`) keeps running exactly as it is until you choose
to swap it. Recommended sequence:

1. Build and run this node **alongside** the existing filter, publishing
   to a different topic for comparison first:
   ```bash
   ros2 launch navpromini_self_filter self_filter.launch.py \
     -- filtered_scan_topic:=/scan_filtered_test
   ```
   (or edit `filtered_scan_topic` in a copy of the config)
2. Compare `/scan_filtered` (old) against `/scan_filtered_test` (new) in
   RViz side by side, with the debug markers on, for a while — including
   deliberately pushing an object into contact, to confirm the exact
   failure mode this package was built to fix is actually fixed.
3. Only once satisfied, edit `navpromini_controller/launch/
   laser_scan_filter.launch.py` (or wherever it's included from) to
   launch this node instead, publishing the real `/scan_filtered` that
   Nav2 consumes.

Do not skip step 2. A scan filter change already broke `/scan_filtered`
outright once this session (a different, now-reverted fix) — treat any
change to this specific pipeline as needing live verification before
it's trusted, this package included.
