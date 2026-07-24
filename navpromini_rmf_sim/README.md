# NavProMini RMF simulation

Multi-robot Gazebo + namespaced Nav2 + RMF site assets from traffic-editor
`site.building.yaml`.

## Update nav graphs after traffic-editor

1. Edit the map in **traffic-editor** and **Save** → `site/site.building.yaml`.
2. Regenerate nav graphs + Gazebo world:

```bash
source /opt/ros/jazzy/setup.bash
source ~/rmf_ws/install/setup.bash
source ~/NavProMini_ws/install/setup.bash

ros2 run navpromini_rmf_sim generate_site_assets
```

That refreshes `site/nav_graphs/0.yaml` (RMF routes), `site/generated/cafe.world`,
fleet/spawn YAMLs, lift cabins, and parking chargers.

3. **Relaunch** Terminal A + B so fleet/Nav2 load the new graph.

Optional flags:

```bash
ros2 run navpromini_rmf_sim generate_site_assets --site-dir ~/NavProMini_ws/src/navpromini_rmf_sim/site
```

If the package was never built in this workspace:

```bash
cd ~/NavProMini_ws
colcon build --packages-select navpromini_rmf_sim --symlink-install
source install/setup.bash
ros2 run navpromini_rmf_sim generate_site_assets
```

### What it writes

| Output                              | Purpose                                        |
| ----------------------------------- | ---------------------------------------------- |
| `site/generated/cafe.world`         | Gazebo Harmonic world (walls, furniture, lift) |
| `site/generated/models/`            | Floor models (`cafe_L1`, `cafe_L2`)            |
| `site/nav_graphs/0.yaml`            | RMF routing graph                              |
| `site/fleet_config/navpromini.yaml` | Robots from `spawn_robot_name`                 |
| `site/spawn_poses.yaml`             | Gazebo spawn x/y/z/yaw                         |

Spawn mapping (from your building):

- `parking2` → **robot1**
- `parking1` → **robot2**

Chargers in fleet config match those parking waypoint names.

## Adapters (demos-style layout)

```text
navpromini_rmf_sim/adapters/
  door_adapter.py          # docs only — Gazebo libdoor owns doors
  lift_adapter.py          # optional fallback if liblift off
  workcell_adapter.py      # dispenser / ingestor
  fleet/
    path_to_nav2_bridge.py # PathRequest → Nav2

launch/include/adapters/
  door_adapter.launch.py
  lift_adapter.launch.py   # default: Gazebo liblift; Python only if requested
  workcell_adapter.launch.py
  fleet_adapter.launch.py  # bridge + rmf_demos EasyFullControl
```

| Role | Runtime |
| --- | --- |
| **Door** | Gazebo `libdoor` + `door_supervisor` (no Python node) |
| **Lift** | Gazebo `liblift` (lift **node**) + `lift_supervisor` (lift **adapter**) — [book](https://osrf.github.io/ros2multirobotbook/integration_lifts.html) |
| **Workcell** | `workcell_adapter` |
| **Fleet** | `path_to_nav2_bridge` + `rmf_demos_fleet_adapter` |

```bash
ros2 run navpromini_rmf_sim door_adapter          # prints ownership note
ros2 launch navpromini_rmf_sim rmf_fleet.launch.py  # → include/adapters/fleet_adapter
```

## Open-source RMF web (API + dashboard, no Gazebo)

Uses this package’s `site/site.building.yaml` maps with open-source
[rmf-web](https://github.com/open-rmf/rmf-web) (Docker by default).

```bash
source /opt/ros/jazzy/setup.bash
source ~/rmf_ws/install/setup.bash
source ~/NavProMini_ws/install/setup.bash

ros2 launch navpromini_rmf_sim rmf_web.launch.py
```

| URL | Login |
| --- | --- |
| Dashboard **http://localhost:3000** | `admin` / `admin` |
| API docs **http://localhost:8000/docs** | — |

What it starts:

- `building_map_server` ← `site/site.building.yaml`
- RMF core (traffic schedule, blockade, task dispatcher, door/lift supervisors)
- Schedule visualizer websocket `:8006` (dashboard trajectories)
- **Fleet adapter** (`rmf_fleet.launch.py`) so `robot1` / `robot2` appear in the UI
- Docker: `api-server` + `demo-dashboard`

### Robots + path graph in the dashboard

- **Robots**: need the fleet adapter (`start_fleet:=true`, default). Restart
  `rmf_web.launch.py` after rebuilding. Robots use `/robotN/odom` when Gazebo
  is up, otherwise spawn poses.
- **Motion**: `path_to_nav2_bridge` follows EasyFullControl one-waypoint
  PathRequests (not the full schedule polyline). Arrival tol is tight (0.25 m)
  so DoorOpen runs when the robot reaches the door approach vertex — not at
  task start. Requires Gazebo + `start_nav:=true` and `use_sim_time:=true`.
- **Doors**: Gazebo `rmf_simulation_door_manager` (`libdoor`) is the sole
  `/door_states` publisher. `door_supervisor` (RMF core) mediates open/close.
  Do not run a parallel DoorState stub — dual publishers leave doors stuck
  MOVING and robots wait forever.
- **Path lanes**: open-source rmf-web does **not** draw lane lines on the map —
  only waypoint markers + floorplan. Routing still uses `site/nav_graphs/0.yaml`.
  Planned routes show under the **Trajectories** layer once robots have tasks
  (and the schedule visualizer is healthy on `:8006`).
- Layers panel: enable **Waypoints** / **Waypoint labels** if you want parking
  spots etc. (Pickup/Dropoff dots are on by default).

### Starting RMF simulation

Source workspaces in **both** terminals first:

```bash
source /opt/ros/jazzy/setup.bash
source ~/rmf_ws/install/setup.bash
source ~/NavProMini_ws/install/setup.bash
```

Then run one command per terminal:

```bash
# Terminal A — Gazebo + namespaced Nav2
ros2 launch navpromini_rmf_sim rmf_sim.launch.py start_nav:=true

# Terminal B — RMF core + fleet + workcells + local rmf-web dashboard
ros2 launch navpromini_rmf_sim rmf_web.launch.py web_mode:=local use_sim_time:=true \
  start_building_map:=false start_workcells:=true start_fleet:=true \
  rmf_web_dir:=~/rmf_ws/src/rmf-web
```

**Relaunch tip:** stop both terminals fully before starting again. A leftover
`fleet_adapter` / `path_to_nav2_bridge` from a previous run publishes a second
`/fleet_states` stream and the dashboard robot marker **jumps**. Check with
`ros2 topic info /fleet_states` (Publisher count should be **1**).

**Lidar / costmap:** the cafe world must include `gz-sim-sensors-system` or
Gazebo never publishes `/robotN/scan` (bridge sits idle, AMCL/costmap get no
lidar). After regenerating the world, restart Terminal A. In Gazebo you can
add the **Visualize Lidar** GUI plugin to see rays; ROS check:

```bash
gz topic -i -t /robot1/scan    # should list a Publisher
ros2 topic hz /robot1/scan
```

| URL | Login |
| --- | --- |
| Dashboard **http://localhost:5173** | `admin` / `admin` |
| API docs **http://localhost:8000/docs** | — |

`start_building_map:=false` avoids a second map server (sim already provides it).

**Dashboard place names (exact):**

| Role | L1 | L2 |
| --- | --- | --- |
| Pickup | `pantry` | `pantry_L2` |
| Dropoff | `Table1_L1` … `Table5_L1` | `Table1_L2` … `Table5_L2` |

**Handlers** (must match nav-graph `pickup_dispenser` / `dropoff_ingestor`):

| Place | Handler |
| --- | --- |
| `pantry` | `pantry` |
| `pantry_L2` | `pantry_L2` |
| `TableN_L1` / `TableN_L2` | same as place (`Table3_L2`, …) |

Do **not** use office-demo names like `coke_dispenser` / `coke_ingestor` — that
leaves the robot stuck on **Loaditem** / **Unload** at the place. Prefer the
handler autofilled from the map; the workcell adapter will still complete
mismatched GUIDs as a sim fallback.

Cross-floor (e.g. `pantry` → `Table3_L2`) uses the book lift stack:
fleet → `/adapter_lift_requests` → `lift_supervisor` → `/lift_requests` →
Gazebo `liblift` (publishes `/lift_states`, moves cabin + doors). Requires
`lifts.Lift1.plugins: true` in `site.building.yaml` and a regenerated world.
Do **not** also run the Python lift adapter (dual `/lift_states` publishers).
Fallback only: `use_python_lift_adapter:=true` with `plugins: false`.

Fleet-only (API already running):

```bash
ros2 launch navpromini_rmf_sim rmf_fleet.launch.py use_sim_time:=true
```

Useful args:

```bash
# Docker dashboard instead of local pnpm (:3000)
ros2 launch navpromini_rmf_sim rmf_web.launch.py use_sim_time:=true start_workcells:=true
```

First Docker run may pull images; needs Docker + host network.

**Conflict with fleet-server:** if `navpro-rmf-api-sim` / `navpro-rmf-core-sim`
are running on host network they own `:8000` / `:8006` and `ROS_DOMAIN_ID=0`.
Either stop those containers, or use free ports + a different domain:

```bash
# stop only RMF containers (optional)
docker stop navpro-rmf-api-sim navpro-rmf-core-sim

# or isolate open-source stack
ROS_DOMAIN_ID=42 ros2 launch navpromini_rmf_sim rmf_web.launch.py web_mode:=local \
  websocket_port:=8016 \
  trajectory_url:=ws://localhost:8016 \
  api_url:=http://localhost:8010 \
  server_uri:=http://localhost:8010/_internal
```

Local mode starts API via `rmf-web/.venv` (`python -m api_server`) with a
fresh sqlite DB under `~/.cache/navpromini_rmf_api/` (avoids stale office-demo
door rows crashing pydantic), and the dashboard via the `vite` binary directly
(no `pnpm install` / `pnpm exec`).

## 3) Launch Gazebo simulation (fleet GUI separately)

Default is **Gazebo + robots + Nav2 + cafe RMF endpoints + building_map_server**.
Use `navpro-fleet-server` collab script to connect the fleet GUI.

```bash
source ~/NavProMini_ws/install/setup.bash
source ~/rmf_ws/install/setup.bash

# Cafe world + robot1/robot2 + Nav2 + doors/workcells + building map
ros2 launch navpromini_rmf_sim rmf_sim.launch.py start_nav:=true
```

Then in another terminal (one command — self-heals nginx / trajectory / robots):

```bash
cd /path/to/navpro-fleet-server
./scripts/sim/up.sh
```

Open `http://<LAN_IP>/` and hard-refresh once. (`./scripts/gui_up.sh` still works as a shim.)

Or Gazebo only (no building map / cafe infra):

```bash
ros2 launch navpromini_rmf_sim multi_robot_gazebo.launch.py world_name:=cafe
```

Optional RViz:

```bash
ros2 launch navpromini_rmf_sim rmf_sim.launch.py start_nav:=true use_rviz:=true
```

Defaults:

- World: `cafe` (transparent walls, ground plane)
- Robots: `robot1,robot2` at parking spawn poses (spawn delayed ~4 s)
- `building_map_server` on (for fleet GUI `/map`)
- Doors: Gazebo `libdoor` + `door_supervisor` (no Python DoorState stub)
- Workcells: `workcell_adapter` from nav graph GUIDs (`start_workcells:=true`)
- Nav2 on when using `start_nav:=true` (required for collab delivery tasks)

## 4) Sync + open fleet web GUI

```bash
# 1) Sync site assets into fleet-server
bash $(ros2 pkg prefix navpromini_rmf_sim)/share/navpromini_rmf_sim/scripts/sync_site_to_fleet_server.sh

# 2) Start fleet stack (if not already running)
cd /mnt/68185C18185BE39A/Botforge/Projects/navpro-fleet-server
docker compose up -d
docker compose restart rmf-core
```

### Open the GUI

1. Browser: **`http://<LAN_IP>/`** (from fleet-server `.env`, e.g. `http://192.168.0.101/`)
2. Login: **`admin` / `navpro-admin`** (defaults in `.env` — change in production)
3. Useful tabs:

| Tab                  | Use                                      |
| -------------------- | ---------------------------------------- |
| **Overview / Map**   | See building floorplan + robots          |
| **Building**         | Edit lanes/chargers → **Apply to fleet** |
| **Tasks**            | Dispatch loop / delivery tasks           |
| **Robots / Devices** | Commission / status                      |

### Sim + GUI together

```bash
# Terminal A — Gazebo robots
ros2 launch navpromini_rmf_sim rmf_sim.launch.py

# Terminal B — if host sim must talk to Docker RMF (DDS bridge)
cd /mnt/68185C18185BE39A/Botforge/Projects/navpro-fleet-server
./scripts/run_gui_rmf_demo.sh status   # check map/fleets
```

For host Gazebo ↔ Docker RMF discovery, use the same ROS domain / bridge pattern
as `navpro-fleet-server/scripts/run_gui_rmf_demo.sh`.

## Site layout

```text
navpromini_rmf_sim/
  adapters/                   # door / lift / workcell / fleet
  launch/
    include/adapters/         # demos-style adapter launches
    rmf_sim.launch.py
    rmf_web.launch.py
    rmf_fleet.launch.py       # → include/adapters/fleet_adapter.launch.py
  site/
    site.building.yaml        # traffic-editor output (source of truth)
    maps/L1/map.pgm + map.yaml
    maps/L1/floorplan.png
    nav_graphs/0.yaml
    fleet_config/navpromini.yaml
    spawn_poses.yaml
    generated/cafe.world
    generated/models/
```

## Notes

- If launch fails with `package 'navpromini_fleet' not found`, remove stale leftovers:
  `rm -rf ~/NavProMini_ws/{build,install}/navpromini_fleet` then re-`source install/setup.bash`.
- Lift uses Gazebo **`liblift`** + **`lift_supervisor`** ([book](https://osrf.github.io/ros2multirobotbook/integration_lifts.html)); set `lifts.Lift1.plugins: true` and regenerate.
- Generated world adds **opaque floor slabs** (L1/L2) with a **lift-shaft cutout** so the cabin floor is flush (no lip / no embedding on L2), plus a ground plane; walls stay **opaque**.
- Building **must** have a **measurement** on the **reference** level (`reference_level_name: L1`). Without it, world/nav graph stay in pixels.
- Multi-floor: add **two+ matching named fiducials** on L1 and L2 (same names, same pixel corners) so L2 inherits L1 scale. Empty-named fiducials crash traffic-editor.
- `L2.elevation` should be > 0 when using a lift (e.g. `3.0`).
- Door names must not be `"null"`.
- Robot names come from vertex `spawn_robot_name` (parking2→robot1, parking1→robot2).
- Sim `maps/L*/map.yaml` origin is set to `[0, -(H-1)*res, 0]` so Nav2 matches RMF/Gazebo (not the live SLAM origin).
- After every traffic-editor save, re-run `generate_site_assets` then relaunch
  (see **Update nav graphs after traffic-editor** at the top).
