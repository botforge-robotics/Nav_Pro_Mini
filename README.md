# NavProMini

ROS 2 **Jazzy** + Gazebo **Harmonic** workspace for the NavProMini differential-drive robot: simulation, teleop, SLAM mapping, and Nav2 navigation. The same topic/TF contract is intended for a **real robot** (ESP32 micro-ROS + lidar + odom).

---

## Packages

| Package | Description |
|---------|-------------|
| `navpromini_description` | URDF/xacro, meshes, sensors, DiffDrive plugins, RViz display |
| `navpromini_gazebo` | Gazebo Harmonic worlds, spawn, ros_gz bridge |
| `navpromini_teleop` | Joystick / keyboard teleoperation |
| `navpromini_mapping` | slam_toolbox online mapping + map saver |
| `navpromini_navigation` | Nav2 localization + navigation |

---

## Robot model (shared)

| Item | Value |
|------|--------|
| Drive | Differential drive |
| Wheel Ø | **65 mm** (`WHEEL_RADIUS ≈ 0.0325 m`) |
| Track | ~**0.187 m** (CAD) |
| Sim motors | **10 kg·cm**, **300 RPM** (Nav2 capped ~0.4 m/s) |
| Lidar | RPLIDAR A1M8-like `gpu_lidar` → `/scan`, frame `lidar_1` |
| Control | `/cmd_vel` (`geometry_msgs/Twist`) |

### TF tree

```text
map                         ← SLAM (mapping) or AMCL (navigation)
 └── odom                   ← Gazebo odom (sim) or wheel/odom node (real)
      └── base_link         ← robot base (+X forward)
           └── chassis
                ├── leftWheel_1 / rightWheel_1
                ├── casterWheel_1, FLLC_1, FRLC_1, BLC_1, FC_1
                └── lidar_1   ← LaserScan frame_id
```

---

## Dependencies & build

```bash
cd ~/NavProMini_ws
source /opt/ros/jazzy/setup.bash

sudo apt update
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install
source install/setup.bash
```

Key metas (if needed manually):

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-navigation2 \
  ros-jazzy-slam-toolbox ros-jazzy-teleop-twist-joy \
  ros-jazzy-teleop-twist-keyboard ros-jazzy-joy
```

---

## Launch files (all)

### 1. `navpromini_gazebo` → `gazebo.launch.py`

**Purpose:** Start Gazebo Harmonic, spawn NavProMini, bridge sim topics.

| Argument | Default | Description |
|----------|---------|-------------|
| `world_name` | `empty` | `empty` \| `office` \| `cafe` |
| `use_sim_time` | `true` | Use `/clock` from Gazebo |
| `use_rviz` | **`false`** | Optional companion RViz (`navpromini_sim.rviz`) |
| `x` `y` `z` `yaw` | `0 0 0.06 0` | Spawn pose (world defaults applied if left at defaults) |

**World spawn defaults** (when x/y/z still `0,0,0.06`):

| World | x | y | z |
|-------|---|---|---|
| `empty` | 0.0 | 0.0 | 0.06 |
| `office` | 0.0 | -6.5 | 0.06 |
| `cafe` | 0.0 | -3.0 | **0.28** (above cafe floor) |

**Nodes / includes**

| Name | Role |
|------|------|
| `gz_sim` (`ros_gz_sim`) | Gazebo Harmonic (`-r` run) |
| `robot_state_publisher` | URDF → `/robot_description`, `/tf_static` |
| `create` | Spawn model `NavProMini` |
| `parameter_bridge` | ros_gz bridge (`config/ros_gz_bridge.yaml`) |
| `rviz2` | Only if `use_rviz:=true` |

```bash
# Default: Gazebo only (no RViz)
ros2 launch navpromini_gazebo gazebo.launch.py world_name:=cafe

# With Gazebo RViz
ros2 launch navpromini_gazebo gazebo.launch.py world_name:=cafe use_rviz:=true
```

---

### 2. `navpromini_description` → `display.launch.py`

**Purpose:** RViz-only model check (no Gazebo).

| Argument | Default | Description |
|----------|---------|-------------|
| `gui` | `True` | Joint state GUI vs plain JSP |

**Nodes:** `robot_state_publisher`, `joint_state_publisher(_gui)`, `rviz2`

```bash
ros2 launch navpromini_description display.launch.py
```

---

### 3. `navpromini_teleop` → `joystick.launch.py`

**Purpose:** Gamepad → `/cmd_vel`.

| Argument | Default | Description |
|----------|---------|-------------|
| `joy_config` | `xbox` | `xbox` or `ps4` |
| `joy_dev` | `0` | `/dev/input/js0` |
| `cmd_vel_topic` | `cmd_vel` | Remap target |
| `use_sim_time` | `true` | Set `false` on real robot |

**Nodes:** `joy_node` → `/joy`; `teleop_twist_joy_node` → `/cmd_vel`  
**Controls:** hold **LB** to enable; **RB** turbo.

```bash
ros2 launch navpromini_teleop joystick.launch.py
ros2 launch navpromini_teleop joystick.launch.py joy_config:=ps4 use_sim_time:=false
```

---

### 4. `navpromini_teleop` → `keyboard.launch.py`

| Argument | Default |
|----------|---------|
| `cmd_vel_topic` | `cmd_vel` |
| `use_sim_time` | `true` |

**Node:** `teleop_twist_keyboard`

```bash
ros2 launch navpromini_teleop keyboard.launch.py
# real robot:
ros2 launch navpromini_teleop keyboard.launch.py use_sim_time:=false
```

---

### 5. `navpromini_mapping` → `slam.launch.py`

**Purpose:** Online async SLAM (`slam_toolbox`).

| Argument | Default | Description |
|----------|---------|-------------|
| `use_sim_time` | `true` | |
| `autostart` | `true` | Auto activate lifecycle |
| `use_lifecycle_manager` | `false` | |
| `slam_params_file` | `mapper_params_online_async.yaml` | |
| `use_rviz` | `true` | Mapping RViz |

**Nodes:** `async_slam_toolbox_node`, optional `rviz2`

**Frames / topics:** `odom_frame=odom`, `map_frame=map`, `base_frame=base_link`, `scan_topic=/scan`

```bash
ros2 launch navpromini_mapping slam.launch.py
```

---

### 6. `navpromini_mapping` → `map_saver.launch.py`

**Purpose:** Save SLAM map to disk.

| Argument | Default | Description |
|----------|---------|-------------|
| `map_name` | `navpromini_map` | Bare name (no path/extension) |

**Output:** `~/NavProMini_ws/src/navpromini_mapping/maps/<map_name>.pgm` + `.yaml`

```bash
ros2 launch navpromini_mapping map_saver.launch.py map_name:=cafe
```

---

### 7. `navpromini_navigation` → `navigation.launch.py`

**Purpose:** Full Nav2 (localization + navigation) + optional RViz.

| Argument | Default | Description |
|----------|---------|-------------|
| `map_name` | `navpromini_map` | Bare name → resolved under mapping `maps/` |
| `use_sim_time` | `true` | **`true` for Gazebo; `false` for real robot** |
| `autostart` | `true` | Lifecycle autostart |
| `params_file` | `config/nav2_params.yaml` | |
| `use_rviz` | **`true`** | Nav RViz (map, costmaps, plans) |

**Includes:** `nav2_bringup/bringup_launch.py` (`slam:=False`, localization on)

**Typical Nav2 nodes**

| Node | Role |
|------|------|
| `map_server` | Load map → `/map` |
| `amcl` | Localization → `map`→`odom` TF |
| `lifecycle_manager_localization` | Activate map_server + amcl |
| `controller_server` | MPPI local controller |
| `planner_server` | Global NavFn planner |
| `smoother_server` | Path smoother |
| `behavior_server` | Spin / backup / wait / … |
| `bt_navigator` | Behavior trees |
| `waypoint_follower` | Waypoints |
| `velocity_smoother` | Smooth cmd → `cmd_vel_smoothed` |
| `collision_monitor` | Safety filter → `/cmd_vel` |
| `route_server` / `docking_server` | Optional Nav2 extras |
| `lifecycle_manager_navigation` | Activate navigation stack |
| `rviz2` | If `use_rviz:=true` |

**RViz displays (navigation.rviz):** `/map`, `/global_costmap/costmap`, `/local_costmap/costmap`, `/plan` (global), `/local_plan` (local), `/scan`, robot model, AMCL pose/particles.

```bash
# Simulation (after Gazebo)
ros2 launch navpromini_navigation navigation.launch.py \
  map_name:=cafe use_sim_time:=true

# Real robot
ros2 launch navpromini_navigation navigation.launch.py \
  map_name:=cafe use_sim_time:=false
```

In RViz (`Fixed Frame: map`):

1. **2D Pose Estimate** → `/initialpose` (required for AMCL)  
2. **2D Goal Pose** → `/goal_pose`  

---

### 8. `navpromini_navigation` → `localization.launch.py`

**Purpose:** Map + AMCL only (no planner/controller).

| Argument | Default |
|----------|---------|
| `map_name` | `navpromini_map` |
| `use_sim_time` | `true` |
| `autostart` | `true` |
| `params_file` | `nav2_params.yaml` |

**Nodes (via Nav2 localization bringup):** `map_server`, `amcl`, lifecycle manager.

```bash
ros2 launch navpromini_navigation localization.launch.py map_name:=cafe
```

---

## Topics (main)

### Simulation bridge (`ros_gz_bridge.yaml`)

| Topic | Type | Direction |
|-------|------|-----------|
| `/clock` | `rosgraph_msgs/Clock` | GZ → ROS |
| `/cmd_vel` | `geometry_msgs/Twist` | ROS → GZ |
| `/odom` | `nav_msgs/Odometry` | GZ → ROS (ground truth) |
| `/tf` | `tf2_msgs/TFMessage` | GZ → ROS (`odom`→`base_link`) |
| `/joint_states` | `sensor_msgs/JointState` | GZ → ROS |
| `/scan` | `sensor_msgs/LaserScan` | GZ → ROS |

> Gazebo also has `/odom_wheels` (open-loop DiffDrive odom) — **not bridged** to ROS. Nav2 uses bridged `/odom`.

### Navigation / SLAM extras

| Topic | Notes |
|-------|--------|
| `/map` | Occupancy grid (slam_toolbox or map_server) |
| `/amcl_pose` | Localized pose |
| `/particlecloud` | AMCL particles |
| `/plan` | Global path |
| `/local_plan` | Local / controller path |
| `/global_costmap/costmap` | Global costmap |
| `/local_costmap/costmap` | Local costmap |
| `/cmd_vel_nav` | Often used internally by controller |
| `/cmd_vel_smoothed` | Into collision_monitor |
| `/cmd_vel` | Final command out to robot / Gazebo |
| `/initialpose` | RViz 2D Pose Estimate |
| `/goal_pose` | RViz 2D Goal Pose |
| `/joy` | Joystick |

### ESP32 micro-ROS firmware (real robot)

| Topic | Dir | Notes |
|-------|-----|--------|
| `cmd_vel` | sub | Wheel speed / motor control |
| `display_text` | sub | OLED string |
| `led_strip` / `led_command` | sub | WS2812 |
| `imu` | pub | QMI8658 |

> Firmware currently focuses on **cmd_vel motors**; full Nav2 on hardware still needs **`/odom` + `odom`→`base_link` TF** and a lidar `/scan` source on the PC (or bridged).

---

## Frames cheat sheet

| Frame | Published by (sim) | Published by (real — typical) |
|-------|--------------------|--------------------------------|
| `map` | slam_toolbox / AMCL | same |
| `odom` | Gazebo odometry + bridge | wheel odom node / ESP32 |
| `base_link` | Gazebo TF / RSP child | robot odom TF |
| `lidar_1` | URDF static | URDF / static TF |
| `chassis`, wheels, … | `robot_state_publisher` | same |

---

## Simulation workflows

### A) Drive in Gazebo

```bash
# T1 — Gazebo (RViz off by default)
ros2 launch navpromini_gazebo gazebo.launch.py world_name:=cafe

# T2 — teleop
source ~/NavProMini_ws/install/setup.bash
ros2 launch navpromini_teleop joystick.launch.py
```

### B) Map an environment

```bash
# T1 gazebo, T2 teleop, then:
ros2 launch navpromini_mapping slam.launch.py
# drive around …
ros2 launch navpromini_mapping map_saver.launch.py map_name:=cafe
```

### C) Navigate on a map

```bash
# T1
ros2 launch navpromini_gazebo gazebo.launch.py world_name:=cafe

# T2
source ~/NavProMini_ws/install/setup.bash
ros2 launch navpromini_navigation navigation.launch.py \
  map_name:=cafe use_sim_time:=true
```

1. Set **2D Pose Estimate** in Nav RViz  
2. Set **2D Goal Pose**  
3. Watch global plan (red), local plan (green), costmaps  

### Quick checks (sim)

```bash
ros2 topic echo /clock --once
ros2 topic echo /odom --once
ros2 topic echo /scan --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map odom   # after initial pose
```

---

## Real robot workflows

### Required interfaces on the ROS PC

| Interface | Required for |
|-----------|----------------|
| `/cmd_vel` → robot base | Teleop / Nav2 |
| `/scan` (`frame_id` matching lidar TF) | SLAM / AMCL / costmaps |
| `/odom` + TF `odom`→`base_link` | Localization / Nav2 |
| `/tf` `/tf_static` | Full tree (`base_link`→`lidar_1`) |
| `use_sim_time:=false` | All launches |

### Teleop only

```bash
# Bring up: micro-ROS agent + ESP32 firmware (motors)
# Then:
ros2 launch navpromini_teleop keyboard.launch.py use_sim_time:=false
```

### SLAM on real robot

```bash
# Provide /scan + odom TF, then:
ros2 launch navpromini_mapping slam.launch.py use_sim_time:=false
ros2 launch navpromini_mapping map_saver.launch.py map_name:=home
```

### Nav2 on real robot

```bash
ros2 launch navpromini_navigation navigation.launch.py \
  map_name:=home use_sim_time:=false

# Optional RViz off:
#   … use_rviz:=false
```

Then **2D Pose Estimate** → **2D Goal Pose** (same as sim).

### Common failure modes

| Symptom | Likely cause |
|---------|----------------|
| `Invalid frame ID "odom"` | No odometry TF / sim not running / wrong `use_sim_time` |
| AMCL “set the initial pose” | Need RViz **2D Pose Estimate** |
| Costmaps empty | No `/scan` or TF to `lidar_1` |
| Nav stuck / no motion | No `/cmd_vel` consumer (agent / Gazebo) |
| Time / TF “frozen” | `use_sim_time:=true` without `/clock` |

---

## Typical flow diagram

```text
colcon build && source install/setup.bash
        │
        ├─ SIM ── gazebo.launch.py  [use_rviz:=false by default]
        │           use_rviz:=true  → optional Gazebo RViz
        │
        ├─ TELEOP ── joystick / keyboard  → /cmd_vel
        │
        ├─ MAP ── slam.launch.py → map_saver.launch.py
        │
        └─ NAV ── navigation.launch.py  [use_rviz:=true by default]
                    Fixed Frame: map
                    2D Pose Estimate → 2D Goal Pose
                    Displays: map, local/global costmaps, local/global plans
```

---

## Notes

- Cafe floor is raised; launch auto-spawns near `z≈0.28` for `world_name:=cafe`.
- Lidar min range **0.35 m** reduces body self-hits.
- First `office` / `cafe` launch downloads Gazebo Fuel models (needs internet); later uses `~/.gz/fuel/`.
- Map path resolution for Nav2: `~/NavProMini_ws/src/navpromini_mapping/maps/<map_name>.yaml` (then package share dirs).
- Nav2 motion limits (params): ~**0.4 m/s** linear, ~**1.5 rad/s** angular (velocity smoother).

---

## License

Apache-2.0 (package defaults; declare per package as needed).
