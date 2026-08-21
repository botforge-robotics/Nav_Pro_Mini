# NavProMini

ROS 2 **Jazzy** + Gazebo **Harmonic** workspace for the NavProMini differential-drive robot: simulation, teleop, SLAM mapping, Nav2 navigation, and **Mission Planner** (Botforge rosbridge + React web UI). The same topic/TF contract is intended for a **real robot** (ESP32 micro-ROS + lidar + odom).

---

## Packages

| Package                       | Description                                                              |
| ----------------------------- | ------------------------------------------------------------------------ |
| `navpromini_description`      | URDF/xacro, meshes, sensors, DiffDrive plugins, RViz display             |
| `navpromini_gazebo`           | Gazebo Harmonic worlds, spawn, ros_gz bridge                             |
| `navpromini_teleop`           | Joystick / keyboard teleoperation                                        |
| `navpromini_mapping`          | slam_toolbox online mapping + map saver                                  |
| `navpromini_navigation`       | Nav2 localization + navigation                                           |
| `navpromini_controller`       | **Real robot:** micro-ROS agent, RPLidar, wheel odom, bringup            |
| `navpromini_setup`            | Pi install, Wi‑Fi portal, systemd (robot / display / mission planner)    |
| `navpromini_mission_planner`  | Launch wrappers for Mission Planner (`map` → `map_name`)                 |
| `nav2_mission_planner`        | Companion services (`launch_with_args`, map list/delete, save map)       |
| `nav2_mission_planner_interfaces` | Custom srv/action types for Mission Planner                          |
| `rosbridge_suite`             | Botforge fork — WebSocket bridge on port **9090** (not apt rosbridge)    |

---

## Robot model (shared)

| Item         | Value                                                          |
| ------------ | -------------------------------------------------------------- |
| Drive        | Differential drive                                             |
| Wheel radius | **0.0325 m** (65 mm dia; URDF + firmware + odom)               |
| Track        | **0.225 m** (22.5 cm; URDF + firmware + odom)                  |
| Encoder      | **1470** ticks/rev → **≈7198.7** ticks/m                       |
| Sim motors   | **10 kg·cm**, **300 RPM** (Nav2 capped ~0.4 m/s)               |
| Lidar        | RPLIDAR A1M8 → `/scan`, frame `lidar_1`                        |
| Control      | `/cmd_vel` (`geometry_msgs/Twist`)                             |
| Encoders     | 420 ticks/rev → `/joint_states` (ESP32) → `/odom` (controller) |

### TF tree

```text
map                         ← SLAM (mapping) or AMCL (navigation)
 └── odom                   ← Gazebo odom (sim) or wheel odom node (real)
      └── base_link         ← robot base (+X forward)
           ├── imu_link     ← QMI8658 (real)
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
  ros-jazzy-teleop-twist-keyboard ros-jazzy-joy \
  ros-jazzy-rplidar-ros
```

---

## Launch files (all)

### 0. `navpromini_controller` → `robot.launch.py` (real robot)

**Purpose:** Bring up hardware on Raspberry Pi 5 + [Waveshare General Driver](https://www.waveshare.com/wiki/General_Driver_for_Robots) + RPLidar.

| Argument                                     | Default          | Description                                    |
| -------------------------------------------- | ---------------- | ---------------------------------------------- |
| `use_sim_time`                               | `false`          | Always false on real robot                     |
| `microros_port`                              | `/dev/ttyAMA0`   | Pi 5 GPIO UART to ESP32 (Pi 4: `/dev/serial0`) |
| `lidar_port`                                 | `/dev/rplidar`   | RPLidar CP2102 stable udev device              |
| `lidar_frame`                                | `lidar_1`        | Must match URDF                                |
| `start_agent` / `start_lidar` / `start_odom` | `true`           | Toggle subsystems                              |
| `start_slam`                                 | `false`          | Include `navpromini_mapping`                   |
| `start_nav`                                  | `false`          | Include Nav2                                   |
| `map_name`                                   | `navpromini_map` | Used when `start_nav:=true`                    |
| `use_rviz`                                   | `false`          | For slam/nav includes                          |

**Nodes:** `robot_state_publisher`, native micro-ROS agent, `rplidar_composition`, `odom_node`, Daly `battery_node`.

```bash
# Hardware only
ros2 launch navpromini_controller robot.launch.py

# + SLAM
ros2 launch navpromini_controller robot.launch.py start_slam:=true use_rviz:=true

# + Nav2 on a saved map
ros2 launch navpromini_controller robot.launch.py \
  start_nav:=true map_name:=home use_rviz:=true
```

Also: `microros_agent.launch.py`, `rplidar.launch.py`, `odom.launch.py`, `battery.launch.py`.

#### Battery (Daly Smart BMS Blue, FTDI USB–RS485)

Port: `/dev/battery_bms` @ 9600 8N1 (classic Daly UART/485 `0xA5` protocol; Modbus fallback).

| Topic                   | Type                              | Content                                      |
| ----------------------- | --------------------------------- | -------------------------------------------- |
| `/battery/state`        | `sensor_msgs/BatteryState`        | V, I, SOC%, charge status, cell voltages     |
| `/battery/soc`          | `std_msgs/Float32`                | SOC percent 0–100                            |
| `/battery/voltage`      | `std_msgs/Float32`                | Pack voltage (V)                             |
| `/battery/current`      | `std_msgs/Float32`                | Current (A; +charge / −discharge)            |
| `/battery/cells`        | `std_msgs/Float32MultiArray`      | Per-cell voltages (V)                        |
| `/battery/temperatures` | `std_msgs/Float32MultiArray`      | NTC temperatures (°C)                        |
| `/battery/info`         | `std_msgs/String`                 | Full JSON snapshot (MOS, charger, faults, …) |
| `/diagnostics`          | `diagnostic_msgs/DiagnosticArray` | Human-readable BMS status                    |

```bash
ros2 topic echo /battery/state --once
ros2 topic echo /battery/info --once
```

---

### 1. `navpromini_gazebo` → `gazebo.launch.py`

**Purpose:** Start Gazebo Harmonic, spawn NavProMini, bridge sim topics.

| Argument          | Default      | Description                                             |
| ----------------- | ------------ | ------------------------------------------------------- |
| `world_name`      | `empty`      | `empty`                                                 |
| `use_sim_time`    | `true`       | Use `/clock` from Gazebo                                |
| `use_rviz`        | `false`      | Optional companion RViz (`navpromini_sim.rviz`)         |
| `x` `y` `z` `yaw` | `0 0 0.06 0` | Spawn pose (world defaults applied if left at defaults) |

**World spawn defaults** (when x/y/z still `0,0,0.06`):

| World    | x   | y    | z                           |
| -------- | --- | ---- | --------------------------- |
| `empty`  | 0.0 | 0.0  | 0.06                        |
| `office` | 0.0 | -6.5 | 0.06                        |
| `cafe`   | 0.0 | -3.0 | **0.28** (above cafe floor) |

**Nodes / includes**

| Name                    | Role                                        |
| ----------------------- | ------------------------------------------- |
| `gz_sim` (`ros_gz_sim`) | Gazebo Harmonic (`-r` run)                  |
| `robot_state_publisher` | URDF → `/robot_description`, `/tf_static`   |
| `create`                | Spawn model `NavProMini`                    |
| `parameter_bridge`      | ros_gz bridge (`config/ros_gz_bridge.yaml`) |
| `rviz2`                 | Only if `use_rviz:=true`                    |

```bash
# Default: Gazebo only (no RViz)
ros2 launch navpromini_gazebo gazebo.launch.py world_name:=cafe

# With Gazebo RViz
ros2 launch navpromini_gazebo gazebo.launch.py world_name:=cafe use_rviz:=true
```

---

### 2. `navpromini_description` → `display.launch.py`

**Purpose:** RViz-only model check (no Gazebo).

| Argument | Default | Description                  |
| -------- | ------- | ---------------------------- |
| `gui`    | `True`  | Joint state GUI vs plain JSP |

**Nodes:** `robot_state_publisher`, `joint_state_publisher(_gui)`, `rviz2`

```bash
ros2 launch navpromini_description display.launch.py
```

---

### 3. `navpromini_teleop` → `joystick.launch.py`

**Purpose:** Gamepad → `/cmd_vel`.

| Argument        | Default   | Description               |
| --------------- | --------- | ------------------------- |
| `joy_config`    | `xbox`    | `xbox` or `ps4`           |
| `joy_dev`       | `0`       | `/dev/input/js0`          |
| `cmd_vel_topic` | `cmd_vel` | Remap target              |
| `use_sim_time`  | `true`    | Set `false` on real robot |

**Nodes:** `joy_node` → `/joy`; `teleop_twist_joy_node` → `/cmd_vel`
**Controls:** hold **LB** to enable; **RB** turbo.

```bash
ros2 launch navpromini_teleop joystick.launch.py
ros2 launch navpromini_teleop joystick.launch.py joy_config:=ps4 use_sim_time:=false
```

---

### 4. `navpromini_teleop` → `keyboard.launch.py`

| Argument        | Default   |
| --------------- | --------- |
| `cmd_vel_topic` | `cmd_vel` |
| `use_sim_time`  | `true`    |

**Node:** `teleop_twist_keyboard`

```bash
ros2 launch navpromini_teleop keyboard.launch.py
# real robot:
ros2 launch navpromini_teleop keyboard.launch.py use_sim_time:=false
```

---

### 5. `navpromini_mapping` → `slam.launch.py`

**Purpose:** Online async SLAM (`slam_toolbox`).

| Argument                | Default                           | Description             |
| ----------------------- | --------------------------------- | ----------------------- |
| `use_sim_time`          | `true`                            |                         |
| `autostart`             | `true`                            | Auto activate lifecycle |
| `use_lifecycle_manager` | `false`                           |                         |
| `slam_params_file`      | `mapper_params_online_async.yaml` |                         |
| `use_rviz`              | `true`                            | Mapping RViz            |

**Nodes:** `async_slam_toolbox_node`, optional `rviz2`

**Frames / topics:** `odom_frame=odom`, `map_frame=map`, `base_frame=base_link`, `scan_topic=/scan`

```bash
ros2 launch navpromini_mapping slam.launch.py
```

---

### 6. `navpromini_mapping` → `map_saver.launch.py`

**Purpose:** Save SLAM map to disk.

| Argument   | Default          | Description                   |
| ---------- | ---------------- | ----------------------------- |
| `map_name` | `navpromini_map` | Bare name (no path/extension) |

**Output:** `~/NavProMini_ws/src/navpromini_mapping/maps/<map_name>.pgm` + `.yaml`

```bash
ros2 launch navpromini_mapping map_saver.launch.py map_name:=cafe
```

---

### 7. `navpromini_navigation` → `navigation.launch.py`

**Purpose:** Full Nav2 (localization + navigation) + optional RViz.

| Argument       | Default                   | Description                                       |
| -------------- | ------------------------- | ------------------------------------------------- |
| `map_name`     | `navpromini_map`          | Bare name → resolved under mapping `maps/`        |
| `use_sim_time` | `true`                    | `true` **for Gazebo;** `false` **for real robot** |
| `autostart`    | `true`                    | Lifecycle autostart                               |
| `params_file`  | `config/nav2_params.yaml` |                                                   |
| `use_rviz`     | `true`                    | Nav RViz (map, costmaps, plans)                   |

**Includes:** `nav2_bringup/bringup_launch.py` (`slam:=False`, localization on)

**Typical Nav2 nodes**

| Node                              | Role                            |
| --------------------------------- | ------------------------------- |
| `map_server`                      | Load map → `/map`               |
| `amcl`                            | Localization → `map`→`odom` TF  |
| `lifecycle_manager_localization`  | Activate map_server + amcl      |
| `controller_server`               | MPPI local controller           |
| `planner_server`                  | Global NavFn planner            |
| `smoother_server`                 | Path smoother                   |
| `behavior_server`                 | Spin / backup / wait / …        |
| `bt_navigator`                    | Behavior trees                  |
| `waypoint_follower`               | Waypoints                       |
| `velocity_smoother`               | Smooth cmd → `cmd_vel_smoothed` |
| `collision_monitor`               | Safety filter → `/cmd_vel`      |
| `route_server` / `docking_server` | Optional Nav2 extras            |
| `lifecycle_manager_navigation`    | Activate navigation stack       |
| `rviz2`                           | If `use_rviz:=true`             |

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

| Argument       | Default            |
| -------------- | ------------------ |
| `map_name`     | `navpromini_map`   |
| `use_sim_time` | `true`             |
| `autostart`    | `true`             |
| `params_file`  | `nav2_params.yaml` |

**Nodes (via Nav2 localization bringup):** `map_server`, `amcl`, lifecycle manager.

```bash
ros2 launch navpromini_navigation localization.launch.py map_name:=cafe
```

---

## Topics (main)

### Simulation bridge (`ros_gz_bridge.yaml`)

| Topic           | Type                     | Direction                     |
| --------------- | ------------------------ | ----------------------------- |
| `/clock`        | `rosgraph_msgs/Clock`    | GZ → ROS                      |
| `/cmd_vel`      | `geometry_msgs/Twist`    | ROS → GZ                      |
| `/odom`         | `nav_msgs/Odometry`      | GZ → ROS (ground truth)       |
| `/tf`           | `tf2_msgs/TFMessage`     | GZ → ROS (`odom`→`base_link`) |
| `/joint_states` | `sensor_msgs/JointState` | GZ → ROS                      |
| `/scan`         | `sensor_msgs/LaserScan`  | GZ → ROS                      |

> Gazebo also has `/odom_wheels` (open-loop DiffDrive odom) — **not bridged** to ROS. Nav2 uses bridged `/odom`.

### Navigation / SLAM extras

| Topic                     | Notes                                       |
| ------------------------- | ------------------------------------------- |
| `/map`                    | Occupancy grid (slam_toolbox or map_server) |
| `/amcl_pose`              | Localized pose                              |
| `/particlecloud`          | AMCL particles                              |
| `/plan`                   | Global path                                 |
| `/local_plan`             | Local / controller path                     |
| `/global_costmap/costmap` | Global costmap                              |
| `/local_costmap/costmap`  | Local costmap                               |
| `/cmd_vel_nav`            | Often used internally by controller         |
| `/cmd_vel_smoothed`       | Into collision_monitor                      |
| `/cmd_vel`                | Final command out to robot / Gazebo         |
| `/initialpose`            | RViz 2D Pose Estimate                       |
| `/goal_pose`              | RViz 2D Goal Pose                           |
| `/joy`                    | Joystick                                    |

### ESP32 micro-ROS firmware (real robot)

| Topic                       | Dir | Notes                                       |
| --------------------------- | --- | ------------------------------------------- |
| `cmd_vel`                   | sub | Diff-drive; **500 ms** timeout stops motors |
| `display_text`              | sub | OLED string                                 |
| `led_strip` / `led_command` | sub | WS2812                                      |
| `imu`                       | pub | QMI8658, `imu_link`                         |
| `joint_states`              | pub | `LeftWheelJoint` / `RightWheelJoint` (rad)  |

Host `navpromini_controller`: `joint_states` → `/odom` + TF `odom`→`base_link`; RPLidar → `/scan`.

---

## Frames cheat sheet

| Frame                | Published by (sim)       | Published by (real — typical) |
| -------------------- | ------------------------ | ----------------------------- |
| `map`                | slam_toolbox / AMCL      | same                          |
| `odom`               | Gazebo odometry + bridge | wheel odom node / ESP32       |
| `base_link`          | Gazebo TF / RSP child    | robot odom TF                 |
| `lidar_1`            | URDF static              | URDF / static TF              |
| `chassis`, wheels, … | `robot_state_publisher`  | same                          |

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

**Hardware:** Raspberry Pi 5 + Waveshare General Driver (ESP32 stacked, GPIO UART) + RPLidar USB + NavProMini firmware (flash via Type-C).

### Bringup

```bash
# Enable UART (once): raspi-config → Serial → console No, hardware Yes
# Flash ESP32 over Type-C, then disconnect monitor and stack/run on Pi UART.

cd ~/NavProMini_ws && source install/setup.bash
ros2 launch navpromini_controller robot.launch.py
```

### Teleop

```bash
ros2 launch navpromini_teleop keyboard.launch.py use_sim_time:=false
# or joystick.launch.py use_sim_time:=false
```

### SLAM

```bash
# On Pi — prefer use_rviz:=false and open RViz on your PC (see Multi-machine below)
ros2 launch navpromini_controller robot.launch.py start_slam:=true use_rviz:=false
# drive, then:
ros2 launch navpromini_mapping map_saver.launch.py map_name:=home
```

### Nav2

```bash
ros2 launch navpromini_controller robot.launch.py \
  start_nav:=true map_name:=home use_rviz:=false
```

Then on the PC in RViz: **2D Pose Estimate** → **2D Goal Pose**.

### Quick checks

```bash
ros2 topic echo /joint_states --once
ros2 topic echo /odom --once
ros2 topic echo /scan --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link lidar_1
```

### Common failure modes

| Symptom                                                       | Likely cause                                                                                 |
| ------------------------------------------------------------- | -------------------------------------------------------------------------------------------- |
| No micro-ROS topics                                           | Agent not on `/dev/ttyAMA0`, UART disabled, or firmware waiting for agent                    |
| `/imu` visible but empty `/odom` / no `/joint_states` on host | `ROS_DOMAIN_ID` mismatch — launch inherits the shell value; it must match the ESP32 firmware |
| `Invalid frame ID "odom"`                                     | odom node not running / no `/joint_states`                                                   |
| Costmaps empty                                                | No `/scan` or wrong `frame_id` (must be `lidar_1`)                                           |
| Robot keeps driving                                           | Old firmware without cmd_vel timeout — reflash                                               |
| Lidar / ESP32 port clash                                      | Lidar is CP2102 (`/dev/rplidar`); ESP32 uses GPIO UART (`ttyAMA0`)                           |

---

## Multi-machine: robot on Pi, RViz on PC

Run the robot stack on the **Raspberry Pi 5**. Run **RViz** (and optional teleop) on your **PC**. Both machines must be on the same LAN/Wi‑Fi with matching ROS 2 domain settings.

```text
Raspberry Pi 5                          Your PC
─────────────────                       ─────────────────
robot.launch.py                         rviz2 (+ optional teleop)
  micro-ROS agent
  lidar, odom, RSP
  (optional slam / nav)                 same Wi‑Fi / LAN
         │                                     │
         └──────── ROS 2 DDS discovery ────────┘
```

### 1. Network

1. Put Pi and PC on the **same Wi‑Fi or Ethernet** (avoid guest / AP-isolation SSIDs).
2. Get IPs and ping both ways:

```bash
 hostname -I
 ping <pi_ip>
 ping <pc_ip>
```

### 2. ROS 2 environment (both machines)

In **every** terminal on Pi and PC before launching:

```bash
source /opt/ros/jazzy/setup.bash
source ~/NavProMini_ws/install/setup.bash

export ROS_DOMAIN_ID=0           # must match ESP32 micro-ROS (firmware default 0)
# After firmware sets domain 42, use 42 on Pi + PC instead.
export ROS_LOCALHOST_ONLY=0      # allow discovery over the network
# If set, also unset localhost-only discovery:
# unset ROS_AUTOMATIC_DISCOVERY_RANGE
```

Optional: add the two `export` lines to `~/.bashrc` on both machines.

**Firewall:** DDS needs UDP multicast / high ports. For a quick test you can allow the other host or temporarily disable `ufw`. Build/install the workspace on the PC too (at least `navpromini_description` + mapping/navigation RViz configs).

### 3. Verify discovery

On Pi (after robot bringup) and on PC:

```bash
ros2 topic list
```

Both should show the same topics (`/scan`, `/odom`, `/tf`, `/joint_states`, …). From the PC:

```bash
ros2 topic hz /scan
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_link
```

If the PC topic list is empty: check `ROS_DOMAIN_ID`, `ROS_LOCALHOST_ONLY`, Wi‑Fi AP isolation, and firewall. Try Ethernet once to rule out Wi‑Fi.

### 4. Pi — robot bringup (no RViz)

```bash
# Pi
source /opt/ros/jazzy/setup.bash
source ~/NavProMini_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

ros2 launch navpromini_controller robot.launch.py use_rviz:=false

# With SLAM or Nav2 still on the Pi (recommended — PC only visualizes):
ros2 launch navpromini_controller robot.launch.py start_slam:=true use_rviz:=false
ros2 launch navpromini_controller robot.launch.py \
  start_nav:=true map_name:=home use_rviz:=false
```

### 5. PC — RViz and teleop

```bash
# PC
source /opt/ros/jazzy/setup.bash
source ~/NavProMini_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

rviz2
```

In RViz:

| Setting / display     | Value                                       |
| --------------------- | ------------------------------------------- |
| Fixed Frame           | `odom` (or `map` if SLAM / Nav2 is running) |
| RobotModel            | from `/robot_description`                   |
| TF                    | show tree                                   |
| LaserScan             | topic `/scan`                               |
| Odometry              | topic `/odom`                               |
| Map / Path / Costmaps | when Nav2 or SLAM is active                 |

Nav2 tools on PC: **2D Pose Estimate** → `/initialpose`, **2D Goal Pose** → `/goal_pose`.

Teleop from the PC:

```bash
ros2 launch navpromini_teleop keyboard.launch.py use_sim_time:=false
# or joystick.launch.py use_sim_time:=false
```

**Do not** start a second slam/nav stack on the PC — only RViz (and teleop). Let the Pi own hardware + SLAM/Nav2.

### 6. Time sync

Keep clocks close (NTP) on both machines or TF may show extrapolation errors:

```bash
timedatectl status
sudo timedatectl set-ntp true
```

### Checklist

| Check                     | Pi  | PC  |
| ------------------------- | --- | --- |
| Same Wi‑Fi / LAN, ping OK | ✓   | ✓   |
| Same `ROS_DOMAIN_ID`      | ✓   | ✓   |
| `ROS_LOCALHOST_ONLY=0`    | ✓   | ✓   |
| `robot.launch.py`         | ✓   | ✗   |
| `rviz2`                   | ✗   | ✓   |
| `use_sim_time:=false`     | ✓   | ✓   |

### Multi-machine failure modes

| Symptom                    | Likely cause                                                           |
| -------------------------- | ---------------------------------------------------------------------- |
| PC `ros2 topic list` empty | Different domain ID, `ROS_LOCALHOST_ONLY=1`, firewall, or AP isolation |
| RViz “No tf data”          | Odom not running on Pi; wrong Fixed Frame                              |
| Scan empty in RViz         | Lidar not up on Pi; check `ros2 topic hz /scan` from PC                |
| Laggy RViz / TF jumps      | Weak Wi‑Fi; keep Nav2 on Pi; sync clocks (NTP)                         |
| Transform extrapolation    | Clock skew between Pi and PC                                           |

---

## Mission Planner (React web UI)

Browser UI for teleop, SLAM, Nav2, and missions. **Rosbridge runs on the robot** (Botforge fork in this workspace). The PC Docker stack serves the React app **and a SQLite API** (claim + settings). Opens on the home screen → scan/claim robot (no connection wizard). Topics/launches are locked to NavProMini defaults.

```text
Browser  →  Docker React UI (PC :8080) + API/SQLite (:3001)
Browser  →  ws://<claimed-robot-ip>:9090  (Botforge rosbridge)
                →  nav2_mission_planner services + Nav2 / SLAM / twist_mux
```

### Robot (one-time)

```bash
# Prefer source build of Botforge rosbridge (already under src/rosbridge_suite)
sudo apt remove ros-jazzy-rosbridge* ros-jazzy-rosapi* || true

cd ~/NavProMini_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash

# ROS_LOCALHOST_ONLY must be 0 (default in navpromini_setup env / systemd)
export ROS_LOCALHOST_ONLY=0

# Hardware bringup (systemd: navpro-robot) then companion stack:
ros2 launch nav2_mission_planner nav2_mission_planner.launch.py
# or: sudo systemctl start navpro-mission-planner
```

Re-run Pi install to enable the unit: `sudo bash src/navpromini_setup/scripts/install_navpro.sh`  
(see [`navpromini_setup/README.md`](navpromini_setup/README.md)).

### PC (Docker UI)

From the Mission Planner **`react-web`** branch:

```bash
cd /path/to/nav2_mission_planner   # React web app repo
docker compose up --build -d
# open http://localhost:8080 → Scan nearby / Claim (or enter robot IP)
# Topics & launches are locked to NavProMini defaults (SQLite stores claim + missions)
```

### NavProMini locked defaults (app)

| Setting            | Value                                              |
| ------------------ | -------------------------------------------------- |
| Mapping launch     | `navpromini_mission_planner/mapping_launch`        |
| Navigation launch  | `navpromini_mission_planner/navigation_launch`     |
| Maps path          | `navpromini_mapping/maps`                          |
| cmd_vel            | `/cmd_vel_teleop`                                  |
| Twist type         | `geometry_msgs/msg/Twist`                          |
| Pose (nav)         | `/amcl_pose`                                       |
| Path               | `/plan`                                            |
| Lidar              | `/scan`                                            |
| Camera             | empty / disabled                                   |

Wrappers follow the Mission Planner docs: mapping = Nav2 + slam_toolbox; navigation accepts `map:=office.yaml` and joins `navpromini_mapping/maps/<file>`.

---

## Typical flow diagram

```text
colcon build && source install/setup.bash
        │
        ├─ REAL ── navpromini_controller/robot.launch.py  [use_sim_time:=false]
        │           optional start_slam / start_nav  (use_rviz:=false on Pi)
        │           PC: RViz + teleop over ROS_DOMAIN_ID (see Multi-machine)
        │           or: Mission Planner web → rosbridge :9090 (see above)
        │
        ├─ MISSION PLANNER ── nav2_mission_planner.launch.py  (rosbridge + services)
        │           wrappers: navpromini_mission_planner/mapping|navigation_launch
        │
        ├─ SIM ── gazebo.launch.py  [use_rviz:=false by default]
        │           use_rviz:=true  → optional Gazebo RViz
        │
        ├─ TELEOP ── joystick / keyboard  → /cmd_vel
        │              (mux: /cmd_vel_teleop + /cmd_vel_nav → /cmd_vel)
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
- Mission Planner uses Botforge **rosbridge** on the robot (`:9090`); do not install apt `ros-jazzy-rosbridge*`.
- With `twist_mux`, Mission Planner teleop should publish **`/cmd_vel_teleop`** (unstamped `Twist`), not `/cmd_vel` directly.

---

## License

Apache-2.0 (package defaults; declare per package as needed).
