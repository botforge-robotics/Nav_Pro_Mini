# navpromini_manipulation

Autonomous mobile manipulation package for the **NavProMini Mobile Manipulator** (differential drive base + SO101 6-DOF robotic arm + gripper wrist camera).

---

## Features

- **Coordinated Mobile Manipulation**: Combines base driving and arm articulation into unified mission pipelines.
- **Station Navigation**: Moves autonomously between the Home Dock Bay, Arena Doorway, Assembly Table (Pick Station), and Sorting Trays.
  - Supports Nav2 `NavigateToPose` when Nav2 stack is active.
  - Automatically falls back to autonomous closed-loop Odometry Waypoint Tracking if Nav2 is not launched.
- **SO101 6-DOF Kinematics**:
  - Analytical Inverse Kinematics for arbitrary 3D target coordinates $(x, y, z)$.
  - Pre-calibrated Playground world targets (`red_cube`, `green_cylinder`, `blue_box`, `yellow_cylinder`, `red_tray`, `blue_tray`).
  - Safe transit states: `IDLE` $\to$ `NAVIGATE_TO_PICK` $\to$ `ARM_APPROACH` $\to$ `VISION_VERIFY` $\to$ `DESCEND` $\to$ `GRASP` $\to$ `LIFT` $\to$ `CARRY` $\to$ `NAVIGATE_TO_PLACE` $\to$ `PLACE` $\to$ `RELEASE` $\to$ `HOME`.
- **Perception**: Integrated gripper wrist camera (`/arm/wrist_camera/image_raw`) for color detection and target verification before picking.

---

## Quickstart

### 1. Launch Simulation
```bash
ros2 launch navpromini_gazebo gazebo.launch.py \
  world_name:=playground \
  use_arm:=true \
  use_rviz:=true
```

### 2. Launch Navigation & Manipulation Executive

You can run navigation and manipulation in either of two ways:

#### Option A: All-in-One Launch (starts Nav2 stack + Manipulation node)
```bash
ros2 launch navpromini_manipulation manipulation.launch.py use_sim_time:=true start_navigation:=true
```

#### Option B: Standalone Terminals (independent Nav2 stack)
**Terminal 2 — Nav2 Navigation:**
```bash
ros2 launch navpromini_navigation navigation.launch.py map_name:=playground use_sim_time:=true
```
**Terminal 3 — Manipulation Node:**
```bash
ros2 launch navpromini_manipulation manipulation.launch.py use_sim_time:=true
```

*(Note: If Nav2 is not launched, `object_manipulation_node` will automatically use its built-in closed-loop odometry waypoint navigation fallback).*

### 3. Command Tasks
```bash
# Full Mobile Pick-and-Place:
ros2 run navpromini_manipulation manipulation_client --action pick_and_place --target red_cube --destination red_tray

# Sort Green Cylinder into Blue Tray:
ros2 run navpromini_manipulation manipulation_client --action pick_and_place --target green_cylinder --destination blue_tray

# Pick only:
ros2 run navpromini_manipulation manipulation_client --action pick --target red_cube

# Place only:
ros2 run navpromini_manipulation manipulation_client --action place --destination red_tray

# Visual inspection with wrist camera:
ros2 run navpromini_manipulation manipulation_client --action inspect --target red_cube

# Return arm to home:
ros2 run navpromini_manipulation manipulation_client --action home
```

---

## ROS 2 Interfaces

- **Subscribed Topics**:
  - `/manipulation/command` (`std_msgs/msg/String`): Action commands (e.g. `pick_and_place red_cube red_tray`).
  - `/odom` (`nav_msgs/msg/Odometry`): Mobile base telemetry.
  - `/joint_states` (`sensor_msgs/msg/JointState`): Arm telemetry.
  - `/arm/wrist_camera/image_raw` (`sensor_msgs/msg/Image`): Gripper camera image.
- **Published Topics**:
  - `/cmd_vel` (`geometry_msgs/msg/Twist`): Base navigation velocity commands.
  - `/arm/joint_goals` (`std_msgs/msg/Float64MultiArray`): 6-DOF arm joint targets.
  - `/arm/named_pose` (`std_msgs/msg/String`): Named pose requests.
  - `/manipulation/status` (`std_msgs/msg/String`): Real-time JSON telemetry.
- **Services**:
  - `/manipulation/execute_task` (`std_srvs/srv/Trigger`): Trigger default demo cycle.
