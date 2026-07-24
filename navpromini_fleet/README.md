# NavPro Mini fleet agent (Phase A)

On-robot package for production boot → hotspot → register → hardware + heartbeat.

## Phase A contents

| Piece | Role |
|-------|------|
| `provision_portal` | Modern setup UI at `http://10.42.0.1/` → Wi‑Fi dropdown → **status page** after submit (LED guide) |
| `register_robot` | `POST /api/v1/robots/register` |
| `heartbeat_node` | `POST /robots/:id/heartbeat` + `nav_mode` |
| `status_display_node` | `/display_text` + `/led_command` → ESP32 |
| `robot_with_mux.launch` | Hardware bringup + `twist_mux` → `/cmd_vel` |
| systemd | `navpro-provision`, `navpro-robot`, `navpro-fleet`, `navpro-display` |
| zenoh template | CLIENT with `namespace: /<robot_name>` |

**cmd_vel sources (priority):** `fleet_teleop` > `cmd_vel_teleop` > `cmd_vel_nav`  
Manual keyboard: `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_teleop`

Later phases add `mode_manager`, map upload/claim.

## Install (on the Pi)

```bash
cd ~/NavProMini_ws
colcon build --packages-select navpromini_fleet
source install/setup.bash

sudo bash src/navpromini_fleet/scripts/setup_robot_pi.sh
sudo bash src/navpromini_fleet/scripts/install_zenoh_bridge.sh
sudo bash src/navpromini_fleet/systemd/install_fleet_services.sh

# First-boot hotspot (only when unprovisioned — fleet.yaml must NOT exist)
sudo rm -f /etc/navpro/fleet.yaml
sudo systemctl daemon-reload
sudo systemctl restart navpro-provision navpro-display navpro-robot
# optional: sudo reboot
```

**Hotspot only starts if `/etc/navpro/fleet.yaml` is missing.** If that file
already exists (even from an older test), `navpro-provision` is skipped and
the Pi stays on normal Wi‑Fi. To re-enter setup mode:

```bash
sudo systemctl stop navpro-fleet
sudo rm -f /etc/navpro/fleet.yaml
sudo systemctl restart navpro-provision navpro-display navpro-robot
# Phone: join AP NavPro-Setup-<MAC6> / password navprosetup → http://10.42.0.1/
```

Check services:

```bash
systemctl status navpro-provision navpro-robot navpro-display navpro-fleet --no-pager
journalctl -u navpro-robot -b --no-pager -n 40
```

## What is `serial`?

`serial` in `fleet.yaml` is the **board identity** used when the robot calls
`POST /api/v1/robots/register` (unique DB key). It is **not** the Wi‑Fi MAC and
**not** typed in the setup form.

| Field | Source | Example |
|-------|--------|---------|
| `serial` | Pi CPU / device-tree serial-number (fallback: machine-id) | `15e8f5efdf7f4b23` |
| Hotspot SSID | Last 6 hex of **wlan0 MAC** | `NavPro-Setup-0DE67E` |
| Hotspot password | Fixed for all robots | `navprosetup` |
| `name` | Operator types in form | `bot-1` |
| `mac` | Sent on register from wlan0 | `a2:a8:71:0d:e6:7e` |

## fleet.yaml (written by portal)

```yaml
name: bot-1
serial: 15e8f5efdf7f4b23   # auto from board — do not invent
server_ip: 192.168.1.10
provisioning_token: <from server .env>
wifi_ssid: SiteWifi         # chosen from scanned dropdown
robot_id: <uuid from register>
nav_mode: HARDWARE
```

## Display states

| State | OLED (approx) | LED |
|-------|---------------|-----|
| setup | AP + http://10.42.0.1/ | amber blink |
| joining | Joining fleet… | cyan |
| need_map | Need map | yellow |
| ready / nav | Ready | green |

## Devices shows “offline” but OLED looks fine

OLED **Need map** only means no site map yet — that is expected after claim
without a map. **Online** in the GUI comes only from heartbeats
(`POST /api/v1/robots/:id/heartbeat` every ~2s → Redis `robots:online`).

On the Pi:

```bash
# 1) Identity complete?
sudo cat /etc/navpro/fleet.yaml
# must have robot_id, server_ip, provisioning_token (same as server .env)

# 2) Fleet agent running?
systemctl status navpro-fleet --no-pager
journalctl -u navpro-fleet -b --no-pager -n 80

# 3) Can Pi reach the server?
curl -sS -m 3 "http://$(python3 -c "import yaml;print(yaml.safe_load(open('/etc/navpro/fleet.yaml'))['server_ip'])")/api/v1/health" || true

# 4) Restart fleet agent after Wi‑Fi is up
sudo systemctl restart navpro-fleet
```

Deploy updated agent:

```bash
cd ~/NavProMini_ws && colcon build --packages-select navpromini_fleet && source install/setup.bash
sudo bash src/navpromini_fleet/systemd/install_fleet_services.sh
sudo systemctl restart navpro-fleet
```
