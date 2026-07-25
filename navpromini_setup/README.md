# navpromini_setup (nav2 branch)

Standalone robot setup — **no RMF / fleet / zenoh**.

## What it includes

| Piece | Role |
|-------|------|
| Wi‑Fi portal | Hotspot only if no saved Wi‑Fi is online/nearby; form = Wi‑Fi + robot name |
| `navpro-provision.service` | Runs the portal |
| `navpro-robot.service` | Hardware bringup (separate) |
| `navpro-display.service` | OLED / LED |
| `udev/99-navpro.rules` | `/dev/rplidar`, `/dev/battery_bms` |
| `scripts/setup_robot_pi.sh` | One-time Pi OS + udev + NetworkManager |

Config written to: `/etc/navpro/robot.yaml` (`name`, `serial`, `wifi_ssid`).

## Install on Pi

```bash
cd ~/NavProMini_ws
colcon build --packages-select navpromini_setup
source install/setup.bash
sudo bash src/navpromini_setup/scripts/setup_robot_pi.sh
sudo bash src/navpromini_setup/systemd/install_robot_services.sh
sudo systemctl start navpro-robot navpro-display navpro-provision
```

Hotspot SSID: `NavPro-Setup-<MAC6>` · password: `navprosetup` · portal: `http://10.42.0.1/`
