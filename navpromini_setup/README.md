# navpromini_setup

Robot Wi‑Fi setup, OLED/LED, hardware bringup, and USB device rules.

## Install (one script)

```bash
cd ~/NavProMini_ws
colcon build --packages-select navpromini_setup
source install/setup.bash
sudo bash src/navpromini_setup/scripts/install_navpro.sh
```

That single script sets up:

| Step | What |
|------|------|
| Environment | ROS Jazzy, micro-ROS (`uros_ws`), NavPro workspace, `ROS_DOMAIN_ID` |
| USB udev | `/dev/rplidar`, `/dev/battery_bms` |
| `navpro-provision` | Wi‑Fi setup hotspot (only when no known Wi‑Fi is nearby) |
| `navpro-display` | OLED / LED status |
| `navpro-robot` | Hardware ROS (lidar, odom, micro-ROS) |

## Wi‑Fi setup

When the robot needs setup, connect to:

- **SSID:** `NavPro-Setup-<last6 of MAC>`
- **Password:** `navprosetup`
- **Portal:** http://10.42.0.1/ → Wi‑Fi + robot name

Config is saved to `/etc/navpro/robot.yaml`.

## Check / remove

```bash
systemctl status navpro-display navpro-robot navpro-provision
sudo bash src/navpromini_setup/scripts/uninstall_navpro.sh   # optional full remove
```
