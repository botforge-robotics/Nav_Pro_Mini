#!/usr/bin/env python3
"""Drive ESP32 OLED + LED from robot lifecycle state.

Publishes:
  /display_text  (std_msgs/String)
  /led_command   (std_msgs/String)  — firmware presets: solid/blink/...

ESP32 micro-ROS uses BEST_EFFORT + VOLATILE — no latch. We wait until a
subscriber (or /joint_states) appears, then push. Same OLED text is not
re-sent while connected. On ESP reconnect we push again.

Dynamic updates (do NOT publish OLED/LED from other processes):
  1) /run/navpro/display_state hint file (Wi‑Fi portal writes this)
  2) /navpro/display_state topic (optional; mapping/nav can publish later)
  3) /etc/navpro/robot.yaml presence (leave setup after Wi‑Fi saved)
  4) /battery/state (battery_node) — overrides just the LED while the pack is
     actually taking charge (charging → red breathe, full → green breathe).
     OLED text and the underlying lifecycle state are untouched.

     Driven by the BATTERY, deliberately, not by dock_manager's /dock_status.
     The LED should reflect what is physically true, and charging is true
     whether the robot drove onto the dock itself or someone pushed it on by
     hand. Keying off the docking action meant a manually docked robot sat
     there charging with an idle LED, and — worse — a failed dock attempt
     could leave the LED claiming "charging" when nothing was connected.

Without this node running, hint-file writes do nothing on the ESP.
"""

from __future__ import annotations

import os
import time
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState, JointState, LaserScan
from std_msgs.msg import String

from navpromini_setup.robot_config import DEFAULT_ROBOT_PATH, config_path_present, load_robot_config

STATE_FX: dict[str, tuple[str, str]] = {
    'boot': ('NavProMini', 'solid,255,0,0'),
    'setup': ('Setup WiFi - pass: navprosetup', 'blink,255,160,0,400'),
    'joining': ('Connecting WiFi...', 'solid,0,200,220'),
    'ready': ('', 'solid,0,200,40'),
    'mapping': ('Mapping...', 'chase,0,120,255,80'),
    'nav': ('Nav ready', 'solid,0,200,40'),
    'error': ('Error', 'blink,255,0,0,300'),
    'offline': ('Offline', 'solid,80,80,80'),
}

# LED-only override while the pack is actually taking charge, taking priority
# over STATE_FX's LED (OLED text is untouched). Anything else falls through to
# the normal state LED — 'nav'/'ready' are already solid green, so "not
# charging" needs no entry here.
CHARGE_LED: dict[str, str] = {
    'charging': 'breathe,255,0,0,1500',
    'full': 'breathe,0,255,0,1500',
}

# Current thresholds, in amps, with the sign convention battery_node uses
# (positive = into the pack). A deadband rather than a zero crossing because
# a topped-off pack on the charger floats around zero and would otherwise
# flicker between "full" and "not connected".
CHARGE_CURRENT_A = 0.2
DISCHARGE_CURRENT_A = -0.2

# How stale joint_states/scan may be before "ready" is downgraded back to a
# "boot" display — see _hardware_ready(). Doc §9/§10: READY means Wi-Fi AND
# required hardware, and the OLED previously only checked the former (see
# _sync_from_disk's robot.yaml/hint-driven 'ready' transitions below), so it
# could show "Ready" — and a person's name on it — before the encoders or
# LiDAR had actually come up.
HARDWARE_READY_TIMEOUT_SEC = 5.0

# The pogo-pin contact bounces on arrival: a real dock was observed reporting
# CHARGING and then DISCHARGING 1.2s apart while the robot settled. Requiring
# a state to persist before showing it keeps the LED from strobing during
# those first seconds.
CHARGE_DEBOUNCE_SEC = 0.6

HINT_PATH = Path(os.environ.get('NAVPRO_DISPLAY_HINT', '/run/navpro/display_state'))
ROBOT_PATH = Path(os.environ.get('NAVPRO_ROBOT_YAML', str(DEFAULT_ROBOT_PATH)))

ESP_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

JS_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


class StatusDisplayNode(Node):
    def __init__(self) -> None:
        super().__init__('navpro_status_display')
        self.declare_parameter('state', os.environ.get('NAVPRO_DISPLAY_STATE', 'boot'))
        self.declare_parameter('robot_name', os.environ.get('NAVPRO_ROBOT_NAME', ''))
        self.declare_parameter('ap_ssid', os.environ.get('NAVPRO_AP_SSID', ''))
        self.declare_parameter('ap_password', os.environ.get('NAVPRO_AP_PASSWORD', 'navprosetup'))
        self.declare_parameter('refresh_hz', 2.0)
        self.declare_parameter('esp_alive_timeout_sec', 3.0)

        self._pub_text = self.create_publisher(String, 'display_text', ESP_QOS)
        self._pub_led = self.create_publisher(String, 'led_command', ESP_QOS)
        self.create_subscription(String, 'navpro/display_state', self._on_state_msg, 10)
        self.create_subscription(JointState, 'joint_states', self._on_joint_states, JS_QOS)
        self.create_subscription(BatteryState, 'battery/state', self._on_battery, 10)
        # Liveness-only (see _hardware_ready) — not processed for anything
        # else, so best-effort/shallow queue is fine, same as joint_states.
        self.create_subscription(LaserScan, 'scan', self._on_scan, JS_QOS)
        self.create_subscription(String, 'dock_status', self._on_dock_status, 10)
        self._dock_status: Optional[str] = None
        # What the battery currently says, and what we have committed to
        # showing — separated so the debounce can hold the LED steady while
        # contact settles.
        self._charge_raw: Optional[str] = None
        self._charge_shown: Optional[str] = None
        self._charge_since = 0.0

        self._state = str(self.get_parameter('state').value).strip().lower() or 'boot'
        self._pending_text: Optional[str] = None
        self._pending_led: Optional[str] = None
        self._delivered_text: Optional[str] = None
        self._delivered_led: Optional[str] = None
        self._esp_seen = False
        self._last_js_ns: Optional[int] = None
        self._last_scan_ns: Optional[int] = None
        self._waiting_logged = False
        self._hint_mtime: Optional[float] = None
        self._robot_mtime: Optional[float] = None
        # micro-ROS is BEST_EFFORT: burst + periodic LED push so chase/blink
        # actually stop when mapping ends (one-shot drops leave ESP stuck).
        self._burst_left = 0
        self._ticks_since_led = 0
        self._led_repost_every = max(
            1, int(round(2.0 / max(float(self.get_parameter('refresh_hz').value), 0.2)))
        )

        period = 1.0 / max(float(self.get_parameter('refresh_hz').value), 0.2)
        self.create_timer(period, self._tick)
        self.get_logger().info(f'status_display starting in state={self._state}')
        self._sync_from_disk(force=True)
        self._compose_pending(self._state)
        self._burst_left = 6

    def _apply_state(self, new_state: str) -> None:
        """Set state and force ESP re-push (OLED + LED)."""
        if new_state == self._state and self._pending_led is not None:
            return
        self._state = new_state
        self._delivered_text = None
        self._delivered_led = None
        self._burst_left = 8
        self._ticks_since_led = 0
        self._compose_pending(new_state)

    def _on_state_msg(self, msg: String) -> None:
        new_state = (msg.data or '').strip().lower()
        if not new_state:
            return
        # Don't let a late setup message undo post-setup states.
        if new_state == 'setup' and self._state in ('joining', 'ready', 'nav', 'mapping'):
            return
        if new_state == self._state:
            return
        self.get_logger().info(f'display_state topic → {new_state}')
        self._apply_state(new_state)

    def _on_joint_states(self, _msg: JointState) -> None:
        self._last_js_ns = self.get_clock().now().nanoseconds

    def _on_scan(self, _msg: LaserScan) -> None:
        self._last_scan_ns = self.get_clock().now().nanoseconds

    def _on_dock_status(self, msg: String) -> None:
        status = (msg.data or '').strip().lower()
        if status != self._dock_status:
            self._dock_status = status
            self._note_state_composed()
            self._compose_pending(self._state)

    def _hardware_ready(self) -> bool:
        """Encoders (joint_states) and LiDAR (scan) both publishing recently
        — a good-enough stand-in for "required hardware is up" (doc §9/§10),
        not a full diagnostic breakdown; navpromini_sdk's /system/lifecycle
        is the place for that. Used only to gate what the OLED/LED show, see
        _compose_pending — never touches self._state's own bookkeeping."""
        now_ns = self.get_clock().now().nanoseconds

        def _fresh(last_ns: Optional[int]) -> bool:
            return (last_ns is not None
                   and (now_ns - last_ns) * 1e-9 <= HARDWARE_READY_TIMEOUT_SEC)

        return _fresh(self._last_js_ns) and _fresh(self._last_scan_ns)

    @staticmethod
    def _charge_state_of(msg: BatteryState) -> Optional[str]:
        """'charging' | 'full' | None, from the pack itself.

        Uses power_supply_status where the BMS reports it, and falls back to
        current so a pack that only reports UNKNOWN still lights up. FULL only
        counts while the robot is still connected — a full pack running on
        battery must not sit there glowing "charged on dock".
        """
        status = msg.power_supply_status
        current = float(msg.current) if msg.current == msg.current else 0.0

        if status == BatteryState.POWER_SUPPLY_STATUS_CHARGING:
            return 'charging'
        if status == BatteryState.POWER_SUPPLY_STATUS_FULL:
            return 'full' if current > DISCHARGE_CURRENT_A else None
        if current > CHARGE_CURRENT_A:
            return 'charging'
        return None

    def _on_battery(self, msg: BatteryState) -> None:
        raw = self._charge_state_of(msg)
        now = time.monotonic()
        if raw != self._charge_raw:
            self._charge_raw = raw
            self._charge_since = now
            return
        # Only commit once the reading has held — see CHARGE_DEBOUNCE_SEC.
        if raw == self._charge_shown or now - self._charge_since < CHARGE_DEBOUNCE_SEC:
            return
        self.get_logger().info(
            f'charge state → {raw or "not charging"} (LED follows the battery, '
            'so manual docking shows it too)')
        self._charge_shown = raw
        self._note_state_composed()
        self._compose_pending(self._state)

    def _set_robot_name(self, name: str) -> bool:
        name = (name or '').strip()
        if not name or name == self._param_str('robot_name'):
            return False
        self.set_parameters([Parameter('robot_name', Parameter.Type.STRING, name)])
        return True

    def _set_ap(self, ssid: str, password: str) -> bool:
        ssid = (ssid or '').strip()
        password = (password or '').strip() or 'navprosetup'
        changed = False
        if ssid and ssid != self._param_str('ap_ssid'):
            self.set_parameters([Parameter('ap_ssid', Parameter.Type.STRING, ssid)])
            changed = True
        if password != self._param_str('ap_password'):
            self.set_parameters([Parameter('ap_password', Parameter.Type.STRING, password)])
            changed = True
        return changed

    def _read_hint_state(self) -> Optional[str]:
        try:
            if not HINT_PATH.is_file():
                return None
            lines = HINT_PATH.read_text(encoding='utf-8').splitlines()
            if not lines:
                return None
            return (lines[0] or '').strip().lower()
        except OSError:
            return None

    def _setup_ap_really_up(self) -> bool:
        """True only if nmcli reports the setup AP connection active."""
        try:
            import subprocess
            r = subprocess.run(
                ['nmcli', '-t', '-f', 'NAME,STATE', 'connection', 'show', '--active'],
                capture_output=True, text=True, timeout=5,
            )
            for line in (r.stdout or '').splitlines():
                if line.startswith('navpro-setup-ap:') and 'activated' in line.lower():
                    return True
        except Exception:  # noqa: BLE001
            pass
        return False

    def _wifi_site_online(self) -> bool:
        try:
            import subprocess
            r = subprocess.run(
                ['nmcli', '-t', '-f', 'DEVICE,TYPE,STATE,CONNECTION', 'device', 'status'],
                capture_output=True, text=True, timeout=5,
            )
            for line in (r.stdout or '').splitlines():
                parts = line.split(':')
                if len(parts) < 4:
                    continue
                _dev, dtype, state, conn = parts[0], parts[1], parts[2], parts[3]
                if dtype != 'wifi' or state != 'connected':
                    continue
                if not conn or conn == 'navpro-setup-ap':
                    continue
                ip = subprocess.run(['hostname', '-I'], capture_output=True, text=True, timeout=3)
                if ip.returncode == 0 and (ip.stdout or '').strip():
                    return True
        except Exception:  # noqa: BLE001
            pass
        return False

    def _sync_from_disk(self, force: bool = False) -> bool:
        """React to robot.yaml / display hint changes (post-portal)."""
        changed = False
        hint_live = self._read_hint_state()

        # Reality check: never keep OLED in "setup" if hotspot is not actually up
        # and site Wi‑Fi is already connected (stale hint / race on boot).
        if self._state == 'setup' and not self._setup_ap_really_up() and self._wifi_site_online():
            self.get_logger().info('Wi‑Fi online and no setup AP — display setup → ready')
            self._state = 'ready'
            try:
                HINT_PATH.parent.mkdir(parents=True, exist_ok=True)
                HINT_PATH.write_text(f'ready\n{self._param_str("robot_name")}\n\n', encoding='utf-8')
            except OSError:
                pass
            changed = True

        # 1) Config file is authoritative over setup hotspot — but do not
        # clobber an in-progress joining/error hint (portal still driving UI).
        try:
            robot_mtime = ROBOT_PATH.stat().st_mtime if ROBOT_PATH.is_file() else None
        except OSError:
            robot_mtime = None
        if force or robot_mtime != self._robot_mtime:
            self._robot_mtime = robot_mtime
            if robot_mtime is not None or config_path_present():
                cfg = load_robot_config()
                if cfg is not None:
                    if self._set_robot_name(cfg.name):
                        changed = True
                    # Stale setup hint while AP is down: leave setup.
                    if hint_live == 'setup' and not self._setup_ap_really_up():
                        if self._state in ('boot', 'setup', 'joining', 'error'):
                            self._state = 'ready'
                            changed = True
                    elif hint_live in ('joining', 'error'):
                        pass
                    elif self._state in ('boot', 'setup', 'joining', 'error'):
                        self.get_logger().info(
                            f'robot.yaml present — display {self._state} → ready'
                        )
                        self._state = 'ready'
                        changed = True

        # 2) Hint file (setup/joining/ready/error during / after portal).
        try:
            hint_mtime = HINT_PATH.stat().st_mtime if HINT_PATH.is_file() else None
        except OSError:
            hint_mtime = None
        if force or hint_mtime != self._hint_mtime:
            self._hint_mtime = hint_mtime
            if hint_mtime is None:
                return changed
            try:
                lines = HINT_PATH.read_text(encoding='utf-8').splitlines()
            except OSError:
                return changed
            if not lines:
                return changed
            hint_state = (lines[0] if lines else '').strip().lower()
            line2 = (lines[1] if len(lines) > 1 else '').strip()
            line3 = (lines[2] if len(lines) > 2 else '').strip()

            if hint_state == 'setup':
                # Only honor setup hint when the AP is really active.
                if not self._setup_ap_really_up():
                    if self._wifi_site_online() and self._state == 'setup':
                        self._state = 'ready'
                        changed = True
                    return changed
                if self._set_ap(line2, line3 or 'navprosetup'):
                    changed = True
                if self._state != 'setup':
                    self._state = 'setup'
                    changed = True
            else:
                if line2 and self._set_robot_name(line2):
                    changed = True
                if hint_state in STATE_FX and hint_state != self._state:
                    regress = {'ready', 'nav', 'mapping'}
                    if self._state in regress and hint_state == 'setup':
                        pass
                    elif self._state in regress and hint_state == 'joining':
                        self.get_logger().info(f'display hint → {hint_state}')
                        self._state = hint_state
                        changed = True
                    else:
                        self.get_logger().info(f'display hint → {hint_state}')
                        self._state = hint_state
                        changed = True
        return changed

    def _note_state_composed(self) -> None:
        """After disk sync mutated _state, clear delivered so ESP gets a burst."""
        self._delivered_text = None
        self._delivered_led = None
        self._burst_left = max(self._burst_left, 8)
        self._ticks_since_led = 0

    def _param_str(self, name: str) -> str:
        raw = str(self.get_parameter(name).value).strip()
        return '' if raw in ('', '_') else raw

    def _get_robot_ip(self) -> str:
        try:
            import subprocess
            r = subprocess.run(['hostname', '-I'], capture_output=True, text=True, timeout=2)
            if r.returncode == 0 and r.stdout:
                ips = r.stdout.strip().split()
                for addr in ips:
                    if not addr.startswith('127.') and not addr.startswith('10.42.'):
                        return addr
                if ips:
                    return ips[0]
        except Exception:
            pass
        return ""

    def _compose_text(self, state: str, default_text: str) -> str:
        name = self._param_str('robot_name')
        ap = self._param_str('ap_ssid')
        pw = self._param_str('ap_password') or 'navprosetup'
        if state == 'setup':
            if ap:
                return f'{ap}  pw:{pw}  http://10.42.0.1/'
            return f'Setup WiFi  pw:{pw}  http://10.42.0.1/'
        if state == 'joining':
            return f'{name} - Connecting WiFi...' if name else 'Connecting WiFi...'
        if state == 'ready':
            # Always show robot IP when everything is OK (never show 'Ready' text)
            ip = self._get_robot_ip()
            if ip:
                return ip
            return name or default_text or 'No IP'
        if state in ('nav', 'mapping', 'error') and name:
            return f'{name} - {default_text}'
        return default_text

    @staticmethod
    def _oled_ascii(text: str) -> str:
        """OLED fonts are usually ASCII-only; strip fancy punctuation."""
        repl = {
            '\u2014': '-',
            '\u2013': '-',
            '\u2026': '...',
            '\u00b7': '-',
            '\u2022': '-',
            '\u2018': "'",
            '\u2019': "'",
            '\u201c': '"',
            '\u201d': '"',
            '\u00a0': ' ',
        }
        out = text
        for src, dst in repl.items():
            out = out.replace(src, dst)
        return ''.join(ch if 32 <= ord(ch) <= 126 else '?' for ch in out)

    def _compose_pending(self, state: str) -> None:
        # self._state (what's passed in as `state`) can say 'ready' purely
        # from Wi-Fi/config presence — see _sync_from_disk. Downgrading only
        # what's actually shown here, not self._state itself, keeps that
        # method's already-intricate hint/mtime race handling untouched.
        display_state = 'boot' if state == 'ready' and not self._hardware_ready() else state
        text, led = STATE_FX.get(display_state, STATE_FX['boot'])
        if self._dock_status in ('staging', 'searching', 'servo', 'blind_creep'):
            self._pending_text = self._oled_ascii('Docking...')[:192]
            self._pending_led = 'solid,255,255,255'
        else:
            self._pending_text = self._oled_ascii(self._compose_text(display_state, text))[:192]
            self._pending_led = CHARGE_LED.get(self._charge_shown or '', led)

    def _esp_ready(self) -> bool:
        text_subs = self._pub_text.get_subscription_count()
        led_subs = self._pub_led.get_subscription_count()
        has_sub = text_subs > 0 or led_subs > 0

        alive = False
        if self._last_js_ns is not None:
            age_s = (self.get_clock().now().nanoseconds - self._last_js_ns) * 1e-9
            timeout = float(self.get_parameter('esp_alive_timeout_sec').value)
            alive = age_s <= timeout

        ready = has_sub or alive
        if ready and not self._esp_seen:
            self._esp_seen = True
            self.get_logger().info(
                f'ESP ready (display_subs={text_subs} led_subs={led_subs} '
                f'joint_states_alive={alive}) — pushing display/LED'
            )
        return ready

    def _publish_if_needed(self, force: bool = False) -> None:
        if self._pending_text is None or self._pending_led is None:
            return
        if not self._esp_ready():
            if not self._waiting_logged:
                self._waiting_logged = True
                self.get_logger().info(
                    'Waiting for ESP (/joint_states or display/led subscribers) '
                    'before OLED/LED push…'
                )
            return

        self._waiting_logged = False
        text = self._pending_text
        led = self._pending_led
        burst = self._burst_left > 0
        self._ticks_since_led += 1
        repost_led = self._ticks_since_led >= self._led_repost_every

        if force or burst or text != self._delivered_text:
            msg = String()
            msg.data = text
            self._pub_text.publish(msg)
            prev_text = self._delivered_text
            self._delivered_text = text
            if force or text != prev_text or self._burst_left >= 7:
                self.get_logger().info(f'OLED state={self._state} text={text!r}')

        # Always re-push LED on burst / periodic interval — chase must be cancelled.
        if force or burst or repost_led or led != self._delivered_led:
            msg = String()
            msg.data = led
            self._pub_led.publish(msg)
            prev = self._delivered_led
            self._delivered_led = led
            self._ticks_since_led = 0
            if force or led != prev or self._burst_left >= 7:
                self.get_logger().info(f'LED state={self._state} cmd={led!r}')

        if self._burst_left > 0:
            self._burst_left -= 1

    def _tick(self) -> None:
        was_ready = self._esp_seen and (
            self._delivered_text is not None or self._delivered_led is not None
        )
        ready = self._esp_ready()
        force = False
        if was_ready and not ready:
            self._esp_seen = False
            self._delivered_text = None
            self._delivered_led = None
            self.get_logger().warn('ESP lost — will re-push display/LED when back')
        elif not was_ready and ready:
            force = True

        if self._sync_from_disk():
            self._note_state_composed()
        # Recomposed every tick, not only on a disk change above: hardware
        # readiness (_hardware_ready, used inside _compose_pending) moves on
        # its own schedule, independent of the hint file / robot.yaml. Cheap
        # (string formatting only) — _publish_if_needed below only pushes to
        # the ESP when the composed value actually differs from what's
        # already delivered.
        self._compose_pending(self._state)
        self._publish_if_needed(force=force)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = StatusDisplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
