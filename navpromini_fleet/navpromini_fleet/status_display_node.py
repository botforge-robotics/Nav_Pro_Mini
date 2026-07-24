#!/usr/bin/env python3
"""Drive ESP32 OLED + LED from robot lifecycle state (Phase A).

Publishes:
  /display_text  (std_msgs/String)
  /led_command   (std_msgs/String)  — firmware presets: solid/blink/...

States (env NAVPRO_DISPLAY_STATE or ROS param `state`, or /navpro/display_state):
  setup | joining | need_map | ready | mapping | error | boot
"""

from __future__ import annotations

import os
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# OLED text + LED command per phase (plan Phase A).
STATE_FX: dict[str, tuple[str, str]] = {
    'boot': ('NavProMini', 'solid,255,0,0'),
    'setup': ('Setup WiFi — pass: navprosetup', 'blink,255,160,0,400'),
    'joining': ('Joining fleet…', 'solid,0,200,220'),
    'need_map': ('Need map', 'solid,255,200,0'),
    'ready': ('Ready', 'solid,0,200,40'),
    'mapping': ('Mapping…', 'chase,0,120,255,80'),
    'nav': ('Nav ready', 'solid,0,200,40'),
    'error': ('Error', 'blink,255,0,0,300'),
    'offline': ('Offline', 'solid,80,80,80'),
}


class StatusDisplayNode(Node):
    def __init__(self) -> None:
        super().__init__('navpro_status_display')
        self.declare_parameter('state', os.environ.get('NAVPRO_DISPLAY_STATE', 'boot'))
        self.declare_parameter('robot_name', os.environ.get('NAVPRO_ROBOT_NAME', ''))
        self.declare_parameter('ap_ssid', os.environ.get('NAVPRO_AP_SSID', ''))
        self.declare_parameter('ap_password', os.environ.get('NAVPRO_AP_PASSWORD', 'navprosetup'))
        self.declare_parameter('refresh_hz', 0.5)

        self._pub_text = self.create_publisher(String, 'display_text', 10)
        self._pub_led = self.create_publisher(String, 'led_command', 10)
        self.create_subscription(String, 'navpro/display_state', self._on_state_msg, 10)

        self._state = str(self.get_parameter('state').value).strip().lower() or 'boot'
        self._last_sent: Optional[tuple[str, str]] = None
        period = 1.0 / max(float(self.get_parameter('refresh_hz').value), 0.1)
        self.create_timer(period, self._tick)
        self.get_logger().info(f'status_display starting in state={self._state}')
        self._apply(self._state, force=True)

    def _on_state_msg(self, msg: String) -> None:
        new_state = (msg.data or '').strip().lower()
        if not new_state:
            return
        self._state = new_state
        self._apply(new_state, force=True)

    def _compose_text(self, state: str, default_text: str) -> str:
        name = str(self.get_parameter('robot_name').value).strip()
        ap = str(self.get_parameter('ap_ssid').value).strip()
        pw = str(self.get_parameter('ap_password').value).strip() or 'navprosetup'
        if state == 'setup':
            if ap:
                return f'{ap}  pw:{pw}  http://10.42.0.1/'
            return f'Setup WiFi  pw:{pw}  http://10.42.0.1/'
        if state in ('ready', 'nav', 'need_map') and name:
            return f'{name} · {default_text}'
        return default_text

    def _apply(self, state: str, force: bool = False) -> None:
        text, led = STATE_FX.get(state, STATE_FX['boot'])
        text = self._compose_text(state, text)
        key = (text, led)
        if not force and key == self._last_sent:
            return
        self._last_sent = key
        t = String()
        t.data = text[:192]
        self._pub_text.publish(t)
        l = String()
        l.data = led
        self._pub_led.publish(l)
        self.get_logger().info(f'display state={state} text={text!r} led={led!r}')

    def _tick(self) -> None:
        # Re-publish so reconnecting ESP32 picks up current state.
        self._apply(self._state, force=False)
        if self._last_sent is None:
            self._apply(self._state, force=True)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = StatusDisplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
