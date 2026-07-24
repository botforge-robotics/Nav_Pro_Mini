#!/usr/bin/env python3
"""Captive portal for first-boot Wi‑Fi + fleet join (Phase A).

Runs only when /etc/navpro/fleet.yaml is missing (systemd ConditionPathExists).
Creates AP NavPro-Setup-XXXX, serves http://10.42.0.1/, then joins site Wi‑Fi
and registers with the fleet server.
"""

from __future__ import annotations

import html
import os
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from typing import Optional
from urllib.parse import parse_qs

from navpromini_fleet.fleet_config import (
    DEFAULT_AP_PASSWORD,
    DEFAULT_FLEET_PATH,
    FleetConfig,
    ap_ssid_from_mac,
    primary_mac,
    read_cpu_serial,
    save_fleet_config,
)
from navpromini_fleet.register_robot import register

AP_IFACE = os.environ.get('NAVPRO_WIFI_IFACE', 'wlan0')
# Same password on every robot (do not randomize).
AP_PASSWORD = os.environ.get('NAVPRO_AP_PASSWORD', DEFAULT_AP_PASSWORD)
AP_ADDR = '10.42.0.1'
CONN_AP = 'navpro-setup-ap'
CONN_SITE = 'navpro-site-wifi'


FORM_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1"/>
<title>NavPro Setup</title>
<style>
  body {{ font-family: system-ui, sans-serif; margin: 1.5rem; background: #0f1419; color: #e7ecf1; }}
  h1 {{ font-size: 1.35rem; }}
  label {{ display: block; margin-top: 0.9rem; font-size: 0.85rem; color: #9aa7b5; }}
  input, select {{ width: 100%; max-width: 28rem; padding: 0.55rem 0.65rem; margin-top: 0.25rem;
           border: 1px solid #2a3540; border-radius: 6px; background: #1a222b; color: #fff;
           box-sizing: border-box; }}
  button {{ margin-top: 1.25rem; padding: 0.65rem 1.2rem; border: 0; border-radius: 6px;
            background: #1f8f4e; color: #fff; font-weight: 600; cursor: pointer; }}
  button.linkish {{ background: #2a3540; margin-top: 0.5rem; margin-right: 0.5rem; }}
  .hint {{ color: #9aa7b5; font-size: 0.85rem; margin-top: 0.5rem; }}
  .err {{ color: #ff7b7b; margin-top: 1rem; }}
  .ok {{ color: #6ddea0; margin-top: 1rem; }}
  #custom_ssid_wrap {{ display: none; }}
</style>
<script>
function onSsidChange(sel) {{
  var wrap = document.getElementById('custom_ssid_wrap');
  var custom = document.getElementById('wifi_ssid_custom');
  if (sel.value === '__other__') {{
    wrap.style.display = 'block';
    custom.required = true;
  }} else {{
    wrap.style.display = 'none';
    custom.required = false;
    custom.value = '';
  }}
}}
function beforeSubmit(form) {{
  var sel = document.getElementById('wifi_ssid_select');
  var hidden = document.getElementById('wifi_ssid');
  if (sel.value === '__other__') {{
    hidden.value = document.getElementById('wifi_ssid_custom').value.trim();
  }} else {{
    hidden.value = sel.value;
  }}
  if (!hidden.value) {{
    alert('Select or enter a Wi‑Fi network');
    return false;
  }}
  return true;
}}
</script>
</head>
<body>
  <h1>NavPro robot setup</h1>
  <p class="hint">AP: <strong>{ap_ssid}</strong> · Password: <strong>{ap_password}</strong></p>
  <p class="hint">MAC: {mac} · Board serial: {serial}</p>
  <form method="POST" action="/save" onsubmit="return beforeSubmit(this)">
    <input type="hidden" name="wifi_ssid" id="wifi_ssid" value=""/>
    <label>Site Wi‑Fi network
      <select id="wifi_ssid_select" required onchange="onSsidChange(this)">
        <option value="">— select network —</option>
        {wifi_options}
        <option value="__other__">Other (type SSID)…</option>
      </select>
    </label>
    <label id="custom_ssid_wrap">Custom SSID
      <input id="wifi_ssid_custom" name="wifi_ssid_custom" autocomplete="off" placeholder="Hidden / not listed"/>
    </label>
    <p class="hint">
      <a href="/rescan" style="color:#6ddea0">Refresh network list</a>
      {wifi_hint}
    </p>
    <label>Site Wi‑Fi password
      <input name="wifi_password" type="password" required autocomplete="off"/>
    </label>
    <label>Robot name
      <input name="robot_name" required placeholder="bot-1" autocomplete="off"/>
    </label>
    <label>Fleet server IP
      <input name="server_ip" required placeholder="192.168.1.10" autocomplete="off"/>
    </label>
    <label>Provisioning token
      <input name="provisioning_token" required autocomplete="off"/>
    </label>
    <button type="submit">Save &amp; join</button>
  </form>
  <p class="hint">Do not set fleet/floor here — assign those in the GUI Devices tab.</p>
  {message}
</body>
</html>
"""


def _run(cmd: list[str], check: bool = False, timeout: float = 60.0) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        cmd, check=check, text=True, capture_output=True, timeout=timeout
    )


def _nmcli(*args: str, check: bool = False) -> subprocess.CompletedProcess[str]:
    return _run(['nmcli', *args], check=check)


def scan_wifi_networks(rescan: bool = True) -> list[tuple[str, int]]:
    """Return [(ssid, signal%), ...] sorted by signal, unique SSIDs.

    Best results before AP mode; also tries while AP is up.
    """
    args = ['-t', '-f', 'SSID,SIGNAL', 'device', 'wifi', 'list', 'ifname', AP_IFACE]
    if rescan:
        args.append('--rescan')
        args.append('yes')
    r = _nmcli(*args)
    if r.returncode != 0:
        # Retry without forced rescan
        r = _nmcli('-t', '-f', 'SSID,SIGNAL', 'device', 'wifi', 'list', 'ifname', AP_IFACE)
    seen: dict[str, int] = {}
    for line in (r.stdout or '').splitlines():
        # nmcli -t uses : as separator; SSID may contain \: escapes
        parts: list[str] = []
        buf = ''
        i = 0
        raw = line
        while i < len(raw):
            if raw[i] == '\\' and i + 1 < len(raw):
                buf += raw[i + 1]
                i += 2
                continue
            if raw[i] == ':':
                parts.append(buf)
                buf = ''
                i += 1
                continue
            buf += raw[i]
            i += 1
        parts.append(buf)
        if len(parts) < 2:
            continue
        ssid = parts[0].strip()
        if not ssid or ssid.startswith('NavPro-Setup-'):
            continue
        try:
            signal = int(parts[1].strip() or '0')
        except ValueError:
            signal = 0
        prev = seen.get(ssid, -1)
        if signal > prev:
            seen[ssid] = signal
    ranked = sorted(seen.items(), key=lambda kv: (-kv[1], kv[0].lower()))
    return ranked


def wifi_options_html(networks: list[tuple[str, int]]) -> str:
    if not networks:
        return ''
    opts: list[str] = []
    for ssid, signal in networks:
        label = f'{ssid}  ({signal}%)'
        opts.append(
            f'<option value="{html.escape(ssid, quote=True)}">'
            f'{html.escape(label)}</option>'
        )
    return '\n'.join(opts)


def write_display_hint(ap_ssid: str, ap_password: str = AP_PASSWORD) -> None:
    """Best-effort hint file for start_display.sh before ROS is up."""
    path = '/run/navpro/display_state'
    try:
        os.makedirs('/run/navpro', exist_ok=True)
        with open(path, 'w', encoding='utf-8') as f:
            f.write(f'setup\n{ap_ssid}\n{ap_password}\n')
    except OSError:
        pass


def start_access_point(_serial: str) -> tuple[str, str, list[tuple[str, int]]]:
    """Start setup AP. Returns (ssid, mac, wifi_list). SSID uses Wi‑Fi MAC."""
    mac = primary_mac(AP_IFACE)
    ssid = ap_ssid_from_mac(mac)
    write_display_hint(ssid, AP_PASSWORD)
    # Tear down previous navpro connections if present.
    for name in (CONN_AP, CONN_SITE):
        _nmcli('connection', 'delete', name)
    # Ensure Wi‑Fi is managed by NetworkManager.
    _nmcli('device', 'set', AP_IFACE, 'managed', 'yes')
    _nmcli('radio', 'wifi', 'on')

    # Scan site networks BEFORE enabling AP (best scan quality on one radio).
    print('Scanning Wi‑Fi networks…')
    networks = scan_wifi_networks(rescan=True)
    print(f'Found {len(networks)} SSIDs')

    r = _nmcli(
        'connection', 'add',
        'type', 'wifi',
        'ifname', AP_IFACE,
        'con-name', CONN_AP,
        'autoconnect', 'yes',
        'ssid', ssid,
        'mode', 'ap',
        'wifi-sec.key-mgmt', 'wpa-psk',
        'wifi-sec.psk', AP_PASSWORD,
        'ipv4.method', 'shared',
        'ipv4.addresses', f'{AP_ADDR}/24',
    )
    if r.returncode != 0:
        raise RuntimeError(f'nmcli add AP failed: {r.stderr or r.stdout}')
    r = _nmcli('connection', 'up', CONN_AP)
    if r.returncode != 0:
        raise RuntimeError(f'nmcli up AP failed: {r.stderr or r.stdout}')
    return ssid, mac, networks


def connect_site_wifi(ssid: str, password: str) -> None:
    _nmcli('connection', 'down', CONN_AP)
    _nmcli('connection', 'delete', CONN_SITE)
    r = _nmcli(
        'device', 'wifi', 'connect', ssid,
        'password', password,
        'ifname', AP_IFACE,
        'name', CONN_SITE,
    )
    if r.returncode != 0:
        # Restore AP so operator can retry.
        _nmcli('connection', 'up', CONN_AP)
        raise RuntimeError(f'Wi‑Fi join failed: {r.stderr or r.stdout}')
    # Wait for DHCP / route
    for _ in range(30):
        time.sleep(1.0)
        ping = _run(['ping', '-c', '1', '-W', '1', '8.8.8.8'])
        # Server may be LAN-only; also try nothing if ping blocked — just wait.
        ip = _run(['hostname', '-I'])
        if ip.returncode == 0 and ip.stdout.strip():
            return
    # Continue anyway; register will fail clearly if no route.


def maybe_restart_fleet_units() -> None:
    for unit in ('navpro-display', 'navpro-robot', 'navpro-fleet'):
        _run(['systemctl', 'restart', unit], check=False)


class PortalState:
    def __init__(
        self,
        serial: str,
        ap_ssid: str,
        mac: str,
        networks: list[tuple[str, int]] | None = None,
    ) -> None:
        self.serial = serial
        self.ap_ssid = ap_ssid
        self.mac = mac
        self.networks = list(networks or [])
        self.message = ''
        self.busy = False
        self.done = False


def make_handler(state: PortalState):  # noqa: ANN201
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt: str, *args) -> None:  # noqa: ANN002
            sys.stderr.write(f'[provision] {self.address_string()} {fmt % args}\n')

        def _send_html(self, code: int, body: str) -> None:
            data = body.encode('utf-8')
            self.send_response(code)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def _render_form(self) -> None:
            hint = (
                f' · {len(state.networks)} networks found'
                if state.networks
                else ' · no networks cached — try Refresh or Other'
            )
            page = FORM_HTML.format(
                ap_ssid=html.escape(state.ap_ssid),
                ap_password=html.escape(AP_PASSWORD),
                mac=html.escape(state.mac or '—'),
                serial=html.escape(state.serial or '—'),
                wifi_options=wifi_options_html(state.networks),
                wifi_hint=hint,
                message=state.message,
            )
            self._send_html(200, page)

        def do_GET(self) -> None:  # noqa: N802
            path = self.path.split('?', 1)[0]
            if path == '/rescan':
                try:
                    # Soft rescan while AP is up (may return partial list).
                    found = scan_wifi_networks(rescan=True)
                    if found:
                        state.networks = found
                        state.message = (
                            f'<p class="ok">Refreshed — {len(found)} networks.</p>'
                        )
                    else:
                        state.message = (
                            '<p class="err">Rescan returned empty (normal while AP is on). '
                            'Use the list from boot or choose Other.</p>'
                        )
                except Exception as exc:  # noqa: BLE001
                    state.message = f'<p class="err">Rescan failed: {html.escape(str(exc))}</p>'
                self.send_response(302)
                self.send_header('Location', '/')
                self.end_headers()
                return
            self._render_form()

        def do_POST(self) -> None:  # noqa: N802
            length = int(self.headers.get('Content-Length', '0') or 0)
            raw = self.rfile.read(length).decode('utf-8', errors='replace')
            form = {k: (v[0] if v else '') for k, v in parse_qs(raw, keep_blank_values=True).items()}
            if state.busy:
                state.message = '<p class="err">Already joining — wait…</p>'
                self._render_form()
                return
            wifi_ssid = form.get('wifi_ssid', '').strip() or form.get('wifi_ssid_custom', '').strip()
            wifi_password = form.get('wifi_password', '')
            robot_name = form.get('robot_name', '').strip()
            server_ip = form.get('server_ip', '').strip()
            token = form.get('provisioning_token', '').strip()
            if not all([wifi_ssid, wifi_password, robot_name, server_ip, token]):
                state.message = '<p class="err">All fields are required (pick a Wi‑Fi from the list).</p>'
                self._render_form()
                return

            state.busy = True
            state.message = '<p class="ok">Joining Wi‑Fi and registering… keep this page open.</p>'
            self._render_form()

            def worker() -> None:
                try:
                    connect_site_wifi(wifi_ssid, wifi_password)
                    cfg = FleetConfig(
                        name=robot_name,
                        serial=state.serial,
                        server_ip=server_ip,
                        provisioning_token=token,
                        wifi_ssid=wifi_ssid,
                        nav_mode='HARDWARE',
                    )
                    # Persist before register so reboot keeps credentials.
                    save_fleet_config(cfg)
                    # Register (retry a few times while DHCP settles).
                    last_err: Optional[Exception] = None
                    for attempt in range(1, 9):
                        try:
                            data = register(cfg)
                            save_fleet_config(cfg)
                            state.message = (
                                f'<p class="ok">Registered as <strong>{html.escape(cfg.name)}</strong> '
                                f'(id {html.escape(cfg.robot_id)}). '
                                f'Status: {html.escape(str(data.get("status", "")))}. '
                                'You can close this page and open the fleet GUI.</p>'
                            )
                            state.done = True
                            maybe_restart_fleet_units()
                            return
                        except Exception as exc:  # noqa: BLE001
                            last_err = exc
                            time.sleep(2.0 * attempt)
                    raise RuntimeError(f'register failed after retries: {last_err}')
                except Exception as exc:  # noqa: BLE001
                    state.message = f'<p class="err">{html.escape(str(exc))}</p>'
                finally:
                    state.busy = False

            threading.Thread(target=worker, daemon=True).start()

    return Handler


def main(argv: Optional[list[str]] = None) -> int:
    _ = argv
    if DEFAULT_FLEET_PATH.is_file():
        print(f'{DEFAULT_FLEET_PATH} exists — provision portal exiting (already provisioned).')
        return 0

    serial = read_cpu_serial()
    print(f'NavPro provision portal starting (serial={serial})')
    try:
        ap_ssid, mac, networks = start_access_point(serial)
    except Exception as exc:  # noqa: BLE001
        print(f'Failed to start AP: {exc}', file=sys.stderr)
        return 1
    print(
        f'AP up: {ap_ssid}  password={AP_PASSWORD} (fixed)  mac={mac}  '
        f'portal=http://{AP_ADDR}/  ssids={len(networks)}'
    )

    state = PortalState(serial=serial, ap_ssid=ap_ssid, mac=mac, networks=networks)
    handler = make_handler(state)
    # Bind all interfaces on port 80 (needs root / CAP_NET_BIND_SERVICE).
    port = int(os.environ.get('NAVPRO_PROVISION_PORT', '80'))
    server = HTTPServer(('0.0.0.0', port), handler)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print('provision portal stopped')
    finally:
        server.server_close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
