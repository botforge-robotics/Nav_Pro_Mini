#!/usr/bin/env python3
"""Captive portal for first-boot Wi‑Fi + fleet join (Phase A).

Runs only when /etc/navpro/fleet.yaml is missing (systemd ConditionPathExists).
Creates AP NavPro-Setup-<MAC>, serves http://10.42.0.1/, then joins site Wi‑Fi
and registers with the fleet server.
"""

from __future__ import annotations

import html
import json
import os
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from typing import Any, Optional
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
AP_PASSWORD = os.environ.get('NAVPRO_AP_PASSWORD', DEFAULT_AP_PASSWORD)
AP_ADDR = '10.42.0.1'
CONN_AP = 'navpro-setup-ap'
CONN_SITE = 'navpro-site-wifi'

# Shared modern shell CSS (setup form + status page).
SHELL_CSS = """
@import url('https://fonts.googleapis.com/css2?family=DM+Sans:ital,opsz,wght@0,9..40,400;0,9..40,500;0,9..40,600;0,9..40,700;1,9..40,400&family=Syne:wght@600;700;800&display=swap');
:root {
  --bg0: #071018;
  --bg1: #0c1a24;
  --card: rgba(14, 28, 38, 0.88);
  --line: rgba(120, 180, 200, 0.18);
  --text: #e8f2f6;
  --muted: #8aa3b0;
  --accent: #1ec98a;
  --accent2: #2ec4b6;
  --warn: #e8b84a;
  --err: #ff6b6b;
  --cyan: #3ecfff;
  --radius: 16px;
}
* { box-sizing: border-box; }
html, body { margin: 0; min-height: 100%; }
body {
  font-family: 'DM Sans', system-ui, sans-serif;
  color: var(--text);
  background:
    radial-gradient(1200px 600px at 10% -10%, rgba(46, 196, 182, 0.18), transparent 55%),
    radial-gradient(900px 500px at 100% 0%, rgba(30, 201, 138, 0.12), transparent 50%),
    linear-gradient(165deg, var(--bg0), var(--bg1) 45%, #08141c);
  background-attachment: fixed;
}
.wrap {
  max-width: 440px;
  margin: 0 auto;
  padding: 1.75rem 1.25rem 2.5rem;
}
.brand {
  display: flex; align-items: center; gap: 0.75rem;
  margin-bottom: 1.5rem;
}
.mark {
  width: 42px; height: 42px; border-radius: 12px;
  background: linear-gradient(135deg, var(--accent2), var(--accent));
  display: grid; place-items: center;
  font-family: Syne, sans-serif; font-weight: 800; font-size: 0.95rem;
  color: #042018; letter-spacing: -0.02em;
  box-shadow: 0 8px 24px rgba(30, 201, 138, 0.25);
}
.brand h1 {
  font-family: Syne, sans-serif; font-weight: 700;
  font-size: 1.35rem; margin: 0; letter-spacing: -0.02em;
}
.brand p { margin: 0.15rem 0 0; color: var(--muted); font-size: 0.85rem; }
.card {
  background: var(--card);
  border: 1px solid var(--line);
  border-radius: var(--radius);
  padding: 1.35rem 1.25rem 1.5rem;
  backdrop-filter: blur(12px);
  box-shadow: 0 20px 50px rgba(0,0,0,0.35);
}
.meta {
  display: grid; gap: 0.45rem;
  margin-bottom: 1.15rem;
  padding: 0.75rem 0.85rem;
  border-radius: 12px;
  background: rgba(0,0,0,0.22);
  border: 1px solid var(--line);
  font-size: 0.8rem; color: var(--muted);
}
.meta strong { color: var(--text); font-weight: 600; }
.meta code {
  font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
  font-size: 0.78rem; color: var(--accent2);
}
label {
  display: block; margin-top: 0.95rem;
  font-size: 0.78rem; font-weight: 600;
  letter-spacing: 0.04em; text-transform: uppercase; color: var(--muted);
}
input, select {
  width: 100%; margin-top: 0.35rem;
  padding: 0.7rem 0.85rem;
  border-radius: 10px;
  border: 1px solid var(--line);
  background: rgba(4, 14, 20, 0.75);
  color: var(--text); font: inherit; font-size: 0.95rem;
  outline: none; transition: border-color 0.15s, box-shadow 0.15s;
}
input:focus, select:focus {
  border-color: rgba(46, 196, 182, 0.55);
  box-shadow: 0 0 0 3px rgba(46, 196, 182, 0.15);
}
select option { background: #0c1a24; }
.row-actions { display: flex; gap: 0.5rem; flex-wrap: wrap; margin-top: 0.55rem; }
a.ghost, button.ghost {
  display: inline-flex; align-items: center; gap: 0.35rem;
  padding: 0.4rem 0.7rem; border-radius: 8px;
  border: 1px solid var(--line); background: transparent;
  color: var(--accent2); font: inherit; font-size: 0.82rem;
  text-decoration: none; cursor: pointer;
}
a.ghost:hover, button.ghost:hover { background: rgba(46,196,182,0.08); }
button.primary {
  width: 100%; margin-top: 1.35rem;
  padding: 0.85rem 1rem; border: 0; border-radius: 12px;
  background: linear-gradient(135deg, var(--accent2), var(--accent));
  color: #042018; font: inherit; font-weight: 700; font-size: 1rem;
  cursor: pointer; letter-spacing: -0.01em;
  box-shadow: 0 10px 28px rgba(30, 201, 138, 0.28);
}
button.primary:disabled { opacity: 0.55; cursor: wait; }
.hint { margin: 0.85rem 0 0; color: var(--muted); font-size: 0.82rem; line-height: 1.45; }
#custom_ssid_wrap { display: none; }
.flash { margin-top: 0.9rem; padding: 0.7rem 0.85rem; border-radius: 10px; font-size: 0.88rem; }
.flash.ok { background: rgba(30,201,138,0.12); color: #8dffc4; border: 1px solid rgba(30,201,138,0.3); }
.flash.err { background: rgba(255,107,107,0.12); color: #ffb4b4; border: 1px solid rgba(255,107,107,0.3); }

/* Status page */
.status-hero { text-align: center; padding: 0.5rem 0 0.25rem; }
.status-hero h2 {
  font-family: Syne, sans-serif; font-weight: 700;
  font-size: 1.45rem; margin: 0.75rem 0 0.35rem; letter-spacing: -0.02em;
}
.status-hero p { margin: 0; color: var(--muted); font-size: 0.92rem; line-height: 1.45; }
.orb {
  width: 72px; height: 72px; margin: 0 auto;
  border-radius: 50%;
  display: grid; place-items: center;
  position: relative;
}
.orb::before {
  content: ''; position: absolute; inset: -6px; border-radius: 50%;
  border: 2px solid transparent;
}
.orb.spin::before {
  border-top-color: var(--cyan);
  border-right-color: rgba(62,207,255,0.25);
  animation: spin 0.9s linear infinite;
}
.orb.ok { background: rgba(30,201,138,0.15); color: var(--accent); }
.orb.err { background: rgba(255,107,107,0.15); color: var(--err); }
.orb.busy { background: rgba(62,207,255,0.12); color: var(--cyan); }
.orb svg { width: 34px; height: 34px; }
@keyframes spin { to { transform: rotate(360deg); } }
.steps { list-style: none; margin: 1.25rem 0 0; padding: 0; }
.steps li {
  display: flex; gap: 0.75rem; align-items: flex-start;
  padding: 0.7rem 0; border-top: 1px solid var(--line);
  font-size: 0.9rem;
}
.steps li:first-child { border-top: 0; }
.dot {
  width: 22px; height: 22px; border-radius: 50%; flex-shrink: 0;
  margin-top: 1px; display: grid; place-items: center;
  font-size: 0.7rem; font-weight: 700;
  background: rgba(255,255,255,0.06); color: var(--muted);
  border: 1px solid var(--line);
}
.dot.done { background: rgba(30,201,138,0.2); color: var(--accent); border-color: rgba(30,201,138,0.4); }
.dot.active { background: rgba(62,207,255,0.2); color: var(--cyan); border-color: rgba(62,207,255,0.45); }
.dot.fail { background: rgba(255,107,107,0.2); color: var(--err); border-color: rgba(255,107,107,0.4); }
.step-body strong { display: block; font-weight: 600; }
.step-body span { color: var(--muted); font-size: 0.82rem; }
.led-guide {
  margin-top: 1.15rem; padding: 0.85rem;
  border-radius: 12px; background: rgba(0,0,0,0.25);
  border: 1px solid var(--line); font-size: 0.82rem; color: var(--muted);
}
.led-guide h3 {
  margin: 0 0 0.55rem; font-size: 0.72rem; text-transform: uppercase;
  letter-spacing: 0.06em; color: var(--text); font-weight: 700;
}
.leds { display: grid; gap: 0.4rem; }
.led-row { display: flex; align-items: center; gap: 0.55rem; }
.swatch {
  width: 12px; height: 12px; border-radius: 50%;
  box-shadow: 0 0 10px currentColor;
}
.swatch.amber { color: #e8b84a; background: #e8b84a; }
.swatch.cyan { color: #3ecfff; background: #3ecfff; }
.swatch.yellow { color: #f0d060; background: #f0d060; }
.swatch.green { color: #1ec98a; background: #1ec98a; }
.detail {
  margin-top: 0.85rem; padding: 0.7rem 0.85rem;
  border-radius: 10px; background: rgba(0,0,0,0.2);
  font-size: 0.84rem; color: var(--muted); word-break: break-word;
}
.detail strong { color: var(--text); }
"""

FORM_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1"/>
<title>NavPro · Robot setup</title>
<style>{css}</style>
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
  var btn = form.querySelector('button.primary');
  if (btn) {{ btn.disabled = true; btn.textContent = 'Submitting…'; }}
  return true;
}}
</script>
</head>
<body>
  <div class="wrap">
    <div class="brand">
      <div class="mark">NP</div>
      <div>
        <h1>NavPro</h1>
        <p>Robot setup</p>
      </div>
    </div>
    <div class="card">
      <div class="meta">
        <div>Hotspot <strong><code>{ap_ssid}</code></strong></div>
        <div>Password <strong><code>{ap_password}</code></strong></div>
        <div>MAC {mac} · Board {serial}</div>
      </div>
      <form method="POST" action="/save" onsubmit="return beforeSubmit(this)">
        <input type="hidden" name="wifi_ssid" id="wifi_ssid" value=""/>
        <label>Site Wi‑Fi</label>
        <select id="wifi_ssid_select" required onchange="onSsidChange(this)">
          <option value="">Select network…</option>
          {wifi_options}
          <option value="__other__">Other (type SSID)…</option>
        </select>
        <div id="custom_ssid_wrap">
          <label>Custom SSID</label>
          <input id="wifi_ssid_custom" name="wifi_ssid_custom" autocomplete="off" placeholder="Hidden / not listed"/>
        </div>
        <div class="row-actions">
          <a class="ghost" href="/rescan">Refresh networks</a>
          <span class="hint" style="margin:0;align-self:center">{wifi_hint}</span>
        </div>
        <label>Wi‑Fi password</label>
        <input name="wifi_password" type="password" required autocomplete="off" placeholder="Site Wi‑Fi password"/>
        <label>Robot name</label>
        <input name="robot_name" required placeholder="bot-1" autocomplete="off"/>
        <label>Fleet server IP</label>
        <input name="server_ip" required placeholder="192.168.1.10" autocomplete="off"/>
        <label>Provisioning token</label>
        <input name="provisioning_token" required autocomplete="off" placeholder="From server .env"/>
        <button class="primary" type="submit">Save &amp; join fleet</button>
      </form>
      <p class="hint">Fleet and floor are assigned later in the fleet GUI · Devices.</p>
      {message}
    </div>
  </div>
</body>
</html>
"""

STATUS_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1"/>
<title>NavPro · Joining…</title>
<style>{css}</style>
</head>
<body>
  <div class="wrap">
    <div class="brand">
      <div class="mark">NP</div>
      <div>
        <h1>NavPro</h1>
        <p>Connection status</p>
      </div>
    </div>
    <div class="card">
      <div class="status-hero">
        <div id="orb" class="orb busy spin" aria-hidden="true">
          <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
            <path d="M12 3v3M12 18v3M3 12h3M18 12h3M5.6 5.6l2.1 2.1M16.3 16.3l2.1 2.1M5.6 18.4l2.1-2.1M16.3 7.7l2.1-2.1"/>
          </svg>
        </div>
        <h2 id="title">Connecting…</h2>
        <p id="subtitle">Watch the robot LED strip while we join Wi‑Fi and register.</p>
      </div>
      <ul class="steps">
        <li>
          <div id="d1" class="dot">1</div>
          <div class="step-body"><strong>Leave setup hotspot</strong><span id="s1">Waiting…</span></div>
        </li>
        <li>
          <div id="d2" class="dot">2</div>
          <div class="step-body"><strong>Join site Wi‑Fi</strong><span id="s2">Waiting…</span></div>
        </li>
        <li>
          <div id="d3" class="dot">3</div>
          <div class="step-body"><strong>Register with fleet</strong><span id="s3">Waiting…</span></div>
        </li>
      </ul>
      <div class="led-guide">
        <h3>Robot LED guide</h3>
        <div class="leds">
          <div class="led-row"><span class="swatch amber"></span> Amber blink — setup / waiting</div>
          <div class="led-row"><span class="swatch cyan"></span> Cyan — joining Wi‑Fi &amp; fleet</div>
          <div class="led-row"><span class="swatch yellow"></span> Yellow — registered, need map</div>
          <div class="led-row"><span class="swatch green"></span> Green — nav ready</div>
        </div>
      </div>
      <div id="detail" class="detail" style="display:none"></div>
      <div id="actions" style="display:none;margin-top:1rem">
        <a class="ghost" href="/" id="retry">Back to setup</a>
      </div>
    </div>
  </div>
<script>
(function() {{
  var orb = document.getElementById('orb');
  var title = document.getElementById('title');
  var subtitle = document.getElementById('subtitle');
  var detail = document.getElementById('detail');
  var actions = document.getElementById('actions');
  function setStep(n, cls, text) {{
    var d = document.getElementById('d' + n);
    var s = document.getElementById('s' + n);
    d.className = 'dot ' + (cls || '');
    if (cls === 'done') d.textContent = '✓';
    else if (cls === 'fail') d.textContent = '!';
    else d.textContent = String(n);
    s.textContent = text || '';
  }}
  function paint(st) {{
    var phase = st.phase || 'idle';
    var msg = st.message || '';
    if (phase === 'joining_wifi') {{
      orb.className = 'orb busy spin';
      title.textContent = 'Joining Wi‑Fi…';
      subtitle.textContent = 'Robot LED should turn cyan. Your phone may lose this hotspot.';
      setStep(1, 'done', 'Leaving setup AP');
      setStep(2, 'active', 'Connecting to ' + (st.wifi_ssid || 'site Wi‑Fi') + '…');
      setStep(3, '', 'Waiting…');
    }} else if (phase === 'registering') {{
      orb.className = 'orb busy spin';
      title.textContent = 'Registering…';
      subtitle.textContent = 'Talking to the fleet server at ' + (st.server_ip || '…');
      setStep(1, 'done', 'Left setup hotspot');
      setStep(2, 'done', 'On ' + (st.wifi_ssid || 'site Wi‑Fi'));
      setStep(3, 'active', 'Registering robot…');
    }} else if (phase === 'success') {{
      orb.className = 'orb ok';
      orb.innerHTML = '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5"><path d="M5 13l4 4L19 7"/></svg>';
      title.textContent = 'Submitted successfully';
      subtitle.textContent = 'Robot “‘ + (st.robot_name || '') + '” is on the fleet. LED may show yellow (need map) until you commission.';
      setStep(1, 'done', 'Left setup hotspot');
      setStep(2, 'done', 'Connected to ' + (st.wifi_ssid || 'Wi‑Fi'));
      setStep(3, 'done', 'Registered · status ' + (st.robot_status || 'ok'));
      detail.style.display = 'block';
      detail.innerHTML = '<strong>Name</strong> ' + (st.robot_name || '') +
        '<br/><strong>ID</strong> ' + (st.robot_id || '') +
        '<br/><strong>Next</strong> Open the fleet GUI on your PC → Devices, then Mapping if needed.';
      actions.style.display = 'none';
    }} else if (phase === 'error') {{
      orb.className = 'orb err';
      orb.innerHTML = '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5"><path d="M6 6l12 12M18 6L6 18"/></svg>';
      title.textContent = 'Setup failed';
      subtitle.textContent = 'Check Wi‑Fi password / server IP / token. LED may blink red.';
      setStep(1, st.wifi_ok ? 'done' : 'fail', st.wifi_ok ? 'Left setup AP' : 'Could not leave / join');
      setStep(2, st.wifi_ok ? 'done' : 'fail', st.wifi_ok ? ('On ' + (st.wifi_ssid || 'Wi‑Fi')) : 'Wi‑Fi join failed');
      setStep(3, 'fail', 'Registration incomplete');
      detail.style.display = 'block';
      detail.innerHTML = '<strong>Error</strong> ' + (msg || 'Unknown error');
      actions.style.display = 'block';
    }} else if (phase === 'idle' || phase === 'submitted') {{
      orb.className = 'orb busy spin';
      title.textContent = 'Starting…';
      subtitle.textContent = 'Submitted — starting Wi‑Fi join.';
      setStep(1, 'active', 'Preparing…');
    }}
  }}
  function tick() {{
    fetch('/api/status', {{ cache: 'no-store' }}).then(function(r) {{ return r.json(); }}).then(function(st) {{
      paint(st);
      if (st.phase !== 'success' && st.phase !== 'error') {{
        setTimeout(tick, 800);
      }}
    }}).catch(function() {{ setTimeout(tick, 1200); }});
  }}
  tick();
}})();
</script>
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
    """Return [(ssid, signal%), ...] sorted by signal, unique SSIDs."""
    args = ['-t', '-f', 'SSID,SIGNAL', 'device', 'wifi', 'list', 'ifname', AP_IFACE]
    if rescan:
        args.extend(['--rescan', 'yes'])
    r = _nmcli(*args)
    if r.returncode != 0:
        r = _nmcli('-t', '-f', 'SSID,SIGNAL', 'device', 'wifi', 'list', 'ifname', AP_IFACE)
    seen: dict[str, int] = {}
    for line in (r.stdout or '').splitlines():
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
        if signal > seen.get(ssid, -1):
            seen[ssid] = signal
    return sorted(seen.items(), key=lambda kv: (-kv[1], kv[0].lower()))


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


def write_display_hint(
    state: str,
    line2: str = '',
    line3: str = '',
) -> None:
    """Hint file for status_display / start_display.sh."""
    path = '/run/navpro/display_state'
    try:
        os.makedirs('/run/navpro', exist_ok=True)
        with open(path, 'w', encoding='utf-8') as f:
            f.write(f'{state}\n{line2}\n{line3}\n')
    except OSError:
        pass


def start_access_point(_serial: str) -> tuple[str, str, list[tuple[str, int]]]:
    """Start setup AP. Returns (ssid, mac, wifi_list)."""
    mac = primary_mac(AP_IFACE)
    ssid = ap_ssid_from_mac(mac)
    write_display_hint('setup', ssid, AP_PASSWORD)
    for name in (CONN_AP, CONN_SITE):
        _nmcli('connection', 'delete', name)
    _nmcli('device', 'set', AP_IFACE, 'managed', 'yes')
    _nmcli('radio', 'wifi', 'on')

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
        _nmcli('connection', 'up', CONN_AP)
        raise RuntimeError(f'Wi‑Fi join failed: {r.stderr or r.stdout}')
    for _ in range(30):
        time.sleep(1.0)
        ip = _run(['hostname', '-I'])
        if ip.returncode == 0 and ip.stdout.strip():
            return


def maybe_restart_fleet_units() -> None:
    # Let Wi‑Fi/DHCP settle, then stop portal and start fleet agent (heartbeat).
    time.sleep(3.0)
    _run(['systemctl', 'stop', 'navpro-provision'], check=False)
    for unit in ('navpro-robot', 'navpro-fleet', 'navpro-display'):
        # start (not only restart) so ConditionPathExists is re-evaluated
        # after fleet.yaml was created during this session.
        _run(['systemctl', 'start', unit], check=False)
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
        # Status page fields
        self.phase = 'idle'  # idle|submitted|joining_wifi|registering|success|error
        self.wifi_ssid = ''
        self.server_ip = ''
        self.robot_name = ''
        self.robot_id = ''
        self.robot_status = ''
        self.wifi_ok = False
        self.error = ''
        self.lock = threading.Lock()

    def status_dict(self) -> dict[str, Any]:
        with self.lock:
            return {
                'phase': self.phase,
                'message': self.error or self.message,
                'wifi_ssid': self.wifi_ssid,
                'server_ip': self.server_ip,
                'robot_name': self.robot_name,
                'robot_id': self.robot_id,
                'robot_status': self.robot_status,
                'wifi_ok': self.wifi_ok,
                'busy': self.busy,
                'done': self.done,
            }

    def set_phase(self, phase: str, **kwargs: Any) -> None:
        with self.lock:
            self.phase = phase
            for k, v in kwargs.items():
                if hasattr(self, k):
                    setattr(self, k, v)


def make_handler(state: PortalState):  # noqa: ANN201
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt: str, *args) -> None:  # noqa: ANN002
            sys.stderr.write(f'[provision] {self.address_string()} {fmt % args}\n')

        def _send_html(self, code: int, body: str) -> None:
            data = body.encode('utf-8')
            self.send_response(code)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(data)

        def _send_json(self, code: int, payload: dict[str, Any]) -> None:
            data = json.dumps(payload).encode('utf-8')
            self.send_response(code)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(data)

        def _redirect(self, location: str) -> None:
            self.send_response(302)
            self.send_header('Location', location)
            self.end_headers()

        def _render_form(self, flash: str = '') -> None:
            hint = (
                f'{len(state.networks)} found'
                if state.networks
                else 'none cached — Refresh or Other'
            )
            msg = flash or state.message
            page = FORM_HTML.format(
                css=SHELL_CSS,
                ap_ssid=html.escape(state.ap_ssid),
                ap_password=html.escape(AP_PASSWORD),
                mac=html.escape(state.mac or '—'),
                serial=html.escape(state.serial or '—'),
                wifi_options=wifi_options_html(state.networks),
                wifi_hint=hint,
                message=msg,
            )
            self._send_html(200, page)

        def _render_status(self) -> None:
            page = STATUS_HTML.format(css=SHELL_CSS)
            self._send_html(200, page)

        def do_GET(self) -> None:  # noqa: N802
            path = self.path.split('?', 1)[0]
            # Captive-portal probes → show setup
            if path in (
                '/generate_204', '/hotspot-detect.html', '/ncsi.txt',
                '/connecttest.txt', '/success.txt', '/canonical.html',
            ):
                self._redirect('/')
                return
            if path == '/api/status':
                self._send_json(200, state.status_dict())
                return
            if path == '/status':
                self._render_status()
                return
            if path == '/rescan':
                try:
                    found = scan_wifi_networks(rescan=True)
                    if found:
                        state.networks = found
                        flash = f'<div class="flash ok">Refreshed — {len(found)} networks.</div>'
                    else:
                        flash = (
                            '<div class="flash err">Rescan empty (normal while AP is on). '
                            'Use the boot list or Other.</div>'
                        )
                except Exception as exc:  # noqa: BLE001
                    flash = f'<div class="flash err">Rescan failed: {html.escape(str(exc))}</div>'
                self._render_form(flash)
                return
            # Default / and anything else → form (unless already mid-join)
            if state.phase not in ('idle', 'error') and path == '/':
                self._redirect('/status')
                return
            self._render_form()

        def do_POST(self) -> None:  # noqa: N802
            path = self.path.split('?', 1)[0]
            if path not in ('/save', '/'):
                self._send_html(404, '<h1>Not found</h1>')
                return
            length = int(self.headers.get('Content-Length', '0') or 0)
            raw = self.rfile.read(length).decode('utf-8', errors='replace')
            form = {k: (v[0] if v else '') for k, v in parse_qs(raw, keep_blank_values=True).items()}

            if state.busy:
                self._redirect('/status')
                return

            wifi_ssid = form.get('wifi_ssid', '').strip() or form.get('wifi_ssid_custom', '').strip()
            wifi_password = form.get('wifi_password', '')
            robot_name = form.get('robot_name', '').strip()
            server_ip = form.get('server_ip', '').strip()
            token = form.get('provisioning_token', '').strip()
            if not all([wifi_ssid, wifi_password, robot_name, server_ip, token]):
                self._render_form(
                    '<div class="flash err">All fields are required (pick a Wi‑Fi from the list).</div>'
                )
                return

            state.busy = True
            state.set_phase(
                'submitted',
                wifi_ssid=wifi_ssid,
                server_ip=server_ip,
                robot_name=robot_name,
                error='',
                wifi_ok=False,
                done=False,
            )
            self._redirect('/status')

            def worker() -> None:
                try:
                    state.set_phase('joining_wifi')
                    write_display_hint('joining', robot_name)
                    connect_site_wifi(wifi_ssid, wifi_password)
                    state.set_phase('registering', wifi_ok=True)
                    write_display_hint('joining', robot_name)
                    cfg = FleetConfig(
                        name=robot_name,
                        serial=state.serial,
                        server_ip=server_ip,
                        provisioning_token=token,
                        wifi_ssid=wifi_ssid,
                        nav_mode='HARDWARE',
                    )
                    save_fleet_config(cfg)
                    last_err: Optional[Exception] = None
                    data: dict[str, Any] = {}
                    for attempt in range(1, 9):
                        try:
                            data = register(cfg)
                            save_fleet_config(cfg)
                            write_display_hint('need_map', cfg.name)
                            state.set_phase(
                                'success',
                                robot_id=cfg.robot_id,
                                robot_name=cfg.name,
                                robot_status=str(data.get('status') or 'registered'),
                                done=True,
                                error='',
                            )
                            maybe_restart_fleet_units()
                            return
                        except Exception as exc:  # noqa: BLE001
                            last_err = exc
                            time.sleep(2.0 * attempt)
                    raise RuntimeError(f'register failed after retries: {last_err}')
                except Exception as exc:  # noqa: BLE001
                    write_display_hint('error', robot_name)
                    state.set_phase('error', error=str(exc), done=False)
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
