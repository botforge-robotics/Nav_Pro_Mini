"""Load / write /etc/navpro/fleet.yaml (robot identity after provisioning)."""

from __future__ import annotations

import os
import socket
import uuid
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Optional

import yaml

DEFAULT_FLEET_PATH = Path(os.environ.get('NAVPRO_FLEET_YAML', '/etc/navpro/fleet.yaml'))
DEFAULT_MAPS_DIR = Path(os.environ.get('NAVPRO_MAPS_DIR', '/var/lib/navpro/maps'))


@dataclass
class FleetConfig:
    name: str = ''
    serial: str = ''
    server_ip: str = ''
    provisioning_token: str = ''
    wifi_ssid: str = ''
    robot_id: str = ''
    server_port: int = 80
    ros_domain_id: int = 0
    zenoh_port: int = 7447
    nav_mode: str = 'HARDWARE'  # HARDWARE | MAPPING | NAV_READY | NAV_ACTIVE
    extra: dict[str, Any] = field(default_factory=dict)

    @property
    def api_base(self) -> str:
        return f'http://{self.server_ip}:{self.server_port}/api/v1'

    @property
    def zenoh_endpoint(self) -> str:
        return f'tcp/{self.server_ip}:{self.zenoh_port}'

    def to_dict(self) -> dict[str, Any]:
        d = asdict(self)
        extra = d.pop('extra', {}) or {}
        d.update(extra)
        return {k: v for k, v in d.items() if v != '' and v is not None}


def read_cpu_serial() -> str:
    """Stable board id from device-tree or machine-id."""
    for path in (
        Path('/sys/firmware/devicetree/base/serial-number'),
        Path('/proc/device-tree/serial-number'),
    ):
        try:
            raw = path.read_bytes().rstrip(b'\x00').decode('utf-8', errors='ignore').strip()
            if raw:
                return raw
        except OSError:
            pass
    try:
        mid = Path('/etc/machine-id').read_text(encoding='utf-8').strip()
        if mid:
            return mid[:16]
    except OSError:
        pass
    return f'navpro-{uuid.getnode():012x}'


def primary_mac(iface: str = 'wlan0') -> str:
    """Stable Wi‑Fi MAC (prefer wlan0), lowercase colon form e.g. aa:bb:cc:dd:ee:ff."""
    candidates = [
        Path(f'/sys/class/net/{iface}/address'),
        Path('/sys/class/net/wlan0/address'),
        Path('/sys/class/net/wlp1s0/address'),
    ]
    for p in candidates:
        try:
            mac = p.read_text(encoding='utf-8').strip().lower()
            if mac and mac != '00:00:00:00:00:00' and len(mac) >= 11:
                return mac
        except OSError:
            continue
    # Fallback: first non-loopback iface
    try:
        for entry in sorted(Path('/sys/class/net').iterdir()):
            name = entry.name
            if name == 'lo' or name.startswith('docker') or name.startswith('veth'):
                continue
            addr = entry / 'address'
            mac = addr.read_text(encoding='utf-8').strip().lower()
            if mac and mac != '00:00:00:00:00:00':
                return mac
    except OSError:
        pass
    try:
        return ':'.join(f'{(uuid.getnode() >> (8 * i)) & 0xff:02x}' for i in reversed(range(6)))
    except Exception:  # noqa: BLE001
        return ''


def mac_tail(mac: str, nibble_chars: int = 6) -> str:
    """Last N hex chars of MAC without colons (default 6 → unique-ish SSID suffix)."""
    hex_only = ''.join(c for c in (mac or '') if c.isalnum())
    if len(hex_only) < nibble_chars:
        return (hex_only or '000000').upper().rjust(nibble_chars, '0')[-nibble_chars:]
    return hex_only[-nibble_chars:].upper()


def ap_ssid_from_mac(mac: str | None = None) -> str:
    """Hotspot SSID: NavPro-Setup-<last6 of MAC> e.g. NavPro-Setup-DDEEFF."""
    m = mac or primary_mac()
    return f'NavPro-Setup-{mac_tail(m, 6)}'


# Fixed password for every robot setup AP (operators use the same password site-wide).
DEFAULT_AP_PASSWORD = 'navprosetup'


def ap_suffix(serial: str) -> str:
    """Deprecated: prefer ap_ssid_from_mac(). Kept for older call sites."""
    cleaned = ''.join(c for c in serial if c.isalnum())
    return (cleaned[-4:] or '0000').upper()


def load_fleet_config(path: Path = DEFAULT_FLEET_PATH) -> Optional[FleetConfig]:
    if not path.is_file():
        return None
    with path.open(encoding='utf-8') as f:
        raw = yaml.safe_load(f) or {}
    if not isinstance(raw, dict):
        return None
    robot = raw.get('robot') if isinstance(raw.get('robot'), dict) else {}

    server_ip = str(raw.get('server_ip') or '')
    if not server_ip:
        url = str(raw.get('server_url') or '')
        if '://' in url:
            url = url.split('://', 1)[1]
        server_ip = url.split('/')[0].split(':')[0] if url else ''

    zenoh_port = int(raw.get('zenoh_port') or 7447)
    zenoh_ep = str(raw.get('zenoh_endpoint') or '')
    if zenoh_ep.startswith('tcp/') and ':' in zenoh_ep:
        try:
            zenoh_port = int(zenoh_ep.rsplit(':', 1)[-1])
        except ValueError:
            pass

    known = {
        'name', 'serial', 'server_ip', 'provisioning_token', 'wifi_ssid',
        'robot_id', 'server_port', 'ros_domain_id', 'zenoh_port', 'nav_mode',
        'robot', 'server_url', 'zenoh_endpoint', 'map_name', 'start_nav', 'start_slam',
    }
    extra = {k: v for k, v in raw.items() if k not in known}
    return FleetConfig(
        name=str(raw.get('name') or robot.get('name') or ''),
        serial=str(raw.get('serial') or robot.get('serial') or read_cpu_serial()),
        server_ip=server_ip,
        provisioning_token=str(raw.get('provisioning_token') or ''),
        wifi_ssid=str(raw.get('wifi_ssid') or ''),
        robot_id=str(raw.get('robot_id') or ''),
        server_port=int(raw.get('server_port') or 80),
        ros_domain_id=int(raw.get('ros_domain_id') or 0),
        zenoh_port=zenoh_port,
        nav_mode=str(raw.get('nav_mode') or 'HARDWARE'),
        extra=extra,
    )


def save_fleet_config(cfg: FleetConfig, path: Path = DEFAULT_FLEET_PATH) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    data = cfg.to_dict()
    tmp = path.with_suffix('.yaml.tmp')
    with tmp.open('w', encoding='utf-8') as f:
        yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False)
    tmp.replace(path)


def hostname() -> str:
    return socket.gethostname()
