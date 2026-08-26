"""Load / write /etc/navpro/robot.yaml (robot identity after Wi‑Fi setup)."""

from __future__ import annotations

import os
import socket
import uuid
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Optional

import yaml

DEFAULT_ROBOT_PATH = Path(os.environ.get('NAVPRO_ROBOT_YAML', '/etc/navpro/robot.yaml'))
# Older installs may still have this path; treated as configured if present.
_ALT_ROBOT_PATH = Path('/etc/navpro/fleet.yaml')


@dataclass
class RobotConfig:
    name: str = ''
    serial: str = ''
    wifi_ssid: str = ''
    # IANA zone (e.g. "Asia/Kolkata"), set once during first-time setup —
    # see provision_portal.py. Empty means never set (an older install, or
    # setup run before this field existed): the system clock keeps whatever
    # timezone the OS image shipped with, which is very likely wrong for the
    # robot's actual deployment site.
    timezone: str = ''
    extra: dict[str, Any] = field(default_factory=dict)

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
    """Stable Wi‑Fi MAC (prefer wlan0), lowercase colon form."""
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
    hex_only = ''.join(c for c in (mac or '') if c.isalnum())
    if len(hex_only) < nibble_chars:
        return (hex_only or '000000').upper().rjust(nibble_chars, '0')[-nibble_chars:]
    return hex_only[-nibble_chars:].upper()


def ap_ssid_from_mac(mac: str | None = None) -> str:
    """Hotspot SSID: NavPro-Setup-<last6 of MAC>."""
    m = mac or primary_mac()
    return f'NavPro-Setup-{mac_tail(m, 6)}'


DEFAULT_AP_PASSWORD = 'navprosetup'


def config_path_present() -> bool:
    return DEFAULT_ROBOT_PATH.is_file() or _ALT_ROBOT_PATH.is_file()


def load_robot_config(path: Path | None = None) -> Optional[RobotConfig]:
    path = path or (
        DEFAULT_ROBOT_PATH if DEFAULT_ROBOT_PATH.is_file()
        else _ALT_ROBOT_PATH if _ALT_ROBOT_PATH.is_file()
        else DEFAULT_ROBOT_PATH
    )
    if not path.is_file():
        return None
    with path.open(encoding='utf-8') as f:
        raw = yaml.safe_load(f) or {}
    if not isinstance(raw, dict):
        return None
    robot = raw.get('robot') if isinstance(raw.get('robot'), dict) else {}
    known = {'name', 'serial', 'wifi_ssid', 'timezone', 'robot'}
    extra = {k: v for k, v in raw.items() if k not in known}
    return RobotConfig(
        name=str(raw.get('name') or robot.get('name') or ''),
        serial=str(raw.get('serial') or robot.get('serial') or read_cpu_serial()),
        wifi_ssid=str(raw.get('wifi_ssid') or ''),
        timezone=str(raw.get('timezone') or ''),
        extra=extra,
    )


def save_robot_config(cfg: RobotConfig, path: Path = DEFAULT_ROBOT_PATH) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    data = cfg.to_dict()
    tmp = path.with_suffix('.yaml.tmp')
    with tmp.open('w', encoding='utf-8') as f:
        yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False)
    tmp.replace(path)


def hostname() -> str:
    return socket.gethostname()
