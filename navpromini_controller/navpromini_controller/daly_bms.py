#!/usr/bin/env python3
"""Daly Smart BMS drivers over RS485 (FTDI USB-RS485).

Supports:
  1) Classic Daly UART/485 frame protocol (header 0xA5) — common on Smart BMS
  2) Modbus RTU (slave 0xD2) — common on newer Daly Blue / Home Storage units

Physical: 9600 8N1. Use an isolated USB–RS485 adapter when possible (BMS
UART ground is tied to battery negative).
"""

from __future__ import annotations

import struct
import time
from dataclasses import dataclass, field
from typing import List, Optional, Sequence

import serial


def _u16_be(data: bytes, i: int) -> int:
    return (data[i] << 8) | data[i + 1]


def _checksum_a5(frame: Sequence[int]) -> int:
    return sum(frame) & 0xFF


def _crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


@dataclass
class DalySnapshot:
    """All fields the node could decode from the BMS."""

    protocol: str = ''
    pack_voltage_v: float = float('nan')
    pack_current_a: float = float('nan')  # +charge, -discharge (A5 convention after offset)
    soc_percent: float = float('nan')
    remain_capacity_ah: float = float('nan')
    nominal_capacity_ah: float = float('nan')
    cell_count: int = 0
    temp_count: int = 0
    cell_voltages_v: List[float] = field(default_factory=list)
    temperatures_c: List[float] = field(default_factory=list)
    max_cell_v: float = float('nan')
    min_cell_v: float = float('nan')
    max_cell_index: int = 0
    min_cell_index: int = 0
    max_temp_c: float = float('nan')
    min_temp_c: float = float('nan')
    # 0 idle, 1 charging, 2 discharging
    charge_state: int = 0
    charge_mos_on: bool = False
    discharge_mos_on: bool = False
    charger_connected: bool = False
    load_connected: bool = False
    bms_life: int = 0
    failure_bits: int = 0
    balance_bits: int = 0
    raw_notes: str = ''


class DalySerial:
    def __init__(self, port: str, baud: int = 9600, timeout: float = 0.4):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self._ser: Optional[serial.Serial] = None
        self.protocol: Optional[str] = None  # 'a5' | 'modbus'

    def open(self) -> None:
        self._ser = serial.Serial(
            port=self.port,
            baudrate=self.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.timeout,
            write_timeout=self.timeout,
        )
        # RS485 USB adapters often need DTR/RTS for TX enable; leave defaults.
        time.sleep(0.05)
        self._ser.reset_input_buffer()

    def close(self) -> None:
        if self._ser and self._ser.is_open:
            self._ser.close()
        self._ser = None

    def _write_read(self, payload: bytes, expect_min: int) -> bytes:
        assert self._ser is not None
        self._ser.reset_input_buffer()
        self._ser.write(payload)
        self._ser.flush()
        time.sleep(0.05)
        raw = self._ser.read(expect_min)
        # Drain a bit more if adapter is slow
        t_end = time.time() + 0.15
        while time.time() < t_end and len(raw) < expect_min + 32:
            chunk = self._ser.read(64)
            if not chunk:
                break
            raw += chunk
        return raw

    # --- Classic A5 UART/485 -------------------------------------------------

    def a5_request(self, cmd: int, host_addr: int = 0x40) -> Optional[bytes]:
        """Send 13-byte A5 request; return 8 data bytes or None."""
        frame = [0xA5, host_addr & 0xFF, cmd & 0xFF, 0x08] + [0x00] * 8
        frame.append(_checksum_a5(frame))
        raw = self._write_read(bytes(frame), 13)
        # Scan for response frame
        for i in range(max(0, len(raw) - 12)):
            if raw[i] != 0xA5:
                continue
            if i + 13 > len(raw):
                break
            resp = raw[i:i + 13]
            if resp[2] != (cmd & 0xFF) or resp[3] != 0x08:
                continue
            if _checksum_a5(resp[:12]) != resp[12]:
                continue
            return bytes(resp[4:12])
        return None

    def probe_a5(self) -> bool:
        for addr in (0x40, 0x80):
            data = self.a5_request(0x90, host_addr=addr)
            if data is not None:
                self._a5_host = addr
                self.protocol = 'a5'
                return True
        return False

    def read_a5(self) -> DalySnapshot:
        host = getattr(self, '_a5_host', 0x40)
        snap = DalySnapshot(protocol='a5')

        d90 = self.a5_request(0x90, host)
        if d90:
            # Byte0-1 cumulative V 0.1V, 2-3 gather V 0.1V, 4-5 current offset 30000 @0.1A, 6-7 SOC 0.1%
            snap.pack_voltage_v = _u16_be(d90, 0) * 0.1
            snap.pack_current_a = (_u16_be(d90, 4) - 30000) * 0.1
            snap.soc_percent = _u16_be(d90, 6) * 0.1

        d91 = self.a5_request(0x91, host)
        if d91:
            snap.max_cell_v = _u16_be(d91, 0) * 0.001
            snap.max_cell_index = d91[2]
            snap.min_cell_v = _u16_be(d91, 3) * 0.001
            snap.min_cell_index = d91[5]

        d92 = self.a5_request(0x92, host)
        if d92:
            snap.max_temp_c = float(d92[0] - 40)
            snap.min_temp_c = float(d92[2] - 40)

        d93 = self.a5_request(0x93, host)
        if d93:
            snap.charge_state = int(d93[0])
            snap.charge_mos_on = bool(d93[1])
            snap.discharge_mos_on = bool(d93[2])
            snap.bms_life = int(d93[3])
            # remain capacity mAh
            rem_mah = struct.unpack('>I', d93[4:8])[0]
            snap.remain_capacity_ah = rem_mah / 1000.0

        d94 = self.a5_request(0x94, host)
        if d94:
            snap.cell_count = int(d94[0])
            snap.temp_count = int(d94[1])
            snap.charger_connected = bool(d94[2])
            snap.load_connected = bool(d94[3])

        # Cell voltages: response packs 3 cells per frame; byte0 = frame index
        cells: List[float] = []
        if snap.cell_count > 0:
            frames_needed = (snap.cell_count + 2) // 3
            for _ in range(frames_needed + 2):
                d95 = self.a5_request(0x95, host)
                if not d95:
                    break
                frame_idx = d95[0]
                for k in range(3):
                    mv = _u16_be(d95, 1 + 2 * k)
                    if mv == 0:
                        continue
                    cells.append(mv * 0.001)
                if len(cells) >= snap.cell_count:
                    break
            snap.cell_voltages_v = cells[: snap.cell_count]

        temps: List[float] = []
        if snap.temp_count > 0:
            for _ in range(max(2, (snap.temp_count + 6) // 7)):
                d96 = self.a5_request(0x96, host)
                if not d96:
                    break
                # byte0 = frame index; following bytes temperatures +40
                for b in d96[1:]:
                    if b == 0:
                        continue
                    temps.append(float(b - 40))
                if len(temps) >= snap.temp_count:
                    break
            snap.temperatures_c = temps[: snap.temp_count]
        elif snap.max_temp_c == snap.max_temp_c:  # not nan
            snap.temperatures_c = [snap.min_temp_c, snap.max_temp_c]

        d97 = self.a5_request(0x97, host)
        if d97:
            # balance bits packed
            snap.balance_bits = int.from_bytes(d97[:6], 'big')

        d98 = self.a5_request(0x98, host)
        if d98:
            snap.failure_bits = int.from_bytes(d98[:8], 'big')

        return snap

    # --- Modbus RTU ----------------------------------------------------------

    def modbus_read_holding(self, start: int, count: int, slave: int = 0xD2) -> Optional[List[int]]:
        req = bytes([
            slave & 0xFF, 0x03,
            (start >> 8) & 0xFF, start & 0xFF,
            (count >> 8) & 0xFF, count & 0xFF,
        ])
        crc = _crc16_modbus(req)
        req += bytes([crc & 0xFF, (crc >> 8) & 0xFF])
        expect = 5 + 2 * count
        raw = self._write_read(req, expect)
        for i in range(max(0, len(raw) - expect + 1)):
            if raw[i] != (slave & 0xFF) or raw[i + 1] != 0x03:
                continue
            byte_count = raw[i + 2]
            if byte_count != 2 * count:
                continue
            end = i + 3 + byte_count
            if end + 2 > len(raw):
                continue
            payload = raw[i:end]
            got_crc = raw[end] | (raw[end + 1] << 8)
            if _crc16_modbus(payload) != got_crc:
                continue
            regs = []
            for r in range(count):
                regs.append((raw[i + 3 + 2 * r] << 8) | raw[i + 4 + 2 * r])
            return regs
        return None

    def probe_modbus(self) -> bool:
        regs = self.modbus_read_holding(0x28, 3)
        if regs is not None and len(regs) >= 3:
            self.protocol = 'modbus'
            return True
        return False

    def read_modbus(self) -> DalySnapshot:
        snap = DalySnapshot(protocol='modbus')
        # Pack V / I / SOC — community map for newer Daly
        pack = self.modbus_read_holding(0x28, 4)
        if pack and len(pack) >= 3:
            snap.pack_voltage_v = pack[0] * 0.1
            # Some firmwares use 30000 offset like A5; detect by magnitude
            raw_i = pack[1]
            if raw_i > 10000:
                snap.pack_current_a = (raw_i - 30000) * 0.1
            else:
                # signed interpretation: values > 32767 mean discharge on some maps
                if raw_i > 32767:
                    snap.pack_current_a = (raw_i - 65536) * 0.1
                else:
                    snap.pack_current_a = raw_i * 0.1
            snap.soc_percent = pack[2] * 0.1 if pack[2] > 100 else float(pack[2])

        # Cell count unknown — read up to 16 cells from 0x00
        cells_regs = self.modbus_read_holding(0x00, 16)
        if cells_regs:
            cells = [r * 0.001 for r in cells_regs if 2000 < r < 4500]
            snap.cell_voltages_v = cells
            snap.cell_count = len(cells)
            if cells:
                snap.max_cell_v = max(cells)
                snap.min_cell_v = min(cells)
                snap.max_cell_index = cells.index(snap.max_cell_v) + 1
                snap.min_cell_index = cells.index(snap.min_cell_v) + 1

        temps_regs = self.modbus_read_holding(0x20, 4)
        if temps_regs:
            temps = [float(r - 40) for r in temps_regs if 0 < r < 140]
            snap.temperatures_c = temps
            snap.temp_count = len(temps)
            if temps:
                snap.max_temp_c = max(temps)
                snap.min_temp_c = min(temps)

        # Charge / discharge MOS & status — maps vary; try common block
        status = self.modbus_read_holding(0x2F, 4)
        if status:
            # Heuristic: if current > 0.2 charging, < -0.2 discharging
            pass
        if snap.pack_current_a == snap.pack_current_a:
            if snap.pack_current_a > 0.2:
                snap.charge_state = 1
                snap.charger_connected = True
            elif snap.pack_current_a < -0.2:
                snap.charge_state = 2
                snap.load_connected = True
            else:
                snap.charge_state = 0

        snap.raw_notes = 'modbus map is best-effort for newer Daly Blue units'
        return snap

    def probe(self) -> str:
        if self.probe_a5():
            return 'a5'
        if self.probe_modbus():
            return 'modbus'
        raise RuntimeError(
            f'No Daly BMS response on {self.port} @ {self.baud}. '
            'Check RS485 A/B wiring, baud 9600, and that FTDI is /dev/battery_bms.'
        )

    def read(self) -> DalySnapshot:
        if self.protocol is None:
            self.probe()
        if self.protocol == 'a5':
            return self.read_a5()
        if self.protocol == 'modbus':
            return self.read_modbus()
        raise RuntimeError('BMS protocol not selected')
