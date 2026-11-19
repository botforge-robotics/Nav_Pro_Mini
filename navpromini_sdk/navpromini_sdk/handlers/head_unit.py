#!/usr/bin/env python3
"""NavPro Mini - Aria Sense Management Subsystem.

Provides:
- Real-time BLE / Wi-Fi discovery of nearby Aria Sense Units (top mic + camera)
- 1-Tap automatic Wi-Fi credential provisioning from the robot screen UI
- In-memory WebSocket audio receiver (:8095) that feeds directly into Gemini Live
- UDP dynamic IP discovery beacon broadcaster (:8093)
- Live Aria Sense status & camera stream proxy
"""

from __future__ import annotations

import asyncio
import json
import logging
import os
import re
import socket
import subprocess
import threading
import time
import queue
from typing import Any, Optional

import websockets
from .base import ApiError, BaseHandler

logger = logging.getLogger("navpromini_sdk.head_unit")

# Head Unit Global State
_HEAD_UNIT_STATE = {
    "connected": False,
    "ip": None,
    "mac": None,
    "name": None,
    "last_seen": 0,
    "streaming_audio": False,
    "audio_rms": 0.0,
    "camera_url": None,
    "paired_token": "navpro_aria_default"
}
_HEAD_UNIT_LOCK = threading.Lock()
_AUDIO_LISTENERS: list[Any] = []
_HEAD_MIC_RECORDING_SUBSCRIBERS: list[queue.Queue] = []

def is_aria_sense_active() -> bool:
    """Check if Aria Sense top microphone is currently streaming 16kHz PCM audio."""
    with _HEAD_UNIT_LOCK:
        now = time.time()
        return bool(_HEAD_UNIT_STATE.get("streaming_audio", False) and (now - _HEAD_UNIT_STATE.get("last_seen", 0) <= 3.5))

def record_aria_sense_audio(duration_sec: float) -> Optional[bytes]:
    """Capture duration_sec of clean 16kHz PCM audio directly from Aria Sense top mic."""
    if not is_aria_sense_active():
        return None
    target_bytes = int(duration_sec * 16000 * 2)
    accumulated = bytearray()
    start_time = time.time()
    q: queue.Queue = queue.Queue(maxsize=150)
    _HEAD_MIC_RECORDING_SUBSCRIBERS.append(q)
    try:
        while len(accumulated) < target_bytes and (time.time() - start_time < duration_sec + 1.5):
            try:
                chunk = q.get(timeout=0.2)
                accumulated.extend(chunk)
            except queue.Empty:
                pass
    finally:
        if q in _HEAD_MIC_RECORDING_SUBSCRIBERS:
            _HEAD_MIC_RECORDING_SUBSCRIBERS.remove(q)

    if len(accumulated) >= 16000 * 2 * 0.5:
        return bytes(accumulated[:target_bytes])
    return None

HEAD_UNIT_WS_PORT = 8095
UDP_BEACON_PORT = 8093


def get_robot_local_ip() -> str:
    """Get the primary local LAN IP of the robot."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "127.0.0.1"


def get_current_wifi_credentials() -> dict[str, str]:
    """Extract active Wi-Fi SSID and password from NetworkManager.

    Detects if the robot is on a 5GHz band and looks up the 2.4GHz counterpart
    since ESP32-S3 is 2.4GHz only.
    """
    res = {"ssid": "", "ssid_24ghz": "", "password": ""}
    try:
        cmd = ["nmcli", "-t", "-f", "NAME,TYPE,DEVICE", "con", "show", "--active"]
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=3).stdout
        wifi_con_name = ""
        for line in out.splitlines():
            parts = line.split(":")
            if len(parts) >= 2 and ("wifi" in parts[1] or "wireless" in parts[1]):
                wifi_con_name = parts[0]
                break

        if wifi_con_name:
            cmd = ["sudo", "nmcli", "-s", "-g", "802-11-wireless.ssid,802-11-wireless-security.psk", "connection", "show", wifi_con_name]
            p_out = subprocess.run(cmd, capture_output=True, text=True, timeout=3).stdout.splitlines()
            if len(p_out) >= 1:
                res["ssid"] = p_out[0].strip()
            if len(p_out) >= 2:
                res["password"] = p_out[1].strip()

        # If on 5GHz, check nearby Wi-Fi scan for 2.4GHz equivalent
        if res["ssid"].endswith("-5GHZ") or res["ssid"].endswith("_5G"):
            base_name = re.sub(r'[-_]5GH?Z?$', '', res["ssid"], flags=re.IGNORECASE)
            scan_cmd = ["sudo", "nmcli", "-t", "-f", "SSID,FREQ", "dev", "wifi", "list"]
            scan_out = subprocess.run(scan_cmd, capture_output=True, text=True, timeout=4).stdout
            for s_line in scan_out.splitlines():
                s_parts = s_line.split(":")
                if len(s_parts) >= 2:
                    ssid_candidate = s_parts[0]
                    freq_mhz = int(s_parts[1].split()[0]) if s_parts[1].split() and s_parts[1].split()[0].isdigit() else 0
                    if 2400 <= freq_mhz <= 2500:
                        if base_name.lower() in ssid_candidate.lower():
                            res["ssid_24ghz"] = ssid_candidate
                            break
            if not res["ssid_24ghz"]:
                res["ssid_24ghz"] = base_name + "-2.4GHZ"
        else:
            res["ssid_24ghz"] = res["ssid"]

    except Exception as e:
        logger.warning(f"Error reading Wi-Fi credentials: {e}")

    return res


# ==============================================================================
# UDP Beacon Broadcaster (Announces Robot IP so ESP32 auto-syncs on DHCP change)
# ==============================================================================
def _run_udp_beacon_broadcaster():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    while True:
        try:
            local_ip = get_robot_local_ip()
            payload = json.dumps({
                "robot": "navpro",
                "ip": local_ip,
                "port": HEAD_UNIT_WS_PORT,
                "token": _HEAD_UNIT_STATE["paired_token"],
                "timestamp": time.time()
            }).encode('utf-8')
            sock.sendto(payload, ("255.255.255.255", UDP_BEACON_PORT))
        except Exception:
            pass
        time.sleep(2.0)


# ==============================================================================
# Dedicated Head Unit Audio WebSocket Server (:8092/ws/head_mic)
# ==============================================================================
async def _head_mic_ws_handler(websocket: Any):
    client_ip = websocket.remote_address[0] if websocket.remote_address else "unknown"
    logger.info(f"Incoming connection from Head Unit at {client_ip}")

    with _HEAD_UNIT_LOCK:
        _HEAD_UNIT_STATE["connected"] = True
        _HEAD_UNIT_STATE["ip"] = client_ip
        _HEAD_UNIT_STATE["camera_url"] = f"http://{client_ip}:81/stream"
        _HEAD_UNIT_STATE["last_seen"] = time.time()

    import struct
    try:
        async for message in websocket:
            if isinstance(message, bytes) and len(message) > 0:
                with _HEAD_UNIT_LOCK:
                    _HEAD_UNIT_STATE["connected"] = True
                    _HEAD_UNIT_STATE["last_seen"] = time.time()
                    _HEAD_UNIT_STATE["streaming_audio"] = True

                    # Fast RMS calculation for UI audio meter (throttled to ~4 Hz with DC removal)
                    _meter_cnt = getattr(_head_mic_ws_handler, "_meter_cnt", 0) + 1
                    _head_mic_ws_handler._meter_cnt = _meter_cnt
                    if _meter_cnt % 8 == 0:
                        n_samples = len(message) // 2
                        if n_samples > 0:
                            samples = struct.unpack(f'<{n_samples}h', message)
                            mean_s = sum(samples) / n_samples
                            rms = (sum((s - mean_s) ** 2 for s in samples) / n_samples) ** 0.5
                            _HEAD_UNIT_STATE["audio_rms"] = float(rms)

                # Push to single-turn recording subscribers
                for q in list(_HEAD_MIC_RECORDING_SUBSCRIBERS):
                    try:
                        q.put_nowait(message)
                    except Exception:
                        pass

                # Push to Gemini Live full-duplex session as primary default audio stream
                try:
                    from ..gemini_live import get_live_session
                    live_sess = get_live_session()
                    if live_sess:
                        live_sess.inject_pcm_audio(message)
                except Exception:
                    pass

    except (websockets.exceptions.ConnectionClosed, asyncio.CancelledError):
        pass
    finally:
        with _HEAD_UNIT_LOCK:
            _HEAD_UNIT_STATE["connected"] = False
            _HEAD_UNIT_STATE["streaming_audio"] = False
        logger.info(f"Head Unit {client_ip} disconnected")


async def _start_head_mic_ws():
    async with websockets.serve(_head_mic_ws_handler, "0.0.0.0", HEAD_UNIT_WS_PORT, ping_interval=15, ping_timeout=15):
        logger.info(f"Head Unit Audio WebSocket server listening on port {HEAD_UNIT_WS_PORT}")
        await asyncio.get_running_loop().create_future()


def _run_head_mic_server():
    try:
        asyncio.run(_start_head_mic_ws())
    except Exception as e:
        logger.error(f"Head Unit WebSocket server error: {e}")


# Start background services once
_SERVERS_STARTED = False

def ensure_head_unit_services():
    global _SERVERS_STARTED
    if _SERVERS_STARTED:
        return
    _SERVERS_STARTED = True

    t_beacon = threading.Thread(target=_run_udp_beacon_broadcaster, daemon=True, name="head_unit_beacon")
    t_beacon.start()

    t_ws = threading.Thread(target=_run_head_mic_server, daemon=True, name="head_unit_ws_server")
    t_ws.start()
    logger.info(f"Head Unit background services initialized (WS: {HEAD_UNIT_WS_PORT}, Beacon: {UDP_BEACON_PORT})")


# ==============================================================================
# REST API Handlers for Robot UI
# ==============================================================================

class HeadUnitStatusHandler(BaseHandler):
    """GET /api/v1/head_unit/status - get current head unit pairing & stream state."""

    async def get(self) -> None:
        ensure_head_unit_services()
        with _HEAD_UNIT_LOCK:
            # Check timeout (if no packet in 6s, mark inactive)
            now = time.time()
            if _HEAD_UNIT_STATE["last_seen"] > 0 and (now - _HEAD_UNIT_STATE["last_seen"] <= 6.0):
                _HEAD_UNIT_STATE["connected"] = True
            else:
                _HEAD_UNIT_STATE["connected"] = False
                _HEAD_UNIT_STATE["streaming_audio"] = False
            data = dict(_HEAD_UNIT_STATE)

        self.send(data)


class HeadUnitWifiInfoHandler(BaseHandler):
    """GET /api/v1/head_unit/wifi_info - get active robot Wi-Fi network for 1-tap sync."""

    async def get(self) -> None:
        info = get_current_wifi_credentials()
        self.send({
            "current_ssid": info.get("ssid", ""),
            "target_ssid_24ghz": info.get("ssid_24ghz", "") or info.get("ssid", ""),
            "saved_password": info.get("password", ""),
            "has_password": bool(info.get("password")),
            "robot_ip": get_robot_local_ip()
        })


class HeadUnitWifiScanHandler(BaseHandler):
    """GET /api/v1/head_unit/wifi_scan - scan available Wi-Fi networks in range."""

    async def get(self) -> None:
        networks = []
        try:
            cmd = ["sudo", "nmcli", "-t", "-f", "SSID,SIGNAL,FREQ,SECURITY", "dev", "wifi", "list"]
            proc = await asyncio.create_subprocess_exec(
                *cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL
            )
            stdout, _ = await proc.communicate()
            text = stdout.decode("utf-8", errors="ignore")

            seen: dict[str, dict[str, Any]] = {}
            for line in text.splitlines():
                parts = line.split(":")
                if len(parts) >= 4 and parts[0]:
                    ssid = parts[0]
                    sig = int(parts[1]) if parts[1].isdigit() else 0
                    freq_str = parts[2].split()[0] if parts[2].split() else "0"
                    freq = int(freq_str) if freq_str.isdigit() else 0
                    is_24ghz = (2400 <= freq <= 2500)
                    sec = parts[3]
                    if ssid not in seen or sig > seen[ssid]["signal"]:
                        seen[ssid] = {
                            "ssid": ssid,
                            "signal": sig,
                            "freq": freq,
                            "is_24ghz": is_24ghz,
                            "security": sec
                        }
            # Sort: 2.4GHz networks first, then by signal strength desc
            networks = sorted(seen.values(), key=lambda x: (not x["is_24ghz"], -x["signal"]))
        except Exception as e:
            logger.warning(f"Error scanning Wi-Fi: {e}")

        self.send({"networks": networks, "count": len(networks)})


class HeadUnitScanHandler(BaseHandler):
    """GET /api/v1/head_unit/scan - scan for nearby advertising Aria-Sense BLE devices."""

    async def get(self) -> None:
        ensure_head_unit_services()
        devices = []
        try:
            try:
                from bleak import BleakScanner
                scanned = await BleakScanner.discover(timeout=2.5)
                for d in scanned:
                    name = d.name or ""
                    mac = d.address.upper()
                    if "Aria-Sense" in name or "Aria-Head" in name or "Sense" in name or "Head" in name or "XIAO" in name:
                        clean_name = name
                        if "Aria-Head" in clean_name:
                            clean_name = clean_name.replace("Aria-Head", "Aria-Sense")
                        elif "XIAO" in clean_name or "Seeed" in clean_name:
                            clean_name = f"Aria-Sense-{mac.replace(':', '')[-4:]}"
                        devices.append({
                            "name": clean_name or "Aria-Sense",
                            "mac": mac,
                            "rssi": getattr(d, "rssi", -55)
                        })
            except Exception as be:
                logger.debug(f"BleakScanner scan fallback: {be}")
                proc = await asyncio.create_subprocess_exec(
                    "bluetoothctl", "--timeout", "3", "scan", "on",
                    stdout=subprocess.PIPE,
                    stderr=subprocess.DEVNULL
                )
                stdout, _ = await proc.communicate()
                text = stdout.decode("utf-8", errors="ignore")

                seen_macs = set()
                for line in text.splitlines():
                    m = re.search(r'Device\s+([0-9A-Fa-f:]{17})\s+(.*)$', line)
                    if m:
                        mac = m.group(1).upper()
                        name = m.group(2).strip()
                        if mac not in seen_macs and ("Aria-Sense" in name or "Aria-Head" in name or "Sense" in name or "Head" in name or "XIAO" in name):
                            seen_macs.add(mac)
                            clean_name = name
                            if "Aria-Head" in clean_name:
                                clean_name = clean_name.replace("Aria-Head", "Aria-Sense")
                            elif "XIAO" in clean_name or "Seeed" in clean_name:
                                clean_name = f"Aria-Sense-{mac.replace(':', '')[-4:]}"
                            devices.append({
                                "name": clean_name,
                                "mac": mac,
                                "rssi": -55
                            })
        except Exception as e:
            logger.warning(f"Error scanning for BLE Aria Sense units: {e}")

        self.send({"devices": devices, "count": len(devices)})


class HeadUnitPairHandler(BaseHandler):
    """POST /api/v1/head_unit/pair - provision Wi-Fi credentials to Aria Sense over BLE."""

    async def post(self) -> None:
        data = self.body()
        target_mac = data.get("mac", "").strip().upper()
        if not target_mac:
            raise ApiError(400, "missing_mac", "MAC address of Aria Sense is required")

        wifi_info = get_current_wifi_credentials()
        ssid = (data.get("ssid") or wifi_info.get("ssid_24ghz") or wifi_info.get("ssid") or "").strip()
        password = data.get("password") if data.get("password") is not None else wifi_info.get("password", "")
        robot_ip = get_robot_local_ip()

        if not ssid:
            raise ApiError(400, "missing_ssid", "Please select or enter a Wi-Fi network SSID.")

        payload = json.dumps({
            "ssid": ssid,
            "pass": password,
            "robot_host": robot_ip,
            "robot_port": HEAD_UNIT_WS_PORT,
            "robot_token": _HEAD_UNIT_STATE["paired_token"]
        })

        # Send configuration payload over BLE using BleakClient
        success = False
        err_msg = ""
        try:
            py_code = f"""
import asyncio
from bleak import BleakClient

UUID_CONFIG = "beb5483e-36e1-4688-b7f5-ea07361b26a8"
async def send():
    async with BleakClient("{target_mac}", timeout=10.0) as client:
        await client.write_gatt_char(UUID_CONFIG, {repr(payload)}.encode('utf-8'), response=True)
        print("PROVISIONED_OK")

asyncio.run(send())
"""
            proc = await asyncio.create_subprocess_exec(
                "python3", "-c", py_code,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            try:
                out, err = await asyncio.wait_for(proc.communicate(), timeout=12.0)
                if b"PROVISIONED_OK" in out:
                    success = True
                else:
                    err_msg = err.decode('utf-8', errors='ignore').strip()
            except asyncio.TimeoutError:
                proc.kill()
                err_msg = "Bluetooth transmission timed out"
        except Exception as e:
            err_msg = str(e)

        if not success:
            logger.warning(f"BLE write to Aria Sense {target_mac} failed: {err_msg}")
            raise ApiError(500, "ble_write_failed", f"Could not reach Aria Sense via Bluetooth ({err_msg or 'timeout'}). Ensure it is powered on and within range.")

        with _HEAD_UNIT_LOCK:
            _HEAD_UNIT_STATE["mac"] = target_mac

        self.send({
            "status": "provisioning_sent",
            "target_mac": target_mac,
            "target_ssid": ssid,
            "robot_ip": robot_ip,
            "message": f"Credentials accepted by Aria Sense! Connecting to '{ssid}'..."
        })
