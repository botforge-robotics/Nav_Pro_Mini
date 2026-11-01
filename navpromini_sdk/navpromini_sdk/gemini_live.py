#!/usr/bin/env python3
"""Gemini 2.0 Flash Multimodal Live Stream Subsystem.

Provides continuous bidirectional voice-to-voice streaming with:
- Zero-latency 16 kHz PCM microphone capture uplink
- Real-time 24 kHz PCM speaker playback downlink
- Native barge-in live interruption (cuts off speaker immediately on user speech)
- Native function / tool calling directly within live conversation (dock, stop, navigate, etc.)
- Real-time Mute / Live toggle support
"""

from __future__ import annotations

import asyncio
import base64
import json
import logging
import os
import shutil
import subprocess
import threading
import time
from typing import Any, Optional
import websockets

logger = logging.getLogger("navpromini_sdk.gemini_live")

GEMINI_LIVE_URL = "wss://generativelanguage.googleapis.com/ws/google.ai.generativelanguage.v1beta.GenerativeService.BidiGenerateContent"
GEMINI_LIVE_MODELS = [
    "models/gemini-2.5-flash-native-audio-preview-12-2025",
    "models/gemini-2.5-flash-native-audio",
    "models/gemini-2.0-flash-exp",
]


def get_pulse_env() -> dict[str, str]:
    env = dict(os.environ)
    env.setdefault('PULSE_SERVER', 'unix:/run/user/1000/pulse/native')
    env.setdefault('XDG_RUNTIME_DIR', '/run/user/1000')
    for cookie_path in ['/root/.config/pulse/cookie', '/home/navpromini/.config/pulse/cookie']:
        if os.path.isfile(cookie_path):
            env.setdefault('PULSE_COOKIE', cookie_path)
            break
    return env


class GeminiLiveSession:
    """Manages a single continuous bidirectional voice-to-voice Gemini Live session."""

    def __init__(self) -> None:
        self.active: bool = False
        self.muted: bool = False
        self.connecting: bool = False
        self.last_error: str = ""
        self.current_user_text: str = ""
        self.current_robot_text: str = ""
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._player_proc: Optional[subprocess.Popen] = None
        self._player_lock = threading.Lock()
        self._bridge: Any = None
        self._opts: dict[str, Any] = {}
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._head_mic_queue: Optional[asyncio.Queue] = None

    def inject_pcm_audio(self, pcm_data: bytes) -> None:
        """Inject 16kHz PCM audio chunk from the wireless Aria Sense module."""
        if self._loop and self._head_mic_queue and not self.muted:
            def _push():
                # Keep queue ultra-tight: max 2 packets (~64ms max buffer) for instant zero-latency speech
                while self._head_mic_queue.qsize() >= 2:
                    try:
                        self._head_mic_queue.get_nowait()
                    except Exception:
                        break
                try:
                    self._head_mic_queue.put_nowait(pcm_data)
                except Exception:
                    pass
            try:
                self._loop.call_soon_threadsafe(_push)
            except Exception:
                pass

    @property
    def is_speaking(self) -> bool:
        with self._player_lock:
            proc_alive = self._player_proc is not None and self._player_proc.poll() is None
            recent_audio = (time.time() - getattr(self, '_last_speech_time', 0)) < 0.6
            return bool(proc_alive and recent_audio and getattr(self, '_is_speaking_state', False))

    def get_status(self) -> dict[str, Any]:
        with self._lock:
            from .handlers.head_unit import is_aria_sense_active
            aria_active = is_aria_sense_active()
            speaking = self.is_speaking
            user_speaking = getattr(self, '_prev_user_speaking', False) and not speaking
            return {
                "active": self.active,
                "muted": self.muted,
                "connecting": self.connecting,
                "speaking": speaking,
                "user_speaking": user_speaking,
                "last_error": self.last_error,
                "engine": "gemini_live",
                "aria_sense_active": aria_active,
                "audio_source": "aria_sense" if aria_active else "chassis_mic"
            }

    def start(self, api_key: str, bridge: Any, opts: dict[str, Any]) -> tuple[bool, str]:
        with self._lock:
            if self.active or self.connecting:
                return True, "already_running"
            if not api_key or not api_key.strip():
                self.last_error = "Gemini API key is required"
                return False, self.last_error
            self._stop_event.clear()
            self.muted = False
            self.connecting = True
            self.last_error = ""
            self._bridge = bridge
            self._opts = opts

        self._thread = threading.Thread(
            target=self._run_event_loop,
            args=(api_key.strip(),),
            daemon=True,
            name="gemini_live_session"
        )
        self._thread.start()
        return True, "starting"

    def stop(self) -> None:
        self._stop_event.set()
        self._kill_player()
        with self._lock:
            self.active = False
            self.connecting = False

    def toggle_mute(self) -> bool:
        with self._lock:
            self.muted = not self.muted
            new_muted = self.muted
        if new_muted:
            self._kill_player()
            if self._bridge and hasattr(self._bridge, 'publish_led_command'):
                self._bridge.publish_led_command('solid,255,60,0')  # Red indicator
        else:
            if self._bridge and hasattr(self._bridge, 'publish_led_command'):
                self._bridge.publish_led_command('pulse,0,255,160')  # Glowing cyan/green
        return new_muted

    def set_mute(self, muted: bool) -> None:
        with self._lock:
            self.muted = muted
        if muted:
            self._kill_player()
            if self._bridge and hasattr(self._bridge, 'publish_led_command'):
                self._bridge.publish_led_command('solid,255,60,0')  # Orange = muted
        else:
            if self._bridge and hasattr(self._bridge, 'publish_led_command'):
                self._bridge.publish_led_command('pulse,0,255,160')  # Cyan/green = listening

    def unpause_and_listen(self) -> None:
        """Instant Barge-In: kill any playing audio and ensure listening state."""
        self._kill_player()
        with self._lock:
            self.muted = False
        if self._bridge and hasattr(self._bridge, 'publish_led_command'):
            self._bridge.publish_led_command('pulse,0,255,160')

    def _kill_player(self) -> None:
        with self._player_lock:
            if self._player_proc is not None:
                try:
                    self._player_proc.terminate()
                except Exception:
                    pass
                try:
                    self._player_proc.kill()
                except Exception:
                    pass
                self._player_proc = None

    def _ensure_player(self) -> Optional[subprocess.Popen]:
        with self._player_lock:
            if self._player_proc is not None and self._player_proc.poll() is None:
                if self._player_proc.stdin and not self._player_proc.stdin.closed:
                    return self._player_proc
                self._kill_player()

            env = get_pulse_env()
            cmd = ['paplay', '--raw', '--rate', '24000', '--channels', '1', '--format', 's16le', '--latency-msec=30']
            try:
                self._player_proc = subprocess.Popen(
                    cmd,
                    stdin=subprocess.PIPE,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    env=env,
                    bufsize=0
                )
                return self._player_proc
            except Exception as e:
                logger.error(f"Failed to spawn 24kHz paplay: {e}")
                self._player_proc = None
                return None

    def _run_event_loop(self, api_key: str) -> None:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        self._loop = loop
        self._head_mic_queue = asyncio.Queue(maxsize=3)
        backoff = 1.0
        while not self._stop_event.is_set():
            try:
                loop.run_until_complete(self._session_loop(api_key))
                backoff = 1.0
            except Exception as e:
                if self._stop_event.is_set():
                    break
                logger.warning(f"Gemini live session loop ended ({e}), reconnecting in {backoff:.1f}s...")
                with self._lock:
                    self.active = False
                    self.connecting = True
                time.sleep(backoff)
                backoff = min(backoff * 1.5, 10.0)
            else:
                if self._stop_event.is_set():
                    break
                logger.info(f"Gemini live session closed, reconnecting in {backoff:.1f}s...")
                time.sleep(backoff)
        loop.close()
        with self._lock:
            self.active = False
            self.connecting = False

    async def _session_loop(self, api_key: str) -> None:
        from .handlers.voice import (
            ROBOT_OPENAI_TOOLS,
            _build_robot_context_str,
            execute_robot_voice_tool,
            _VOICE_LOCK,
            _CONVERSATION_HISTORY
        )

        ws_url = f"{GEMINI_LIVE_URL}?key={api_key}"
        robot_context = _build_robot_context_str(self._bridge, self._opts)

        system_prompt = (
            "You are Aria, an attentive, gracious, elegant hospitality servant and companion robot developed by Botforge. "
            "You serve humans with utmost warmth, polite courtesy, and helpful dedication, like a 5-star concierge.\n\n"
            f"{robot_context}\n\n"
            "CRITICAL IDENTITY & FIRST-PERSON RULES:\n"
            "1. YOU ARE THE ROBOT: Speak exclusively in the first person ('I', 'me', 'my', 'myself'). "
            "NEVER refer to yourself as 'the robot' or 'this robot', and NEVER say 'send the robot' or 'where do you want to send the robot'. "
            "Instead, always say: 'Where should I go?', 'Where would you like me to take you?', 'How may I assist you?', or 'Right away, I am heading there now!'.\n"
            "2. HOSPITALITY SERVANT PERSONALITY: Be warm, gracious, polite, and eager to assist (e.g. 'Right away, I am stopping here for you!', 'Certainly, stepping off the charging dock for you!'). NEVER use cold, stiff, robotic phrases like 'Robot has been commanded to stop'.\n"
            "3. BRISK & FAST: Speak concisely (1 short, natural spoken sentence per turn). Everything you say will be spoken out loud immediately. Avoid lists or formatting.\n"
            "4. LANGUAGE & VOICE CONSISTENCY (NO SPLIT PERSONALITY):\n"
            "   - Your default and primary language is ENGLISH. Always speak English with a consistent, warm, natural, friendly female tone.\n"
            "   - NEVER switch languages randomly or change accents mid-conversation!\n"
            "   - DO NOT switch to Hindi, Telugu, or any other language unless the guest specifically and clearly addresses you in that language for the entire sentence.\n"
            "   - If you hear ambient background noise, static, or accented English, DO NOT interpret it as a foreign language; stay firmly in English.\n"
            "   - Maintain one unified, steady, charming Aria persona at all times.\n"
            "5. PHYSICAL ACTION TOOLS (MANDATORY TO CALL ON MOTION COMMANDS):\n"
            "   - When the user asks you to move, undock, dock, stop, or navigate, you MUST invoke the appropriate tool!\n"
            "   - 'can you come out from charger', 'come off charger', 'undock', 'leave charger', 'step off dock', 'move off charger' -> MUST CALL undock_robot!\n"
            "   - 'dock', 'go to charger', 'charge yourself', 'park in dock', 'return to charging station' -> MUST CALL dock_robot!\n"
            "   - 'stop', 'halt', 'freeze', 'cancel' -> MUST CALL emergency_stop!\n"
            "   - 'take me to [destination]', 'go to [destination]' -> MUST CALL navigate_to_destination!\n"
            "   - ONLY answer with pure voice without tools when the guest is merely asking an informational question (e.g. 'what is your battery level?', 'where are you?').\n"
            "6. DESTINATION INQUIRIES: If the guest asks where you can go or what places are available, ONLY mention the Saved Named Locations from your status above (e.g. Kitchen, Office, Charging Dock), and ask: 'Where would you like me to take you?'. NEVER recite raw graph node numbers or 'Node 1, Node 2' to guests!\n"
            "7. ALWAYS TALK BACK: When any guest speaks, always answer graciously out loud. If their voice was too faint or unclear, politely ask them to repeat. NEVER stay silent!\n"
            "8. NO META-REASONING: Do NOT speak internal thoughts, analysis, or prefixes like 'Acknowledging User's Statement'. Only speak what you want the human guest to hear."
        )

        # Format tools for Gemini Live functionDeclarations (Only physical motion tools to ensure maximum speed!)
        function_decls = []
        for t in ROBOT_OPENAI_TOOLS:
            fn = t["function"]
            if fn["name"] in ("ignore_background_speech", "get_robot_telemetry"):
                continue  # Never delay Gemini Live with non-physical tool round-trips!
            props = {}
            for k, v in fn["parameters"]["properties"].items():
                props[k] = {
                    "type": "INTEGER" if v.get("type") == "integer" else "STRING",
                    "description": v.get("description", "")
                }
            function_decls.append({
                "name": fn["name"],
                "description": fn["description"],
                "parameters": {
                    "type": "OBJECT",
                    "properties": props,
                    "required": fn["parameters"].get("required", [])
                }
            })

        setup_message = {
            "setup": {
                "model": "models/gemini-2.5-flash-native-audio-preview-12-2025",
                "generationConfig": {
                    "responseModalities": ["AUDIO"],
                    "speechConfig": {
                        "voiceConfig": {
                            "prebuiltVoiceConfig": {
                                "voiceName": "Aoede"
                            }
                        }
                    },
                    "thinkingConfig": {
                        "thinkingBudget": 0
                    }
                },
                "inputAudioTranscription": {},
                "outputAudioTranscription": {},
                "systemInstruction": {
                    "parts": [{"text": system_prompt}]
                },
                "tools": [
                    {
                        "functionDeclarations": function_decls
                    }
                ]
            }
        }

        logger.info(f"Connecting to Gemini Live WebSocket: {GEMINI_LIVE_URL}...")
        async with websockets.connect(ws_url) as ws:
            # 1. Send Setup Handshake
            await ws.send(json.dumps(setup_message))
            initial_resp = await ws.recv()
            logger.info(f"Gemini Live setup response received: {initial_resp[:200]}")

            with self._lock:
                self.active = True
                self.connecting = False

            if self._bridge and hasattr(self._bridge, 'publish_led_command'):
                self._bridge.publish_led_command('pulse,0,255,160')  # Glowing active pulse

            # 2. Launch Mic Uplink Task and Downlink Receiver
            mic_task = asyncio.create_task(self._mic_sender_loop(ws))
            receiver_task = asyncio.create_task(self._receiver_loop(ws))

            done, pending = await asyncio.wait(
                [mic_task, receiver_task],
                return_when=asyncio.FIRST_COMPLETED
            )
            for t in pending:
                t.cancel()

    @staticmethod
    def _remove_echo_cancellation(env: dict[str, str]) -> None:
        """Remove PulseAudio echo cancellation and restore raw hardware defaults.

        Echo-cancel virtual source severely attenuates mic audio making Gemini's
        VAD unable to detect speech. We use the raw hardware mic directly.
        """
        try:
            # Unload all echo-cancel modules
            res = subprocess.run(['pactl', 'list', 'modules', 'short'], capture_output=True, text=True, env=env)
            for line in res.stdout.splitlines():
                parts = line.split('\t')
                if len(parts) >= 2 and 'echo-cancel' in parts[1]:
                    module_id = parts[0].strip()
                    subprocess.run(['pactl', 'unload-module', module_id], env=env,
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    logger.info(f"Unloaded module-echo-cancel (id={module_id})")

            # Restore raw hardware defaults
            src_res = subprocess.run(['pactl', 'list', 'sources', 'short'], capture_output=True, text=True, env=env)
            for line in src_res.stdout.splitlines():
                parts = line.split('\t')
                if len(parts) >= 2 and 'echo-cancel' not in parts[1] and 'monitor' not in parts[1] and 'usb' in parts[1].lower():
                    subprocess.run(['pactl', 'set-default-source', parts[1]], env=env,
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    logger.info(f"Set default source to raw hardware: {parts[1]}")
                    break

            sink_res = subprocess.run(['pactl', 'list', 'sinks', 'short'], capture_output=True, text=True, env=env)
            for line in sink_res.stdout.splitlines():
                parts = line.split('\t')
                if len(parts) >= 2 and 'echo-cancel' not in parts[1] and 'hdmi' in parts[1].lower():
                    subprocess.run(['pactl', 'set-default-sink', parts[1]], env=env,
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    logger.info(f"Set default sink to raw hardware: {parts[1]}")
                    break
        except Exception as e:
            logger.warning(f"Could not remove echo cancellation: {e}")

    async def _mic_sender_loop(self, ws: Any) -> None:
        """Stream 16kHz PCM audio continuously from microphone to Gemini Live (Full-Duplex).

        Prioritizes Aria Sense Top Microphone as PRIMARY DEFAULT source.
        Only falls back to bottom chassis microphone if Aria Sense is disconnected.
        """
        env = get_pulse_env()
        self._remove_echo_cancellation(env)

        p_mic: Optional[subprocess.Popen] = None
        loop = asyncio.get_event_loop()
        _diag_count = 0
        _diag_rms_sum = 0.0
        active_source = None  # "aria_sense" or "chassis_mic"

        def _ensure_chassis_mic():
            nonlocal p_mic
            if p_mic is not None and p_mic.poll() is None:
                return
            hw_source = None
            try:
                src_res = subprocess.run(['pactl', 'list', 'sources', 'short'], capture_output=True, text=True, env=env)
                for line in src_res.stdout.splitlines():
                    parts = line.split('\t')
                    if len(parts) >= 2 and 'usb' in parts[1].lower() and 'echo-cancel' not in parts[1] and 'monitor' not in parts[1]:
                        hw_source = parts[1]
                        break
            except Exception:
                pass

            cmd = ['parec', '--raw', '--rate=16000', '--channels=1', '--format=s16le', '--latency-msec=20', '--process-time-msec=20']
            if hw_source:
                cmd.extend(['--device', hw_source])
            try:
                p_mic = subprocess.Popen(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.DEVNULL,
                    env=env,
                    bufsize=0
                )
                logger.info(f"Fallback chassis mic started ({hw_source or 'default'})")
            except Exception as e:
                logger.error(f"Failed to spawn chassis mic parec: {e}")

        def _stop_chassis_mic():
            nonlocal p_mic
            if p_mic is not None:
                try:
                    p_mic.terminate()
                    p_mic.wait(timeout=0.3)
                except Exception:
                    try: p_mic.kill()
                    except Exception: pass
                p_mic = None

        def _read_chassis_chunk(n: int) -> bytes:
            if not p_mic or not p_mic.stdout:
                return b''
            buf = bytearray()
            while len(buf) < n:
                if self._stop_event.is_set():
                    break
                part = p_mic.stdout.read(n - len(buf))
                if not part:
                    break
                buf.extend(part)
            return bytes(buf)

        from .handlers.head_unit import is_aria_sense_active

        try:
            while not self._stop_event.is_set():
                data: bytes = b''
                aria_active = is_aria_sense_active()

                if aria_active:
                    if active_source != "aria_sense":
                        active_source = "aria_sense"
                        _stop_chassis_mic()
                        logger.info("🎙️ [Aria Voice] Using Aria Sense Top Microphone as PRIMARY DEFAULT source")
                        print("[Aria Voice] Using Aria Sense Top Microphone as PRIMARY DEFAULT source (16kHz)", flush=True)

                    try:
                        data = await asyncio.wait_for(self._head_mic_queue.get(), timeout=0.08)
                        # Zero-latency backlog drain: batch up to 2 extra accumulated chunks (~96ms max)
                        extra_chunks = []
                        while not self._head_mic_queue.empty() and len(extra_chunks) < 2:
                            try:
                                extra_chunks.append(self._head_mic_queue.get_nowait())
                            except Exception:
                                break
                        if extra_chunks:
                            data = data + b"".join(extra_chunks)
                    except asyncio.TimeoutError:
                        data = b''
                else:
                    if active_source != "chassis_mic":
                        active_source = "chassis_mic"
                        logger.info("ℹ️ [Aria Voice] Aria Sense not active, falling back to chassis mic")
                        print("[Aria Voice] Aria Sense not active, falling back to chassis mic", flush=True)
                        _ensure_chassis_mic()

                    _ensure_chassis_mic()
                    data = await loop.run_in_executor(None, _read_chassis_chunk, 1024)

                if not data or len(data) < 320:  # Must be at least 10ms of 16kHz audio (160 samples * 2 bytes)
                    if self._stop_event.is_set():
                        break
                    await asyncio.sleep(0.005)
                    continue

                if self.muted:
                    continue

                # Remove hardware DC offset so Gemini VAD detects speech boundaries with zero delay
                import numpy as np
                samples = np.frombuffer(data, dtype=np.int16).astype(np.float32)
                mean_offset = float(np.mean(samples)) if len(samples) > 0 else 0.0
                samples = samples - mean_offset
                rms = float(np.sqrt(np.mean(samples ** 2))) if len(samples) > 0 else 0.0
                peak = int(np.max(np.abs(samples))) if len(samples) > 0 else 0
                clean_pcm = np.clip(samples, -32768, 32767).astype(np.int16).tobytes()

                _diag_count += 1
                _diag_rms_sum += rms
                if _diag_count % 30 == 0:
                    avg_rms = _diag_rms_sum / 30
                    src_name = "Aria-Sense" if active_source == "aria_sense" else "Chassis-Mic"
                    print(f"[GeminiLive] [{src_name}] Mic RMS={avg_rms:.0f} Peak={peak} (chunk #{_diag_count})", flush=True)
                    _diag_rms_sum = 0.0

                # Ambient-adaptive voice sensitivity tracking (adapts to room acoustic floor)
                amb_floor = getattr(self, '_ambient_floor', 85.0)
                if not self.is_speaking:
                    if rms < amb_floor * 1.5:
                        amb_floor = 0.94 * amb_floor + 0.06 * rms
                    elif rms < amb_floor:
                        amb_floor = 0.85 * amb_floor + 0.15 * rms
                self._ambient_floor = max(40.0, min(amb_floor, 300.0))

                # Dynamic speech detection threshold (ambient adopting)
                speech_threshold = max(95.0, min(self._ambient_floor + 35.0, self._ambient_floor * 1.35))
                now_time = time.time()
                raw_spk = bool(rms > speech_threshold and not self.is_speaking)
                if raw_spk:
                    self._user_spk_hangover = now_time + 0.38

                is_user_spk = bool(now_time < getattr(self, '_user_spk_hangover', 0.0) and not self.is_speaking)
                if is_user_spk != getattr(self, '_prev_user_speaking', False):
                    self._prev_user_speaking = is_user_spk
                    if self._bridge and hasattr(self._bridge, 'emit_event'):
                        norm_level = float(min(1.0, max(0.0, (rms - self._ambient_floor) / 2200.0)))
                        self._bridge.emit_event('voice.user_speaking', {
                            'speaking': is_user_spk,
                            'level': norm_level
                        })

                b64_audio = base64.b64encode(clean_pcm).decode('ascii')
                realtime_msg = {
                    "realtimeInput": {
                        "mediaChunks": [
                            {
                                "mimeType": "audio/pcm;rate=16000",
                                "data": b64_audio
                            }
                        ]
                    }
                }
                await ws.send(json.dumps(realtime_msg))

        except (websockets.exceptions.ConnectionClosed, asyncio.CancelledError):
            pass
        except Exception as e:
            logger.warning(f"Gemini live mic loop exception: {e}")
        finally:
            _stop_chassis_mic()

    async def _receiver_loop(self, ws: Any) -> None:
        """Receive audio playback chunks, barge-in interruptions, and tool calls from Gemini Live."""
        from .handlers.voice import (
            execute_robot_voice_tool,
            _VOICE_LOCK,
            _CONVERSATION_HISTORY
        )

        while not self._stop_event.is_set():
            try:
                raw_msg = await ws.recv()
                msg = json.loads(raw_msg)
            except Exception as e:
                logger.warn(f"Gemini live recv ended: {e}")
                break

            # 1. Handle Server Content (User & Model Transcripts, Audio, Barge-in)
            server_content = msg.get("serverContent")
            if server_content:
                if server_content.get("interrupted"):
                    print("[GeminiLive] User barge-in detected: interrupting playback!", flush=True)
                    self._kill_player()
                    if getattr(self, '_is_speaking_state', False):
                        self._is_speaking_state = False
                        if self._bridge and hasattr(self._bridge, 'emit_event'):
                            self._bridge.emit_event('voice.speaking', {'speaking': False})

                # Capture User Speech Transcription (appears on right side of chat)
                user_trans = server_content.get("inputTranscription") or server_content.get("interimInputTranscription")
                if user_trans and isinstance(user_trans, dict):
                    u_text = user_trans.get("text", "").strip()
                    if u_text:
                        print(f"[GeminiLive] Human transcript: {u_text}", flush=True)
                        now_ts = time.time()
                        with _VOICE_LOCK:
                            if (_CONVERSATION_HISTORY and
                                _CONVERSATION_HISTORY[-1].get("sender") == "user" and
                                (now_ts - _CONVERSATION_HISTORY[-1].get("timestamp", 0)) < 4.0):
                                _CONVERSATION_HISTORY[-1]["text"] = u_text
                                _CONVERSATION_HISTORY[-1]["timestamp"] = now_ts
                            else:
                                _CONVERSATION_HISTORY.append({
                                    "id": f"user_live_{int(now_ts*1000)}",
                                    "sender": "user",
                                    "text": u_text,
                                    "timestamp": now_ts,
                                    "action": None
                                })
                        if self._bridge and hasattr(self._bridge, 'emit_event'):
                            self._bridge.emit_event('voice.transcript', {
                                'sender': 'user',
                                'text': u_text,
                                'timestamp': now_ts
                            })

                # Process Model Turn Audio & Text FIRST before turnComplete flags
                model_turn = server_content.get("modelTurn")
                if model_turn:
                    parts = model_turn.get("parts", [])
                    for part in parts:
                        # Audio stream chunk (Gemini Multimodal Live returns inlineData with audio/pcm;rate=24000)
                        inline = part.get("inlineData")
                        b64_pcm = None
                        if inline and inline.get("mimeType", "").startswith("audio/pcm"):
                            b64_pcm = inline.get("data", "")
                        elif part.get("mimeType", "").startswith("audio/pcm"):
                            b64_pcm = part.get("data", "")

                        if b64_pcm and not self.muted:
                            pcm_bytes = base64.b64decode(b64_pcm)
                            player = self._ensure_player()
                            if player and player.stdin:
                                try:
                                    player.stdin.write(pcm_bytes)
                                    player.stdin.flush()
                                    self._last_speech_time = time.time()
                                    if not getattr(self, '_is_speaking_state', False):
                                        self._is_speaking_state = True
                                        if self._bridge and hasattr(self._bridge, 'emit_event'):
                                            self._bridge.emit_event('voice.speaking', {'speaking': True})
                                    _play_chunk_cnt = getattr(self, '_play_chunk_cnt', 0) + 1
                                    self._play_chunk_cnt = _play_chunk_cnt
                                    if _play_chunk_cnt == 1 or _play_chunk_cnt % 25 == 0:
                                        print(f"[GeminiLive] Playing audio ({len(pcm_bytes)} B, chunk #{_play_chunk_cnt})", flush=True)
                                except Exception as e:
                                    print(f"[GeminiLive] Audio write error: {e}", flush=True)
                                    self._kill_player()
                                    if getattr(self, '_is_speaking_state', False):
                                        self._is_speaking_state = False
                                        if self._bridge and hasattr(self._bridge, 'emit_event'):
                                            self._bridge.emit_event('voice.speaking', {'speaking': False})

                        # Text transcript chunk
                        if "text" in part and part["text"].strip():
                            text_chunk = part["text"].strip()
                            if (not text_chunk.startswith("**") and
                                "ignore_background_speech" not in text_chunk and
                                "Acknowledging User" not in text_chunk):
                                print(f"[GeminiLive] Aria transcript: {text_chunk}", flush=True)
                                with _VOICE_LOCK:
                                    if _CONVERSATION_HISTORY and _CONVERSATION_HISTORY[-1]["sender"] == "robot":
                                        _CONVERSATION_HISTORY[-1]["text"] += " " + text_chunk
                                    else:
                                        _CONVERSATION_HISTORY.append({
                                            "id": f"robot_live_{int(time.time()*1000)}",
                                            "sender": "robot",
                                            "text": text_chunk,
                                            "timestamp": time.time(),
                                            "action": "gemini_live"
                                        })
                                if self._bridge and hasattr(self._bridge, 'emit_event'):
                                    self._bridge.emit_event('voice.transcript', {
                                        'sender': 'robot',
                                        'text': text_chunk
                                    })

                if server_content.get("turnComplete"):
                    print("[GeminiLive] Model turn complete", flush=True)
                    if getattr(self, '_is_speaking_state', False):
                        self._is_speaking_state = False
                        if self._bridge and hasattr(self._bridge, 'emit_event'):
                            self._bridge.emit_event('voice.speaking', {'speaking': False})
                    # Turn complete: flush player buffer so audio finishes playing smoothly
                    with self._player_lock:
                        if self._player_proc and self._player_proc.stdin and not self._player_proc.stdin.closed:
                            try:
                                self._player_proc.stdin.flush()
                            except Exception:
                                pass

            # 2. Handle Tool Calling (Robot actions: undock, dock, stop, navigate)
            tool_call = msg.get("toolCall")
            if tool_call:
                function_calls = tool_call.get("functionCalls", [])
                responses = []
                for fc in function_calls:
                    call_id = fc.get("id", "")
                    name = fc.get("name", "")
                    args = fc.get("args", {})
                    print(f"[GeminiLive] Native tool call received: {name}({args})", flush=True)

                    if name == "emergency_stop":
                        self._kill_player()

                    spoken_reply, action_tag = await execute_robot_voice_tool(
                        self._bridge, self._opts, name, args
                    )

                    # Immediate vocal feedback using onboard speech synthesizer
                    if spoken_reply:
                        try:
                            from ..audio import play_speech
                            play_speech(spoken_reply, wait=False)
                        except Exception:
                            pass

                    if action_tag != "ignored" and (spoken_reply or name != "ignore_background_speech"):
                        with _VOICE_LOCK:
                            _CONVERSATION_HISTORY.append({
                                "id": f"tool_{int(time.time()*1000)}",
                                "sender": "robot",
                                "text": spoken_reply or f"Executing {name}...",
                                "timestamp": time.time(),
                                "action": action_tag
                            })
                        if self._bridge and hasattr(self._bridge, 'emit_event'):
                            self._bridge.emit_event('voice.transcript', {
                                'sender': 'robot',
                                'text': spoken_reply or f"Executing {name}..."
                            })

                    responses.append({
                        "id": call_id,
                        "name": name,
                        "response": {
                            "result": {
                                "status": "success",
                                "action": action_tag,
                                "message": spoken_reply or "Action executed successfully."
                            }
                        }
                    })

                # Send tool response back to Gemini Live
                tool_response_msg = {
                    "toolResponse": {
                        "functionResponses": responses
                    }
                }
                await ws.send(json.dumps(tool_response_msg))


# Global singleton live session
_GLOBAL_LIVE_SESSION = GeminiLiveSession()

def get_live_session() -> GeminiLiveSession:
    return _GLOBAL_LIVE_SESSION
