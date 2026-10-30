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

    def get_status(self) -> dict[str, Any]:
        with self._lock:
            return {
                "active": self.active,
                "muted": self.muted,
                "connecting": self.connecting,
                "last_error": self.last_error,
                "engine": "gemini_live"
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
                return self._player_proc

            env = get_pulse_env()
            cmd = ['paplay', '--raw', '--rate', '24000', '--channels', '1', '--format', 's16le']
            try:
                self._player_proc = subprocess.Popen(
                    cmd,
                    stdin=subprocess.PIPE,
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
        import websockets
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
            "You are Aria, an intelligent, warm, friendly AI companion on this autonomous robot developed by Botforge. "
            "You speak and listen in real-time with human users in a natural, warm, conversational voice.\n"
            f"{robot_context}\n\n"
            "LIVE VOICE-TO-VOICE INTERACTION RULES:\n"
            "1. CASUAL & NATURAL: Speak naturally and warmly like a real conversational friend (e.g. 'Okay okay, I am stopping right now!', 'Sure thing, heading over to recharge!'). NEVER use cold, stiff, robotic phrases like 'Robot has been commanded to stop'.\n"
            "2. CONCISE & SPOKEN: Speak concisely (1-2 short sentences per turn). Avoid lists or formatting; everything you say will be spoken out loud.\n"
            "3. MULTILINGUAL TOOL CALLING: The user may speak English, Hindi, Telugu, Tamil, Spanish, German, French, etc. Understand their intent in ANY language, call the appropriate tool, and reply in the user's spoken language (e.g. in Telugu: 'సరే సరే, ఇప్పుడే ఆగుతున్నాను!').\n"
            "4. ACTIVE ROBOT TOOLS:\n"
            "   - dock_robot: Return to charging dock, latch, and charge (e.g. 'dock', 'charge', 'డాల్ కి వెళ్లు')\n"
            "   - undock_robot: Disengage and roll forward off the charging dock\n"
            "   - emergency_stop: Immediately halt all robot motion, stop motors, and cancel active navigation (e.g. 'stop', 'halt', 'ఆగు', 'రోకో')\n"
            "   - navigate_to_road_node: Drive to a specific station or node number on the road network (e.g. 'go to node 1', 'drive to station 2')\n"
            "   - get_robot_telemetry: Check current battery %, charging state, position, or current map\n"
            "   - ignore_background_speech: When ambient speech is clearly background noise or chatter not directed at you.\n"
            "5. NO META-REASONING: Do NOT speak internal thoughts, analysis, or prefixes like 'Acknowledging User's Statement'. Only speak what you want the human user to hear."
        )

        # Format tools for Gemini Live functionDeclarations
        function_decls = []
        for t in ROBOT_OPENAI_TOOLS:
            fn = t["function"]
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
                    }
                },
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

    async def _mic_sender_loop(self, ws: Any) -> None:
        """Stream 16kHz PCM audio continuously from microphone to Gemini Live."""
        if shutil.which('arecord'):
            cmd = ['arecord', '-D', 'default', '-t', 'raw', '-f', 'S16_LE', '-r', '16000', '-c', '1', '-q']
        else:
            cmd = ['parec', '--raw', '--rate=16000', '--channels=1', '--format=s16le']

        p_mic: Optional[subprocess.Popen] = None
        try:
            p_mic = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                bufsize=0
            )
        except Exception as e:
            logger.error(f"Failed to start mic capture for Gemini Live mic stream: {e}")
            return

        chunk_size = 3200  # 100 ms of 16kHz 16-bit mono PCM (1600 samples * 2 bytes)
        loop = asyncio.get_event_loop()

        try:
            while not self._stop_event.is_set():
                data = await loop.run_in_executor(None, p_mic.stdout.read, chunk_size)
                if not data:
                    await asyncio.sleep(0.01)
                    continue

                if self.muted:
                    # When muted, do not send user speech to Gemini
                    await asyncio.sleep(0.05)
                    continue

                b64_audio = base64.b64encode(data).decode('ascii')
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
        finally:
            if p_mic:
                try:
                    p_mic.terminate()
                    p_mic.wait(timeout=0.4)
                except Exception:
                    try: p_mic.kill()
                    except Exception: pass

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

            # 1. Handle Barge-in Interruption (User spoke while Gemini was speaking)
            server_content = msg.get("serverContent")
            if server_content:
                if server_content.get("interrupted"):
                    logger.info("Barge-in detected: Gemini interrupted by user speech! Cutting off playback immediately.")
                    self._kill_player()

                if server_content.get("turnComplete"):
                    # Turn complete: flush/close paplay stdin so audio finishes naturally and frees player
                    with self._player_lock:
                        if self._player_proc and self._player_proc.stdin:
                            try:
                                self._player_proc.stdin.close()
                            except Exception:
                                pass
                            self._player_proc = None

                model_turn = server_content.get("modelTurn")
                if model_turn:
                    parts = model_turn.get("parts", [])
                    for part in parts:
                        # Audio stream chunk
                        if part.get("mimeType", "").startswith("audio/pcm"):
                            b64_pcm = part.get("data", "")
                            if b64_pcm and not self.muted:
                                pcm_bytes = base64.b64decode(b64_pcm)
                                player = self._ensure_player()
                                if player and player.stdin:
                                    try:
                                        player.stdin.write(pcm_bytes)
                                        player.stdin.flush()
                                    except (BrokenPipeError, OSError):
                                        self._kill_player()

                        # Text transcript chunk
                        if "text" in part and part["text"].strip():
                            text_chunk = part["text"].strip()
                            if (not text_chunk.startswith("**") and
                                "ignore_background_speech" not in text_chunk and
                                "Acknowledging User" not in text_chunk):
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

            # 2. Handle Tool Calling (Robot actions: dock, stop, navigate, battery)
            tool_call = msg.get("toolCall")
            if tool_call:
                function_calls = tool_call.get("functionCalls", [])
                responses = []
                for fc in function_calls:
                    call_id = fc.get("id", "")
                    name = fc.get("name", "")
                    args = fc.get("args", {})
                    logger.info(f"Gemini Live native tool call received: {name}({args})")

                    spoken_reply, action_tag = await execute_robot_voice_tool(
                        self._bridge, self._opts, name, args
                    )

                    if action_tag != "ignored" and (spoken_reply or name != "ignore_background_speech"):
                        with _VOICE_LOCK:
                            _CONVERSATION_HISTORY.append({
                                "id": f"tool_{int(time.time()*1000)}",
                                "sender": "robot",
                                "text": spoken_reply or f"Executing {name}...",
                                "timestamp": time.time(),
                                "action": action_tag
                            })

                    responses.append({
                        "id": call_id,
                        "name": name,
                        "response": {
                            "output": {
                                "status": "success",
                                "action": action_tag,
                                "result": spoken_reply or "Action executed successfully."
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
