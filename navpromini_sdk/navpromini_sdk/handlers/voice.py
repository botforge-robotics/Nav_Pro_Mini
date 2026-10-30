#!/usr/bin/env python3
"""NavPro Mini Live Voice Assistant & Speech Recognition Subsystem.

Provides:
- Live microphone audio capture and filtering (removes bottom chassis / fan rumble)
- Speech-to-Text (STT) transcription
- Voice Agent intent parser & autonomous robot controller (via SDK / MCP actions)
- Conversational chat dialog history (Left: Robot, Right: User)
- Cute Neural TTS output streaming through robot speakers
- Real-time listening / mute state and audio activity levels
"""

from __future__ import annotations

import asyncio
import os
import re
import shutil
import subprocess
import tempfile
import threading
import time
from typing import Any, Optional
import wave

from .base import ApiError, BaseHandler

_VOICE_LOCK = threading.Lock()
_CONVERSATION_HISTORY: list[dict[str, Any]] = [
    {
        "id": "welcome_1",
        "sender": "robot",
        "text": "Hello! I am NavPro Mini. I can hear your voice and control navigation. Try saying 'dock', 'undock', 'go to node 1', or 'what is your battery?'",
        "timestamp": time.time(),
        "action": None
    }
]
_IS_MUTED = False
_IS_LISTENING = False
_LAST_TRANSCRIPT = ""
_LAST_REPLY = ""
_AUDIO_LEVEL = 0.0


def _get_pulse_env() -> dict[str, str]:
    env = dict(os.environ)
    env.setdefault('PULSE_SERVER', 'unix:/run/user/1000/pulse/native')
    env.setdefault('XDG_RUNTIME_DIR', '/run/user/1000')
    for cookie_path in ['/root/.config/pulse/cookie', '/home/navpromini/.config/pulse/cookie']:
        if os.path.isfile(cookie_path):
            env.setdefault('PULSE_COOKIE', cookie_path)
            break
    return env


def capture_and_filter_audio(duration_sec: float = 4.0) -> Optional[str]:
    """Capture raw microphone audio and apply high-pass filter + peak normalization.

    Returns the path to a clean WAV file ready for STT, or None on failure.
    """
    global _AUDIO_LEVEL
    raw_fd, raw_wav = tempfile.mkstemp(prefix="navpro_raw_", suffix=".wav")
    os.close(raw_fd)
    clean_fd, clean_wav = tempfile.mkstemp(prefix="navpro_clean_", suffix=".wav")
    os.close(clean_fd)

    env = _get_pulse_env()
    cmd: list[str] = []

    # Use parecord with PulseAudio if available (runs as navpromini user if root)
    if shutil.which('parecord'):
        if os.geteuid() == 0:
            cmd = [
                'sudo', '-u', 'navpromini', 'env',
                'XDG_RUNTIME_DIR=/run/user/1000',
                'PULSE_SERVER=unix:/run/user/1000/pulse/native',
                'timeout', str(int(duration_sec + 0.5)),
                'parecord', '-r', '--file-format=wav',
                '--channels=1', '--rate=16000', raw_wav
            ]
        else:
            cmd = [
                'timeout', str(int(duration_sec + 0.5)),
                'parecord', '-r', '--file-format=wav',
                '--channels=1', '--rate=16000', raw_wav
            ]
    elif shutil.which('arecord'):
        cmd = [
            'arecord', '-D', 'hw:0,0', '-f', 'S16_LE',
            '-c', '2', '-r', '16000', '-d', str(int(duration_sec)),
            '-q', raw_wav
        ]
    else:
        return None

    try:
        subprocess.run(cmd, env=env, check=False, timeout=duration_sec + 3.0)
    except Exception:
        pass

    if not os.path.isfile(raw_wav) or os.path.getsize(raw_wav) < 1000:
        if os.path.exists(raw_wav):
            try: os.unlink(raw_wav)
            except OSError: pass
        if os.path.exists(clean_wav):
            try: os.unlink(clean_wav)
            except OSError: pass
        return None

    # Filter and clean the recorded audio
    try:
        import numpy as np
        with wave.open(raw_wav, "rb") as wf:
            channels = wf.getnchannels()
            framerate = wf.getframerate()
            raw_bytes = wf.readframes(wf.getnframes())

        samples = np.frombuffer(raw_bytes, dtype=np.int16)
        if channels == 2:
            # If stereo from arecord, average channels to mono
            samples = samples.reshape(-1, 2)
            samples = (samples[:, 0].astype(np.float32) + samples[:, 1].astype(np.float32)) * 0.5
        else:
            samples = samples.astype(np.float32)

        # High-pass filter (100 Hz cutoff) to eliminate chassis/motor rumble from bottom webcam mic
        try:
            from scipy.signal import butter, sosfilt
            sos = butter(4, 100, "hp", fs=framerate, output="sos")
            filtered = sosfilt(sos, samples)
        except Exception:
            # Fallback lightweight first-order DC-blocker
            filtered = np.zeros_like(samples)
            for i in range(1, len(samples)):
                filtered[i] = samples[i] - samples[i-1] + 0.995 * filtered[i-1]

        # Peak normalization to -3 dBFS (approx 23000 max int16 amplitude)
        peak = np.max(np.abs(filtered))
        rms = np.sqrt(np.mean(filtered**2))
        _AUDIO_LEVEL = float(min(1.0, max(0.0, rms / 8000.0)))

        if peak > 80.0:
            filtered = (filtered / peak) * 23000.0

        int16_samples = np.clip(filtered, -32767, 32767).astype(np.int16)
        with wave.open(clean_wav, "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(framerate)
            wf.writeframes(int16_samples.tobytes())

        try:
            os.chmod(clean_wav, 0o666)
        except OSError:
            pass

        return clean_wav
    except Exception as exc:
        # Fallback: if numpy processing fails, return raw WAV directly
        return raw_wav
    finally:
        if os.path.exists(raw_wav) and raw_wav != clean_wav:
            try: os.unlink(raw_wav)
            except OSError: pass


def transcribe_audio_file(wav_path: str) -> Optional[str]:
    """Transcribe audio file to text using speech_recognition."""
    try:
        import speech_recognition as sr
        r = sr.Recognizer()
        r.energy_threshold = 200
        r.dynamic_energy_threshold = True
        with sr.AudioFile(wav_path) as source:
            audio = r.record(source)
        text = r.recognize_google(audio)
        return text.strip() if text else None
    except Exception:
        return None
    finally:
        if os.path.exists(wav_path):
            try: os.unlink(wav_path)
            except OSError: pass


async def process_voice_agent_command(bridge: Any, opts: dict[str, Any], user_text: str) -> tuple[str, Optional[str]]:
    """Interpret user speech and execute corresponding robot action if matched.

    Returns:
        (robot_reply_text, action_executed_or_none)
    """
    text_lower = user_text.lower().strip()

    # 1. Docking Command
    if re.search(r'\b(dock|docking|charge|recharge|go to dock|return to dock)\b', text_lower):
        try:
            from .docking import send_dock_goal
            await send_dock_goal(bridge, True)
            return ("Heading to the charging dock now.", "dock")
        except Exception as exc:
            return (f"Could not start docking: {exc}", "dock_error")

    # 2. Undocking Command
    if re.search(r'\b(undock|undocking|leave dock|disconnect dock)\b', text_lower):
        try:
            from .docking import send_undock_goal
            await send_undock_goal(bridge)
            return ("Undocking and rolling forward off the charging contacts.", "undock")
        except Exception as exc:
            return (f"Could not undock: {exc}", "undock_error")

    # 3. Emergency / Stop Command
    if re.search(r'\b(stop|halt|freeze|cancel|pause)\b', text_lower):
        try:
            from .navigation import cancel_active_navigation
            await cancel_active_navigation(bridge, 'user_voice_stop')
            return ("Stopping all motion and canceling active goals.", "stop")
        except Exception:
            return ("Robot has been commanded to stop.", "stop")

    # 4. Road Network Navigation (e.g. "go to node 1", "navigate to node 2", "node 3")
    node_match = re.search(r'\b(?:node|waypoint|point)\s*(\d+)\b', text_lower)
    if node_match:
        node_id = int(node_match.group(1))
        try:
            from .navigation import send_route_goal
            await send_route_goal(bridge, opts, goal_id=node_id, routing_mode='road_network')
            return (f"Navigating along the road network to Node {node_id}.", f"route_node_{node_id}")
        except Exception as exc:
            return (f"Could not route to Node {node_id}: {exc}", f"route_error_{node_id}")

    # 5. Battery Status
    if re.search(r'\b(battery|charge level|power status|how much battery)\b', text_lower):
        batt = bridge.get('battery') if hasattr(bridge, 'get') else None
        dock_st = bridge.get('dock_status') if hasattr(bridge, 'get') else None
        pct = (batt.get('percentage', 0.0) if batt else 0.0) * 100.0 if batt and batt.get('percentage', 0.0) <= 1.0 else (batt.get('percentage', 0.0) if batt else 0.0)
        is_charging = bool(dock_st and dock_st.get('charging'))
        chg_str = "charging at the dock" if is_charging else "discharging"
        if pct > 0:
            return (f"My battery is currently at {pct:.0f}% and {chg_str}.", "battery_status")
        return ("Battery monitor is currently active.", "battery_status")

    # 6. Location / Status
    if re.search(r'\b(where are you|position|location|status|what are you doing)\b', text_lower):
        pose = bridge.get('pose') if hasattr(bridge, 'get') else None
        dock_st = bridge.get('dock_status') if hasattr(bridge, 'get') else None
        store = opts.get('store')
        current_map = store.current_map() if store and hasattr(store, 'current_map') else 'active map'
        if dock_st and dock_st.get('charging'):
            return (f"I am docked and charging on map '{current_map}'.", "status")
        if pose and 'x' in pose:
            return (f"I am at coordinates ({pose['x']:.2f}, {pose['y']:.2f}) on map '{current_map}'.", "status")
        return (f"I am ready for navigation on map '{current_map}'.", "status")

    # 7. Greetings / Introductions
    if re.search(r'\b(hello|hi|hey|good morning|good afternoon|who are you|introduce)\b', text_lower):
        return ("Hello! I am NavPro Mini. I can hear you clearly and navigate your facility. What would you like me to do?", "greeting")

    # 8. Help / Capabilities
    if re.search(r'\b(help|what can you do|commands)\b', text_lower):
        return ("You can tell me to dock, undock, go to node 1, check battery, or stop anytime!", "help")

    # 9. Conversational Fallback
    return (f"I heard you say: '{user_text}'. I am ready to navigate or dock whenever you are ready!", "conversational")


class VoiceListenHandler(BaseHandler):
    """Record live audio from robot microphone, transcribe, and optionally execute voice agent reply."""

    async def post(self) -> None:
        global _IS_LISTENING, _LAST_TRANSCRIPT, _LAST_REPLY, _CONVERSATION_HISTORY
        data = self.body()
        duration = float(data.get('duration_sec', 4.0))
        auto_reply = bool(data.get('auto_reply', True))

        if _IS_MUTED:
            raise ApiError(400, 'voice_muted', 'Microphone is currently muted in UI')

        _IS_LISTENING = True
        if hasattr(self.bridge, 'publish_led_command'):
            self.bridge.publish_led_command('pulse,0,180,255')  # Glowing blue while listening

        try:
            # Capture audio in background thread so Tornado event loop is not blocked
            loop = asyncio.get_event_loop()
            clean_wav = await loop.run_in_executor(None, capture_and_filter_audio, duration)

            if not clean_wav:
                _IS_LISTENING = False
                self.send({'status': 'no_audio', 'text': '', 'reply': 'Microphone did not record audio.'})
                return

            transcript = await loop.run_in_executor(None, transcribe_audio_file, clean_wav)
            _IS_LISTENING = False

            if not transcript:
                self.send({'status': 'silence', 'text': '', 'reply': 'No speech detected.'})
                return

            _LAST_TRANSCRIPT = transcript
            user_msg = {
                "id": f"user_{int(time.time()*1000)}",
                "sender": "user",
                "text": transcript,
                "timestamp": time.time(),
                "action": None
            }
            with _VOICE_LOCK:
                _CONVERSATION_HISTORY.append(user_msg)

            reply_text = ""
            action = None
            if auto_reply:
                reply_text, action = await process_voice_agent_command(self.bridge, self.opts, transcript)
                _LAST_REPLY = reply_text
                robot_msg = {
                    "id": f"robot_{int(time.time()*1000)}",
                    "sender": "robot",
                    "text": reply_text,
                    "timestamp": time.time(),
                    "action": action
                }
                with _VOICE_LOCK:
                    _CONVERSATION_HISTORY.append(robot_msg)

                # Speak the reply via cute Piper neural TTS
                try:
                    from ..audio import play_speech
                    play_speech(reply_text, wait=False)
                except Exception as exc:
                    self.bridge.get_logger().warn(f"Voice reply speech playback note: {exc}")

            if hasattr(self.bridge, 'publish_led_command'):
                self.bridge.publish_led_command('solid,0,255,100')

            self.send({
                'status': 'success',
                'text': transcript,
                'reply': reply_text,
                'action': action,
                'messages': _CONVERSATION_HISTORY[-30:]
            })
        except Exception as exc:
            _IS_LISTENING = False
            self.fail(500, 'stt_error', str(exc))


class VoiceChatHandler(BaseHandler):
    """Send text or manual chat message to the voice agent and speak response."""

    async def post(self) -> None:
        global _LAST_REPLY, _CONVERSATION_HISTORY
        data = self.body()
        message = str(data.get('message', '')).strip()
        speak = bool(data.get('speak', True))

        if not message:
            raise ApiError(400, 'invalid_message', 'Message text cannot be empty')

        user_msg = {
            "id": f"user_{int(time.time()*1000)}",
            "sender": "user",
            "text": message,
            "timestamp": time.time(),
            "action": None
        }
        with _VOICE_LOCK:
            _CONVERSATION_HISTORY.append(user_msg)

        reply_text, action = await process_voice_agent_command(self.bridge, self.opts, message)
        _LAST_REPLY = reply_text

        robot_msg = {
            "id": f"robot_{int(time.time()*1000)}",
            "sender": "robot",
            "text": reply_text,
            "timestamp": time.time(),
            "action": action
        }
        with _VOICE_LOCK:
            _CONVERSATION_HISTORY.append(robot_msg)

        if speak:
            try:
                from ..audio import play_speech
                play_speech(reply_text, wait=False)
            except Exception as exc:
                self.bridge.get_logger().warn(f"Chat speech playback note: {exc}")

        self.send({
            'reply': reply_text,
            'action': action,
            'messages': _CONVERSATION_HISTORY[-30:]
        })


class VoiceDialogHandler(BaseHandler):
    """Get or clear conversational dialog history."""

    def get(self) -> None:
        with _VOICE_LOCK:
            msgs = list(_CONVERSATION_HISTORY[-50:])
        self.send({
            'messages': msgs,
            'muted': _IS_MUTED,
            'listening': _IS_LISTENING
        })

    def delete(self) -> None:
        global _CONVERSATION_HISTORY
        with _VOICE_LOCK:
            _CONVERSATION_HISTORY = [
                {
                    "id": f"welcome_{int(time.time()*1000)}",
                    "sender": "robot",
                    "text": "Conversation cleared. How can I assist you?",
                    "timestamp": time.time(),
                    "action": None
                }
            ]
        self.send({'cleared': True, 'messages': _CONVERSATION_HISTORY})


class VoiceStatusHandler(BaseHandler):
    """Get live audio levels, listening state, and mute status."""

    def get(self) -> None:
        self.send({
            'muted': _IS_MUTED,
            'listening': _IS_LISTENING,
            'audio_level': _AUDIO_LEVEL,
            'last_transcript': _LAST_TRANSCRIPT,
            'last_reply': _LAST_REPLY
        })


class VoiceMuteHandler(BaseHandler):
    """Toggle or set microphone listening / mute state."""

    async def post(self) -> None:
        global _IS_MUTED, _IS_LISTENING
        data = self.body()
        if 'muted' in data:
            _IS_MUTED = bool(data['muted'])
        else:
            _IS_MUTED = not _IS_MUTED

        if _IS_MUTED:
            _IS_LISTENING = False
            try:
                from ..audio import stop_active_speech
                stop_active_speech()
            except Exception:
                pass
            if hasattr(self.bridge, 'publish_led_command'):
                self.bridge.publish_led_command('solid,255,60,0')  # Red indicator for muted
        else:
            if hasattr(self.bridge, 'publish_led_command'):
                self.bridge.publish_led_command('solid,0,255,100')

        self.send({
            'muted': _IS_MUTED,
            'listening': _IS_LISTENING,
            'message': 'Microphone muted' if _IS_MUTED else 'Microphone active and listening'
        })
