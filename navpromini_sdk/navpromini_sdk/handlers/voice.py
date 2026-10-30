#!/usr/bin/env python3
"""NavPro Mini Live Voice Assistant & Speech Recognition Subsystem.

Provides:
- Live microphone audio capture and filtering (removes bottom chassis / fan rumble)
- Multi-engine Speech-to-Text (STT) transcription (Groq Whisper-large-v3-turbo / Google STT)
- Voice Agent intent parser & autonomous robot controller (via SDK / MCP actions)
- Multi-engine Conversational LLM reasoning (Groq LLaMA 3.3 / Gemini 2.0 Flash / Local Fallback)
- Conversational chat dialog history (Left: Robot, Right: User)
- Cute Neural TTS output streaming through robot speakers (Piper TTS)
- Real-time listening / mute state and audio activity levels
"""

from __future__ import annotations

import asyncio
import json
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

# Dynamic API configuration for Groq & Gemini (can be set via env or /api/v1/voice/config)
_VOICE_CONFIG = {
    "groq_api_key": os.getenv("GROQ_API_KEY", "").strip(),
    "gemini_api_key": os.getenv("GEMINI_API_KEY", "").strip(),
    "model_preference": os.getenv("VOICE_MODEL_PREF", "groq").strip()  # "groq" or "gemini"
}


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
    try:
        os.chmod(raw_wav, 0o666)
    except OSError:
        pass

    clean_fd, clean_wav = tempfile.mkstemp(prefix="navpro_clean_", suffix=".wav")
    os.close(clean_fd)
    try:
        os.chmod(clean_wav, 0o666)
    except OSError:
        pass

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

        # Discard the initial 0.35s hardware startup pop from the webcam microphone
        startup_samples = int(framerate * 0.35)
        if len(samples) > startup_samples + int(framerate * 0.5):
            samples = samples[startup_samples:]

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
    except Exception:
        # Fallback: if numpy processing fails, return raw WAV directly
        return raw_wav
    finally:
        if os.path.exists(raw_wav) and raw_wav != clean_wav:
            try: os.unlink(raw_wav)
            except OSError: pass


def transcribe_audio_file(wav_path: str) -> Optional[str]:
    """Transcribe audio file to text using Groq Whisper or Google STT."""
    try:
        # 1. Try Groq Whisper (blazing fast, highest accuracy, free tier)
        groq_key = _VOICE_CONFIG.get("groq_api_key") or os.getenv("GROQ_API_KEY", "").strip()
        if groq_key:
            try:
                import requests
                with open(wav_path, "rb") as audio_file:
                    resp = requests.post(
                        "https://api.groq.com/openai/v1/audio/transcriptions",
                        headers={"Authorization": f"Bearer {groq_key}"},
                        files={"file": ("audio.wav", audio_file, "audio/wav")},
                        data={"model": "whisper-large-v3-turbo", "response_format": "json"},
                        timeout=7.0
                    )
                if resp.status_code == 200:
                    text = resp.json().get("text", "").strip()
                    if text:
                        return text
            except Exception:
                pass

        # 2. Fallback: Google Speech Recognition (free, unlimited small clips, no key needed)
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


async def _query_llm_conversational_reply(user_text: str) -> Optional[str]:
    """Generate intelligent natural voice reply via Groq or Gemini if keys are configured."""
    groq_key = _VOICE_CONFIG.get("groq_api_key") or os.getenv("GROQ_API_KEY", "").strip()
    gemini_key = _VOICE_CONFIG.get("gemini_api_key") or os.getenv("GEMINI_API_KEY", "").strip()
    pref = _VOICE_CONFIG.get("model_preference", "groq")

    system_prompt = (
        "You are NavPro Mini, an autonomous mobile robot assistant developed by Botforge. "
        "You have LiDAR, depth cameras, and autonomous navigation. "
        "Keep your response concise, polite, natural, and under 2 sentences, ideal for spoken audio."
    )

    loop = asyncio.get_event_loop()

    # 1. Groq LLaMA 3.3 / 3.1
    if groq_key and (pref == "groq" or not gemini_key):
        def _call_groq():
            import requests
            resp = requests.post(
                "https://api.groq.com/openai/v1/chat/completions",
                headers={"Authorization": f"Bearer {groq_key}", "Content-Type": "application/json"},
                json={
                    "model": "llama-3.3-70b-versatile",
                    "messages": [
                        {"role": "system", "content": system_prompt},
                        {"role": "user", "content": user_text}
                    ],
                    "max_tokens": 100,
                    "temperature": 0.6
                },
                timeout=5.0
            )
            if resp.status_code == 200:
                data = resp.json()
                return data["choices"][0]["message"]["content"].strip()
            return None

        try:
            ans = await loop.run_in_executor(None, _call_groq)
            if ans: return ans
        except Exception:
            pass

    # 2. Gemini 2.0 Flash / 1.5 Flash
    if gemini_key:
        def _call_gemini():
            import requests
            url = f"https://generativelanguage.googleapis.com/v1beta/models/gemini-2.0-flash:generateContent?key={gemini_key}"
            payload = {
                "contents": [
                    {"role": "user", "parts": [{"text": f"{system_prompt}\nUser: {user_text}\nNavPro Mini:"}]}
                ],
                "generationConfig": {"maxOutputTokens": 80, "temperature": 0.6}
            }
            resp = requests.post(url, json=payload, timeout=5.0)
            if resp.status_code == 200:
                data = resp.json()
                parts = data.get("candidates", [{}])[0].get("content", {}).get("parts", [])
                if parts:
                    return parts[0].get("text", "").strip()
            return None

        try:
            ans = await loop.run_in_executor(None, _call_gemini)
            if ans: return ans
        except Exception:
            pass

    return None


async def process_voice_agent_command(bridge: Any, opts: dict[str, Any], user_text: str) -> tuple[str, Optional[str]]:
    """Interpret user speech, execute corresponding robot action, or converse intelligently.

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

    # 4. Road Network Node Navigation: e.g. "go to node 1", "drive to node 2", "navigate to node 5"
    m_node = re.search(r'\b(?:go to|drive to|navigate to|reach)\s+(?:node|point|station)\s+(\d+)\b', text_lower)
    if m_node:
        target_node = int(m_node.group(1))
        try:
            from .roadnet import navigate_roadnet_route
            store = opts.get('store')
            current_map = store.current_map() if store and hasattr(store, 'current_map') else 'f202'
            await navigate_roadnet_route(bridge, opts, current_map, target_node)
            return (f"Navigating to road network node {target_node}.", f"route_node_{target_node}")
        except Exception as exc:
            return (f"Could not navigate to node {target_node}: {exc}", "route_error")

    # 5. Battery Status
    if re.search(r'\b(battery|charge level|power status|how much battery)\b', text_lower):
        batt = bridge.get('battery') if hasattr(bridge, 'get') else None
        dock_raw = bridge.get('dock_status') if hasattr(bridge, 'get') else None
        dock_st: dict[str, Any] = {}
        if isinstance(dock_raw, dict):
            dock_st = dock_raw
        elif isinstance(dock_raw, str):
            try:
                dock_st = json.loads(dock_raw)
            except Exception:
                dock_st = {}

        pct = 0.0
        if batt and isinstance(batt, dict):
            pct_val = float(batt.get('percentage', 0.0) or 0.0)
            pct = pct_val * 100.0 if pct_val <= 1.0 else pct_val

        is_charging = bool(dock_st.get('charging') or (batt and batt.get('charging')))
        chg_str = "charging at the dock" if is_charging else "discharging"
        if pct > 0:
            return (f"My battery is currently at {pct:.0f}% and {chg_str}.", "battery_status")
        return (f"Battery telemetry is initializing, and system is {chg_str}.", "battery_status")

    # 6. Location / Status
    if re.search(r'\b(where are you|position|location|status|what are you doing)\b', text_lower):
        pose = bridge.get('pose') if hasattr(bridge, 'get') else None
        dock_raw = bridge.get('dock_status') if hasattr(bridge, 'get') else None
        dock_st: dict[str, Any] = {}
        if isinstance(dock_raw, dict):
            dock_st = dock_raw
        elif isinstance(dock_raw, str):
            try:
                dock_st = json.loads(dock_raw)
            except Exception:
                dock_st = {}

        store = opts.get('store')
        current_map = store.current_map() if store and hasattr(store, 'current_map') else 'active map'
        if dock_st.get('charging') or dock_st.get('state') == 'charging':
            return (f"I am docked and charging on map '{current_map}'.", "status")
        if pose and isinstance(pose, dict) and 'x' in pose:
            return (f"I am at coordinates ({pose['x']:.2f}, {pose['y']:.2f}) on map '{current_map}'.", "status")
        return (f"I am ready for navigation on map '{current_map}'.", "status")

    # 7. Greetings / Introductions
    if re.search(r'\b(hello|hi|hey|good morning|good afternoon)\b', text_lower):
        return ("Hello! I am NavPro Mini. I can hear you clearly and navigate your facility. What would you like me to do?", "greeting")

    # 8. Help / Capabilities
    if re.search(r'\b(help|what can you do|commands)\b', text_lower):
        return ("You can tell me to dock, undock, go to node 1, check battery, or stop anytime!", "help")

    # 9. Conversational reasoning via LLM (Groq / Gemini)
    llm_reply = await _query_llm_conversational_reply(user_text)
    if llm_reply:
        return (llm_reply, "conversational_llm")

    # 10. Fallback Conversational Response
    return (f"I heard you say: '{user_text}'. I am ready to navigate, dock, or check status whenever you are ready!", "conversational")


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


class VoiceConfigHandler(BaseHandler):
    """Configure voice models, Groq / Gemini API keys, and model preferences."""

    def get(self) -> None:
        groq_set = bool(_VOICE_CONFIG.get("groq_api_key"))
        gemini_set = bool(_VOICE_CONFIG.get("gemini_api_key"))
        self.send({
            "groq_configured": groq_set,
            "gemini_configured": gemini_set,
            "model_preference": _VOICE_CONFIG.get("model_preference", "groq"),
            "available_engines": [
                {"id": "groq", "name": "Groq Whisper Turbo + LLaMA 3.3 (Ultra Fast, Free Daily Tier)"},
                {"id": "gemini", "name": "Google Gemini 2.0 Flash (Multimodal, Free Daily Tier)"},
                {"id": "local", "name": "Google Speech Recognition + Offline Local Agent"}
            ]
        })

    def post(self) -> None:
        data = self.body()
        if "groq_api_key" in data:
            _VOICE_CONFIG["groq_api_key"] = str(data["groq_api_key"]).strip()
        if "gemini_api_key" in data:
            _VOICE_CONFIG["gemini_api_key"] = str(data["gemini_api_key"]).strip()
        if "model_preference" in data:
            pref = str(data["model_preference"]).strip().lower()
            if pref in ("groq", "gemini", "local"):
                _VOICE_CONFIG["model_preference"] = pref

        self.send({
            "status": "updated",
            "groq_configured": bool(_VOICE_CONFIG.get("groq_api_key")),
            "gemini_configured": bool(_VOICE_CONFIG.get("gemini_api_key")),
            "model_preference": _VOICE_CONFIG.get("model_preference")
        })
