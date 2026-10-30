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
from pathlib import Path
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

# Dynamic API configuration for Groq & Gemini (persisted securely on robot disk)
_CONFIG_PATHS = [
    Path("/home/navpromini/.config/navpro/voice_config.json"),
    Path("/root/.config/navpro/voice_config.json"),
    Path("/tmp/navpro_voice_config.json"),
]

def _get_secure_config_path() -> Path:
    for p in _CONFIG_PATHS:
        try:
            p.parent.mkdir(parents=True, exist_ok=True)
            return p
        except Exception:
            continue
    return Path("/tmp/navpro_voice_config.json")

def _sanitize_and_route_keys(groq_key: str, gemini_key: str) -> tuple[str, str]:
    """Sanitize API keys and auto-detect mispasted keys (e.g. gsk_ in Gemini field)."""
    groq = str(groq_key or "").strip()
    gemini = str(gemini_key or "").strip()

    # Clean accidental suffix from copy-paste (e.g. gsk_...allam-2-7b)
    m_groq_in_groq = re.match(r'^(gsk_[A-Za-z0-9]{40,65})', groq)
    if m_groq_in_groq:
        groq = m_groq_in_groq.group(1)

    m_groq_in_gemini = re.match(r'^(gsk_[A-Za-z0-9]{40,65})', gemini)
    if m_groq_in_gemini:
        if not groq:
            groq = m_groq_in_gemini.group(1)
        gemini = ""

    m_gemini_in_groq = re.match(r'^(AIzaSy[A-Za-z0-9_\-]{30,50})', groq)
    if m_gemini_in_groq:
        if not gemini:
            gemini = m_gemini_in_groq.group(1)
        groq = ""

    m_gemini_in_gemini = re.match(r'^(AIzaSy[A-Za-z0-9_\-]{30,50})', gemini)
    if m_gemini_in_gemini:
        gemini = m_gemini_in_gemini.group(1)

    return groq, gemini


def _load_persisted_config() -> dict[str, str]:
    raw_groq = os.getenv("GROQ_API_KEY", "").strip()
    raw_gemini = os.getenv("GEMINI_API_KEY", "").strip()
    model_pref = os.getenv("VOICE_MODEL_PREF", "groq").strip()

    for p in _CONFIG_PATHS:
        if p.is_file():
            try:
                with open(p, "r", encoding="utf-8") as f:
                    data = json.load(f)
                    if isinstance(data, dict):
                        if data.get("groq_api_key"):
                            raw_groq = str(data["groq_api_key"]).strip()
                        if data.get("gemini_api_key"):
                            raw_gemini = str(data["gemini_api_key"]).strip()
                        if data.get("model_preference"):
                            model_pref = str(data["model_preference"]).strip()
                break
            except Exception:
                pass

    groq, gemini = _sanitize_and_route_keys(raw_groq, raw_gemini)
    if groq and not gemini and model_pref == "gemini":
        model_pref = "groq"
    elif gemini and not groq and model_pref == "groq":
        model_pref = "gemini"

    return {
        "groq_api_key": groq,
        "gemini_api_key": gemini,
        "model_preference": model_pref
    }

def _save_persisted_config(cfg: dict[str, str]) -> None:
    target_path = _get_secure_config_path()
    groq, gemini = _sanitize_and_route_keys(cfg.get("groq_api_key", ""), cfg.get("gemini_api_key", ""))
    pref = cfg.get("model_preference", "groq")
    if groq and not gemini and pref == "gemini":
        pref = "groq"
    elif gemini and not groq and pref == "groq":
        pref = "gemini"

    try:
        temp_fd, temp_path = tempfile.mkstemp(prefix="voice_cfg_", dir=str(target_path.parent))
        with os.fdopen(temp_fd, "w", encoding="utf-8") as f:
            json.dump({
                "groq_api_key": groq,
                "gemini_api_key": gemini,
                "model_preference": pref,
                "updated_at": time.time()
            }, f, indent=2)
        os.chmod(temp_path, 0o600)  # Secure: rw------- (owner only)
        os.replace(temp_path, str(target_path))
    except Exception:
        pass

def _mask_key(key: str) -> str:
    if not key:
        return ""
    if len(key) <= 8:
        return "****"
    return f"{key[:4]}...{key[-4:]}"

_VOICE_CONFIG = _load_persisted_config()


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


# ---------------------------------------------------------------------------
# Robot Tools Schema (OpenAI / Groq format and Gemini format)
# ---------------------------------------------------------------------------

ROBOT_OPENAI_TOOLS = [
    {
        "type": "function",
        "function": {
            "name": "dock_robot",
            "description": "Command the robot to return to the charging station and dock. Call this whenever the user wants to dock, charge, recharge, park, or go to dock, in ANY language.",
            "parameters": {
                "type": "object",
                "properties": {
                    "spoken_response": {
                        "type": "string",
                        "description": "Short 1-sentence confirmation spoken to user in their language (e.g. 'Heading to the charging dock now.', 'चार्जिंग डॉक पर जा रहा हूँ।', 'ఛార్జింగ్ డాక్ వద్దకు వెళ్తున్నాను.')."
                    }
                },
                "required": ["spoken_response"]
            }
        }
    },
    {
        "type": "function",
        "function": {
            "name": "undock_robot",
            "description": "Command the robot to disengage and roll forward off the charging dock. Call when user wants to undock or leave the dock, in ANY language.",
            "parameters": {
                "type": "object",
                "properties": {
                    "spoken_response": {
                        "type": "string",
                        "description": "Short 1-sentence confirmation spoken in user's language."
                    }
                },
                "required": ["spoken_response"]
            }
        }
    },
    {
        "type": "function",
        "function": {
            "name": "emergency_stop",
            "description": "CRITICAL SAFETY: Immediately halt all robot motion, zero motor velocity, and abort active navigation goals. Call when user says stop, halt, freeze, cancel, or roko, in ANY language.",
            "parameters": {
                "type": "object",
                "properties": {
                    "spoken_response": {
                        "type": "string",
                        "description": "Short 1-sentence confirmation spoken in user's language (e.g. 'Emergency stop engaged. Stopping all motion.')."
                    }
                },
                "required": ["spoken_response"]
            }
        }
    },
    {
        "type": "function",
        "function": {
            "name": "navigate_to_road_node",
            "description": "Command the robot to navigate to a specific node or station on the road network (e.g. node 1, node 2). Call when user requests going to a node, station, or waypoint, in ANY language.",
            "parameters": {
                "type": "object",
                "properties": {
                    "node_id": {
                        "type": "integer",
                        "description": "Target road network node number (e.g. 1, 2, 3)."
                    },
                    "spoken_response": {
                        "type": "string",
                        "description": "Short 1-sentence confirmation spoken in user's language."
                    }
                },
                "required": ["node_id", "spoken_response"]
            }
        }
    },
    {
        "type": "function",
        "function": {
            "name": "get_robot_telemetry",
            "description": "Query the robot's real-time battery percentage, charging state, coordinates, and active map. Call when user asks about battery, power, location, or status, in ANY language.",
            "parameters": {
                "type": "object",
                "properties": {
                    "spoken_response": {
                        "type": "string",
                        "description": "Spoken answer summarizing telemetry in user's language."
                    }
                },
                "required": ["spoken_response"]
            }
        }
    },
    {
        "type": "function",
        "function": {
            "name": "ignore_background_speech",
            "description": "Call this tool if the user speech is background chatter, TV noise, someone talking to another human in the room, or ambient noise NOT directly intended for the robot.",
            "parameters": {
                "type": "object",
                "properties": {
                    "reason": {
                        "type": "string",
                        "description": "Brief explanation of why this speech is background chatter."
                    }
                }
            }
        }
    }
]

ROBOT_GEMINI_TOOLS = [
    {
        "function_declarations": [
            {
                "name": t["function"]["name"],
                "description": t["function"]["description"],
                "parameters": {
                    "type": "OBJECT",
                    "properties": {
                        k: {
                            "type": "INTEGER" if v.get("type") == "integer" else "STRING",
                            "description": v.get("description", "")
                        }
                        for k, v in t["function"]["parameters"]["properties"].items()
                    },
                    "required": t["function"]["parameters"].get("required", [])
                }
            }
            for t in ROBOT_OPENAI_TOOLS
        ]
    }
]


def _build_robot_context_str(bridge: Any, opts: dict[str, Any]) -> str:
    """Build telemetry summary injected into conversational LLM context."""
    batt = bridge.get('battery') if hasattr(bridge, 'get') else None
    dock_raw = bridge.get('dock_status') if hasattr(bridge, 'get') else None
    pose = bridge.get('pose') if hasattr(bridge, 'get') else None
    store = opts.get('store') if opts else None
    current_map = store.current_map() if store and hasattr(store, 'current_map') else 'active map'

    pct = 0.0
    if batt and isinstance(batt, dict):
        p = float(batt.get('percentage', 0.0) or 0.0)
        pct = p * 100.0 if p <= 1.0 else p

    dock_st = dock_raw if isinstance(dock_raw, dict) else {}
    is_chg = bool(dock_st.get('charging') or (batt and batt.get('charging')))
    dock_str = "Docked & Charging" if is_chg else "Discharging / Off dock"

    pose_str = "unknown"
    if pose and isinstance(pose, dict) and 'x' in pose:
        pose_str = f"x={pose['x']:.2f}, y={pose['y']:.2f}"

    return (
        f"[CURRENT ROBOT STATE: Map: '{current_map}', Battery: {pct:.0f}%, State: {dock_str}, Pose: {pose_str}]"
    )


async def execute_robot_voice_tool(
    bridge: Any,
    opts: dict[str, Any],
    tool_name: str,
    args: dict[str, Any]
) -> tuple[str, Optional[str]]:
    """Execute robot action triggered by LLM tool calling.

    Returns:
        (spoken_response, action_tag)
    """
    spoken = str(args.get("spoken_response", "")).strip()

    if tool_name == "ignore_background_speech":
        return ("", "ignored")

    if tool_name == "dock_robot":
        try:
            from .docking import send_dock_goal
            await send_dock_goal(bridge, True)
            reply = spoken or "Heading to the charging dock now."
            return (reply, "dock")
        except Exception as exc:
            return (f"Could not initiate docking: {exc}", "dock_error")

    if tool_name == "undock_robot":
        try:
            from .docking import send_undock_goal
            await send_undock_goal(bridge)
            reply = spoken or "Undocking and rolling forward off the charging dock."
            return (reply, "undock")
        except Exception as exc:
            return (f"Could not undock: {exc}", "undock_error")

    if tool_name == "emergency_stop":
        try:
            from .navigation import cancel_active_navigation
            await cancel_active_navigation(bridge, 'voice_emergency_stop')
            reply = spoken or "Emergency stop engaged. All motion halted."
            return (reply, "stop")
        except Exception:
            return ("Robot has been commanded to stop.", "stop")

    if tool_name == "navigate_to_road_node":
        node_id = int(args.get("node_id", 1))
        try:
            from .roadnet import navigate_roadnet_route
            store = opts.get('store')
            current_map = store.current_map() if store and hasattr(store, 'current_map') else 'f202'
            await navigate_roadnet_route(bridge, opts, current_map, node_id)
            reply = spoken or f"Navigating to road network node {node_id}."
            return (reply, f"route_node_{node_id}")
        except Exception as exc:
            return (f"Could not navigate to node {node_id}: {exc}", "route_error")

    if tool_name == "get_robot_telemetry":
        if spoken:
            return (spoken, "battery_status")
        batt = bridge.get('battery') if hasattr(bridge, 'get') else None
        dock_raw = bridge.get('dock_status') if hasattr(bridge, 'get') else None
        dock_st = dock_raw if isinstance(dock_raw, dict) else {}
        pct = 0.0
        if batt and isinstance(batt, dict):
            p = float(batt.get('percentage', 0.0) or 0.0)
            pct = p * 100.0 if p <= 1.0 else p
        is_chg = bool(dock_st.get('charging') or (batt and batt.get('charging')))
        chg_str = "charging at dock" if is_chg else "discharging"
        return (f"Battery is at {pct:.0f}% and {chg_str}.", "battery_status")

    return (spoken or "Action executed.", tool_name)


async def _query_llm_with_tool_calling(
    bridge: Any,
    opts: dict[str, Any],
    user_text: str
) -> Optional[tuple[str, Optional[str]]]:
    """Execute conversational reasoning with native tool calling via Groq LLaMA 3.3 or Gemini 2.0 Flash."""
    groq_key = _VOICE_CONFIG.get("groq_api_key") or os.getenv("GROQ_API_KEY", "").strip()
    gemini_key = _VOICE_CONFIG.get("gemini_api_key") or os.getenv("GEMINI_API_KEY", "").strip()
    pref = _VOICE_CONFIG.get("model_preference", "groq")

    if not groq_key and not gemini_key:
        return None

    robot_context = _build_robot_context_str(bridge, opts)
    system_prompt = (
        "You are NavPro Mini, an intelligent autonomous robotics voice assistant developed by Botforge. "
        "You are deployed on an autonomous mobile robot equipped with LiDAR, depth cameras, and ROS2 navigation.\n"
        f"{robot_context}\n\n"
        "CORE CONVERSATION & SAFETY RULES:\n"
        "1. TELEPHONY STYLE & CONCISENESS: Keep all spoken replies under 2 natural, concise sentences suitable for spoken TTS audio.\n"
        "2. MULTILINGUAL TOOL CALLING: The user may speak English, Hindi, Telugu, Tamil, Spanish, German, French, etc. You must understand their intent, execute tools, and speak your reply in the SAME language the user spoke.\n"
        "3. TOOL CALLING: Use your direct tools to control the robot:\n"
        "   - dock_robot: Return to charging dock, latch, and charge (e.g. 'dock', 'charge', 'डॉक पर जाओ', 'ఛార్జింగ్ డాక్ కి వెళ్లు')\n"
        "   - undock_robot: Disengage and roll forward off the charging dock\n"
        "   - emergency_stop: Immediately halt all robot motion, stop motors, and cancel active navigation (e.g. 'stop', 'halt', 'freeze', 'रोको', 'ఆగు')\n"
        "   - navigate_to_road_node: Drive to a specific station or node number on the road network (e.g. 'go to node 1', 'drive to station 2')\n"
        "   - get_robot_telemetry: Check current battery %, charging state, position, or current map\n"
        "   - ignore_background_speech: BACKGROUND CHATTER FILTER\n"
        "4. FILTERING UNNECESSARY SURROUNDING CHATTER: If the audio transcript is background chatter, someone talking to another human in the room, background TV/music, or ambient noise not directly addressing you, you MUST call 'ignore_background_speech'. Do NOT speak or interrupt human conversations.\n"
        "5. When calling any action tool, always supply the 'spoken_response' argument in the user's spoken language."
    )

    loop = asyncio.get_event_loop()

    # 1. Groq LLaMA 3.3 70B (ultra-fast, native tool calling, free daily tier)
    if groq_key and (pref == "groq" or not gemini_key):
        def _call_groq():
            import requests
            models_to_try = [
                "llama-3.3-70b-versatile",
                "qwen/qwen3.8-27b",
                "llama-3.1-8b-instant",
                "allam-2-7b"
            ]
            for m in models_to_try:
                try:
                    resp = requests.post(
                        "https://api.groq.com/openai/v1/chat/completions",
                        headers={"Authorization": f"Bearer {groq_key}", "Content-Type": "application/json"},
                        json={
                            "model": m,
                            "messages": [
                                {"role": "system", "content": system_prompt},
                                {"role": "user", "content": user_text}
                            ],
                            "tools": ROBOT_OPENAI_TOOLS,
                            "tool_choice": "auto",
                            "max_tokens": 120,
                            "temperature": 0.3
                        },
                        timeout=5.5
                    )
                    if resp.status_code == 200:
                        data = resp.json()
                        choice = data.get("choices", [{}])[0].get("message", {})
                        tool_calls = choice.get("tool_calls", [])
                        if tool_calls:
                            fn = tool_calls[0].get("function", {})
                            name = fn.get("name", "")
                            raw_args = fn.get("arguments", "{}")
                            try:
                                args = json.loads(raw_args) if isinstance(raw_args, str) else raw_args
                            except Exception:
                                args = {}
                            return ("tool_call", name, args)
                        content = choice.get("content", "")
                        if content:
                            return ("text", content.strip(), None)
                    elif resp.status_code in (400, 404):
                        continue
                except Exception:
                    continue
            return None

        try:
            res = await loop.run_in_executor(None, _call_groq)
            if res:
                kind, val1, val2 = res
                if kind == "tool_call":
                    return await execute_robot_voice_tool(bridge, opts, val1, val2 or {})
                elif kind == "text":
                    return (val1, "conversational_llm")
        except Exception:
            pass

    # 2. Gemini 2.0 Flash (multimodal, native tool calling, free daily tier)
    if gemini_key:
        def _call_gemini():
            import requests
            url = f"https://generativelanguage.googleapis.com/v1beta/models/gemini-2.0-flash:generateContent?key={gemini_key}"
            payload = {
                "contents": [
                    {"role": "user", "parts": [{"text": user_text}]}
                ],
                "systemInstruction": {
                    "parts": [{"text": system_prompt}]
                },
                "tools": ROBOT_GEMINI_TOOLS,
                "generationConfig": {"maxOutputTokens": 120, "temperature": 0.3}
            }
            resp = requests.post(url, json=payload, timeout=5.5)
            if resp.status_code == 200:
                data = resp.json()
                parts = data.get("candidates", [{}])[0].get("content", {}).get("parts", [])
                for part in parts:
                    if "functionCall" in part:
                        fc = part["functionCall"]
                        return ("tool_call", fc.get("name", ""), fc.get("args", {}))
                    if "text" in part and part["text"].strip():
                        return ("text", part["text"].strip(), None)
            return None

        try:
            res = await loop.run_in_executor(None, _call_gemini)
            if res:
                kind, val1, val2 = res
                if kind == "tool_call":
                    return await execute_robot_voice_tool(bridge, opts, val1, val2 or {})
                elif kind == "text":
                    return (val1, "conversational_llm")
        except Exception:
            pass

    return None


async def process_voice_agent_command(bridge: Any, opts: dict[str, Any], user_text: str) -> tuple[str, Optional[str]]:
    """Interpret user speech, execute corresponding robot action, or converse intelligently.

    Returns:
        (robot_reply_text, action_executed_or_none)
    """
    # 1. Primary: Intelligent LLM with Native Tool Calling (if Groq or Gemini key is configured)
    groq_key = _VOICE_CONFIG.get("groq_api_key") or os.getenv("GROQ_API_KEY", "").strip()
    gemini_key = _VOICE_CONFIG.get("gemini_api_key") or os.getenv("GEMINI_API_KEY", "").strip()
    if groq_key or gemini_key:
        try:
            llm_res = await _query_llm_with_tool_calling(bridge, opts, user_text)
            if llm_res is not None:
                return llm_res
        except Exception:
            pass

    # 2. Fallback: Fast Local Regex Engine (Zero Cloud / Offline Operation)
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

    # 9. Fallback Local Conversational Response
    return (f"I heard you say: '{user_text}'. I am ready to navigate, dock, or check status whenever you are ready!", "conversational")


class VoiceInterruptHandler(BaseHandler):
    """Instantly interrupt and cancel any active speech synthesis (barge-in reflex)."""

    async def post(self) -> None:
        try:
            from ..audio import stop_active_speech
            stop_active_speech()
        except Exception:
            pass
        self.send({'interrupted': True})


class VoiceListenHandler(BaseHandler):
    """Record live audio from robot microphone, transcribe, and optionally execute voice agent reply."""

    async def post(self) -> None:
        global _IS_LISTENING, _LAST_TRANSCRIPT, _LAST_REPLY, _CONVERSATION_HISTORY
        data = self.body()
        duration = float(data.get('duration_sec', 4.0))
        auto_reply = bool(data.get('auto_reply', True))

        if _IS_MUTED:
            raise ApiError(400, 'voice_muted', 'Microphone is currently muted in UI')

        # Instant Barge-In: immediately interrupt any currently playing audio so robot stops speaking
        try:
            from ..audio import stop_active_speech
            stop_active_speech()
        except Exception:
            pass

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

            reply_text = ""
            action = None
            if auto_reply:
                reply_text, action = await process_voice_agent_command(self.bridge, self.opts, transcript)

                # If speech was classified as background noise/third-party chatter, stay silent!
                if action == "ignored":
                    if hasattr(self.bridge, 'publish_led_command'):
                        self.bridge.publish_led_command('solid,0,255,100')
                    self.send({
                        'status': 'ignored',
                        'text': transcript,
                        'reply': '',
                        'action': 'ignored',
                        'messages': _CONVERSATION_HISTORY[-30:]
                    })
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

            if auto_reply:
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

        # Instant Barge-In: immediately interrupt any currently playing audio
        try:
            from ..audio import stop_active_speech
            stop_active_speech()
        except Exception:
            pass

        reply_text, action = await process_voice_agent_command(self.bridge, self.opts, message)

        if action == "ignored":
            self.send({
                'reply': '',
                'action': 'ignored',
                'messages': _CONVERSATION_HISTORY[-30:]
            })
            return

        user_msg = {
            "id": f"user_{int(time.time()*1000)}",
            "sender": "user",
            "text": message,
            "timestamp": time.time(),
            "action": None
        }
        with _VOICE_LOCK:
            _CONVERSATION_HISTORY.append(user_msg)

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

        if speak and reply_text:
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
    """Configure voice models, Groq / Gemini API keys, and model preferences (securely persisted)."""

    def get(self) -> None:
        groq_key = _VOICE_CONFIG.get("groq_api_key", "")
        gemini_key = _VOICE_CONFIG.get("gemini_api_key", "")
        self.send({
            "groq_configured": bool(groq_key),
            "groq_masked_key": _mask_key(groq_key),
            "gemini_configured": bool(gemini_key),
            "gemini_masked_key": _mask_key(gemini_key),
            "model_preference": _VOICE_CONFIG.get("model_preference", "groq"),
            "available_engines": [
                {"id": "groq", "name": "Groq Whisper Turbo + LLaMA 3.3 (Ultra Fast, Free Daily Tier)"},
                {"id": "gemini", "name": "Google Gemini 2.0 Flash (Multimodal, Free Daily Tier)"},
                {"id": "local", "name": "Google Speech Recognition + Offline Local Agent"}
            ]
        })

    def post(self) -> None:
        global _VOICE_CONFIG
        data = self.body()
        changed = False
        if "groq_api_key" in data:
            _VOICE_CONFIG["groq_api_key"] = str(data["groq_api_key"]).strip()
            changed = True
        if "gemini_api_key" in data:
            _VOICE_CONFIG["gemini_api_key"] = str(data["gemini_api_key"]).strip()
            changed = True
        if "model_preference" in data:
            pref = str(data["model_preference"]).strip().lower()
            if pref in ("groq", "gemini", "local"):
                _VOICE_CONFIG["model_preference"] = pref
                changed = True

        if changed:
            _save_persisted_config(_VOICE_CONFIG)
            _VOICE_CONFIG = _load_persisted_config()

        groq_key = _VOICE_CONFIG.get("groq_api_key", "")
        gemini_key = _VOICE_CONFIG.get("gemini_api_key", "")
        self.send({
            "status": "updated",
            "groq_configured": bool(groq_key),
            "groq_masked_key": _mask_key(groq_key),
            "gemini_configured": bool(gemini_key),
            "gemini_masked_key": _mask_key(gemini_key),
            "model_preference": _VOICE_CONFIG.get("model_preference")
        })
