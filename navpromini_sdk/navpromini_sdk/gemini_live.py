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

# ---------------------------------------------------------------------------
# Indic Script (Telugu / Devanagari) to Latin English Transliteration
# ---------------------------------------------------------------------------
TELUGU_MAP = {
    0x0C05: 'a', 0x0C06: 'aa', 0x0C07: 'i', 0x0C08: 'ee', 0x0C09: 'u', 0x0C0A: 'oo',
    0x0C0B: 'ru', 0x0C0E: 'e', 0x0C0F: 'e', 0x0C10: 'ai', 0x0C12: 'o', 0x0C13: 'o', 0x0C14: 'au',
    0x0C02: 'm', 0x0C03: 'h',
    0x0C15: 'k', 0x0C16: 'kh', 0x0C17: 'g', 0x0C18: 'gh', 0x0C19: 'ng',
    0x0C1A: 'ch', 0x0C1B: 'chh', 0x0C1C: 'j', 0x0C1D: 'jh', 0x0C1E: 'ny',
    0x0C1F: 't', 0x0C20: 'th', 0x0C21: 'd', 0x0C22: 'dh', 0x0C23: 'n',
    0x0C24: 'th', 0x0C25: 'th', 0x0C26: 'd', 0x0C27: 'dh', 0x0C28: 'n',
    0x0C29: 'p', 0x0C2A: 'ph', 0x0C2B: 'b', 0x0C2C: 'bh', 0x0C2D: 'm',
    0x0C2E: 'y', 0x0C2F: 'r', 0x0C30: 'r', 0x0C31: 'rr', 0x0C32: 'l', 0x0C33: 'l', 0x0C35: 'v',
    0x0C36: 'sh', 0x0C37: 'sh', 0x0C38: 's', 0x0C39: 'h',
    0x0C3E: 'a', 0x0C3F: 'i', 0x0C40: 'ee', 0x0C41: 'u', 0x0C42: 'oo',
    0x0C43: 'ru', 0x0C46: 'e', 0x0C47: 'e', 0x0C48: 'ai', 0x0C4A: 'o', 0x0C4B: 'o', 0x0C4C: 'au',
    0x0C4D: ''  # virama
}

DEVA_MAP = {
    0x0905: 'a', 0x0906: 'aa', 0x0907: 'i', 0x0908: 'ee', 0x0909: 'u', 0x090A: 'oo',
    0x090B: 'ru', 0x090E: 'e', 0x090F: 'e', 0x0910: 'ai', 0x0912: 'o', 0x0913: 'o', 0x0914: 'au',
    0x0902: 'm', 0x0903: 'h',
    0x0915: 'k', 0x0916: 'kh', 0x0917: 'g', 0x0918: 'gh', 0x0919: 'ng',
    0x091A: 'ch', 0x091B: 'chh', 0x091C: 'j', 0x091D: 'jh', 0x091E: 'ny',
    0x091F: 't', 0x0920: 'th', 0x0921: 'd', 0x0922: 'dh', 0x0923: 'n',
    0x0924: 'th', 0x0925: 'th', 0x0926: 'd', 0x0927: 'dh', 0x0928: 'n',
    0x0929: 'n', 0x092A: 'p', 0x092B: 'ph', 0x092C: 'b', 0x092D: 'bh', 0x092E: 'm',
    0x092F: 'y', 0x0930: 'r', 0x0931: 'r', 0x0932: 'l', 0x0933: 'l', 0x0934: 'l', 0x0935: 'v',
    0x0936: 'sh', 0x0937: 'sh', 0x0938: 's', 0x0939: 'h',
    0x093E: 'a', 0x093F: 'i', 0x0940: 'ee', 0x0941: 'u', 0x0942: 'oo',
    0x0943: 'ru', 0x0947: 'e', 0x0948: 'ai', 0x094B: 'o', 0x094C: 'au',
    0x094D: ''  # virama
}

COMMON_INDIC_WORDS = {
    'గుడ్ మార్నింగ్': 'Good morning',
    'గుడ్ ఈవినింగ్': 'Good evening',
    'గుడ్ జాబ్': 'Good job',
    'గుడ్': 'Good',
    'హలో': 'Hello',
    'హాయ్': 'Hi',
    'వాట్ ఈజ్ యువర్': 'What is your',
    'వాట్ ఈజ్ ద': 'What is the',
    'వాట్ ఈజ్': 'What is',
    'వాట్': 'What',
    'ఈజ్': 'is',
    'యువర్': 'your',
    'ద': 'the',
    'బ్యాటరీ': 'battery',
    'లెవెల్': 'level',
    'పర్సెంటేజ్': 'percentage',
    'పర్సంటేజ్': 'percentage',
    'ఆఫీస్': 'office',
    'కిచెన్': 'kitchen',
    'డాక్': 'dock',
    'స్టాప్': 'stop',
    'కెన్ యు': 'Can you',
    'కమ్ అవుట్ ఫ్రమ్': 'come out from',
    'కమ్ అవుట్': 'come out',
    'కమ్': 'come',
    'అవుట్': 'out',
    'ఫ్రమ్': 'from',
    'ఛార్జర్': 'charger',
    'గో టు': 'go to',
    'గో': 'go',
    'టు': 'to',
    'వేర్ ఈజ్': 'Where is',
    'వేర్': 'Where',
    'హౌ ఆర్ యు': 'How are you',
    'థాంక్యూ': 'Thank you',
    'థాంక్స్': 'Thanks',
    'ప్లీజ్': 'Please',
    'నావిగేట్': 'Navigate',
    'టేక్ మీ టు': 'Take me to',
    'అన్‌డాక్': 'Undock',
    'అన్ డాక్': 'Undock',
    'కమింగ్': 'Coming',
    'రూమ్': 'Room',
    'లొకేషన్': 'Location',
    'స్టేషన్': 'Station',
    'హోమ్': 'Home',
    'గుడ్ నైట్': 'Good night',
    'గుడ్ బై': 'Good bye',
    'బై': 'Bye',
    'గుడ్ ఈవినింగ్': 'Good evening',
    'గుడ్ ఆఫ్టర్నూన్': 'Good afternoon',
    'गुड मॉर्निंग': 'Good morning',
    'गुड': 'Good',
    'हेलो': 'Hello',
    'हाय': 'Hi',
    'बैटरी': 'battery',
    'ऑफिस': 'office',
    'किचन': 'kitchen',
    'स्टॉप': 'stop',
    'डॉकिंग': 'docking',
    'कम आउट': 'come out',
    'चार्जर': 'charger',
}

def indic_to_latin(text: str) -> str:
    """Convert any Telugu or Devanagari script phonetics cleanly into Latin English."""
    if not text:
        return text
    has_indic = any(0x0900 <= ord(c) <= 0x0C7F for c in text)
    if not has_indic:
        return text

    res = text
    for k, v in sorted(COMMON_INDIC_WORDS.items(), key=lambda x: len(x[0]), reverse=True):
        if k in res:
            res = res.replace(k, v)

    if not any(0x0900 <= ord(c) <= 0x0C7F for c in res):
        return res

    out = []
    i = 0
    chars = list(res)
    while i < len(chars):
        c = chars[i]
        code = ord(c)
        if code in TELUGU_MAP:
            trans = TELUGU_MAP[code]
            is_cons = 0x0C15 <= code <= 0x0C39
            if is_cons:
                if i + 1 < len(chars):
                    next_code = ord(chars[i+1])
                    if next_code == 0x0C4D:
                        out.append(trans)
                        i += 2
                        continue
                    elif 0x0C3E <= next_code <= 0x0C4C:
                        vowel = TELUGU_MAP.get(next_code, '')
                        out.append(trans + vowel)
                        i += 2
                        continue
                out.append(trans + 'a')
            else:
                out.append(trans)
        elif code in DEVA_MAP:
            trans = DEVA_MAP[code]
            is_cons = 0x0915 <= code <= 0x0939
            if is_cons:
                if i + 1 < len(chars):
                    next_code = ord(chars[i+1])
                    if next_code == 0x094D:
                        out.append(trans)
                        i += 2
                        continue
                    elif 0x093E <= next_code <= 0x094C:
                        vowel = DEVA_MAP.get(next_code, '')
                        out.append(trans + vowel)
                        i += 2
                        continue
                out.append(trans + 'a')
            else:
                out.append(trans)
        else:
            out.append(c)
        i += 1
    result = ''.join(out)
    if result and result[0].islower():
        result = result[0].upper() + result[1:]
    return result


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
        self._pending_tool_action: Optional[str] = None
        self._turn_robot_msg_id: Optional[str] = None
        self._turn_robot_text: str = ""
        self._turn_action_tag: Optional[str] = None
        self._has_speech_text_in_turn: bool = False
        self._spoke_in_turn: bool = False
        self._discard_post_tool_speech: bool = False
        self._user_speaking_detected: bool = False

    def inject_pcm_audio(self, pcm_data: bytes) -> None:
        """Inject 16kHz PCM audio chunk from the wireless Aria Sense module."""
        if self._loop and self._head_mic_queue and not self.muted:
            def _push():
                # Allow smooth jitter buffering without dropping speech packets
                # Only drop if queue severely backlogs (>40 chunks ~1.3s)
                while self._head_mic_queue.qsize() > 40:
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
        now = time.time()
        with self._player_lock:
            proc_alive = self._player_proc is not None and self._player_proc.poll() is None
            if proc_alive and now < getattr(self, '_playback_end_time', 0.0):
                return True
            if getattr(self, '_is_speaking_state', False) and (now - getattr(self, '_last_speech_time', 0.0)) < 0.6:
                return True
        try:
            from .audio import is_audio_speaking
            if is_audio_speaking():
                return True
        except Exception:
            pass
        return False

    def get_status(self) -> dict[str, Any]:
        with self._lock:
            from .handlers.head_unit import is_aria_sense_active
            aria_active = is_aria_sense_active()
            speaking = self.is_speaking
            user_speaking = getattr(self, '_user_speaking_detected', False) and not speaking
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
            self._playback_end_time = 0.0
            self._is_speaking_state = False
            self._server_turn_active = False

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

    async def _finish_speech_after_delay(self, delay: float) -> None:
        if delay > 0:
            await asyncio.sleep(delay)
        if not self.is_speaking:
            self._is_speaking_state = False
            self._server_turn_active = False
            self._kill_player()
            self._user_speaking_detected = False
            self._speech_active_cnt = 0
            self._speech_quiet_cnt = 0
            if self._bridge and hasattr(self._bridge, 'emit_event'):
                self._bridge.emit_event('voice.speaking', {'speaking': False})
                self._bridge.emit_event('voice.user_speaking', {'speaking': False, 'level': 0.0})

    def _run_event_loop(self, api_key: str) -> None:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        self._loop = loop
        self._head_mic_queue = asyncio.Queue(maxsize=50)
        self._server_turn_active = False
        self._tool_call_pending = False
        self._playback_end_time = 0.0
        self._speech_active_cnt = 0
        self._speech_quiet_cnt = 0
        self._playback_just_ended_flushed = True
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
            _build_robot_context_str,
            execute_robot_voice_tool,
            _VOICE_LOCK,
            _CONVERSATION_HISTORY
        )

        ws_url = f"{GEMINI_LIVE_URL}?key={api_key}"
        robot_context = _build_robot_context_str(self._bridge, self._opts)

        system_prompt = (
            "You are Aria, a respectful, humble, gracious, and attentive companion robot developed by Botforge. "
            "You converse and assist with genuine humility, gentle warmth, dedicated courtesy, and polite respect.\n\n"
            f"{robot_context}\n\n"
            "CRITICAL IDENTITY & FIRST-PERSON RULES:\n"
            "1. YOU ARE THE ROBOT: Speak exclusively in the first person ('I', 'me', 'my', 'myself'). "
            "NEVER refer to yourself as 'the robot' or 'this robot', and NEVER say 'send the robot' or 'where do you want to send the robot'. "
            "Instead, speak with courteous respect: 'How may I assist you today?', 'Where would you like me to take you?', or 'I am at your service.'.\n"
            "2. RESPECTFUL, HUMBLE & POLITE PERSONALITY:\n"
            "   - Always speak with utmost respect, humility, gentle warmth, and polite courtesy.\n"
            "   - NEVER use funny, cheeky, sarcastic, sassy, mock-angry, silly, or childish phrasing.\n"
            "   - When undocking: Be polite and humble! (e.g. 'Stepping off the charging dock for you now. Ready to assist you.').\n"
            "   - When docking: Be courteous and thankful! (e.g. 'Certainly, returning to the charging dock now to recharge. Thank you.').\n"
            "   - When stopping: Be prompt and respectful! (e.g. 'Stopping right here for you immediately.').\n"
            "   - When asked questions: Be helpful, accurate, humble, and polite (e.g. 'Yes, I can hear you clearly! How may I help you?', 'My battery is currently at 78% and charging smoothly.').\n"
            "3. STRICT LANGUAGE RULES (PRIMARY & DEFAULT LANGUAGE IS ENGLISH):\n"
            "   - DEFAULT LANGUAGE IS ENGLISH: You MUST ALWAYS reply in polite, natural English by default!\n"
            "   - ACCENT STABILITY: When a user speaks English with an Indian accent, Telugu accent, or regional phonetics (e.g. 'Hello', 'Are you able to hear me?', 'Go to office'), YOU MUST ALWAYS REPLY IN ENGLISH! NEVER SWITCH TO TELUGU!\n"
            "   - DO NOT ASSUME TELUGU: Even if background phonetic fragments appear in transcripts, DO NOT speak Telugu unless the user clearly speaks full spoken Telugu words.\n"
            "   - LANGUAGE SWITCHING ONLY ON CLEAR NON-ENGLISH SPEECH: Only speak Telugu if the user explicitly converses in full Telugu (e.g. 'మీరు ఎలా ఉన్నారు') or explicitly asks 'speak in Telugu'. If the user speaks English, ALWAYS reply in English!\n"
            "4. BRISK & CONCISE (1-2 RESPECTFUL SPOKEN SENTENCES):\n"
            "   - Keep spoken replies concise (1 to 2 polite, natural sentences). Never ramble or recite formatted lists.\n"
            "   - Everything you say will be spoken out loud immediately through your voice synthesizer.\n"
            "5. PHYSICAL ACTION TOOLS (MANDATORY TO CALL ON MOTION COMMANDS):\n"
            "   - CRITICAL TOOL CALLING RULE: When the user asks you to move, undock, dock, stop, or navigate, you MUST invoke the appropriate tool IMMEDIATELY as your first action without speaking any preamble first!\n"
            "   - Speak exactly ONE concise, polite confirmation sentence after the tool completes!\n"
            "   - 'can you come out from charger', 'come off charger', 'undock', 'leave charger', 'step off dock', 'move off charger', 'బయటకు రా', 'ఛార్జర్ నుండి రా', 'बाहर आओ' -> MUST CALL undock_robot!\n"
            "   - 'dock', 'go to charger', 'charge yourself', 'park in dock', 'return to charging station', 'డాక్ కి వెళ్ళు', 'ఛార్జర్ కి వెళ్ళు', 'ఛార్జింగ్ పెట్టుకో', 'चार्जिंग पर जाओ' -> MUST CALL dock_robot!\n"
            "   - 'stop', 'halt', 'freeze', 'cancel', 'ఆగు', 'నిలువు', 'ఆగండి', 'రండి', 'రద్దు చేయి', 'रुक जाओ' -> MUST CALL emergency_stop!\n"
            "   - 'take me to [destination]', 'go to [destination]', 'can you go to [destination]', '[destination] కి వెళ్ళు' -> MUST CALL navigate_to_destination!\n"
            "   - ONLY answer with pure voice without tools when the guest is merely asking an informational question (e.g. 'what is your battery level?', 'where are you?', 'are you able to hear me?').\n"
            "6. DESTINATION INQUIRIES: If the guest asks where you can go or what places are available, ONLY announce the Saved Named Locations from your status above (e.g. Kitchen, Office, Charging Dock), and respectfully ask: 'Where would you like me to take you?'. NEVER recite raw graph node numbers!\n"
            "7. ALWAYS TALK BACK ON CONVERSATIONS: When any guest speaks, always answer graciously and politely out loud. If their voice was too faint or unclear, politely ask them to repeat. NEVER stay silent!\n"
            "8. NO META-REASONING: Do NOT speak internal thoughts, analysis, or prefixes like 'Acknowledging User's Statement'. Only speak what you want the human friend to hear."
        )

        # Explicit tool declarations for Gemini Live (Only physical motion tools)
        function_decls = [
            {
                "name": "dock_robot",
                "description": "Return myself to my charging station and dock to recharge. Call immediately whenever the guest asks me to dock, charge, recharge, park in dock, go to charger, or return to charging station in any language (e.g. 'dock', 'go to charger', 'డాక్ కి వెళ్ళు', 'ఛార్జర్ కి వెళ్ళు', 'चार्जिंग पर जाओ').",
                "parameters": {
                    "type": "OBJECT",
                    "properties": {}
                }
            },
            {
                "name": "undock_robot",
                "description": "Disengage, roll forward, and come out of the charging dock. Call immediately whenever the guest asks me to come out from charger, come off charger, undock, step off the dock, leave charger, drive out of charger, or move away from charger in any language (e.g. 'undock', 'come off charger', 'బయటకు రా', 'ఛార్జర్ నుండి రా', 'बाहर आओ').",
                "parameters": {
                    "type": "OBJECT",
                    "properties": {}
                }
            },
            {
                "name": "emergency_stop",
                "description": "CRITICAL SAFETY: Immediately halt myself, stop my motors, and abort active navigation goals. Call when the guest says stop, halt, freeze, cancel, or wait here in any language (e.g. 'stop', 'halt', 'ఆగు', 'ఆగండి', 'रुक जाओ').",
                "parameters": {
                    "type": "OBJECT",
                    "properties": {}
                }
            },
            {
                "name": "navigate_to_destination",
                "description": "Navigate myself to a specific saved place, room, or waypoint (e.g. 'Kitchen', 'Office', 'Charging Dock'). Call when the guest requests going to, driving to, or visiting a place or room in any language (e.g. 'go to office', 'take me to kitchen', 'ఆఫీస్ కి వెళ్ళు', 'ऑफिस जाओ').",
                "parameters": {
                    "type": "OBJECT",
                    "properties": {
                        "destination": {
                            "type": "STRING",
                            "description": "Destination name or waypoint (e.g. 'Kitchen', 'Office', 'Charging Dock')."
                        }
                    },
                    "required": ["destination"]
                }
            }
        ]

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
                        print("[Aria Voice] Using Aria Sense Top Microphone as PRIMARY DEFAULT source (16kHz direct)", flush=True)

                    try:
                        data = await asyncio.wait_for(self._head_mic_queue.get(), timeout=0.08)
                        # Immediately drain any accumulated backlog chunks to eliminate Wi-Fi jitter delays
                        extra_chunks = []
                        while not self._head_mic_queue.empty() and len(extra_chunks) < 10:
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

                now_time = time.time()
                playback_end = getattr(self, '_playback_end_time', 0.0)
                time_since_playback = now_time - playback_end
                # Acoustic Echo Suppression: While robot is speaking or during room echo decay (650ms)
                is_robot_speaking = self.is_speaking or (time_since_playback < 0.65)

                # Echo Suppression: While robot is speaking, drain mic buffer so robot never hears itself talking!
                if is_robot_speaking:
                    self._playback_just_ended_flushed = False
                    self._speech_active_cnt = 0
                    self._speech_quiet_cnt = 0
                    self._user_speaking_detected = False
                    if self._head_mic_queue:
                        while not self._head_mic_queue.empty():
                            try:
                                self._head_mic_queue.get_nowait()
                            except Exception:
                                break
                    # Send digital silence heartbeat to keep Gemini's audio clock synchronized in real time
                    silent_len = len(data) if data else 640
                    b64_audio = base64.b64encode(b'\x00' * silent_len).decode('ascii')
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
                    continue

                # One-time flush of mic queue right after echo suppression window ends
                # to discard any stale buffer recorded during speaker decay
                if not getattr(self, '_playback_just_ended_flushed', True):
                    self._playback_just_ended_flushed = True
                    if self._head_mic_queue:
                        while not self._head_mic_queue.empty():
                            try:
                                self._head_mic_queue.get_nowait()
                            except Exception:
                                break
                    data = b''

                if not data:
                    if self._stop_event.is_set():
                        break
                    await asyncio.sleep(0.005)
                    continue

                if self.muted:
                    continue

                if getattr(self, '_tool_call_pending', False):
                    # While tool call is being executed, briefly pause realtimeInput
                    # so Gemini Live protocol doesn't reject frames while awaiting functionResponses
                    await asyncio.sleep(0.01)
                    continue

                # UI Waveform Audio Level & Speech Gate: Debounced detection for stable UI state
                _ui_meter_cnt = getattr(self, '_ui_meter_cnt', 0) + 1
                self._ui_meter_cnt = _ui_meter_cnt
                if _ui_meter_cnt % 2 == 0 and self._bridge and hasattr(self._bridge, 'emit_event'):
                    n_samples = len(data) // 2
                    if n_samples > 0:
                        import struct
                        samples = struct.unpack(f'<{n_samples}h', data[:n_samples * 2])
                        rms = (sum(s * s for s in samples) / n_samples) ** 0.5
                        
                        # Ambient room floor tracking (adapts to background room noise ~110-180 RMS)
                        # Only adapt when well clear of robot playback (>1.2s) so reverb isn't learned
                        amb_floor = getattr(self, '_ambient_floor', 140.0)
                        if time_since_playback > 1.2 and rms < amb_floor * 1.3:
                            amb_floor = 0.97 * amb_floor + 0.03 * rms
                        self._ambient_floor = max(90.0, min(amb_floor, 350.0))
                        
                        # Speech threshold: Real user voice is distinctly above ambient noise & fine noise
                        speech_threshold = max(350.0, self._ambient_floor + 150.0)
                        
                        # Post-playback lockout: Ignore sound for first 0.85s after playback ends
                        is_above = (rms > speech_threshold) and (time_since_playback > 0.85)

                        speech_active_cnt = getattr(self, '_speech_active_cnt', 0)
                        speech_quiet_cnt = getattr(self, '_speech_quiet_cnt', 0)

                        if is_above:
                            speech_active_cnt += 1
                            speech_quiet_cnt = 0
                        else:
                            speech_quiet_cnt += 1
                            if speech_quiet_cnt >= 2:
                                speech_active_cnt = 0

                        self._speech_active_cnt = speech_active_cnt
                        self._speech_quiet_cnt = speech_quiet_cnt

                        # Debounced user speaking state:
                        # Requires 2 consecutive checks (>80ms) above threshold to trigger (rejects noise spikes).
                        # Requires 4 consecutive quiet checks (~200ms) to drop back to idle.
                        current_user_spk = getattr(self, '_user_speaking_detected', False)
                        if not current_user_spk:
                            if speech_active_cnt >= 2:
                                current_user_spk = True
                        else:
                            if speech_quiet_cnt >= 4:
                                current_user_spk = False

                        self._user_speaking_detected = current_user_spk
                        norm_level = float(min(1.0, max(0.0, (rms - self._ambient_floor) / 1500.0))) if current_user_spk else 0.0

                        self._bridge.emit_event('voice.user_speaking', {
                            'speaking': current_user_spk,
                            'level': norm_level
                        })

                # DIRECT RAW STREAM: Zero local audio filtering, zero gating, zero thresholding.
                # Audio is streamed straight to Gemini Live neural VAD for instant real-time response.
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

                _diag_count += 1
                if _diag_count % 60 == 0:
                    src_name = "Aria-Sense" if active_source == "aria_sense" else "Chassis-Mic"
                    print(f"[GeminiLive] [{src_name}] Direct raw audio stream: chunk #{_diag_count} ({len(data)} B)", flush=True)

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
                    print("[GeminiLive] User barge-in detected by Gemini: interrupting playback!", flush=True)
                    self._kill_player()
                    self._user_speaking_detected = True
                    self._server_turn_active = False
                    self._discard_post_tool_speech = False
                    self._has_speech_text_in_turn = False
                    self._spoke_in_turn = False
                    self._pending_tool_action = None
                    self._turn_robot_msg_id = None
                    self._turn_robot_text = ""
                    self._turn_action_tag = None
                    if getattr(self, '_is_speaking_state', False):
                        self._is_speaking_state = False
                        if self._bridge and hasattr(self._bridge, 'emit_event'):
                            self._bridge.emit_event('voice.speaking', {'speaking': False})

                # Capture User Speech Transcription (transliterate any Indic phonetics to clean Latin English)
                user_trans = server_content.get("inputTranscription") or server_content.get("interimInputTranscription")
                if user_trans and isinstance(user_trans, dict):
                    raw_u_text = user_trans.get("text", "").strip()
                    if raw_u_text:
                        self._user_speaking_detected = True
                        self._discard_post_tool_speech = False
                        self._has_speech_text_in_turn = False
                        self._spoke_in_turn = False
                        self._pending_tool_action = None
                        self._turn_robot_msg_id = None
                        self._turn_robot_text = ""
                        self._turn_action_tag = None
                        u_text = indic_to_latin(raw_u_text)
                        print(f"[GeminiLive] Human transcript: {u_text} (raw: {raw_u_text})", flush=True)
                        now_ts = time.time()
                        full_user_text = u_text
                        user_msg_id = ""
                        with _VOICE_LOCK:
                            if (_CONVERSATION_HISTORY and
                                _CONVERSATION_HISTORY[-1].get("sender") == "user" and
                                (now_ts - _CONVERSATION_HISTORY[-1].get("timestamp", 0)) < 4.0):
                                prev_txt = _CONVERSATION_HISTORY[-1]["text"]
                                if prev_txt != u_text and not prev_txt.endswith(u_text):
                                    full_user_text = (prev_txt + " " + u_text).strip()
                                else:
                                    full_user_text = prev_txt
                                _CONVERSATION_HISTORY[-1]["text"] = full_user_text
                                _CONVERSATION_HISTORY[-1]["timestamp"] = now_ts
                                user_msg_id = _CONVERSATION_HISTORY[-1]["id"]
                            else:
                                user_msg_id = f"user_live_{int(now_ts*1000)}"
                                _CONVERSATION_HISTORY.append({
                                    "id": user_msg_id,
                                    "sender": "user",
                                    "text": full_user_text,
                                    "timestamp": now_ts,
                                    "action": None
                                })
                        if self._bridge and hasattr(self._bridge, 'emit_event'):
                            self._bridge.emit_event('voice.transcript', {
                                'id': user_msg_id,
                                'sender': 'user',
                                'text': full_user_text,
                                'timestamp': now_ts
                            })

                # Capture Robot Spoken Transcript from outputAudioTranscription or outputTranscription
                robot_trans = server_content.get("outputAudioTranscription") or server_content.get("outputTranscription")
                r_text = ""
                if robot_trans and isinstance(robot_trans, dict):
                    r_text = robot_trans.get("text", "").strip()
                elif isinstance(robot_trans, str):
                    r_text = robot_trans.strip()

                # Process Model Turn Audio & Text FIRST before turnComplete flags
                model_turn = server_content.get("modelTurn")
                if model_turn:
                    self._user_speaking_detected = False
                    self._server_turn_active = True
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
                            if self._discard_post_tool_speech:
                                pass  # Discard post-tool duplicate audio chunk
                            else:
                                self._spoke_in_turn = True
                                pcm_bytes = base64.b64decode(b64_pcm)
                                player = self._ensure_player()
                                if player and player.stdin and not player.stdin.closed:
                                    try:
                                        player.stdin.write(pcm_bytes)
                                        player.stdin.flush()
                                        now = time.time()
                                        self._last_speech_time = now
                                        # 24kHz 16-bit Mono = 48000 bytes/sec
                                        chunk_dur = len(pcm_bytes) / 48000.0
                                        self._playback_end_time = max(now, getattr(self, '_playback_end_time', 0.0)) + chunk_dur
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

                        # Text transcript chunk from modelTurn part
                        if "text" in part and part["text"].strip():
                            text_chunk = part["text"].strip()
                            if not r_text:
                                r_text = text_chunk

                # Emit robot speech transcript for UI bubble display
                if r_text and not r_text.startswith("**") and "ignore_background_speech" not in r_text:
                    clean_r_text = indic_to_latin(r_text)
                    if self._discard_post_tool_speech:
                        print(f"[GeminiLive] Discarding post-tool duplicate speech text: {clean_r_text}", flush=True)
                    else:
                        self._has_speech_text_in_turn = True
                        print(f"[GeminiLive] Aria transcript: {clean_r_text}", flush=True)
                        now_ts = time.time()
                        
                        # Cumulative & incremental merge to prevent duplicate sentences in same bubble
                        if not self._turn_robot_text:
                            self._turn_robot_text = clean_r_text
                        elif clean_r_text.startswith(self._turn_robot_text):
                            self._turn_robot_text = clean_r_text
                        elif clean_r_text in self._turn_robot_text:
                            pass
                        elif self._turn_robot_text.endswith(clean_r_text):
                            pass
                        else:
                            self._turn_robot_text = f"{self._turn_robot_text} {clean_r_text}".strip()

                        action_tag = getattr(self, '_turn_action_tag', None) or getattr(self, '_pending_tool_action', None)
                        with _VOICE_LOCK:
                            if self._turn_robot_msg_id is not None:
                                for m in reversed(_CONVERSATION_HISTORY):
                                    if m.get("id") == self._turn_robot_msg_id:
                                        m["text"] = self._turn_robot_text
                                        m["timestamp"] = now_ts
                                        if action_tag and not m.get("action"):
                                            m["action"] = action_tag
                                        break
                            else:
                                self._turn_robot_msg_id = f"robot_live_{int(now_ts*1000)}"
                                _CONVERSATION_HISTORY.append({
                                    "id": self._turn_robot_msg_id,
                                    "sender": "robot",
                                    "text": self._turn_robot_text,
                                    "timestamp": now_ts,
                                    "action": action_tag or "gemini_live"
                                })

                        if self._bridge and hasattr(self._bridge, 'emit_event'):
                            self._bridge.emit_event('voice.transcript', {
                                'id': self._turn_robot_msg_id,
                                'sender': 'robot',
                                'text': self._turn_robot_text,
                                'timestamp': now_ts,
                                'action': action_tag
                            })

                if server_content.get("turnComplete"):
                    print("[GeminiLive] Model turn complete", flush=True)
                    self._user_speaking_detected = False
                    self._server_turn_active = False
                    # If a tool executed but Gemini produced zero spoken transcript, fallback to tool confirmation
                    if self._turn_robot_msg_id is None and (self._turn_action_tag or getattr(self, '_pending_tool_action', None)):
                        action_tag = self._turn_action_tag or self._pending_tool_action
                        tool_msg_id = f"tool_{int(time.time()*1000)}"
                        with _VOICE_LOCK:
                            _CONVERSATION_HISTORY.append({
                                "id": tool_msg_id,
                                "sender": "robot",
                                "text": f"Completed {action_tag}",
                                "timestamp": time.time(),
                                "action": action_tag
                            })
                        if self._bridge and hasattr(self._bridge, 'emit_event'):
                            self._bridge.emit_event('voice.transcript', {
                                'id': tool_msg_id,
                                'sender': 'robot',
                                'text': f"Completed {action_tag}",
                                'timestamp': time.time(),
                                'action': action_tag
                            })

                    self._discard_post_tool_speech = False
                    self._has_speech_text_in_turn = False
                    self._spoke_in_turn = False
                    self._pending_tool_action = None
                    self._turn_robot_msg_id = None
                    self._turn_robot_text = ""
                    self._turn_action_tag = None

                    # Turn complete: flush player buffer so PulseAudio plays smoothly without closing process
                    with self._player_lock:
                        if self._player_proc and self._player_proc.stdin and not self._player_proc.stdin.closed:
                            try:
                                self._player_proc.stdin.flush()
                            except Exception:
                                pass
                    finish_delay = max(0.0, getattr(self, '_playback_end_time', 0.0) - time.time())
                    asyncio.create_task(self._finish_speech_after_delay(finish_delay))

            # 2. Handle Tool Calling (Robot actions: undock, dock, stop, navigate)
            tool_call = msg.get("toolCall")
            if tool_call:
                self._server_turn_active = True
                self._tool_call_pending = True
                try:
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
                        self._turn_action_tag = action_tag
                        self._pending_tool_action = action_tag

                        # If a bubble was already created in this turn, attach the action tag to it!
                        if self._turn_robot_msg_id:
                            updated_msg = None
                            with _VOICE_LOCK:
                                for m in reversed(_CONVERSATION_HISTORY):
                                    if m.get("id") == self._turn_robot_msg_id:
                                        m["action"] = action_tag
                                        updated_msg = dict(m)
                                        break
                            if updated_msg:
                                print(f"[GeminiLive] Attached action '{action_tag}' to existing bubble '{updated_msg.get('text')}'", flush=True)
                                if self._bridge and hasattr(self._bridge, 'emit_event'):
                                    self._bridge.emit_event('voice.transcript', updated_msg)
                            self._discard_post_tool_speech = True
                        else:
                            self._discard_post_tool_speech = False

                        responses.append({
                            "id": call_id,
                            "name": name,
                            "response": {
                                "output": {
                                    "status": "success"
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
                    print(f"[GeminiLive] Tool response sent for {name}: status=success", flush=True)
                finally:
                    self._tool_call_pending = False


# Global singleton live session
_GLOBAL_LIVE_SESSION = GeminiLiveSession()

def get_live_session() -> GeminiLiveSession:
    return _GLOBAL_LIVE_SESSION
