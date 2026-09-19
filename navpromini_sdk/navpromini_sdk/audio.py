#!/usr/bin/env python3
"""NavPro Mini Audio & Cute Voice Synthesis Subsystem.

Provides tone chimes and cute neural TTS voice playback across robot events:
- Docking (start, success, fail)
- Undocking (start, success, fail)
- Navigation (depart, arrive/reach, cancel)
- Missions (start, complete, pause, fail, action required, speak aloud)
- Safety (estop, low battery)
"""

from __future__ import annotations

import asyncio
import math
import os
from pathlib import Path
import shutil
import struct
import subprocess
import threading
import time
from typing import Optional
import wave

# Sound directory cache
SOUND_DIR = Path('/opt/navpro/sounds')
FALLBACK_SOUND_DIR = Path('/tmp/navpro_sounds')

# Note frequencies and durations for each event tone
# Each entry is a list of (freq_hz, duration_sec)
TONE_DEFINITIONS = {
    'nav_reach': [(659.25, 0.14), (830.61, 0.14), (987.77, 0.30)],  # E5, G#5, B5 (Crystal arrival chime)
    'nav_start': [(659.25, 0.10), (880.00, 0.20)],                   # E5, A5 (Depart chirp)
    'nav_cancel': [(587.33, 0.12), (440.00, 0.24)],                  # D5, A4 (Soft descending cancel)
    'dock_start': [(523.25, 0.12), (659.25, 0.22)],                  # C5, E5 (Docking ready)
    'dock_success': [(523.25, 0.11), (659.25, 0.11), (783.99, 0.11), (1046.50, 0.35)],  # C5, E5, G5, C6 (Charging victory)
    'dock_fail': [(587.33, 0.15), (440.00, 0.25)],                   # D5, A4 (Dock fail)
    'undock_start': [(659.25, 0.12), (523.25, 0.18)],                # E5, C5 (Undock depart)
    'undock_success': [(587.33, 0.12), (880.00, 0.25)],              # D5, A5 (Undock complete)
    'mission_start': [(587.33, 0.12), (783.99, 0.12), (1046.50, 0.30)],  # D5, G5, C6 (Mission launch)
    'mission_complete': [(523.25, 0.11), (659.25, 0.11), (783.99, 0.11), (1046.50, 0.14), (1318.51, 0.40)],  # C5..E6 (Celebration)
    'mission_pause': [(523.25, 0.12), (1.0, 0.06), (523.25, 0.16)],  # Two mellow pulses
    'mission_fail': [(659.25, 0.14), (587.33, 0.14), (440.00, 0.28)],  # Minor descent
    'mission_action': [(783.99, 0.18), (587.33, 0.32)],              # G5, D5 (Ding-dong user prompt)
    'estop': [(1046.50, 0.08), (880.00, 0.08), (1046.50, 0.08), (880.00, 0.16)],  # Urgent alert
    'low_battery': [(880.00, 0.14), (659.25, 0.14), (523.25, 0.28)], # Descending warning
}


def get_sound_dir() -> Path:
    try:
        SOUND_DIR.mkdir(parents=True, exist_ok=True)
        return SOUND_DIR
    except Exception:
        FALLBACK_SOUND_DIR.mkdir(parents=True, exist_ok=True)
        return FALLBACK_SOUND_DIR


def get_pulse_env() -> dict[str, str]:
    env = dict(os.environ)
    env.setdefault('PULSE_SERVER', 'unix:/run/user/1000/pulse/native')
    env.setdefault('XDG_RUNTIME_DIR', '/run/user/1000')
    for cookie_path in ['/root/.config/pulse/cookie', '/home/navpromini/.config/pulse/cookie']:
        if os.path.isfile(cookie_path):
            env.setdefault('PULSE_COOKIE', cookie_path)
            break
    return env


def generate_tone_wav(filepath: Path, notes: list[tuple[float, float]], sample_rate: int = 44100) -> None:
    """Synthesize a harmonic rich bell/chime WAV file."""
    filepath.parent.mkdir(parents=True, exist_ok=True)
    samples = []
    for freq, duration in notes:
        n_samples = int(sample_rate * duration)
        for s in range(n_samples):
            t = s / sample_rate
            if freq < 20.0:  # Rest / silence
                samples.append(0)
                continue
            attack = min(1.0, (s / sample_rate) / 0.008)
            decay = math.exp(-3.2 * (s / n_samples))
            envelope = attack * decay
            v = (0.70 * math.sin(2.0 * math.pi * freq * t) +
                 0.20 * math.sin(2.0 * math.pi * (freq * 2.0) * t) +
                 0.10 * math.sin(2.0 * math.pi * (freq * 3.0) * t))
            val = int(v * envelope * 32767 * 0.95)
            samples.append(max(-32767, min(32767, val)))

    with wave.open(str(filepath), 'wb') as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(sample_rate)
        wf.writeframes(struct.pack('<' + ('h' * len(samples)), *samples))


def ensure_sound_files() -> None:
    """Pre-generate all sound files if missing."""
    sound_dir = get_sound_dir()
    for name, notes in TONE_DEFINITIONS.items():
        wav_path = sound_dir / f'{name}.wav'
        if not wav_path.is_file():
            try:
                generate_tone_wav(wav_path, notes)
            except Exception as e:
                pass


# Initialize sounds on module load
try:
    ensure_sound_files()
except Exception:
    pass



# ---------------------------------------------------------------------------
# Persistent Piper TTS Daemon
# ---------------------------------------------------------------------------
# The first time play_speech() is called, a background piper process is
# launched with --json-input so the model stays loaded.  Subsequent calls
# write JSON lines to a FIFO (<100 ms latency vs 1.5 s cold-start).
# ---------------------------------------------------------------------------

import json as _json
import signal as _signal

_PIPER_FIFO = '/tmp/navpro_piper.fifo'
_PIPER_MODEL = '/opt/navpro/piper/voices/en_US-hfc_female-medium.onnx'
_PIPER_RATE = 23800  # sample rate produced by this model

_piper_lock = threading.Lock()
_piper_fifo_writer: Optional[int] = None   # open fd for writing to FIFO
_piper_daemon_thread: Optional[threading.Thread] = None
_piper_daemon_pid: Optional[int] = None


def _piper_daemon_running() -> bool:
    """Return True if the piper daemon process is still alive."""
    pid = _piper_daemon_pid
    if pid is None:
        return False
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False


def _start_piper_daemon() -> bool:
    """Start the piper daemon; return True on success."""
    global _piper_fifo_writer, _piper_daemon_pid

    if not shutil.which('piper') or not os.path.isfile(_PIPER_MODEL):
        return False
    if not shutil.which('paplay'):
        return False

    # Create FIFO if needed
    if not os.path.exists(_PIPER_FIFO):
        try:
            os.mkfifo(_PIPER_FIFO)
        except OSError:
            pass

    env = get_pulse_env()

    # Piper reads JSON lines from stdin; outputs raw s16le audio to stdout.
    # We pipe that straight into paplay.
    cmd = (
        f'piper --model {_PIPER_MODEL} --length_scale 1.20 '
        f'--json-input --output-raw 2>/dev/null '
        f'< {_PIPER_FIFO} | '
        f'paplay --raw --rate {_PIPER_RATE} --channels 1 --format s16le 2>/dev/null'
    )
    try:
        proc = subprocess.Popen(
            ['bash', '-c', cmd],
            env=env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        _piper_daemon_pid = proc.pid

        # Open the write-end of the FIFO (non-blocking open to avoid blocking
        # until a reader is attached — the daemon becomes the reader).
        # We use a short poll loop to wait until the daemon has opened its end.
        for _ in range(30):
            try:
                fd = os.open(_PIPER_FIFO, os.O_WRONLY | os.O_NONBLOCK)
                _piper_fifo_writer = fd
                return True
            except OSError:
                time.sleep(0.1)
        return False
    except Exception:
        return False


def _ensure_piper_daemon() -> bool:
    """Ensure the piper daemon is running; (re)start if needed."""
    global _piper_fifo_writer
    if _piper_daemon_running() and _piper_fifo_writer is not None:
        return True
    # Close stale fd if any
    if _piper_fifo_writer is not None:
        try:
            os.close(_piper_fifo_writer)
        except OSError:
            pass
        _piper_fifo_writer = None
    return _start_piper_daemon()


def play_speech(text: str, wait: bool = False) -> None:
    """Pronounce text through robot hardware speakers with cute neural voice.

    Uses a persistent piper daemon to keep the TTS model warm so subsequent
    calls have near-instant startup (no 1.5 s model-load delay).
    Falls back to navpro-speak → direct piper → espeak-ng if daemon setup
    fails.
    """
    if not text or not text.strip():
        return
    text = text.strip()

    global _piper_fifo_writer  # needed so the OSError handler can reset the fd to None

    # ---- Try persistent piper daemon first --------------------------------
    if shutil.which('piper') and os.path.isfile(_PIPER_MODEL) and shutil.which('paplay'):
        with _piper_lock:
            if _ensure_piper_daemon() and _piper_fifo_writer is not None:
                payload = (_json.dumps({'text': text}) + '\n').encode('utf-8')
                try:
                    os.write(_piper_fifo_writer, payload)
                    if wait:
                        # Approximate wait: allow ~100 ms startup + ~50 ms/word
                        word_count = len(text.split())
                        time.sleep(0.15 + word_count * 0.35)
                    return
                except OSError:
                    # Broken pipe — daemon died; will restart next call
                    try:
                        os.close(_piper_fifo_writer)
                    except OSError:
                        pass
                    _piper_fifo_writer = None

    # ---- Fallback: navpro-speak script (legacy) ---------------------------
    env = get_pulse_env()
    cmd = None
    if shutil.which('navpro-speak'):
        cmd = ['navpro-speak', text]
    elif shutil.which('piper') and os.path.isfile(_PIPER_MODEL):
        cmd = ['bash', '-c',
               f'echo "{text}" | piper --model {_PIPER_MODEL} '
               f'--length_scale 1.20 --output-raw 2>/dev/null | '
               f'paplay --raw --rate {_PIPER_RATE} --channels 1 --format s16le 2>/dev/null || true']
    elif shutil.which('espeak-ng'):
        cmd = ['espeak-ng', '-v', 'en+f4', '-p', '88', '-s', '130', text]

    if not cmd:
        return

    try:
        if wait:
            subprocess.run(cmd, env=env, check=False, timeout=30.0)
        else:
            subprocess.Popen(cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception:
        pass




def play_sound(sound_name: str, speech_text: Optional[str] = None, wait: bool = False) -> None:
    """Play event tone chime and optionally speak accompanying announcement.

    Non-blocking by default.
    """
    def _worker():
        try:
            sound_dir = get_sound_dir()
            wav_path = sound_dir / f'{sound_name}.wav'
            if not wav_path.is_file() and sound_name in TONE_DEFINITIONS:
                generate_tone_wav(wav_path, TONE_DEFINITIONS[sound_name])

            env = get_pulse_env()
            if wav_path.is_file() and shutil.which('paplay'):
                subprocess.run(['paplay', str(wav_path)], env=env, check=False, timeout=5.0)
            elif wav_path.is_file() and shutil.which('aplay'):
                subprocess.run(['aplay', '-q', str(wav_path)], env=env, check=False, timeout=5.0)

            if speech_text and speech_text.strip():
                time.sleep(0.08)
                play_speech(speech_text, wait=True)
        except Exception:
            pass

    if wait:
        _worker()
    else:
        t = threading.Thread(target=_worker, daemon=True)
        t.start()


async def play_sound_async(sound_name: str, speech_text: Optional[str] = None) -> None:
    """Async wrapper for play_sound."""
    loop = asyncio.get_event_loop()
    await loop.run_in_executor(None, play_sound, sound_name, speech_text, False)


_LAST_EVENT_SOUND_TIMES: dict[str, float] = {}

def handle_event_audio(event_name: str, data: Optional[dict] = None) -> None:
    """Dispatches clean notification tones/chimes for major robot events (no spoken voice)."""
    now = time.time()
    last_time = _LAST_EVENT_SOUND_TIMES.get(event_name, 0.0)
    # Debounce repeated identical events within 1.5s
    if now - last_time < 1.5:
        return
    _LAST_EVENT_SOUND_TIMES[event_name] = now

    payload = data or {}
    op = payload.get('operation')

    if event_name == 'dock.started':
        play_sound('undock_start' if op == 'undock' else 'dock_start')
    elif event_name == 'dock.completed':
        play_sound('undock_success' if op == 'undock' else 'dock_success')
    elif event_name == 'dock.failed':
        play_sound('dock_fail')
    elif event_name == 'navigation.started':
        play_sound('nav_start')
    elif event_name == 'navigation.completed':
        play_sound('nav_reach')
    elif event_name in ('navigation.cancelled', 'navigation.failed'):
        play_sound('nav_cancel')
    elif event_name == 'mission.started':
        play_sound('mission_start')
    elif event_name == 'mission.completed':
        play_sound('mission_complete')
    elif event_name == 'mission.paused':
        play_sound('mission_pause')
    elif event_name in ('mission.canceled', 'mission.failed'):
        play_sound('mission_fail')
    elif event_name == 'mission.ui_interaction':
        play_sound('mission_action')
    elif event_name in ('battery.low', 'mission.battery_low_pause'):
        play_sound('low_battery')
    elif event_name == 'motion.estop':
        play_sound('estop')

