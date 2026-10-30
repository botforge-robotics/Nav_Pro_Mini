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

def _run_play_wav(wav_path: str | Path, timeout: float = 10.0) -> None:
    """Execute audio playback via paplay or aplay, adapting for root daemon permissions."""
    path_str = str(wav_path)
    if not os.path.isfile(path_str):
        return

    # If run by root, ensure file is world-readable so navpromini user can access it
    if os.geteuid() == 0:
        try:
            os.chmod(path_str, 0o666)
        except OSError:
            pass

    env = get_pulse_env()
    cmd: list[str] = []
    if shutil.which('paplay'):
        if os.geteuid() == 0:
            cmd = ['sudo', '-u', 'navpromini', 'env', 'XDG_RUNTIME_DIR=/run/user/1000',
                   'PULSE_SERVER=unix:/run/user/1000/pulse/native', 'paplay', path_str]
        else:
            cmd = ['paplay', path_str]
    elif shutil.which('aplay'):
        cmd = ['aplay', '-q', path_str]

    if cmd:
        try:
            subprocess.run(cmd, env=env, check=False, timeout=timeout)
        except Exception:
            pass


_PIPER_MODEL = '/opt/navpro/piper/voices/en_US-hfc_female-medium.onnx'
_PIPER_RATE = 23800  # sample rate produced by this model

_piper_lock = threading.Lock()
_piper_proc: Optional[subprocess.Popen] = None


def _get_piper_proc() -> Optional[subprocess.Popen]:
    """Get or lazily spawn persistent piper process with --json-input."""
    global _piper_proc
    if not shutil.which('piper') or not os.path.isfile(_PIPER_MODEL):
        return None
    if _piper_proc is not None and _piper_proc.poll() is None:
        return _piper_proc

    if _piper_proc is not None:
        try:
            _piper_proc.kill()
        except Exception:
            pass
        _piper_proc = None

    try:
        cmd = [
            'piper',
            '--model', _PIPER_MODEL,
            '--length_scale', '1.20',
            '--json-input'
        ]
        _piper_proc = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            bufsize=1
        )
        return _piper_proc
    except Exception:
        _piper_proc = None
        return None


_audio_busy_until = 0.0
_audio_lock = threading.Lock()
_current_speech_cancel = threading.Event()
_current_speech_proc: Optional[subprocess.Popen] = None


def stop_active_speech() -> None:
    """Instantly interrupt and cancel any currently playing or queued speech synthesis."""
    global _audio_busy_until, _current_speech_proc
    _current_speech_cancel.set()
    with _audio_lock:
        _audio_busy_until = 0.0
        if _current_speech_proc is not None:
            try:
                _current_speech_proc.terminate()
            except Exception:
                pass
            _current_speech_proc = None
    try:
        subprocess.run(['pkill', '-f', 'paplay.*--raw'], check=False, timeout=1.0)
    except Exception:
        pass


def _wait_audio_clear(max_wait: float = 0.3) -> None:
    """Ensure any previous short chime finishes before starting new audio."""
    global _audio_busy_until
    now = time.time()
    wait_time = _audio_busy_until - now
    if wait_time > 0:
        time.sleep(min(wait_time, max_wait))


def _split_into_utterances(text: str) -> list[str]:
    """Split text into short sentences/phrases for instant real-time pipeline synthesis."""
    import re
    chunks = [s.strip() for s in re.split(r'([.!?;:\n]+)', text) if s.strip()]
    if not chunks:
        return [text.strip()] if text.strip() else []
    sentences: list[str] = []
    i = 0
    while i < len(chunks):
        s = chunks[i]
        if i + 1 < len(chunks) and re.match(r'^[.!?;:\n]+$', chunks[i + 1]):
            s += chunks[i + 1]
            i += 1
        if len(s) > 80:
            sub = [p.strip() for p in re.split(r'(,)', s) if p.strip()]
            j = 0
            while j < len(sub):
                chunk_j = sub[j]
                if j + 1 < len(sub) and sub[j + 1] == ',':
                    chunk_j += ','
                    j += 1
                sentences.append(chunk_j)
                j += 1
        else:
            sentences.append(s)
        i += 1
    return [s for s in sentences if s]


def _detect_neural_voice(text: str) -> str:
    """Select the best Microsoft Edge neural voice based on language and script."""
    # Devanagari script (Hindi, Marathi)
    if any('\u0900' <= c <= '\u097F' for c in text):
        return 'hi-IN-SwaraNeural'
    # Telugu script
    if any('\u0C00' <= c <= '\u0C7F' for c in text):
        return 'te-IN-ShrutiNeural'
    # Tamil script
    if any('\u0B80' <= c <= '\u0BFF' for c in text):
        return 'ta-IN-PallaviNeural'
    # Default to natural, clear US English female assistant voice
    return 'en-US-AriaNeural'


def _stream_edge_tts(sentence: str, cancel_event: threading.Event) -> bool:
    """Synthesize sentence using Microsoft Edge Neural TTS in-memory and stream directly to paplay."""
    global _current_speech_proc
    if cancel_event.is_set():
        return True

    try:
        import asyncio
        import edge_tts

        voice = _detect_neural_voice(sentence)

        async def _fetch_audio() -> bytes:
            comm = edge_tts.Communicate(sentence, voice)
            chunks: list[bytes] = []
            async for c in comm.stream():
                if cancel_event.is_set():
                    break
                if c["type"] == "audio":
                    chunks.append(c["data"])
            return b"".join(chunks)

        loop = asyncio.new_event_loop()
        try:
            mp3_bytes = loop.run_until_complete(_fetch_audio())
        finally:
            loop.close()

        if cancel_event.is_set() or not mp3_bytes:
            return bool(mp3_bytes)

        env = get_pulse_env()
        p_ffmpeg = subprocess.Popen(
            ['ffmpeg', '-v', '0', '-i', '-', '-f', 'wav', '-'],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL
        )
        p_pa = subprocess.Popen(
            ['paplay'],
            stdin=p_ffmpeg.stdout,
            stderr=subprocess.DEVNULL,
            env=env
        )
        p_ffmpeg.stdout.close()
        with _audio_lock:
            _current_speech_proc = p_pa

        try:
            p_ffmpeg.stdin.write(mp3_bytes)
            p_ffmpeg.stdin.close()
        except (BrokenPipeError, OSError):
            pass

        while p_pa.poll() is None:
            if cancel_event.is_set():
                try: p_pa.terminate()
                except Exception: pass
                try: p_ffmpeg.terminate()
                except Exception: pass
                break
            time.sleep(0.04)

        try: p_ffmpeg.wait(timeout=1.0)
        except Exception: pass

        return True
    except Exception:
        return False


def _stream_sentence_pcm(sentence: str, cancel_event: threading.Event) -> None:
    """Stream speech in memory: primary Edge Neural Voice, with offline fallback to Piper TTS."""
    global _current_speech_proc
    if cancel_event.is_set():
        return

    # 1. Primary: Microsoft Edge Neural Voice (Studio Quality, Human Intonation, Multilingual)
    try:
        if _stream_edge_tts(sentence, cancel_event):
            return
    except Exception:
        pass

    if cancel_event.is_set():
        return

    # 2. Offline Fallback: Local Piper TTS
    env = get_pulse_env()
    piper_cmd = [
        'piper',
        '--model', _PIPER_MODEL,
        '--length_scale', '1.18',
        '--output-raw'
    ]
    pa_cmd = [
        'paplay',
        '--raw',
        '--rate', str(_PIPER_RATE),
        '--channels', '1',
        '--format', 's16le'
    ]

    p_piper: Optional[subprocess.Popen] = None
    p_pa: Optional[subprocess.Popen] = None
    try:
        p_piper = subprocess.Popen(
            piper_cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL
        )
        p_pa = subprocess.Popen(
            pa_cmd,
            stdin=p_piper.stdout,
            stderr=subprocess.DEVNULL,
            env=env
        )
        p_piper.stdout.close()  # Allow p_pa to receive EOF when p_piper terminates
        with _audio_lock:
            _current_speech_proc = p_pa

        p_piper.stdin.write(sentence.encode('utf-8') + b'\n')
        p_piper.stdin.close()

        while p_pa.poll() is None:
            if cancel_event.is_set():
                try:
                    p_pa.terminate()
                except Exception:
                    pass
                try:
                    p_piper.terminate()
                except Exception:
                    pass
                break
            time.sleep(0.04)

        try:
            p_piper.wait(timeout=2.0)
        except Exception:
            try:
                p_piper.kill()
            except Exception:
                pass
    except Exception:
        if shutil.which('espeak-ng') and not cancel_event.is_set():
            cmd = ['espeak-ng', '-v', 'en+f4', '-p', '88', '-s', '135', sentence]
            subprocess.run(cmd, env=env, check=False, timeout=8.0)
    finally:
        with _audio_lock:
            if _current_speech_proc is not None:
                _current_speech_proc = None


def _synthesize_and_stream(sentences: list[str], cancel_event: threading.Event) -> None:
    """Stream speech utterances in real time directly to the sound card in memory."""
    for sentence in sentences:
        if cancel_event.is_set():
            break
        _stream_sentence_pcm(sentence, cancel_event)


def play_speech(text: str, wait: bool = False) -> None:
    """Pronounce text in real-time in memory without disk writing/deleting."""
    if not text or not text.strip():
        return
    text = text.strip()

    global _current_speech_cancel
    _current_speech_cancel.set()
    _current_speech_cancel = threading.Event()
    this_cancel = _current_speech_cancel

    _wait_audio_clear(max_wait=0.25)
    dur = len(text) / 13.0 + 0.4
    global _audio_busy_until
    with _audio_lock:
        _audio_busy_until = time.time() + dur

    sentences = _split_into_utterances(text)
    if not sentences:
        return

    if wait:
        _synthesize_and_stream(sentences, this_cancel)
    else:
        threading.Thread(target=_synthesize_and_stream, args=(sentences, this_cancel), daemon=True).start()




def play_sound(sound_name: str, speech_text: Optional[str] = None, wait: bool = False) -> None:
    """Play event tone chime and optionally speak accompanying announcement.

    Non-blocking by default.
    """
    dur = sum(s[1] for s in TONE_DEFINITIONS.get(sound_name, [])) or 0.6
    global _audio_busy_until
    with _audio_lock:
        _audio_busy_until = max(_audio_busy_until, time.time() + dur + 0.12)

    def _worker():
        try:
            sound_dir = get_sound_dir()
            wav_path = sound_dir / f'{sound_name}.wav'
            if not wav_path.is_file() and sound_name in TONE_DEFINITIONS:
                generate_tone_wav(wav_path, TONE_DEFINITIONS[sound_name])

            _run_play_wav(wav_path, timeout=5.0)

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

