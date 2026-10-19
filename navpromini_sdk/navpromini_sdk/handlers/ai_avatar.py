#!/usr/bin/env python3
"""AI Avatar & Gemini Flash Live Configuration Handler for NavPro Mini.

Persists and exposes the AI assistant configuration (Gemini API key,
active scenario, voice persona, waypoints knowledge base, and safety parameters)
across ~/.navpromini_ai_avatar.json and /root/.navpromini_ai_avatar.json.
"""

from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import Any, Dict

import tornado.web

from .base import BaseHandler

logger = logging.getLogger(__name__)

PRIMARY_PATH = Path('/home/navpromini/.navpromini_ai_avatar.json')
ROOT_PATH = Path('/root/.navpromini_ai_avatar.json')
LOCAL_PATH = Path.home() / '.navpromini_ai_avatar.json'

DEFAULT_CONFIG: Dict[str, Any] = {
    "enabled": True,
    "apiKey": "",
    "model": "gemini-2.0-flash-exp",
    "voice": "Aoede",
    "scenario": "real_estate",  # "real_estate", "shopping_mall", "custom"
    "avatarStyle": "robot_2d",
    "silenceTimeoutSec": 8,
    "speedLimitMps": 0.35,
    "systemInstruction": (
        "You are the AI Voice Assistant, a helpful and welcoming conversational assistant. "
        "You speak warmly, concisely, and naturally as if having an in-person conversation. "
        "When visitors ask to see a place or go somewhere, use your navigate_to_place tool to guide them. "
        "When they ask for details, floor plans, or photos, use show_media. "
        "Always be welcoming, polite, and helpful."
    ),
    "waypointsKnowledge": {
        "kitchen": "14x11 ft modular kitchen with Italian quartz countertops, soft-close Blum hardware, and integrated induction hob.",
        "master_bedroom": "16x14 ft master bedroom with solid hardwood flooring, attached walk-in wardrobe, and south-facing view.",
        "living_room": "Spacious 20x15 ft living hall with double-glazed French windows, 10-ft ceiling, and smart ambient lighting.",
        "balcony": "East-facing sunrise balcony with anti-skid vitrified tiles and glass balustrades.",
        "foyer": "Main entrance foyer with biometric smart lock and shoe console."
    },
    "mallDirectory": {
        "apple_store": "Ground Floor central atrium, specializing in iPhones, MacBooks, and Apple repairs.",
        "starbucks": "Ground Floor East entrance, coffee, pastries, and seating.",
        "food_court": "Level 3 with multi-cuisine dining options and family seating.",
        "restrooms": "Ground Floor Corridor B, next to elevator bank."
    }
}


def load_ai_avatar_config() -> Dict[str, Any]:
    """Load configuration from the first available location."""
    for path in (PRIMARY_PATH, ROOT_PATH, LOCAL_PATH):
        try:
            if path.is_file():
                with open(path, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                    if isinstance(data, dict):
                        # Merge with defaults so new keys are present
                        merged = dict(DEFAULT_CONFIG)
                        merged.update(data)
                        return merged
        except Exception as exc:
            logger.warning(f"Failed to read {path}: {exc}")
    return dict(DEFAULT_CONFIG)


def save_ai_avatar_config(config: Dict[str, Any]) -> bool:
    """Save configuration to all writable locations."""
    success = False
    for path in (PRIMARY_PATH, ROOT_PATH, LOCAL_PATH):
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            with open(path, 'w', encoding='utf-8') as f:
                json.dump(config, f, indent=2)
            success = True
        except Exception as exc:
            logger.debug(f"Could not write AI avatar config to {path}: {exc}")
    return success


class AiAvatarConfigHandler(BaseHandler):
    """GET/PUT handler for AI Avatar & Gemini Flash Live assistant settings."""

    async def get(self):
        config = load_ai_avatar_config()
        # Return full config (robot UI and mission planner require apiKey to connect)
        self.finish(config)

    async def put(self):
        try:
            payload = json.loads(self.request.body.decode('utf-8'))
            if not isinstance(payload, dict):
                self.set_status(400)
                self.finish({"error": {"code": "invalid_body", "message": "Expected JSON object"}})
                return

            current = load_ai_avatar_config()
            current.update(payload)
            save_ai_avatar_config(current)

            self.finish({"status": "ok", "config": current})
        except Exception as exc:
            logger.exception("Failed to update AI avatar config")
            self.set_status(500)
            self.finish({"error": {"code": "save_failed", "message": str(exc)}})
