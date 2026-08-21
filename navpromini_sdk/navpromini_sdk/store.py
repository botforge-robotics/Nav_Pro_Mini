#!/usr/bin/env python3
"""Robot-side persistence for SDK-owned data (waypoints).

Waypoints live on the ROBOT, not in a client. The Flutter app currently keeps
bookmarks in its own storage, which means a second client — a web backend, a
fleet manager, a script — cannot see them, and reinstalling the app loses them.
Anything the robot can navigate to should be knowable by asking the robot.

Stored per map: a waypoint is a pose in a specific map's frame, so the same
name in two maps is two different places, and carrying them across maps would
send the robot somewhere arbitrary.
"""

from __future__ import annotations

import json
import os
import tempfile
import threading
from pathlib import Path
from typing import Optional

DEFAULT_PATH = Path(os.environ.get(
    'NAVPRO_SDK_DATA', os.path.expanduser('~/.navpromini_sdk.json')))


class Store:
    def __init__(self, path: Path = DEFAULT_PATH) -> None:
        self.path = Path(path)
        self._lock = threading.Lock()
        self._data = self._load()

    def _load(self) -> dict:
        try:
            with self.path.open() as f:
                data = json.load(f)
            if isinstance(data, dict):
                data.setdefault('waypoints', {})
                return data
        except (OSError, ValueError):
            pass
        return {'waypoints': {}}

    def _save(self) -> None:
        """Atomic write: a truncated file on power loss would lose every
        waypoint, and robots lose power without warning."""
        self.path.parent.mkdir(parents=True, exist_ok=True)
        fd, tmp = tempfile.mkstemp(dir=str(self.path.parent), suffix='.tmp')
        try:
            with os.fdopen(fd, 'w') as f:
                json.dump(self._data, f, indent=2)
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp, self.path)
        except OSError:
            try:
                os.unlink(tmp)
            except OSError:
                pass
            raise

    # -- waypoints ---------------------------------------------------------

    def current_map(self) -> str:
        return self._data.get('current_map') or 'default'

    def set_current_map(self, name: str) -> None:
        with self._lock:
            self._data['current_map'] = name
            self._save()

    def list_waypoints(self, map_name: Optional[str] = None) -> list[dict]:
        m = map_name or self.current_map()
        with self._lock:
            return list(self._data.get('waypoints', {}).get(m, {}).values())

    def get_waypoint(self, name: str, map_name: Optional[str] = None) -> Optional[dict]:
        m = map_name or self.current_map()
        with self._lock:
            return self._data.get('waypoints', {}).get(m, {}).get(name)

    def put_waypoint(self, wp: dict, map_name: Optional[str] = None) -> dict:
        m = map_name or self.current_map()
        with self._lock:
            self._data.setdefault('waypoints', {}).setdefault(m, {})[wp['name']] = wp
            self._save()
        return wp

    def delete_waypoint(self, name: str, map_name: Optional[str] = None) -> bool:
        m = map_name or self.current_map()
        with self._lock:
            removed = self._data.get('waypoints', {}).get(m, {}).pop(name, None)
            if removed is not None:
                self._save()
        return removed is not None
