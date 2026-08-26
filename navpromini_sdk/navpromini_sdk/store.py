#!/usr/bin/env python3
"""Robot-side persistence for SDK-owned data (waypoints, missions, schedules,
current map).

SQLite instead of a single JSON file: the JSON store rewrote its *entire*
file on every single put/delete, however small — fine at a few dozen
waypoints, wasteful as missions/waypoints grow or mission-run history gets
added later. SQLite writes touch only the changed row, and WAL mode means a
read (GET /api/v1/waypoints) never blocks behind a concurrent write.

Each row stores its record as a JSON blob rather than one column per field:
what's actually queried is "waypoints for this map" and "mission by id" —
real primary-key lookups — while the record shape itself (a waypoint's
fields, a mission's nested step list) stays exactly as flexible as it was as
plain JSON, with no schema migration needed the next time a field is added.

Durability: synchronous=FULL, matching the old JSON store's own
fsync-before-replace guarantee — robots lose power without warning, same
reasoning as before, just enforced by SQLite's transaction commit instead of
a manual tempfile dance.

On first run, if the pre-SQLite `<path>.json` file this SDK used to write
exists and this database is otherwise empty, its contents are imported once.
The JSON file is left in place afterward — this is a copy-in, not a cutover,
so it stays recoverable if the import ever needs re-checking.
"""

from __future__ import annotations

import json
import os
import sqlite3
import threading
from pathlib import Path
from typing import Optional

DEFAULT_PATH = Path(os.environ.get(
    'NAVPRO_SDK_DATA', os.path.expanduser('~/.navpromini_sdk.db')))

_SCHEMA = """
CREATE TABLE IF NOT EXISTS kv (
    key   TEXT PRIMARY KEY,
    value TEXT NOT NULL
);
CREATE TABLE IF NOT EXISTS waypoints (
    map   TEXT NOT NULL,
    name  TEXT NOT NULL,
    data  TEXT NOT NULL,
    PRIMARY KEY (map, name)
);
CREATE TABLE IF NOT EXISTS missions (
    id    TEXT PRIMARY KEY,
    data  TEXT NOT NULL
);
CREATE TABLE IF NOT EXISTS schedules (
    id    TEXT PRIMARY KEY,
    data  TEXT NOT NULL
);
"""


class Store:
    def __init__(self, path: Path = DEFAULT_PATH) -> None:
        self.path = Path(path)
        self._lock = threading.Lock()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        # navpromini_sdk runs its own dedicated event loop thread + ROS
        # executor threads (see server.py); check_same_thread=False plus
        # self._lock below covers a Store call ever landing on a thread other
        # than the one that opened the connection, same defensive stance the
        # old JSON Store took with its own threading.Lock.
        self._conn = sqlite3.connect(str(self.path), check_same_thread=False)
        self._conn.execute('PRAGMA journal_mode=WAL')
        self._conn.execute('PRAGMA synchronous=FULL')
        self._conn.executescript(_SCHEMA)
        self._conn.commit()
        self._migrate_from_json()

    def _migrate_from_json(self) -> None:
        """One-time import of the pre-SQLite JSON store, if present and this
        database is otherwise empty. Never overwrites existing rows, never
        touches the JSON file itself."""
        legacy = self.path.with_suffix('.json')
        if not legacy.is_file():
            return
        counts = (
            self._conn.execute('SELECT COUNT(*) FROM kv').fetchone()[0]
            + self._conn.execute('SELECT COUNT(*) FROM waypoints').fetchone()[0]
            + self._conn.execute('SELECT COUNT(*) FROM missions').fetchone()[0]
        )
        if counts > 0:
            return
        try:
            with legacy.open() as f:
                data = json.load(f)
        except (OSError, ValueError):
            return
        if not isinstance(data, dict):
            return

        with self._lock, self._conn:
            current_map = data.get('current_map')
            if current_map:
                self._conn.execute(
                    'INSERT OR REPLACE INTO kv (key, value) VALUES (?, ?)',
                    ('current_map', str(current_map)))
            for map_name, wps in (data.get('waypoints') or {}).items():
                for wp_name, wp in (wps or {}).items():
                    self._conn.execute(
                        'INSERT OR REPLACE INTO waypoints (map, name, data) VALUES (?, ?, ?)',
                        (map_name, wp_name, json.dumps(wp)))
            for mission_id, mission in (data.get('missions') or {}).items():
                self._conn.execute(
                    'INSERT OR REPLACE INTO missions (id, data) VALUES (?, ?)',
                    (mission_id, json.dumps(mission)))

    # -- current map -----------------------------------------------------------

    def current_map(self) -> str:
        with self._lock:
            row = self._conn.execute(
                'SELECT value FROM kv WHERE key = ?', ('current_map',)).fetchone()
        return row[0] if row else 'default'

    def set_current_map(self, name: str) -> None:
        with self._lock, self._conn:
            self._conn.execute(
                'INSERT OR REPLACE INTO kv (key, value) VALUES (?, ?)',
                ('current_map', name))

    # -- waypoints ---------------------------------------------------------

    def list_waypoints(self, map_name: Optional[str] = None) -> list[dict]:
        m = map_name or self.current_map()
        with self._lock:
            rows = self._conn.execute(
                'SELECT data FROM waypoints WHERE map = ? ORDER BY name', (m,)).fetchall()
        return [json.loads(r[0]) for r in rows]

    def get_waypoint(self, name: str, map_name: Optional[str] = None) -> Optional[dict]:
        m = map_name or self.current_map()
        with self._lock:
            row = self._conn.execute(
                'SELECT data FROM waypoints WHERE map = ? AND name = ?', (m, name)).fetchone()
        return json.loads(row[0]) if row else None

    def put_waypoint(self, wp: dict, map_name: Optional[str] = None) -> dict:
        m = map_name or self.current_map()
        with self._lock, self._conn:
            self._conn.execute(
                'INSERT OR REPLACE INTO waypoints (map, name, data) VALUES (?, ?, ?)',
                (m, wp['name'], json.dumps(wp)))
        return wp

    def delete_waypoint(self, name: str, map_name: Optional[str] = None) -> bool:
        m = map_name or self.current_map()
        with self._lock, self._conn:
            cur = self._conn.execute(
                'DELETE FROM waypoints WHERE map = ? AND name = ?', (m, name))
        return cur.rowcount > 0

    # -- missions ------------------------------------------------------------

    def list_missions(self) -> list[dict]:
        with self._lock:
            rows = self._conn.execute('SELECT data FROM missions ORDER BY id').fetchall()
        return [json.loads(r[0]) for r in rows]

    def get_mission(self, mission_id: str) -> Optional[dict]:
        with self._lock:
            row = self._conn.execute(
                'SELECT data FROM missions WHERE id = ?', (mission_id,)).fetchone()
        return json.loads(row[0]) if row else None

    def put_mission(self, mission: dict) -> dict:
        with self._lock, self._conn:
            self._conn.execute(
                'INSERT OR REPLACE INTO missions (id, data) VALUES (?, ?)',
                (mission['id'], json.dumps(mission)))
        return mission

    def delete_mission(self, mission_id: str) -> bool:
        with self._lock, self._conn:
            cur = self._conn.execute('DELETE FROM missions WHERE id = ?', (mission_id,))
        return cur.rowcount > 0

    # -- schedules -------------------------------------------------------------

    def list_schedules(self) -> list[dict]:
        with self._lock:
            rows = self._conn.execute('SELECT data FROM schedules ORDER BY id').fetchall()
        return [json.loads(r[0]) for r in rows]

    def get_schedule(self, schedule_id: str) -> Optional[dict]:
        with self._lock:
            row = self._conn.execute(
                'SELECT data FROM schedules WHERE id = ?', (schedule_id,)).fetchone()
        return json.loads(row[0]) if row else None

    def put_schedule(self, schedule: dict) -> dict:
        with self._lock, self._conn:
            self._conn.execute(
                'INSERT OR REPLACE INTO schedules (id, data) VALUES (?, ?)',
                (schedule['id'], json.dumps(schedule)))
        return schedule

    def delete_schedule(self, schedule_id: str) -> bool:
        with self._lock, self._conn:
            cur = self._conn.execute('DELETE FROM schedules WHERE id = ?', (schedule_id,))
        return cur.rowcount > 0
