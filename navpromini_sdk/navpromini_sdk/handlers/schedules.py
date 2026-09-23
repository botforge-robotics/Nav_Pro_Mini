#!/usr/bin/env python3
"""Mission Scheduler — alarm-style: pick a mission, pick a time, pick how it
repeats (once / daily / specific weekdays), and it fires that mission on the
robot itself, independent of any app being open. Reuses the exact same
`_run_mission` the interactive Missions API already uses (missions.py) —
a scheduled mission behaves identically to one started by hand, not a
second, parallel execution path.

Checked, not woken: `check_schedules` is polled on a plain timer (see
server.py), same as mode.reconcile_mode()/watch_localization() — there is no
OS-level wake timer here. A schedule whose minute passes while the SDK
happens to be down is simply missed, like a phone alarm silenced by a dead
battery — it does not fire late when the process comes back up, and does not
queue. Same reasoning for a schedule whose minute arrives while another
mission is already running (RUNNER, missions.py's own singleton): skipped,
not queued, not force-cancelling the mission in progress. Both are
deliberate, not an oversight — see this module's own doc on `check_schedules`
for the exact rule.
"""

from __future__ import annotations

import time
from datetime import datetime
from typing import Any

from .base import ApiError, BaseHandler
from .missions import RUNNER, _run_mission

VALID_REPEATS = ('once', 'daily', 'weekly', 'interval')


def _validate_schedule(data: dict, store) -> dict:
    mission_id = str(data.get('mission_id') or '').strip()
    if not mission_id:
        raise ApiError(400, 'invalid_field', 'mission_id must not be empty')
    if store.get_mission(mission_id) is None:
        raise ApiError(404, 'mission_not_found', f'No mission named {mission_id!r}')

    repeat = data.get('repeat')
    if repeat not in VALID_REPEATS:
        raise ApiError(400, 'invalid_field',
                       f'repeat must be one of: {", ".join(VALID_REPEATS)}')

    hour = data.get('hour')
    minute = data.get('minute')
    interval_minutes = None
    date = None
    weekdays: list[int] = []

    if repeat == 'interval':
        raw_interval = data.get('interval_minutes')
        if raw_interval is None:
            raw_interval = minute if (isinstance(minute, int) and minute > 0) else 30
        try:
            interval_minutes = int(raw_interval)
            if interval_minutes <= 0:
                raise ValueError
        except (TypeError, ValueError):
            raise ApiError(400, 'invalid_field', 'interval_minutes must be a positive integer')
        hour = int(hour) if isinstance(hour, int) and 0 <= hour <= 23 else 0
        minute = int(minute) if isinstance(minute, int) and 0 <= minute <= 59 else 0
    else:
        if not isinstance(hour, int) or not (0 <= hour <= 23):
            raise ApiError(400, 'invalid_field', 'hour must be an integer 0-23')
        if not isinstance(minute, int) or not (0 <= minute <= 59):
            raise ApiError(400, 'invalid_field', 'minute must be an integer 0-59')

        if repeat == 'once':
            date = str(data.get('date') or '')
            try:
                datetime.strptime(date, '%Y-%m-%d')
            except ValueError:
                raise ApiError(400, 'invalid_field', 'date must be YYYY-MM-DD for a one-time schedule')
        elif repeat == 'weekly':
            raw = data.get('weekdays')
            if not isinstance(raw, list) or not raw:
                raise ApiError(400, 'invalid_field',
                               'weekdays must be a non-empty list (0=Monday .. 6=Sunday)')
            try:
                weekdays = sorted({int(d) for d in raw})
            except (TypeError, ValueError):
                raise ApiError(400, 'invalid_field', 'weekdays must be integers 0-6')
            if any(d < 0 or d > 6 for d in weekdays):
                raise ApiError(400, 'invalid_field', 'weekdays must be integers 0-6')

    res = {
        'mission_id': mission_id,
        'name': str(data.get('name') or '').strip(),
        'hour': hour,
        'minute': minute,
        'repeat': repeat,
        'date': date,
        'weekdays': weekdays,
        'enabled': bool(data.get('enabled', True)),
    }
    if interval_minutes is not None:
        res['interval_minutes'] = interval_minutes
    return res


class SchedulesHandler(BaseHandler):
    def get(self) -> None:
        map_name = self.get_argument('map', None)
        self.send({'schedules': self.opts['store'].list_schedules(map_name)})

    def post(self) -> None:
        """Create or replace a schedule — same create-or-replace-by-id shape
        as POST /missions."""
        data = self.body(('id', 'mission_id', 'repeat'))
        schedule_id = str(data['id']).strip()
        if not schedule_id:
            raise ApiError(400, 'invalid_field', 'id must not be empty')
        fields = _validate_schedule(data, self.opts['store'])
        schedule = {'id': schedule_id, **fields}
        self.opts['store'].put_schedule(schedule)
        _last_interval_fired.pop(schedule_id, None)
        _last_fired.pop(schedule_id, None)
        self.send({'schedule': schedule}, status=201)


class ScheduleHandler(BaseHandler):
    def get(self, schedule_id: str) -> None:
        schedule = self.opts['store'].get_schedule(schedule_id)
        if schedule is None:
            raise ApiError(404, 'schedule_not_found', f'No schedule {schedule_id!r}')
        self.send({'schedule': schedule})

    def delete(self, schedule_id: str) -> None:
        _last_interval_fired.pop(schedule_id, None)
        _last_fired.pop(schedule_id, None)
        if not self.opts['store'].delete_schedule(schedule_id):
            raise ApiError(404, 'schedule_not_found', f'No schedule {schedule_id!r}')
        self.send({'deleted': True, 'id': schedule_id})


# schedule_id -> 'YYYY-MM-DDTHH:MM' it last fired at, so a schedule whose
# matching minute is checked more than once (the poll interval doesn't line
# up exactly with minute boundaries) fires exactly once, not once per poll
# tick for the whole minute it's due.
_last_fired: dict[str, str] = {}
_last_interval_fired: dict[str, float] = {}


def check_schedules(bridge, opts: dict[str, Any]) -> None:
    """Polled on a timer (see server.py) — fires any enabled schedule whose
    (hour, minute) matches right now, on the right day for its repeat mode,
    or whose interval has elapsed.

    Both "conflict" cases below are a deliberate skip, not a queue — see
    this module's own docstring:
      - RUNNER already running/paused (missions.py's own singleton): the
        scheduled mission is simply not started this time.
      - The SDK process wasn't running at all when the minute passed: this
        function was never called, so nothing records or catches up on it.
    """
    store = opts['store']
    now = datetime.now()
    now_epoch = time.time()
    minute_key = now.strftime('%Y-%m-%dT%H:%M')

    for schedule in store.list_schedules():
        if not schedule.get('enabled'):
            continue

        repeat = schedule.get('repeat')
        sched_id = schedule.get('id', '')

        if repeat == 'interval':
            interval_min = schedule.get('interval_minutes') or 30
            try:
                interval_min = int(interval_min)
            except (TypeError, ValueError):
                interval_min = 30
            if interval_min <= 0:
                interval_min = 30
            interval_sec = interval_min * 60

            last_epoch = _last_interval_fired.get(sched_id)
            if last_epoch is None:
                # Initialize timing baseline when schedule is first encountered
                _last_interval_fired[sched_id] = now_epoch
                continue
            if (now_epoch - last_epoch) < interval_sec:
                continue

            _last_interval_fired[sched_id] = now_epoch
        else:
            if schedule.get('hour') != now.hour or schedule.get('minute') != now.minute:
                continue
            if _last_fired.get(sched_id) == minute_key:
                continue  # already handled this exact minute

            if repeat == 'once' and schedule.get('date') != now.strftime('%Y-%m-%d'):
                continue
            if repeat == 'weekly' and now.weekday() not in (schedule.get('weekdays') or []):
                continue

            _last_fired[sched_id] = minute_key

        if RUNNER.state in ('running', 'paused'):
            bridge.emit_event('schedule.skipped', {
                'schedule_id': schedule['id'], 'reason': 'mission_active',
            })
            continue

        mission = store.get_mission(schedule['mission_id'])
        if mission is None:
            # The mission it points to was deleted since — disable rather
            # than fail silently forever or error on every future tick.
            schedule['enabled'] = False
            store.put_schedule(schedule)
            bridge.emit_event('schedule.skipped', {
                'schedule_id': schedule['id'], 'reason': 'mission_not_found',
            })
            continue

        current_map = store.current_map()
        mission_map = mission.get('map')
        if mission_map and mission_map != current_map:
            bridge.emit_event('schedule.skipped', {
                'schedule_id': schedule['id'],
                'reason': 'map_mismatch',
                'required_map': mission_map,
                'active_map': current_map,
            })
            bridge.get_logger().warn(
                f"Schedule {schedule['id']!r} skipped: mission requires map {mission_map!r}, "
                f"active map is {current_map!r}"
            )
            continue

        if repeat == 'once':
            # Auto-consumed, like a phone one-time alarm switching itself
            # off after it rings — a "once" schedule left enabled would
            # otherwise sit there matching nothing forever (the date has
            # passed) or, if the date field were ever left instead of
            # cleared, refire; disabling is the honest terminal state.
            schedule['enabled'] = False
            store.put_schedule(schedule)

        bridge.emit_event('schedule.fired', {
            'schedule_id': schedule['id'], 'mission_id': mission['id'],
        })
        opts['spawn'](_run_mission(bridge, opts, mission))
