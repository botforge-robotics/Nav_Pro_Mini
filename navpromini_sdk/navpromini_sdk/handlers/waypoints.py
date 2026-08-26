#!/usr/bin/env python3
"""Named poses the robot can navigate to, stored on the robot."""

from __future__ import annotations

from .base import ApiError, BaseHandler

VALID_TYPES = ('waypoint', 'dock', 'pickup', 'dropoff', 'home')


class WaypointsHandler(BaseHandler):
    def get(self) -> None:
        store = self.opts['store']
        map_name = self.get_argument('map', None)
        self.send({'map': map_name or store.current_map(),
                   'waypoints': store.list_waypoints(map_name)})

    def post(self) -> None:
        """Create or replace a waypoint.

        With no x/y, the robot's current pose is captured — which is how a
        waypoint is normally made: drive there, then name it.
        """
        data = self.body(('name',))
        store = self.opts['store']
        name = str(data['name']).strip()
        if not name:
            raise ApiError(400, 'invalid_field', 'name must not be empty')

        wp_type = str(data.get('type', 'waypoint'))
        if wp_type not in VALID_TYPES:
            raise ApiError(400, 'invalid_field',
                           f'type must be one of: {", ".join(VALID_TYPES)}',
                           {'valid': list(VALID_TYPES)})

        if 'x' in data and 'y' in data:
            x, y = float(data['x']), float(data['y'])
            theta = float(data.get('theta', 0.0))
            source = 'given'
        else:
            pose = self.bridge.get('pose_map')
            if pose is None:
                raise ApiError(409, 'not_localized',
                               'No map-frame pose available, so the current '
                               'position cannot be saved. Start navigation and '
                               'localize first, or pass x/y explicitly.')
            x, y, theta = pose['x'], pose['y'], pose['theta']
            source = 'current_pose'

        wp = {'name': name, 'type': wp_type, 'x': x, 'y': y, 'theta': theta}
        store.put_waypoint(wp, data.get('map'))
        self.send({'waypoint': wp, 'source': source}, status=201)


class WaypointHandler(BaseHandler):
    def get(self, name: str) -> None:
        wp = self.opts['store'].get_waypoint(name, self.get_argument('map', None))
        if wp is None:
            raise ApiError(404, 'waypoint_not_found', f'No waypoint named {name!r}')
        self.send({'waypoint': wp})

    def delete(self, name: str) -> None:
        if not self.opts['store'].delete_waypoint(name, self.get_argument('map', None)):
            raise ApiError(404, 'waypoint_not_found', f'No waypoint named {name!r}')
        self.send({'deleted': True, 'name': name})
