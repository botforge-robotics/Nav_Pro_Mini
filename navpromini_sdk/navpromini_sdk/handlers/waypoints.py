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

        has_x = 'x' in data
        has_y = 'y' in data
        if has_x != has_y:
            raise ApiError(400, 'invalid_field',
                           'both x and y must be provided together')

        if has_x and has_y:
            try:
                x, y = float(data['x']), float(data['y'])
                theta = float(data.get('theta', 0.0))
            except (ValueError, TypeError) as e:
                raise ApiError(400, 'invalid_field',
                               f'x, y, and theta must be numeric: {e}')
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

        # If saving Charging Dock for the active map, sync to dock pose and topic
        req_map = data.get('map')
        if (name == 'Charging Dock' or (wp_type == 'dock' and 'standoff' not in name.lower())) and (not req_map or req_map == store.current_map()):
            import json
            import math
            import os
            from geometry_msgs.msg import PoseStamped
            standoff_dist = 0.70
            standoff_x = x + standoff_dist * math.cos(theta)
            standoff_y = y + standoff_dist * math.sin(theta)
            standoff_theta = theta
            dock_data = {
                'x': x,
                'y': y,
                'theta': theta,
                'frame_id': 'map',
                'standoff_x': standoff_x,
                'standoff_y': standoff_y,
                'standoff_theta': standoff_theta,
            }
            with self.bridge._lock:
                self.bridge._cache['dock_pose'] = (
                    dock_data,
                    self.bridge.get_clock().now().nanoseconds / 1e9,
                )
            if 'dock_pose_pub' in self.opts:
                msg = PoseStamped()
                msg.header.frame_id = 'map'
                msg.header.stamp = self.bridge.get_clock().now().to_msg()
                msg.pose.position.x = x
                msg.pose.position.y = y
                msg.pose.orientation.z = math.sin(theta / 2.0)
                msg.pose.orientation.w = math.cos(theta / 2.0)
                self.opts['dock_pose_pub'].publish(msg)
            dock_dict = {
                'frame_id': 'map',
                'x': x,
                'y': y,
                'z': 0.0,
                'qx': 0.0,
                'qy': 0.0,
                'qz': math.sin(theta / 2.0),
                'qw': math.cos(theta / 2.0),
                'theta': theta,
                'standoff_x': standoff_x,
                'standoff_y': standoff_y,
                'standoff_theta': standoff_theta,
            }
            pid = os.getpid()
            for dock_file in [os.path.expanduser('~/.navpromini_dock_pose.json'),
                              '/home/navpromini/.navpromini_dock_pose.json',
                              '/root/.navpromini_dock_pose.json']:
                dir_name = os.path.dirname(dock_file)
                if not os.path.isdir(dir_name):
                    continue
                try:
                    tmp_path = f"{dock_file}.tmp.{pid}"
                    with open(tmp_path, 'w') as f:
                        json.dump(dock_dict, f, indent=2)
                    try:
                        os.chmod(tmp_path, 0o666)
                    except OSError:
                        pass
                    os.replace(tmp_path, dock_file)
                except Exception:
                    pass

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
