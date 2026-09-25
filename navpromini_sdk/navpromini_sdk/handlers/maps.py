#!/usr/bin/env python3
"""Map listing, saving, deleting and activation."""

from __future__ import annotations

import os
from navpromini_launch_manager_interfaces.srv import DeleteMap, GetMapList, LaunchWithArgs

from .base import ApiError, BaseHandler
from .roscall import call_service

# Maps live in navpromini_mapping's share directory. Note this is inside the
# colcon INSTALL tree, not a stable data path — a clean rebuild can remove
# saved maps. Documented rather than silently worked around, because the fix
# belongs in the workspace layout, not in this API.
MAP_PACKAGE = 'navpromini_mapping'
MAP_RELPATH = 'maps'
MAP_PATH = f'{MAP_PACKAGE}/{MAP_RELPATH}'


FILTER_TOKENS = ('_keepout', '_mask', '_filter', '_speed', '_zone', '_restricted', '_costmap')


def is_valid_base_map(name: str) -> bool:
    """Return True if name is a valid base map and not an internal costmap filter mask."""
    if not name or name.startswith('.'):
        return False
    lower = name.lower()
    return not any(tok in lower for tok in FILTER_TOKENS)


def list_maps_from_disk() -> list[str]:
    """Scan known map directories directly on disk for .yaml map files."""
    dirs = []
    try:
        from ament_index_python.packages import get_package_share_directory
        share = os.path.join(get_package_share_directory('navpromini_mapping'), 'maps')
        if os.path.isdir(share):
            dirs.append(share)
    except Exception:
        pass
    dirs.extend([
        '/home/navpromini/NavProMini_ws/install/navpromini_mapping/share/navpromini_mapping/maps',
        '/home/navpromini/NavProMini_ws/src/navpromini_mapping/maps',
    ])
    found = set()
    for d in dirs:
        if not os.path.isdir(d):
            continue
        try:
            for fname in os.listdir(d):
                if fname.endswith('.yaml'):
                    stem = fname[:-5]
                    if is_valid_base_map(stem):
                        found.add(stem)
        except Exception:
            pass
    return sorted(list(found))


class MapsHandler(BaseHandler):
    async def get(self) -> None:
        maps = []
        try:
            req = GetMapList.Request()
            req.path = MAP_PATH
            resp = await call_service(self.bridge.cli_maplist, req, 'get_map_list', timeout=3.0)
            if resp.success:
                maps = [m for m in list(resp.maplist) if is_valid_base_map(m)]
        except Exception:
            # Fallback to direct disk inspection if ROS service call times out or is slow
            maps = list_maps_from_disk()

        if not maps:
            # Double check disk if ROS service returned empty list
            disk_maps = list_maps_from_disk()
            if disk_maps:
                maps = disk_maps

        self.send({'maps': maps, 'count': len(maps),
                   'current': self.opts['store'].current_map()})

    async def post(self) -> None:
        data = self.body(('name',))
        result = await save_map(self.bridge, self.opts['store'],
                                data['name'], bool(data.get('overwrite')))
        self.send(result, status=201 if not result.get('overwritten') else 200)


class CurrentMapHandler(BaseHandler):
    def get(self) -> None:
        state = self.opts['mode_state']
        self.send({'current': self.opts['store'].current_map(),
                   'mode': state.mode})


class MapHandler(BaseHandler):
    async def delete(self, name: str) -> None:
        if not is_valid_base_map(name):
            raise ApiError(400, 'invalid_map_name',
                           f'{name!r} is an internal costmap filter mask and cannot be deleted as a map. '
                           'To modify or remove zones, use the /zones API.', {'map': name})
        state = self.opts['mode_state']
        if state.mode == 'navigation' and state.map_name == name:
            raise ApiError(409, 'map_in_use',
                           f'{name!r} is the map navigation is currently using. '
                           'Switch mode before deleting it.', {'map': name})
        req = DeleteMap.Request()
        req.map_name = name
        req.map_path = MAP_PATH
        resp = await call_service(self.bridge.cli_delmap, req, 'delete_map')
        if not resp.success:
            raise ApiError(404, 'map_not_found', resp.message or f'no map {name!r}')

        # Clean up database records (zones, waypoints, missions) associated with the deleted map
        store = self.opts['store']
        try:
            for z in store.list_zones(name):
                store.delete_zone(z['id'], name)
            for wp in store.list_waypoints(name):
                store.delete_waypoint(wp['name'], name)
            for m in store.list_missions(name):
                store.delete_mission(m['id'], name)
            if store.current_map() == name:
                store.set_current_map('default')
        except Exception:
            pass

        # Also remove all matching files (base and all filters/masks) from disk
        candidate_dirs = [
            '/home/navpromini/NavProMini_ws/install/navpromini_mapping/share/navpromini_mapping/maps',
            '/home/navpromini/NavProMini_ws/src/navpromini_mapping/maps',
        ]
        try:
            from ament_index_python.packages import get_package_share_directory
            share = os.path.join(get_package_share_directory('navpromini_mapping'), 'maps')
            if share not in candidate_dirs:
                candidate_dirs.append(share)
        except Exception:
            pass

        for d in candidate_dirs:
            if not os.path.isdir(d):
                continue
            try:
                for fname in os.listdir(d):
                    if fname.startswith(f"{name}_") or fname.startswith(f"{name}."):
                        target = os.path.join(d, fname)
                        if os.path.isfile(target):
                            try:
                                os.remove(target)
                            except Exception:
                                pass
            except Exception:
                pass

        self.bridge.emit_event('map.deleted', {'name': name})
        self.send({'deleted': True, 'name': name, 'detail': resp.message})


class ActivateMapHandler(BaseHandler):
    """Switch navigation to a different map (restarts the navigation stack)."""

    async def post(self, name: str) -> None:
        if not is_valid_base_map(name):
            raise ApiError(400, 'invalid_map',
                           f"'{name}' is an internal costmap filter mask, not a navigable map.")
        # Verify map exists on disk before trying to activate
        available = list_maps_from_disk()
        try:
            req = GetMapList.Request()
            req.path = MAP_PATH
            resp = await call_service(self.bridge.cli_maplist, req, 'get_map_list', timeout=3.0)
            if resp.success:
                available = [m for m in list(resp.maplist) if is_valid_base_map(m)]
        except Exception:
            pass
        if name not in available:
            raise ApiError(404, 'map_not_found',
                           f"No saved map named '{name}'. "
                           f"Available maps: {available}",
                           {'available': available})
        from .mode import switch_mode
        result = await switch_mode(self.opts, self.bridge, 'navigation', name)
        self.send(result, status=202)

import io
import numpy as np
from PIL import Image

class CurrentMapInfoHandler(BaseHandler):
    def get(self) -> None:
        m = self.bridge.get('map_msg')
        if not m:
            self.send({'loaded': False, 'current': self.opts['store'].current_map()})
            return
        resp = {
            'loaded': True,
            'current': self.opts['store'].current_map(),
            'width': m.info.width,
            'height': m.info.height,
            'resolution': float(m.info.resolution),
            'origin': {
                'x': float(m.info.origin.position.x),
                'y': float(m.info.origin.position.y)
            }
        }
        if self.get_argument('include_data', '0') in ('1', 'true', 'True'):
            resp['data'] = [int(v) for v in m.data]
        self.send(resp)

class CurrentMapRawHandler(BaseHandler):
    def get(self) -> None:
        m = self.bridge.get('map_msg')
        if not m:
            raise ApiError(404, 'no_map', 'No active occupancy grid map loaded')
        rotate = int(self.get_argument('rotate', 90))

        data = np.array(m.data, dtype=np.int8).reshape((m.info.height, m.info.width))
        data = np.flipud(data)
        if rotate == 90:
            data = np.rot90(data, -1)
        elif rotate == 180:
            data = np.rot90(data, 2)
        elif rotate == 270:
            data = np.rot90(data, 1)

        h, w = data.shape
        # RGB565 native:
        # 0x0863 (#0B0F19 unknown), 0x1926 (#1B2333 free), 0x3DFE (#38BDF8 wall)
        rgb565 = np.full((h, w), 0x0863, dtype=np.uint16)
        rgb565[data == 0] = 0x1926
        rgb565[data > 50] = 0x3DFE

        raw_bytes = rgb565.tobytes()
        self.set_header('Content-Type', 'application/octet-stream')
        self.set_header('X-Map-Width', str(w))
        self.set_header('X-Map-Height', str(h))
        self.set_header('X-Map-Resolution', str(m.info.resolution))
        self.set_header('X-Map-Origin-X', str(m.info.origin.position.x))
        self.set_header('X-Map-Origin-Y', str(m.info.origin.position.y))
        self.set_header('X-Map-Rotated', str(rotate))
        self.write(raw_bytes)

class CurrentMapImageHandler(BaseHandler):
    def get(self) -> None:
        m = self.bridge.get('map_msg')
        if not m:
            raise ApiError(404, 'no_map', 'No active occupancy grid map loaded')
        rotate = int(self.get_argument('rotate', 90))

        data = np.array(m.data, dtype=np.int8).reshape((m.info.height, m.info.width))
        data = np.flipud(data)
        if rotate == 90:
            data = np.rot90(data, -1)
        elif rotate == 180:
            data = np.rot90(data, 2)
        elif rotate == 270:
            data = np.rot90(data, 1)

        h, w = data.shape
        rgb = np.zeros((h, w, 3), dtype=np.uint8)
        rgb[data == -1] = [11, 15, 25]     # Unknown
        rgb[data == 0] = [30, 41, 59]      # Free space
        rgb[data > 50] = [56, 189, 248]    # Obstacle / wall

        im = Image.fromarray(rgb)
        buf = io.BytesIO()
        im.save(buf, format='PNG')
        self.set_header('Content-Type', 'image/png')
        self.write(buf.getvalue())


VALID_ZONE_TYPES = ('restricted', 'speed_limit', 'preferred_lane', 'work_zone')


def _validate_zone_payload(data: dict) -> dict:
    zone_id = str(data.get('id') or '').strip()
    if not zone_id:
        raise ApiError(400, 'invalid_field', "Zone 'id' is required")
    name = str(data.get('name') or zone_id).strip()
    zone_type = str(data.get('type') or 'restricted').strip()
    if zone_type not in VALID_ZONE_TYPES:
        raise ApiError(400, 'invalid_field',
                       f'type must be one of: {", ".join(VALID_ZONE_TYPES)}')
    points_raw = data.get('points')
    if not isinstance(points_raw, list) or len(points_raw) < 3:
        raise ApiError(400, 'invalid_field', 'points must be a list of at least 3 coordinates')
    cleaned_points = []
    for pt in points_raw:
        if not isinstance(pt, dict) or 'x' not in pt or 'y' not in pt:
            raise ApiError(400, 'invalid_field', 'each point must have x and y')
        try:
            cleaned_points.append({'x': float(pt['x']), 'y': float(pt['y'])})
        except (ValueError, TypeError) as e:
            raise ApiError(400, 'invalid_field', f'point coordinates must be numeric: {e}')

    zone = {
        'id': zone_id,
        'name': name,
        'type': zone_type,
        'points': cleaned_points,
    }
    if 'speed_limit_mps' in data and data['speed_limit_mps'] is not None:
        try:
            zone['speed_limit_mps'] = float(data['speed_limit_mps'])
        except (ValueError, TypeError):
            pass
    if 'color' in data and data['color']:
        zone['color'] = str(data['color'])
    return zone


class MapZonesHandler(BaseHandler):
    def get(self, map_name: str) -> None:
        store = self.opts['store']
        self.send({'map': map_name, 'zones': store.list_zones(map_name)})

    def post(self, map_name: str) -> None:
        store = self.opts['store']
        data = self.body()
        zone = _validate_zone_payload(data)
        zone['map'] = map_name
        store.put_zone(zone, map_name)
        self.bridge.emit_event('zone.saved', {'map': map_name, 'zone': zone})
        zone_mgr = self.opts.get('zone_mgr')
        if zone_mgr:
            zone_mgr.update_zones(map_name)
        self.send({'status': 'ok', 'map': map_name, 'zone': zone})


class MapZoneHandler(BaseHandler):
    def get(self, map_name: str, zone_id: str) -> None:
        store = self.opts['store']
        z = store.get_zone(zone_id, map_name)
        if not z:
            raise ApiError(404, 'not_found', f'Zone {zone_id!r} not found on map {map_name!r}')
        self.send(z)

    def delete(self, map_name: str, zone_id: str) -> None:
        store = self.opts['store']
        deleted = store.delete_zone(zone_id, map_name)
        if not deleted:
            raise ApiError(404, 'not_found', f'Zone {zone_id!r} not found on map {map_name!r}')
        self.bridge.emit_event('zone.deleted', {'map': map_name, 'id': zone_id})
        zone_mgr = self.opts.get('zone_mgr')
        if zone_mgr:
            zone_mgr.update_zones(map_name)
        self.send({'status': 'ok', 'map': map_name, 'id': zone_id})


class ZonesHandler(BaseHandler):
    def get(self) -> None:
        store = self.opts['store']
        map_name = self.get_argument('map', None) or store.current_map()
        self.send({'map': map_name, 'zones': store.list_zones(map_name)})

    def post(self) -> None:
        store = self.opts['store']
        data = self.body()
        map_name = str(data.get('map') or self.get_argument('map', None) or store.current_map())
        zone = _validate_zone_payload(data)
        zone['map'] = map_name
        store.put_zone(zone, map_name)
        self.bridge.emit_event('zone.saved', {'map': map_name, 'zone': zone})
        zone_mgr = self.opts.get('zone_mgr')
        if zone_mgr:
            zone_mgr.update_zones(map_name)
        self.send({'status': 'ok', 'map': map_name, 'zone': zone})


class ZoneHandler(BaseHandler):
    def get(self, zone_id: str) -> None:
        store = self.opts['store']
        map_name = self.get_argument('map', None) or store.current_map()
        z = store.get_zone(zone_id, map_name)
        if not z:
            raise ApiError(404, 'not_found', f'Zone {zone_id!r} not found')
        self.send(z)

    def delete(self, zone_id: str) -> None:
        store = self.opts['store']
        map_name = self.get_argument('map', None) or store.current_map()
        deleted = store.delete_zone(zone_id, map_name)
        if not deleted:
            raise ApiError(404, 'not_found', f'Zone {zone_id!r} not found')
        self.bridge.emit_event('zone.deleted', {'map': map_name, 'id': zone_id})
        zone_mgr = self.opts.get('zone_mgr')
        if zone_mgr:
            zone_mgr.update_zones(map_name)
        self.send({'status': 'ok', 'map': map_name, 'id': zone_id})

