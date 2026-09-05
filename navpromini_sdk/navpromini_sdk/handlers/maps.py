#!/usr/bin/env python3
"""Map listing, saving, deleting and activation."""

from __future__ import annotations

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


async def save_map(bridge, store, name: str, overwrite: bool) -> dict:
    """Save the current map under a name. Shared by MapsHandler.post and
    mode.FinishMappingHandler (doc §12's atomic FINISH_MAPPING).

    launch_manager verifies a save by diffing the map list, and returns
    success=False with "already exist" when the name was already present —
    even though the save itself ran. That is ambiguous for an API, so an
    existing name is a 409 unless the caller passes overwrite=True.
    """
    name = str(name).strip()
    if not name or '/' in name or name.startswith('.'):
        raise ApiError(400, 'invalid_name',
                       'name must be a simple file name without "/"')

    req = LaunchWithArgs.Request()
    req.package = 'nav2_mission_planner'
    req.launch_file = 'save_map.launch.py'
    req.arguments = f'map_name:={name} map_path:={MAP_PATH}'
    resp = await call_service(bridge.cli_launch, req, 'save_map', timeout=120.0)

    already = 'already exist' in (resp.message or '').lower()
    if resp.success:
        bridge.emit_event('map.saved', {'name': name})
        return {'saved': True, 'name': name}
    if already and overwrite:
        # The save ran; only the "is this new?" check failed.
        bridge.emit_event('map.saved', {'name': name, 'overwritten': True})
        return {'saved': True, 'name': name, 'overwritten': True}
    if already:
        raise ApiError(409, 'map_exists',
                       f'A map named {name!r} already exists. Resend with '
                       '{"overwrite": true} to replace it.', {'name': name})
    raise ApiError(500, 'save_failed', resp.message or 'map save failed')


class MapsHandler(BaseHandler):
    async def get(self) -> None:
        req = GetMapList.Request()
        req.path = MAP_PATH
        resp = await call_service(self.bridge.cli_maplist, req, 'get_map_list')
        # launch_manager reports failure when the directory contains no yaml
        # files. For an API "there are no maps" is an empty list, not an error.
        maps = list(resp.maplist) if resp.success else []
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
        self.bridge.emit_event('map.deleted', {'name': name})
        self.send({'deleted': True, 'name': name, 'detail': resp.message})


class ActivateMapHandler(BaseHandler):
    """Switch navigation to a different map (restarts the navigation stack)."""

    async def post(self, name: str) -> None:
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
        self.send({
            'loaded': True,
            'current': self.opts['store'].current_map(),
            'width': m.info.width,
            'height': m.info.height,
            'resolution': float(m.info.resolution),
            'origin': {
                'x': float(m.info.origin.position.x),
                'y': float(m.info.origin.position.y)
            }
        })

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
