#!/usr/bin/env python3
"""Road network graph persistence and RouteServer interface.

Maintains road network topology (nodes, directed/bidirectional edges, speed limits,
and operations) per map in SQLite (`road_networks` table) and exported as
`<map_name>_roadnet.geojson` on disk for Nav2's `nav2_route::RouteServer`.
"""

from __future__ import annotations

import json
import os
from typing import Optional

from nav2_msgs.srv import SetRouteGraph
from std_srvs.srv import Trigger

from .base import ApiError, BaseHandler
from .roscall import call_service


def get_roadnet_dirs() -> list[str]:
    """Scan and return all candidate map directories on disk."""
    dirs: list[str] = []
    try:
        from ament_index_python.packages import get_package_share_directory
        share = os.path.join(get_package_share_directory('navpromini_mapping'), 'maps')
        if os.path.isdir(share):
            dirs.append(share)
    except Exception:
        pass
    dirs.extend([
        os.path.join(os.path.expanduser('~'), 'NavProMini_ws', 'install', 'navpromini_mapping', 'share', 'navpromini_mapping', 'maps'),
        os.path.join(os.path.expanduser('~'), 'NavProMini_ws', 'src', 'navpromini_mapping', 'maps'),
        '/home/navpromini/NavProMini_ws/install/navpromini_mapping/share/navpromini_mapping/maps',
        '/home/navpromini/NavProMini_ws/src/navpromini_mapping/maps',
        os.path.expanduser('~/.navpromini_roadnets'),
    ])
    unique = []
    for d in dirs:
        ad = os.path.abspath(d)
        if ad not in unique:
            unique.append(ad)
    return unique


def get_primary_roadnet_path(map_name: str) -> str:
    """Return primary path to <map_name>_roadnet.geojson."""
    dirs = get_roadnet_dirs()
    for d in dirs:
        if os.path.isdir(d):
            return os.path.join(d, f'{map_name}_roadnet.geojson')
    # Fallback to user home
    target_dir = os.path.expanduser('~/.navpromini_roadnets')
    os.makedirs(target_dir, exist_ok=True)
    return os.path.join(target_dir, f'{map_name}_roadnet.geojson')


def save_roadnet_files(map_name: str, geojson: dict) -> str:
    """Write GeoJSON to all map directories on disk."""
    content = json.dumps(geojson, indent=2)
    primary = get_primary_roadnet_path(map_name)
    written = False
    for d in get_roadnet_dirs():
        try:
            if not os.path.isdir(d):
                os.makedirs(d, exist_ok=True)
            path = os.path.join(d, f'{map_name}_roadnet.geojson')
            with open(path, 'w', encoding='utf-8') as f:
                f.write(content)
            written = True
        except Exception:
            pass
    if not written:
        with open(primary, 'w', encoding='utf-8') as f:
            f.write(content)
    return primary


def delete_roadnet_files(map_name: str) -> None:
    """Delete <map_name>_roadnet.geojson from all map directories."""
    for d in get_roadnet_dirs():
        if os.path.isdir(d):
            path = os.path.join(d, f'{map_name}_roadnet.geojson')
            if os.path.isfile(path):
                try:
                    os.remove(path)
                except Exception:
                    pass


def read_roadnet_file(map_name: str) -> Optional[dict]:
    """Read <map_name>_roadnet.geojson from disk if exists."""
    for d in get_roadnet_dirs():
        path = os.path.join(d, f'{map_name}_roadnet.geojson')
        if os.path.isfile(path):
            try:
                with open(path, 'r', encoding='utf-8') as f:
                    return json.load(f)
            except Exception:
                pass
    return None


async def load_route_graph_into_route_server(bridge, map_name: str, store) -> bool:
    """Load the map's road network into Nav2 RouteServer via SetRouteGraph."""
    geojson = store.get_road_network(map_name)
    if not geojson:
        geojson = read_roadnet_file(map_name)
    if not geojson or not geojson.get('features'):
        bridge.get_logger().info(f'No road network features found for map {map_name!r}')
        return False

    path = save_roadnet_files(map_name, geojson)
    req = SetRouteGraph.Request()
    req.graph_filepath = path
    try:
        resp = await call_service(bridge.cli_set_route_graph, req, 'set_route_graph', timeout=5.0)
        success = bool(resp and resp.success)
        if success:
            bridge.get_logger().info(f'Loaded road network for {map_name!r} from {path}')
        else:
            bridge.get_logger().warn(f'RouteServer failed to load road network from {path}')
        return success
    except Exception as exc:
        bridge.get_logger().warn(f'Could not call /route_server/set_route_graph: {exc}')
        return False


class RoadNetworkHandler(BaseHandler):
    """GET /api/v1/road_network?map=<map_name>
       POST /api/v1/road_network (body: {map: str, geojson: dict})
       DELETE /api/v1/road_network?map=<map_name>
    """

    async def get(self) -> None:
        store = self.opts['store']
        map_name = self.get_argument('map', default=None) or store.current_map()
        geojson = store.get_road_network(map_name)
        if not geojson:
            geojson = read_roadnet_file(map_name)
        if not geojson:
            # Return empty FeatureCollection structure conforming to Nav2 RouteServer spec
            geojson = {
                'type': 'FeatureCollection',
                'name': 'graph',
                'crs': {'type': 'name', 'properties': {'name': 'urn:ogc:def:crs:EPSG::3857'}},
                'features': [],
            }
        self.send({
            'map': map_name,
            'geojson': geojson,
            'features_count': len(geojson.get('features', [])),
        })

    async def post(self) -> None:
        store = self.opts['store']
        data = self.body()
        map_name = data.get('map') or self.get_argument('map', default=None) or store.current_map()
        geojson = data.get('geojson') or data

        if not isinstance(geojson, dict) or geojson.get('type') != 'FeatureCollection':
            raise ApiError(400, 'invalid_geojson',
                           'Expected GeoJSON FeatureCollection with "features" array')

        features = geojson.get('features', [])
        if not isinstance(features, list):
            raise ApiError(400, 'invalid_features', '"features" must be a list')

        # Store in SQLite central database
        store.put_road_network(geojson, map_name)

        # Export to disk for Nav2 RouteServer
        primary_path = save_roadnet_files(map_name, geojson)

        # Count nodes and edges
        nodes = [f for f in features if f.get('geometry', {}).get('type') == 'Point']
        edges = [f for f in features if 'LineString' in f.get('geometry', {}).get('type', '')]

        # If this map is current in navigation mode, immediately load it into RouteServer
        loaded_live = False
        mode_state = self.opts.get('mode_state')
        if mode_state and mode_state.mode == 'navigation' and (mode_state.map_name == map_name or store.current_map() == map_name):
            try:
                loaded_live = await load_route_graph_into_route_server(self.bridge, map_name, store)
            except Exception as exc:
                self.bridge.get_logger().warn(f'Live reload into RouteServer failed: {exc}')

        self.bridge.emit_event('road_network.saved', {
            'map': map_name,
            'nodes_count': len(nodes),
            'edges_count': len(edges),
            'loaded_live': loaded_live,
        })

        self.send({
            'saved': True,
            'map': map_name,
            'path': primary_path,
            'nodes_count': len(nodes),
            'edges_count': len(edges),
            'loaded_live': loaded_live,
        }, status=200)

    async def delete(self) -> None:
        store = self.opts['store']
        map_name = self.get_argument('map', default=None) or store.current_map()
        store.delete_road_network(map_name)
        delete_roadnet_files(map_name)

        self.bridge.emit_event('road_network.deleted', {'map': map_name})
        self.send({'deleted': True, 'map': map_name})


class RerouteHandler(BaseHandler):
    """Trigger immediate reroute via Nav2 RouteServer ReroutingService."""

    async def post(self) -> None:
        req = Trigger.Request()
        try:
            resp = await call_service(self.bridge.cli_reroute, req, 'reroute', timeout=3.0)
            self.send({'ok': bool(resp and resp.success), 'message': resp.message if resp else 'No response'})
        except Exception as exc:
            raise ApiError(500, 'reroute_failed', f'Failed to trigger reroute: {exc}')
