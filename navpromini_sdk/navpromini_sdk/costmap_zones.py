"""Publishes Nav2 Costmap Filter Info and Mask for Keepout/Restricted Zones, Preferred Lanes, and Speed Limits."""

from __future__ import annotations

import os
import threading
from typing import Any, Optional

import cv2
import numpy as np
import yaml
from PIL import Image

import rclpy
from nav2_msgs.msg import CostmapFilterInfo
from nav2_msgs.srv import LoadMap
from nav_msgs.msg import MapMetaData, OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import Header

LATCHED_QOS = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
)

# Speed filter resolution: 1 unit in mask = 0.005 m/s (e.g. 16 units = 0.08 m/s, 10 units = 0.05 m/s)
SPEED_LIMIT_MULTIPLIER = 0.005
SPEED_LIMIT_BASE = 0.0

# Non-lethal cost bias for preferred lane background (cost ~127 out of 254)
PREFERRED_LANE_BG_PENALTY = 50
# Non-lethal cost bias for work / caution zones (cost ~88 out of 254)
WORK_ZONE_COST_PENALTY = 35


def _get_map_directories() -> list[str]:
    dirs: list[str] = []
    try:
        from ament_index_python.packages import get_package_share_directory
        share_maps = os.path.join(get_package_share_directory('navpromini_mapping'), 'maps')
        if os.path.isdir(share_maps):
            dirs.append(share_maps)
    except Exception:
        pass

    custom = os.environ.get('NAVPRO_MAPS_DIR')
    if custom and os.path.isdir(custom) and custom not in dirs:
        dirs.append(custom)

    candidates = [
        '/home/navpromini/NavProMini_ws/install/navpromini_mapping/share/navpromini_mapping/maps',
        '/home/navpromini/NavProMini_ws/src/navpromini_mapping/maps',
        os.path.join(os.path.expanduser('~'), 'NavProMini_ws', 'install',
                     'navpromini_mapping', 'share', 'navpromini_mapping', 'maps'),
        os.path.join(os.path.expanduser('~'), 'NavProMini_ws', 'src',
                     'navpromini_mapping', 'maps'),
    ]
    for cand in candidates:
        if os.path.isdir(cand) and cand not in dirs:
            dirs.append(cand)
    return dirs


def _convert_points_to_pixels(points: list[dict], origin_x: float, origin_y: float,
                              resolution: float, width: int, height: int,
                              flip_y_for_image: bool = False) -> list[list[int]]:
    """Converts a list of world coordinate points [{'x': ..., 'y': ...}] to pixel coords."""
    pixel_pts: list[list[int]] = []
    for pt in points:
        wx = float(pt.get('x', 0.0))
        wy = float(pt.get('y', 0.0))
        gx = int(round((wx - origin_x) / resolution))
        gy = int(round((wy - origin_y) / resolution))
        ix = max(0, min(width - 1, gx))
        if flip_y_for_image:
            iy = (height - 1) - gy
            iy = max(0, min(height - 1, iy))
        else:
            iy = max(0, min(height - 1, gy))
        pixel_pts.append([ix, iy])
    return pixel_pts


def generate_keepout_mask(map_name: str, store: Any = None) -> tuple[str, str]:
    """Generate <map>_keepout.pgm and <map>_keepout.yaml from SQLite zones for map_name.

    Handles:
      - restricted zones (lethal cost 254 / black pixel 0, with footprint padding)
      - preferred lanes (free corridor 0 cost / white pixel 255, with background penalty)
      - work/caution zones (elevated cost / gray pixel)

    Returns (keepout_yaml_path, keepout_pgm_path).
    """
    clean_name = str(map_name).strip()
    if clean_name.endswith('.yaml'):
        clean_name = clean_name[:-5]

    map_dirs = _get_map_directories()
    base_yaml_path = None
    for d in map_dirs:
        cand = os.path.join(d, f'{clean_name}.yaml')
        if os.path.isfile(cand):
            base_yaml_path = cand
            break

    if not base_yaml_path:
        raise FileNotFoundError(f"Base map YAML for '{clean_name}' not found in {map_dirs}")

    with open(base_yaml_path, 'r', encoding='utf-8') as f:
        map_meta = yaml.safe_load(f)

    resolution = float(map_meta.get('resolution', 0.05))
    origin = list(map_meta.get('origin', [0.0, 0.0, 0.0]))
    while len(origin) < 3:
        origin.append(0.0)

    image_filename = map_meta.get('image', f'{clean_name}.pgm')
    if not os.path.isabs(image_filename):
        base_image_path = os.path.join(os.path.dirname(base_yaml_path), image_filename)
    else:
        base_image_path = image_filename

    with Image.open(base_image_path) as im:
        width, height = im.size

    # Get zones from store
    if store is None:
        from .store import Store
        store = Store()

    raw_zones = store.list_zones(clean_name)
    restricted_zones = [
        z for z in raw_zones
        if str(z.get('type') or '').lower() == 'restricted'
    ]
    preferred_lanes = [
        z for z in raw_zones
        if str(z.get('type') or '').lower() == 'preferred_lane'
    ]
    work_zones = [
        z for z in raw_zones
        if str(z.get('type') or '').lower() in ('work_zone', 'caution')
    ]

    origin_x = float(origin[0])
    origin_y = float(origin[1])

    # In ROS PGM mask (negate: 0):
    # 255 = Free space (Cost 0)
    # 0 = Occupied / Lethal obstacle (Cost 100/254)
    # Intermediate gray values provide soft cost preference
    if preferred_lanes:
        # Background penalty (~25% occupancy -> cost 64 out of 254)
        bg_pixel = int(round(255 - (255 * PREFERRED_LANE_BG_PENALTY / 100.0)))
        pgm_mask = np.full((height, width), bg_pixel, dtype=np.uint8)
        # Inside preferred corridors: cost 0 (white pixel 255)
        for z in preferred_lanes:
            points = z.get('points', [])
            if len(points) < 3:
                continue
            pixel_pts = _convert_points_to_pixels(
                points, origin_x, origin_y, resolution, width, height, flip_y_for_image=True
            )
            pts_arr = np.array(pixel_pts, dtype=np.int32)
            cv2.fillPoly(pgm_mask, [pts_arr], 255)
    else:
        pgm_mask = np.full((height, width), 255, dtype=np.uint8)

    # Work / Caution zones (cost ~88 out of 254)
    work_pixel = int(round(255 - (255 * WORK_ZONE_COST_PENALTY / 100.0)))
    for z in work_zones:
        points = z.get('points', [])
        if len(points) < 3:
            continue
        pixel_pts = _convert_points_to_pixels(
            points, origin_x, origin_y, resolution, width, height, flip_y_for_image=True
        )
        pts_arr = np.array(pixel_pts, dtype=np.int32)
        cv2.fillPoly(pgm_mask, [pts_arr], work_pixel)

    # Restricted zones (lethal obstacle: black pixel 0)
    for z in restricted_zones:
        points = z.get('points', [])
        if len(points) < 3:
            continue
        pixel_pts = _convert_points_to_pixels(
            points, origin_x, origin_y, resolution, width, height, flip_y_for_image=True
        )
        pts_arr = np.array(pixel_pts, dtype=np.int32)
        cv2.fillPoly(pgm_mask, [pts_arr], 0)

    # Clearance padding (0.16m = robot_radius):
    # Expands lethal keepout zones by robot radius so the global path planner
    # stays safely clear of zone boundaries.
    if restricted_zones:
        lethal_binary = (pgm_mask == 0).astype(np.uint8) * 255
        clearance_m = 0.16
        clearance_px = max(1, int(round(clearance_m / resolution)))
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * clearance_px + 1, 2 * clearance_px + 1))
        dilated_lethal = cv2.dilate(lethal_binary, kernel)
        pgm_mask[dilated_lethal > 0] = 0

    # Save keepout PGM and YAML to map directories
    keepout_yaml_path: Optional[str] = None
    keepout_pgm_path: Optional[str] = None

    yaml_content = {
        'image': f'{clean_name}_keepout.pgm',
        'mode': 'scale',
        'resolution': resolution,
        'origin': [float(origin[0]), float(origin[1]), float(origin[2])],
        'negate': 0,
        'occupied_thresh': 1.0,
        'free_thresh': 0.0,
    }

    for d in map_dirs:
        try:
            target_pgm = os.path.join(d, f'{clean_name}_keepout.pgm')
            target_yaml = os.path.join(d, f'{clean_name}_keepout.yaml')

            im_out = Image.fromarray(pgm_mask)
            im_out.save(target_pgm)

            with open(target_yaml, 'w', encoding='utf-8') as f:
                yaml.dump(yaml_content, f, default_flow_style=False)

            if keepout_yaml_path is None:
                keepout_yaml_path = target_yaml
                keepout_pgm_path = target_pgm
        except Exception:
            pass

    if not keepout_yaml_path or not keepout_pgm_path:
        raise IOError(f"Failed to write keepout mask files for '{clean_name}'")

    return keepout_yaml_path, keepout_pgm_path


def generate_speed_mask(map_name: str, store: Any = None) -> tuple[Optional[str], Optional[str]]:
    """Generate <map>_speed.pgm and <map>_speed.yaml from SQLite speed zones for map_name.

    Returns (speed_yaml_path, speed_pgm_path).
    """
    clean_name = str(map_name).strip()
    if clean_name.endswith('.yaml'):
        clean_name = clean_name[:-5]

    map_dirs = _get_map_directories()
    base_yaml_path = None
    for d in map_dirs:
        cand = os.path.join(d, f'{clean_name}.yaml')
        if os.path.isfile(cand):
            base_yaml_path = cand
            break

    if not base_yaml_path:
        return None, None

    with open(base_yaml_path, 'r', encoding='utf-8') as f:
        map_meta = yaml.safe_load(f)

    resolution = float(map_meta.get('resolution', 0.05))
    origin = list(map_meta.get('origin', [0.0, 0.0, 0.0]))
    while len(origin) < 3:
        origin.append(0.0)

    image_filename = map_meta.get('image', f'{clean_name}.pgm')
    if not os.path.isabs(image_filename):
        base_image_path = os.path.join(os.path.dirname(base_yaml_path), image_filename)
    else:
        base_image_path = image_filename

    with Image.open(base_image_path) as im:
        width, height = im.size

    if store is None:
        from .store import Store
        store = Store()

    raw_zones = store.list_zones(clean_name)
    speed_zones = [
        z for z in raw_zones
        if str(z.get('type') or '').lower() in ('speed_limit', 'work_zone', 'caution')
    ]

    origin_x = float(origin[0])
    origin_y = float(origin[1])

    # Speed filter mask: 0 means NO_SPEED_LIMIT (full speed)
    speed_mask = np.zeros((height, width), dtype=np.uint8)

    for z in speed_zones:
        points = z.get('points', [])
        if len(points) < 3:
            continue
        default_limit = 0.06 if str(z.get('type') or '').lower() in ('work_zone', 'caution') else 0.08
        limit_mps = float(z.get('speed_limit_mps') or default_limit)
        pixel_val = max(1, min(100, int(round(limit_mps / SPEED_LIMIT_MULTIPLIER))))
        pixel_pts = _convert_points_to_pixels(
            points, origin_x, origin_y, resolution, width, height, flip_y_for_image=True
        )
        pts_arr = np.array(pixel_pts, dtype=np.int32)
        cv2.fillPoly(speed_mask, [pts_arr], pixel_val)

    speed_yaml_path: Optional[str] = None
    speed_pgm_path: Optional[str] = None

    yaml_content = {
        'image': f'{clean_name}_speed.pgm',
        'mode': 'scale',
        'resolution': resolution,
        'origin': [float(origin[0]), float(origin[1]), float(origin[2])],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196,
    }

    for d in map_dirs:
        try:
            target_pgm = os.path.join(d, f'{clean_name}_speed.pgm')
            target_yaml = os.path.join(d, f'{clean_name}_speed.yaml')

            im_out = Image.fromarray(speed_mask)
            im_out.save(target_pgm)

            with open(target_yaml, 'w', encoding='utf-8') as f:
                yaml.dump(yaml_content, f, default_flow_style=False)

            if speed_yaml_path is None:
                speed_yaml_path = target_yaml
                speed_pgm_path = target_pgm
        except Exception:
            pass

    return speed_yaml_path, speed_pgm_path


class CostmapZoneManager:
    """Manages rasterization and live publication of Nav2 costmap filter masks for:
      - Keepout / Restricted Zones (Type 0)
      - Preferred Lanes (Type 0 cost bias)
      - Speed Limit Zones (Type 2 absolute speed)
      - Work / Caution Zones (Type 0 cost + Type 2 speed)
    """

    def __init__(self, node: Node, store: Any) -> None:
        self.node = node
        self.store = store
        self.logger = node.get_logger()
        self._lock = threading.Lock()

        self._active_map: Optional[str] = None
        self._cached_metadata: Optional[MapMetaData] = None

        # Publishers with transient-local QoS for Nav2 KeepoutFilter
        self.pub_filter_info = node.create_publisher(
            CostmapFilterInfo, '/costmap_filter_info', LATCHED_QOS
        )
        self.pub_filter_mask = node.create_publisher(
            OccupancyGrid, '/costmap_filter_mask', LATCHED_QOS
        )

        # Publishers with transient-local QoS for Nav2 SpeedFilter
        self.pub_speed_filter_info = node.create_publisher(
            CostmapFilterInfo, '/speed_filter_info', LATCHED_QOS
        )
        self.pub_speed_filter_mask = node.create_publisher(
            OccupancyGrid, '/speed_filter_mask', LATCHED_QOS
        )

        # Service client for filter_mask_server dynamic load_map
        self.cli_load_map = node.create_client(LoadMap, '/filter_mask_server/load_map')

        # Subscribe to /map to automatically track the active map's grid metadata
        self.sub_map = node.create_subscription(
            OccupancyGrid, '/map', self._on_map, LATCHED_QOS
        )

    def _on_map(self, msg: OccupancyGrid) -> None:
        """Called when /map is published by map_server or SLAM."""
        with self._lock:
            self._cached_metadata = msg.info
            self.logger.info(
                f'CostmapZoneManager: Cached /map metadata ({msg.info.width}x{msg.info.height}, '
                f'res={msg.info.resolution:.3f})'
            )
        target_map = (
            self._active_map
            or (self.store.current_map() if hasattr(self.store, 'current_map') else None)
            or 'default'
        )
        self.update_zones(target_map)

    def update_zones(self, map_name: str) -> bool:
        """Rasterize all zones, update files, trigger LoadMap, and publish filter masks."""
        clean_name = str(map_name).strip()
        if clean_name.endswith('.yaml'):
            clean_name = clean_name[:-5]

        with self._lock:
            self._active_map = clean_name

        try:
            yaml_path, _ = generate_keepout_mask(clean_name, self.store)
            generate_speed_mask(clean_name, self.store)
        except Exception as exc:
            self.logger.warn(f'CostmapZoneManager: Failed to generate mask files for {clean_name!r}: {exc}')
            return False

        # Dynamically reload map in filter_mask_server via standard Nav2 LoadMap service
        if self.cli_load_map.service_is_ready():
            req = LoadMap.Request()
            req.map_url = yaml_path
            future = self.cli_load_map.call_async(req)
            future.add_done_callback(lambda f: self._on_load_map_done(f, clean_name))
        else:
            self.logger.info(
                f'CostmapZoneManager: /filter_mask_server/load_map not ready yet (will reload once server is up).'
            )

        # Directly publish OccupancyGrid & CostmapFilterInfo for both keepout and speed filters
        self._publish_direct_masks(clean_name)
        return True

    def _on_load_map_done(self, future: Any, map_name: str) -> None:
        try:
            resp = future.result()
            if resp.result == 0:
                self.logger.info(
                    f'CostmapZoneManager: filter_mask_server successfully reloaded keepout mask for {map_name!r}'
                )
            else:
                self.logger.warn(
                    f'CostmapZoneManager: filter_mask_server load_map returned code {resp.result}'
                )
        except Exception as exc:
            self.logger.warn(f'CostmapZoneManager: load_map service callback error: {exc}')

    def _publish_direct_masks(self, map_name: str) -> None:
        """Construct and publish OccupancyGrid and CostmapFilterInfo directly for Nav2 plugins."""
        with self._lock:
            meta = self._cached_metadata

        if meta is None or meta.width == 0 or meta.height == 0:
            return

        width = meta.width
        height = meta.height
        resolution = meta.resolution
        origin_x = meta.origin.position.x
        origin_y = meta.origin.position.y

        raw_zones = self.store.list_zones(map_name)
        restricted_zones = [
            z for z in raw_zones
            if str(z.get('type') or '').lower() == 'restricted'
        ]
        preferred_lanes = [
            z for z in raw_zones
            if str(z.get('type') or '').lower() == 'preferred_lane'
        ]
        speed_limit_zones = [
            z for z in raw_zones
            if str(z.get('type') or '').lower() == 'speed_limit'
        ]
        work_zones = [
            z for z in raw_zones
            if str(z.get('type') or '').lower() in ('work_zone', 'caution')
        ]

        # ---------------------------------------------------------------------
        # 1. KEEPOUT & PREFERRED LANE & WORK ZONE FILTER MASK (Type 0)
        # ---------------------------------------------------------------------
        if preferred_lanes:
            # Set background to non-lethal penalty (cost ~64)
            keepout_mask = np.full((height, width), PREFERRED_LANE_BG_PENALTY, dtype=np.int8)
            # Free corridors inside preferred lanes (cost 0)
            for z in preferred_lanes:
                points = z.get('points', [])
                if len(points) < 3:
                    continue
                pixel_pts = _convert_points_to_pixels(points, origin_x, origin_y, resolution, width, height)
                pts_arr = np.array(pixel_pts, dtype=np.int32)
                cv2.fillPoly(keepout_mask, [pts_arr], 0)
        else:
            keepout_mask = np.zeros((height, width), dtype=np.int8)

        # Work / Caution zones: non-lethal elevated cost (~88)
        for z in work_zones:
            points = z.get('points', [])
            if len(points) < 3:
                continue
            pixel_pts = _convert_points_to_pixels(points, origin_x, origin_y, resolution, width, height)
            pts_arr = np.array(pixel_pts, dtype=np.int32)
            cv2.fillPoly(keepout_mask, [pts_arr], WORK_ZONE_COST_PENALTY)

        # Restricted zones: lethal cost 100 (254 in costmap)
        restricted_binary = np.zeros((height, width), dtype=np.uint8)
        for z in restricted_zones:
            points = z.get('points', [])
            if len(points) < 3:
                continue
            pixel_pts = _convert_points_to_pixels(points, origin_x, origin_y, resolution, width, height)
            pts_arr = np.array(pixel_pts, dtype=np.int32)
            cv2.fillPoly(restricted_binary, [pts_arr], 255)

        # Clearance padding (0.16m) on restricted zones
        if restricted_zones and np.any(restricted_binary > 0):
            clearance_m = 0.16
            clearance_px = max(1, int(round(clearance_m / resolution)))
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * clearance_px + 1, 2 * clearance_px + 1))
            restricted_binary = cv2.dilate(restricted_binary, kernel)

        keepout_mask[restricted_binary > 0] = 100

        grid_msg = OccupancyGrid()
        grid_msg.header = Header(
            stamp=self.node.get_clock().now().to_msg(),
            frame_id='map',
        )
        grid_msg.info = meta
        grid_msg.data = keepout_mask.flatten().tolist()

        filter_info_msg = CostmapFilterInfo()
        filter_info_msg.header = Header(
            stamp=self.node.get_clock().now().to_msg(),
            frame_id='map',
        )
        filter_info_msg.type = 0  # KEEPOUT_FILTER
        filter_info_msg.filter_mask_topic = '/costmap_filter_mask'
        filter_info_msg.multiplier = 1.0
        filter_info_msg.base = 0.0

        self.pub_filter_mask.publish(grid_msg)
        self.pub_filter_info.publish(filter_info_msg)

        # ---------------------------------------------------------------------
        # 2. SPEED LIMIT FILTER MASK (Type 2: SPEED_FILTER_ABSOLUTE)
        # ---------------------------------------------------------------------
        speed_mask = np.zeros((height, width), dtype=np.int8)

        all_speed_zones = speed_limit_zones + work_zones
        for z in all_speed_zones:
            points = z.get('points', [])
            if len(points) < 3:
                continue
            is_work = str(z.get('type') or '').lower() in ('work_zone', 'caution')
            default_limit = 0.06 if is_work else 0.08
            limit_mps = float(z.get('speed_limit_mps') or default_limit)
            pixel_val = max(1, min(100, int(round(limit_mps / SPEED_LIMIT_MULTIPLIER))))
            pixel_pts = _convert_points_to_pixels(points, origin_x, origin_y, resolution, width, height)
            pts_arr = np.array(pixel_pts, dtype=np.int32)
            cv2.fillPoly(speed_mask, [pts_arr], pixel_val)

        speed_grid_msg = OccupancyGrid()
        speed_grid_msg.header = Header(
            stamp=self.node.get_clock().now().to_msg(),
            frame_id='map',
        )
        speed_grid_msg.info = meta
        speed_grid_msg.data = speed_mask.flatten().tolist()

        speed_filter_info_msg = CostmapFilterInfo()
        speed_filter_info_msg.header = Header(
            stamp=self.node.get_clock().now().to_msg(),
            frame_id='map',
        )
        speed_filter_info_msg.type = 2  # SPEED_FILTER_ABSOLUTE
        speed_filter_info_msg.filter_mask_topic = '/speed_filter_mask'
        speed_filter_info_msg.multiplier = SPEED_LIMIT_MULTIPLIER
        speed_filter_info_msg.base = SPEED_LIMIT_BASE

        self.pub_speed_filter_mask.publish(speed_grid_msg)
        self.pub_speed_filter_info.publish(speed_filter_info_msg)

        self.logger.info(
            f'CostmapZoneManager: Updated filters for {map_name!r}: '
            f'{len(restricted_zones)} restricted, {len(preferred_lanes)} preferred lanes, '
            f'{len(speed_limit_zones)} speed limits, {len(work_zones)} work zones.'
        )
