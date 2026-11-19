#!/usr/bin/env python3
"""
Direct Gazebo SDF World to ROS 2 2D Occupancy Grid Map Generator.

Extracts all collision geometries (<box>, <cylinder>, included models) from any
Gazebo Harmonic SDF world, slices them within the robot LiDAR scan height window,
and rasterizes them into a high-precision 2D occupancy grid (.pgm and .yaml) for Nav2.

Usage:
  ros2 run navpromini_mapping sdf_map_generator --world playground
  ros2 run navpromini_mapping sdf_map_generator --world cafe --resolution 0.05
  ros2 run navpromini_mapping sdf_map_generator --file /path/to/world.sdf --output-dir /path/to/maps
"""

import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np


def parse_pose(pose_elem: Optional[ET.Element]) -> Tuple[float, float, float, float, float, float]:
    """Parse pose element string 'x y z roll pitch yaw'."""
    if pose_elem is None or not pose_elem.text:
        return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    parts = [float(p) for p in pose_elem.text.strip().split()]
    while len(parts) < 6:
        parts.append(0.0)
    return tuple(parts[:6])


def combine_transforms_2d(parent: Tuple[float, float, float],
                          child: Tuple[float, float, float]) -> Tuple[float, float, float]:
    """Combine 2D transforms (x, y, yaw)."""
    px, py, pyaw = parent
    cx, cy, cyaw = child
    cos_p = math.cos(pyaw)
    sin_p = math.sin(pyaw)
    x = px + cx * cos_p - cy * sin_p
    y = py + cx * sin_p + cy * cos_p
    yaw = pyaw + cyaw
    return x, y, yaw


class SDFMapGenerator:

    def __init__(self,
                 sdf_path: str,
                 resolution: float = 0.05,
                 z_min: float = 0.05,
                 z_max: float = 0.45,
                 margin: float = 1.0,
                 model_paths: Optional[List[str]] = None):
        self.sdf_path = sdf_path
        self.resolution = resolution
        self.z_min = z_min
        self.z_max = z_max
        self.margin = margin
        self.model_paths = model_paths or []

        # List of 2D shapes: ('box', corners_list) or ('circle', (cx, cy, radius))
        self.obstacles: List[Tuple[str, any]] = []
        self.bounds = [float('inf'), float('-inf'), float('inf'), float('-inf')]  # xmin, xmax, ymin, ymax

    def _resolve_model_uri(self, uri: str) -> Optional[str]:
        """Resolve model:// or file:// URI to absolute path."""
        if uri.startswith('model://'):
            subpath = uri[len('model://'):]
            for search_dir in self.model_paths:
                candidate = os.path.join(search_dir, subpath)
                if os.path.isdir(candidate):
                    sdf_file = os.path.join(candidate, 'model.sdf')
                    if os.path.isfile(sdf_file):
                        return sdf_file
        elif uri.startswith('file://'):
            path = uri[len('file://'):]
            if os.path.exists(path):
                return path
        return None

    def _process_collision(self,
                           collision_elem: ET.Element,
                           parent_2d: Tuple[float, float, float],
                           parent_z: float) -> None:
        """Extract geometry and test intersection with lidar slice."""
        col_pose = parse_pose(collision_elem.find('pose'))
        x, y, yaw = combine_transforms_2d(parent_2d, (col_pose[0], col_pose[1], col_pose[5]))
        z = parent_z + col_pose[2]

        geom = collision_elem.find('geometry')
        if geom is None:
            return

        box = geom.find('box')
        cylinder = geom.find('cylinder')

        if box is not None:
            size_elem = box.find('size')
            if size_elem is not None and size_elem.text:
                sx, sy, sz = [float(v) for v in size_elem.text.strip().split()]
                # Check vertical slice intersection
                if (z + sz / 2.0 < self.z_min) or (z - sz / 2.0 > self.z_max):
                    return  # Above or below laser scan plane

                # Compute 4 rotated corners in 2D
                cos_y = math.cos(yaw)
                sin_y = math.sin(yaw)
                dx = sx / 2.0
                dy = sy / 2.0
                corners = [
                    (x + (-dx) * cos_y - (-dy) * sin_y, y + (-dx) * sin_y + (-dy) * cos_y),
                    (x + (dx) * cos_y - (-dy) * sin_y, y + (dx) * sin_y + (-dy) * cos_y),
                    (x + (dx) * cos_y - (dy) * sin_y, y + (dx) * sin_y + (dy) * cos_y),
                    (x + (-dx) * cos_y - (dy) * sin_y, y + (-dx) * sin_y + (dy) * cos_y),
                ]
                self.obstacles.append(('box', corners))
                for cx, cy in corners:
                    self.bounds[0] = min(self.bounds[0], cx)
                    self.bounds[1] = max(self.bounds[1], cx)
                    self.bounds[2] = min(self.bounds[2], cy)
                    self.bounds[3] = max(self.bounds[3], cy)

        elif cylinder is not None:
            radius_elem = cylinder.find('radius')
            len_elem = cylinder.find('length')
            if radius_elem is not None and len_elem is not None:
                r = float(radius_elem.text.strip())
                length = float(len_elem.text.strip())
                if (z + length / 2.0 < self.z_min) or (z - length / 2.0 > self.z_max):
                    return

                self.obstacles.append(('circle', (x, y, r)))
                self.bounds[0] = min(self.bounds[0], x - r)
                self.bounds[1] = max(self.bounds[1], x + r)
                self.bounds[2] = min(self.bounds[2], y - r)
                self.bounds[3] = max(self.bounds[3], y + r)

    def _process_model(self, model_elem: ET.Element, world_2d: Tuple[float, float, float] = (0, 0, 0), world_z: float = 0.0) -> None:
        """Parse all links and collisions in a model."""
        m_pose = parse_pose(model_elem.find('pose'))
        m_2d = combine_transforms_2d(world_2d, (m_pose[0], m_pose[1], m_pose[5]))
        m_z = world_z + m_pose[2]

        for link in model_elem.findall('link'):
            l_pose = parse_pose(link.find('pose'))
            l_2d = combine_transforms_2d(m_2d, (l_pose[0], l_pose[1], l_pose[5]))
            l_z = m_z + l_pose[2]

            for collision in link.findall('collision'):
                self._process_collision(collision, l_2d, l_z)

    def _process_include(self, include_elem: ET.Element) -> None:
        """Resolve and parse included submodels (e.g. model://navpromini_dock)."""
        uri_elem = include_elem.find('uri')
        if uri_elem is None or not uri_elem.text:
            return
        inc_pose = parse_pose(include_elem.find('pose'))
        inc_path = self._resolve_model_uri(uri_elem.text.strip())
        if inc_path and os.path.exists(inc_path):
            try:
                tree = ET.parse(inc_path)
                root = tree.getroot()
                model = root.find('model') if root.tag == 'sdf' else (root if root.tag == 'model' else None)
                if model is not None:
                    self._process_model(model, (inc_pose[0], inc_pose[1], inc_pose[5]), inc_pose[2])
            except Exception as e:
                print(f"[WARN] Failed to parse included model {inc_path}: {e}", file=sys.stderr)

    def parse(self) -> None:
        """Parse the SDF world file and populate obstacles."""
        tree = ET.parse(self.sdf_path)
        root = tree.getroot()
        world = root.find('world') if root.tag == 'sdf' else root

        if world is None:
            raise ValueError(f"No <world> element found in {self.sdf_path}")

        # Models directly in world
        for model in world.findall('model'):
            # Skip ground plane
            name = model.get('name', '')
            if 'ground' in name.lower():
                continue
            self._process_model(model)

        # Included models
        for inc in world.findall('include'):
            self._process_include(inc)

    def generate_map(self) -> Tuple[np.ndarray, Dict]:
        """Rasterize obstacles into a 2D numpy occupancy grid."""
        xmin, xmax, ymin, ymax = self.bounds
        if math.isinf(xmin):
            raise ValueError("No collision obstacles found in specified vertical slice window!")

        xmin -= self.margin
        xmax += self.margin
        ymin -= self.margin
        ymax += self.margin

        width = int(math.ceil((xmax - xmin) / self.resolution))
        height = int(math.ceil((ymax - ymin) / self.resolution))

        # Nav2 convention: 254 = Free space, 0 = Occupied, 205 = Unknown
        # We initialize the arena interior as free space (254)
        grid = np.full((height, width), 254, dtype=np.uint8)

        # Coordinate conversion: (x, y) -> (pixel_col, pixel_row)
        # Note: image row 0 is at y = ymax, row height-1 is at y = ymin
        def to_img(x_val: float, y_val: float) -> Tuple[int, int]:
            c = int(round((x_val - xmin) / self.resolution))
            r = int(round((ymax - y_val) / self.resolution))
            c = max(0, min(width - 1, c))
            r = max(0, min(height - 1, r))
            return c, r

        for obs_type, data in self.obstacles:
            if obs_type == 'box':
                pts = np.array([to_img(cx, cy) for cx, cy in data], dtype=np.int32)
                cv2.fillPoly(grid, [pts], color=0)
            elif obs_type == 'circle':
                cx, cy, rad = data
                img_c, img_r = to_img(cx, cy)
                pix_r = max(1, int(round(rad / self.resolution)))
                cv2.circle(grid, (img_c, img_r), pix_r, color=0, thickness=-1)

        metadata = {
            'image': '',
            'mode': 'trinary',
            'resolution': float(self.resolution),
            'origin': [float(xmin), float(ymin), 0.0],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.25,
        }
        return grid, metadata

    def save(self, output_dir: str, map_name: str) -> Tuple[str, str]:
        """Save generated map to .pgm and .yaml files."""
        os.makedirs(output_dir, exist_ok=True)
        grid, meta = self.generate_map()

        pgm_filename = f"{map_name}.pgm"
        yaml_filename = f"{map_name}.yaml"
        pgm_path = os.path.join(output_dir, pgm_filename)
        yaml_path = os.path.join(output_dir, yaml_filename)

        meta['image'] = pgm_filename

        # Write PGM (Binary P5 format)
        cv2.imwrite(pgm_path, grid)

        # Write YAML
        with open(yaml_path, 'w') as f:
            f.write(f"image: {meta['image']}\n")
            f.write(f"mode: {meta['mode']}\n")
            f.write(f"resolution: {meta['resolution']:.4f}\n")
            f.write(f"origin: [{meta['origin'][0]:.3f}, {meta['origin'][1]:.3f}, {meta['origin'][2]:.3f}]\n")
            f.write(f"negate: {meta['negate']}\n")
            f.write(f"occupied_thresh: {meta['occupied_thresh']}\n")
            f.write(f"free_thresh: {meta['free_thresh']}\n")

        return pgm_path, yaml_path


def main(args: Optional[list] = None) -> None:
    parser = argparse.ArgumentParser(
        description='Generate 2D Occupancy Grid Map (.pgm + .yaml) directly from Gazebo SDF world.'
    )
    parser.add_argument('--world', type=str, default='playground',
                        help='World name (empty, office, cafe, playground) or custom name')
    parser.add_argument('--file', type=str, default='',
                        help='Direct path to .sdf world file (overrides --world)')
    parser.add_argument('--resolution', type=float, default=0.05,
                        help='Grid resolution in meters/pixel (default: 0.05)')
    parser.add_argument('--z-min', type=float, default=0.05,
                        help='Minimum height for lidar slice window in meters (default: 0.05)')
    parser.add_argument('--z-max', type=float, default=0.45,
                        help='Maximum height for lidar slice window in meters (default: 0.45)')
    parser.add_argument('--margin', type=float, default=1.0,
                        help='Border margin around obstacles in meters (default: 1.0)')
    parser.add_argument('--output-dir', type=str, default='',
                        help='Output directory to save map files (defaults to maps in navigation & mapping pkgs)')

    parsed = parser.parse_args(args)

    ws_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
    if parsed.file:
        sdf_path = os.path.abspath(parsed.file)
        map_name = Path(sdf_path).stem
    else:
        world_name = parsed.world.lower().strip()
        map_name = world_name
        sdf_path = os.path.join(ws_dir, 'src', 'navpromini_gazebo', 'worlds', f'{world_name}.sdf')
        if not os.path.exists(sdf_path):
            # Try installed package share
            share_candidate = os.path.join(ws_dir, 'install', 'navpromini_gazebo', 'share', 'navpromini_gazebo', 'worlds', f'{world_name}.sdf')
            if os.path.exists(share_candidate):
                sdf_path = share_candidate

    if not os.path.isfile(sdf_path):
        print(f"[ERROR] World file not found: {sdf_path}", file=sys.stderr)
        sys.exit(1)

    # Search paths for included models
    model_paths = [
        os.path.join(ws_dir, 'src', 'navpromini_gazebo', 'models'),
        os.path.join(ws_dir, 'install', 'navpromini_gazebo', 'share', 'navpromini_gazebo', 'models'),
        os.path.expanduser('~/.gz/fuel/fuel.gazebosim.org/openrobotics/models'),
    ]

    print(f"[INFO] Parsing SDF world: {sdf_path}")
    print(f"[INFO] Lidar slice window: [{parsed.z_min:.2f}m, {parsed.z_max:.2f}m], resolution: {parsed.resolution:.3f}m/pix")

    generator = SDFMapGenerator(
        sdf_path=sdf_path,
        resolution=parsed.resolution,
        z_min=parsed.z_min,
        z_max=parsed.z_max,
        margin=parsed.margin,
        model_paths=model_paths
    )
    generator.parse()
    print(f"[INFO] Found {len(generator.obstacles)} collision obstacles inside lidar slice.")

    # Target output directories
    target_dirs = []
    if parsed.output_dir:
        target_dirs.append(parsed.output_dir)
    else:
        nav_maps = os.path.join(ws_dir, 'src', 'navpromini_navigation', 'maps')
        map_maps = os.path.join(ws_dir, 'src', 'navpromini_mapping', 'maps')
        target_dirs.extend([nav_maps, map_maps])

    for out_dir in target_dirs:
        pgm, yml = generator.save(out_dir, map_name)
        print(f"[SUCCESS] Saved map files:")
        print(f"  - {pgm}")
        print(f"  - {yml}")


if __name__ == '__main__':
    main()
