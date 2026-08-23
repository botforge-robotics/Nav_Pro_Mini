#!/usr/bin/env python3
"""Loads configured self-geometry (pillars, wires, ...) from ROS parameters
and transforms it from base_link into the lidar frame.

Kept separate from self_filter_node.py so the parsing and transform math
can be unit-tested without a running ROS graph — see test/test_geometry.py.

Parameter schema (per object, under its own name — see config/self_filter.yaml
for the full worked example):

  self_object_names: ["pillar_1", "pillar_2", ...]   # which objects exist

  <name>.enabled: bool
  <name>.geometry_type: "circle" | "capsule" | "polygon"

  circle:   <name>.x, .y, .z, .radius
  capsule:  <name>.x1, .y1, .z1, .x2, .y2, .z2, .radius
  polygon:  <name>.x, .y, .z, .orientation, .points_x, .points_y
            (points_x/points_y are a LOCAL template around the origin —
            x/y/orientation place and rotate it. This is how a non-round
            pillar cross-section, bracket, or any flat shape gets defined
            once and positioned.)

All coordinates are metres, in base_link, at the pose recorded when the
object was measured (see the README's measurement procedure) — a static
description of the robot's own structure, not something that moves at
runtime.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple, Union

from . import geometry as geo

Vec3 = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]  # x, y, z, w


@dataclass
class CircleConfig:
    name: str
    enabled: bool
    x: float
    y: float
    z: float
    radius: float


@dataclass
class CapsuleConfig:
    name: str
    enabled: bool
    x1: float
    y1: float
    z1: float
    x2: float
    y2: float
    z2: float
    radius: float


@dataclass
class PolygonConfig:
    name: str
    enabled: bool
    x: float
    y: float
    z: float
    orientation: float
    points: List[Tuple[float, float]]  # local template, pre-transform


ObjectConfig = Union[CircleConfig, CapsuleConfig, PolygonConfig]


class ConfigError(ValueError):
    """A configured self-object is malformed. Raised at load time, not
    per-scan — a bad config should fail loudly at startup, not degrade
    into 'never filters anything' silently.
    """


def load_object(get_param, name: str) -> Optional[ObjectConfig]:
    """Build one object's config via `get_param(key, default)`.

    `get_param` is injected rather than importing rclpy here, so this
    function is directly unit-testable with a plain dict-backed stub — see
    test/test_geometry.py.
    """
    enabled = bool(get_param(f'{name}.enabled', False))
    geometry_type = str(get_param(f'{name}.geometry_type', ''))

    if geometry_type == 'circle':
        return CircleConfig(
            name=name, enabled=enabled,
            x=float(get_param(f'{name}.x', 0.0)),
            y=float(get_param(f'{name}.y', 0.0)),
            z=float(get_param(f'{name}.z', 0.0)),
            radius=float(get_param(f'{name}.radius', 0.0)),
        )
    if geometry_type == 'capsule':
        return CapsuleConfig(
            name=name, enabled=enabled,
            x1=float(get_param(f'{name}.x1', 0.0)),
            y1=float(get_param(f'{name}.y1', 0.0)),
            z1=float(get_param(f'{name}.z1', 0.0)),
            x2=float(get_param(f'{name}.x2', 0.0)),
            y2=float(get_param(f'{name}.y2', 0.0)),
            z2=float(get_param(f'{name}.z2', 0.0)),
            radius=float(get_param(f'{name}.radius', 0.0)),
        )
    if geometry_type == 'polygon':
        points_x = list(get_param(f'{name}.points_x', []))
        points_y = list(get_param(f'{name}.points_y', []))
        if len(points_x) != len(points_y) or len(points_x) < 3:
            raise ConfigError(
                f"{name}: polygon needs points_x/points_y of equal length "
                f">= 3, got {len(points_x)}/{len(points_y)}")
        return PolygonConfig(
            name=name, enabled=enabled,
            x=float(get_param(f'{name}.x', 0.0)),
            y=float(get_param(f'{name}.y', 0.0)),
            z=float(get_param(f'{name}.z', 0.0)),
            orientation=float(get_param(f'{name}.orientation', 0.0)),
            points=list(zip((float(v) for v in points_x),
                           (float(v) for v in points_y))),
        )
    raise ConfigError(
        f"{name}: geometry_type must be circle|capsule|polygon, "
        f"got {geometry_type!r}")


def load_all(get_param, names: List[str]) -> List[ObjectConfig]:
    """Load every named object. Raises ConfigError on the first bad one —
    fail at startup, not silently at runtime (see ConfigError docstring).
    """
    return [load_object(get_param, name) for name in names]


# -- transform: base_link -> lidar frame ----------------------------------


def _quat_to_yaw_and_z_axis_tilt_ok(q: Quat) -> bool:
    """Cheap sanity check, not a hard requirement: warn-worthy if the lidar
    mount has meaningful roll/pitch, since transform_point below treats
    self-geometry as if a purely vertical pillar's cross-section is
    invariant to the transform's rotation — true for a pure yaw rotation,
    only approximately true otherwise. See README 'Known limitations'.
    """
    x, y, _z, w = q
    # roll/pitch magnitude via the quaternion's x/y components; a pure yaw
    # rotation has x == y == 0.
    return (x * x + y * y) < 1e-4


def transform_point(p: Vec3, translation: Vec3, rotation: Quat) -> Vec3:
    """Rotate then translate a point by a TF (translation + quaternion)."""
    x, y, z = p
    qx, qy, qz, qw = rotation
    # Standard quaternion-rotate-vector: v' = v + 2*qw*(q_xyz x v) +
    # 2*(q_xyz x (q_xyz x v))
    ux, uy, uz = qx, qy, qz
    cx1, cy1, cz1 = (uy * z - uz * y, uz * x - ux * z, ux * y - uy * x)
    cx2, cy2, cz2 = (uy * cz1 - uz * cy1, uz * cx1 - ux * cz1, ux * cy1 - uy * cx1)
    rx = x + 2.0 * qw * cx1 + 2.0 * cx2
    ry = y + 2.0 * qw * cy1 + 2.0 * cy2
    rz = z + 2.0 * qw * cz1 + 2.0 * cz2
    tx, ty, tz = translation
    return (rx + tx, ry + ty, rz + tz)


def transform_object(cfg: ObjectConfig, translation: Vec3, rotation: Quat
                     ) -> Optional["geo.Circle | geo.Capsule | geo.Polygon"]:
    """Transform one configured object from base_link into the lidar frame.

    Returns None for a disabled object — callers should simply skip it,
    same as if it weren't configured at all.

    Cylinders/capsules: only x/y of each transformed endpoint is used, per
    geometry.py's module docstring — correct for a vertical post crossing
    the lidar's scan plane; see README for the tilt caveat.
    """
    if not cfg.enabled:
        return None

    if isinstance(cfg, CircleConfig):
        cx, cy, _cz = transform_point((cfg.x, cfg.y, cfg.z), translation, rotation)
        return geo.Circle(name=cfg.name, cx=cx, cy=cy, radius=cfg.radius)

    if isinstance(cfg, CapsuleConfig):
        ax, ay, _az = transform_point((cfg.x1, cfg.y1, cfg.z1), translation, rotation)
        bx, by, _bz = transform_point((cfg.x2, cfg.y2, cfg.z2), translation, rotation)
        return geo.Capsule(name=cfg.name, ax=ax, ay=ay, bx=bx, by=by,
                           radius=cfg.radius)

    if isinstance(cfg, PolygonConfig):
        cos_o, sin_o = math.cos(cfg.orientation), math.sin(cfg.orientation)
        placed = []
        for (lx, ly) in cfg.points:
            # rotate the local template point, then place it at (x, y, z)
            px = lx * cos_o - ly * sin_o + cfg.x
            py = lx * sin_o + ly * cos_o + cfg.y
            wx, wy, _wz = transform_point((px, py, cfg.z), translation, rotation)
            placed.append((wx, wy))
        return geo.Polygon(name=cfg.name, points=placed)

    raise ConfigError(f"unknown object config type: {type(cfg)!r}")
