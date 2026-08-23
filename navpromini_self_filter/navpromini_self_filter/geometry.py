#!/usr/bin/env python3
"""Ray-vs-shape intersection math for the self-filter, plus the SelfObject
types built on it.

Everything here is evaluated in the LIDAR's own frame. A LaserScan beam at
angle theta is a ray from the origin (0, 0) in direction
(cos theta, sin theta) — range is measured FROM the sensor, so the ray
always starts at the sensor. Self-object geometry is configured in
base_link and transformed into this frame once per TF update
(self_filter_node.py), not per beam.

Deliberately free of numpy. Each shape test below is a handful of
multiplies — cheap enough that the real cost driver is how many BEAMS get
tested per object, not the arithmetic itself. self_filter_node.py exploits
that: it computes each object's angular extent once and only tests the
beams that actually fall inside it (a few, for something the size of an
8mm pillar), instead of every beam in the scan. See its module docstring.

Safety contract every function here upholds: a return of None means "this
ray does not hit this shape, or the shape does not conclusively explain
the beam" — the caller is expected to KEEP the point in that case. Nothing
in this module ever removes a point; it only ever answers "if this shape
occluded this beam, what range would it read?", and it errs toward None
(not intersecting) at every ambiguous edge case rather than guessing.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

_EPS = 1e-9


# -- raw ray/shape tests ------------------------------------------------------


def ray_circle(ox: float, oy: float, dx: float, dy: float,
                cx: float, cy: float, r: float) -> Optional[float]:
    """Nearest intersection of ray (o, d), d unit, with circle (c, r).

    Returns the ray parameter t >= 0 (== range, since d is unit), or None
    if the ray misses the circle or only intersects it "behind" the sensor.
    """
    fx, fy = ox - cx, oy - cy
    b = fx * dx + fy * dy
    c = fx * fx + fy * fy - r * r
    disc = b * b - c  # a == 1 since d is unit length
    if disc < 0.0:
        return None
    sq = math.sqrt(disc)
    t1, t2 = -b - sq, -b + sq
    if t1 >= _EPS:
        return t1
    if t2 >= _EPS:
        return t2
    return None


def ray_segment(ox: float, oy: float, dx: float, dy: float,
                 ax: float, ay: float, bx: float, by: float) -> Optional[float]:
    """Nearest intersection of ray (o, d) with the finite segment a->b."""
    ex, ey = bx - ax, by - ay
    denom = dx * ey - dy * ex
    if abs(denom) < _EPS:
        return None  # parallel (including the segment being zero-length)
    fx, fy = ax - ox, ay - oy
    t = (fx * ey - fy * ex) / denom
    u = (fx * dy - fy * dx) / denom
    if t >= _EPS and -_EPS <= u <= 1.0 + _EPS:
        return t
    return None


def ray_capsule(ox: float, oy: float, dx: float, dy: float,
                 ax: float, ay: float, bx: float, by: float,
                 r: float) -> Optional[float]:
    """Nearest intersection with a capsule: segment a->b thickened by r.

    A capsule is the Minkowski sum of a segment and a disc of radius r —
    the right shape for a wire/cable (thin) or, with a==b, exactly a
    circle (a pillar's actual cross-section). Modelled as the nearer of:
    the two side edges (the segment offset by +-r along its normal,
    restricted to the segment's own length) and the two end caps (circles
    of radius r at a and b, which also correctly cover a==b).
    """
    ex, ey = bx - ax, by - ay
    seg_len = math.hypot(ex, ey)
    best: Optional[float] = None
    if seg_len > _EPS:
        ux, uy = ex / seg_len, ey / seg_len
        nx, ny = -uy, ux  # left normal, unit
        for s in (1.0, -1.0):
            ax2, ay2 = ax + nx * r * s, ay + ny * r * s
            t = ray_segment(ox, oy, dx, dy, ax2, ay2, ax2 + ex, ay2 + ey)
            if t is not None and (best is None or t < best):
                best = t
    for (cx, cy) in ((ax, ay), (bx, by)):
        t = ray_circle(ox, oy, dx, dy, cx, cy, r)
        if t is not None and (best is None or t < best):
            best = t
    return best


def ray_polygon(ox: float, oy: float, dx: float, dy: float,
                 points: List[Tuple[float, float]]) -> Optional[float]:
    """Nearest intersection with a closed polygon's outline (edges only).

    Tests every edge and keeps the nearest hit — correct for convex and
    non-convex outlines alike, since a ray from outside a closed shape
    always meets its boundary at the shape's actual silhouette first.
    """
    best: Optional[float] = None
    n = len(points)
    if n < 2:
        return None
    for i in range(n):
        ax, ay = points[i]
        bx, by = points[(i + 1) % n]
        t = ray_segment(ox, oy, dx, dy, ax, ay, bx, by)
        if t is not None and (best is None or t < best):
            best = t
    return best


def _norm_angle(a: float) -> float:
    """Wrap to (-pi, pi]."""
    a = math.fmod(a + math.pi, 2.0 * math.pi)
    if a <= 0.0:
        a += 2.0 * math.pi
    return a - math.pi


def _angular_span(bearings: List[float], margin: float) -> Tuple[float, float]:
    """Smallest angular window (with margin) covering every given bearing.

    Handles the +-pi wraparound by trying every bearing as the window's
    start and keeping the tightest span that contains them all — O(n^2)
    but n is at most a handful of corner points per object, once per TF
    update, not per beam.
    """
    if not bearings:
        return (0.0, 0.0)
    bs = [_norm_angle(b) for b in bearings]
    if len(bs) == 1:
        return (bs[0] - margin, bs[0] + margin)
    best_span = None
    best_start = None
    for start in bs:
        # offsets of every bearing relative to `start`, folded into [0, 2pi)
        offsets = [math.fmod(_norm_angle(b - start) + 2 * math.pi, 2 * math.pi)
                   for b in bs]
        span = max(offsets)
        if best_span is None or span < best_span:
            best_span = span
            best_start = start
    lo = best_start - margin
    hi = best_start + best_span + margin
    return (lo, hi)


def angle_in_window(theta: float, lo: float, hi: float) -> bool:
    """True if theta falls in [lo, hi], correctly across the +-pi seam."""
    # Shift everything so the window starts at 0, compare in that space —
    # avoids re-deriving the wraparound logic at every call site.
    span = hi - lo
    off = math.fmod((theta - lo) + 4 * math.pi, 2 * math.pi)
    return 0.0 <= off <= span


# -- self-object types ---------------------------------------------------


@dataclass
class Circle:
    """A round post — the expected shape for the 8mm pallet pillars."""
    name: str
    cx: float
    cy: float
    radius: float

    def expected_range(self, theta: float) -> Optional[float]:
        return ray_circle(0.0, 0.0, math.cos(theta), math.sin(theta),
                          self.cx, self.cy, self.radius)

    def angular_extent(self, margin: float) -> Tuple[float, float]:
        dist = math.hypot(self.cx, self.cy)
        bearing = math.atan2(self.cy, self.cx)
        if dist <= self.radius:
            # Sensor origin is inside/on the circle — degenerate for a real
            # rigid mount, but cover it rather than crash: full circle.
            return (-math.pi, math.pi)
        half = math.asin(min(1.0, self.radius / dist))
        return (bearing - half - margin, bearing + half + margin)


@dataclass
class Capsule:
    """A thin wire/cable, or any segment-shaped object with thickness."""
    name: str
    ax: float
    ay: float
    bx: float
    by: float
    radius: float

    def expected_range(self, theta: float) -> Optional[float]:
        return ray_capsule(0.0, 0.0, math.cos(theta), math.sin(theta),
                           self.ax, self.ay, self.bx, self.by, self.radius)

    def angular_extent(self, margin: float) -> Tuple[float, float]:
        # The capsule's silhouette from the origin is bounded by bearings to
        # its two offset "corner" points at each end, plus each endpoint's
        # own circular cap — using the four offset corners is a safe
        # (non-tight but never-too-tight) cover of both.
        ex, ey = self.bx - self.ax, self.by - self.ay
        seg_len = math.hypot(ex, ey)
        bearings = []
        if seg_len > _EPS:
            ux, uy = ex / seg_len, ey / seg_len
            nx, ny = -uy, ux
            for (px, py) in ((self.ax, self.ay), (self.bx, self.by)):
                for s in (1.0, -1.0):
                    cx, cy = px + nx * self.radius * s, py + ny * self.radius * s
                    bearings.append(math.atan2(cy, cx))
        else:
            bearings.append(math.atan2(self.ay, self.ax))
        # Also fold in each endpoint's own circle extent, in case a corner
        # point ends up closer to the origin than the endpoint itself makes
        # the corner-only bound too tight (near-degenerate geometry).
        for (px, py) in ((self.ax, self.ay), (self.bx, self.by)):
            dist = math.hypot(px, py)
            if dist > self.radius:
                bearing = math.atan2(py, px)
                half = math.asin(min(1.0, self.radius / max(dist, _EPS)))
                bearings.append(bearing - half)
                bearings.append(bearing + half)
        return _angular_span(bearings, margin)


@dataclass
class Polygon:
    """An arbitrary outline — brackets, non-round pillar cross-sections,
    or any future flat/solid self-structure. Points are absolute, already
    placed and rotated (self_filter_node.py applies x/y/orientation to the
    configured local template before constructing this).
    """
    name: str
    points: List[Tuple[float, float]]

    def expected_range(self, theta: float) -> Optional[float]:
        return ray_polygon(0.0, 0.0, math.cos(theta), math.sin(theta), self.points)

    def angular_extent(self, margin: float) -> Tuple[float, float]:
        bearings = [math.atan2(y, x) for (x, y) in self.points]
        return _angular_span(bearings, margin)


SelfObject = "Circle | Capsule | Polygon"
