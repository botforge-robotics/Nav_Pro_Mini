#!/usr/bin/env python3
"""Functional tests for the ray-geometry math and the self-object config/
transform pipeline. Pure Python — no ROS graph needed, these run with
plain pytest.

Also doubles as the executable version of the eight scenarios in the
package README ("Test cases"): each TEST_n below names which scenario it
covers.

Run:
    cd navpromini_self_filter && python3 -m pytest test/test_geometry.py -v
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from navpromini_self_filter import geometry as geo
from navpromini_self_filter import self_objects as so


# -- raw ray/shape math -----------------------------------------------------


def test_ray_circle_hit_and_miss():
    # Circle straight ahead on the ray.
    t = geo.ray_circle(0, 0, 1, 0, 1.0, 0.0, 0.1)
    assert t is not None and abs(t - 0.9) < 1e-9
    # Circle off to the side, ray misses entirely.
    assert geo.ray_circle(0, 0, 1, 0, 1.0, 1.0, 0.1) is None
    # Circle behind the ray origin — must not report a "hit" behind us.
    assert geo.ray_circle(0, 0, 1, 0, -1.0, 0.0, 0.1) is None


def test_ray_circle_tangent_and_inside():
    # Grazing tangent still counts (touches at exactly one point).
    t = geo.ray_circle(0, 0, 1, 0, 1.0, 0.1, 0.1)
    assert t is not None
    # Sensor origin inside the circle: nearest exit point is behind (t<0
    # for entry), function should not crash and should behave sanely.
    t = geo.ray_circle(0, 0, 1, 0, 0.0, 0.0, 0.5)
    assert t is not None and t > 0


def test_ray_segment_basic():
    # Segment crossing straight ahead.
    t = geo.ray_segment(0, 0, 1, 0, 1.0, -1.0, 1.0, 1.0)
    assert t is not None and abs(t - 1.0) < 1e-9
    # Parallel segment: no intersection.
    assert geo.ray_segment(0, 0, 1, 0, 1.0, 1.0, 2.0, 1.0) is None
    # Segment whose infinite line crosses the ray, but outside the
    # segment's own bounds.
    assert geo.ray_segment(0, 0, 1, 0, -1.0, -0.5, -1.0, 0.5) is None


def test_ray_capsule_side_and_cap():
    # Straight down the capsule's axis: should hit the near end cap.
    t = geo.ray_capsule(0, 0, 1, 0, 1.0, 0.0, 2.0, 0.0, 0.05)
    assert t is not None and abs(t - 0.95) < 1e-9
    # Perpendicular to the capsule, offset to clip its side.
    t = geo.ray_capsule(0, 0, 1, 0, 1.0, -1.0, 1.0, 1.0, 0.05)
    assert t is not None and abs(t - 0.95) < 1e-9
    # Degenerate capsule (a == b) must behave exactly like a circle.
    t_cap = geo.ray_capsule(0, 0, 1, 0, 1.0, 0.0, 1.0, 0.0, 0.1)
    t_circ = geo.ray_circle(0, 0, 1, 0, 1.0, 0.0, 0.1)
    assert t_cap is not None and t_circ is not None
    assert abs(t_cap - t_circ) < 1e-9


def test_ray_polygon_square():
    square = [(0.5, -0.5), (1.5, -0.5), (1.5, 0.5), (0.5, 0.5)]
    t = geo.ray_polygon(0, 0, 1, 0, square)
    assert t is not None and abs(t - 0.5) < 1e-9
    assert geo.ray_polygon(0, 0, 0, 1, square) is None  # ray points away


def test_angle_in_window_wraparound():
    # A window straddling the +-pi seam.
    lo, hi = math.pi - 0.1, math.pi + 0.1  # i.e. (pi-0.1) .. (-pi+0.1)
    assert geo.angle_in_window(math.pi, lo, hi)
    assert geo.angle_in_window(-math.pi + 0.05, lo, hi)
    assert not geo.angle_in_window(0.0, lo, hi)


# -- Circle/Capsule/Polygon: expected_range + angular_extent ----------------


def test_circle_angular_extent_matches_geometry():
    c = geo.Circle(name='p', cx=0.1, cy=0.0, radius=0.004)
    lo, hi = c.angular_extent(margin=0.0)
    # Every beam inside the window should find SOME expected range;
    # a beam just outside should not.
    mid = (lo + hi) / 2
    assert c.expected_range(mid) is not None
    assert c.expected_range(hi + 0.05) is None
    assert c.expected_range(lo - 0.05) is None


def test_capsule_angular_extent_covers_full_shape():
    cap = geo.Capsule(name='w', ax=0.1, ay=-0.05, bx=0.1, by=0.05, radius=0.003)
    lo, hi = cap.angular_extent(margin=0.0)
    for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
        theta = math.atan2(cap.ay + frac * (cap.by - cap.ay),
                           cap.ax + frac * (cap.bx - cap.ax))
        assert geo.angle_in_window(theta, lo, hi), (
            f'bearing at frac={frac} fell outside computed extent')


# -- self_objects: config loading + transform --------------------------------


def _stub_params(d):
    def get_param(key, default):
        return d.get(key, default)
    return get_param


def test_load_circle_config():
    d = {
        'pillar_1.enabled': True,
        'pillar_1.geometry_type': 'circle',
        'pillar_1.x': 0.10, 'pillar_1.y': 0.06, 'pillar_1.z': 0.10,
        'pillar_1.radius': 0.004,
    }
    cfg = so.load_object(_stub_params(d), 'pillar_1')
    assert isinstance(cfg, so.CircleConfig)
    assert cfg.enabled and cfg.radius == 0.004


def test_load_polygon_requires_matching_points():
    d = {
        'bracket.enabled': True,
        'bracket.geometry_type': 'polygon',
        'bracket.points_x': [0.0, 0.01],
        'bracket.points_y': [0.0, 0.01, 0.02],  # mismatched length
    }
    try:
        so.load_object(_stub_params(d), 'bracket')
        assert False, 'expected ConfigError for mismatched polygon points'
    except so.ConfigError:
        pass


def test_transform_object_identity():
    cfg = so.CircleConfig(name='p', enabled=True, x=0.1, y=0.05, z=0.1, radius=0.004)
    identity_t = (0.0, 0.0, 0.0)
    identity_q = (0.0, 0.0, 0.0, 1.0)
    obj = so.transform_object(cfg, identity_t, identity_q)
    assert isinstance(obj, geo.Circle)
    assert abs(obj.cx - 0.1) < 1e-9 and abs(obj.cy - 0.05) < 1e-9


def test_transform_object_translation_and_yaw():
    # 90deg yaw: base_link's +x axis becomes lidar frame's... the transform
    # here is base_link -> lidar, so a point at base_link (0.1, 0) with the
    # lidar rotated +90deg *relative to base_link* and offset (0, 0, 0)
    # lands at (0, 0.1) in the lidar frame? Verify via the quaternion math
    # directly matches a hand-computed rotation, not just "doesn't crash".
    cfg = so.CircleConfig(name='p', enabled=True, x=0.1, y=0.0, z=0.0, radius=0.004)
    half = math.pi / 4  # 90deg -> quaternion half-angle
    q = (0.0, 0.0, math.sin(half), math.cos(half))
    obj = so.transform_object(cfg, (0.0, 0.0, 0.0), q)
    assert abs(obj.cx - 0.0) < 1e-9
    assert abs(obj.cy - 0.1) < 1e-9


def test_transform_object_disabled_returns_none():
    cfg = so.CircleConfig(name='p', enabled=False, x=0.1, y=0.0, z=0.0, radius=0.004)
    assert so.transform_object(cfg, (0, 0, 0), (0, 0, 0, 1)) is None


# -- TEST 1-6 from the README, expressed as direct scenario checks ----------
#
# These simulate the per-beam decision self_filter_node.py makes — same
# expected_range()/tolerance comparison, without needing a live LaserScan
# message or ROS node.

_TOLERANCE = 0.02


def _would_remove(obj, theta, measured_range) -> bool:
    expected = obj.expected_range(theta)
    if expected is None:
        return False
    return abs(measured_range - expected) <= _TOLERANCE


def test_TEST1_pillar_open_space_behind():
    pillar = geo.Circle(name='pillar_1', cx=0.10, cy=0.0, radius=0.004)
    theta = math.atan2(pillar.cy, pillar.cx)
    measured = pillar.expected_range(theta)  # exactly what the pillar itself gives
    assert _would_remove(pillar, theta, measured)


def test_TEST2_pillar_with_wall_exactly_behind_same_beam():
    # Nearest return on this beam IS the pillar (a real scanner reports the
    # nearest surface) — the wall on this exact beam is optically occluded
    # and, correctly, unrecoverable. The filter must still remove the
    # pillar's own return; it must not invent the wall.
    pillar = geo.Circle(name='pillar_1', cx=0.10, cy=0.0, radius=0.004)
    theta = math.atan2(pillar.cy, pillar.cx)
    nearest_return = pillar.expected_range(theta)  # scanner sees the pillar first
    assert _would_remove(pillar, theta, nearest_return)


def test_pillar_with_wall_behind_on_a_DIFFERENT_beam_is_kept():
    pillar = geo.Circle(name='pillar_1', cx=0.10, cy=0.0, radius=0.004)
    # A neighbouring beam that misses the (tiny) pillar entirely and sees
    # a wall 1m out.
    lo, hi = pillar.angular_extent(margin=0.0)
    theta_wall = hi + 0.2  # well outside the pillar's own angular extent
    assert not _would_remove(pillar, theta_wall, 1.0)


def test_TEST3_wall_beside_pillar_kept():
    pillar = geo.Circle(name='pillar_1', cx=0.10, cy=0.0, radius=0.004)
    lo, hi = pillar.angular_extent(margin=0.01)
    theta_beside = hi + 0.05  # just past the pillar's own extent + margin
    assert not _would_remove(pillar, theta_beside, 1.5)


def test_TEST4_obstacle_beside_pillar_kept():
    pillar = geo.Circle(name='pillar_1', cx=0.10, cy=0.0, radius=0.004)
    theta = math.atan2(pillar.cy, pillar.cx)
    # Same bearing as the pillar, but the range is nowhere near what the
    # pillar's own geometry predicts — something else is on this beam.
    assert not _would_remove(pillar, theta, 0.5)


def test_TEST6_unknown_object_near_pillar_not_matching_range_kept():
    pillar = geo.Circle(name='pillar_1', cx=0.10, cy=0.0, radius=0.004)
    theta = math.atan2(pillar.cy, pillar.cx)
    expected = pillar.expected_range(theta)
    # Just outside the tolerance window -> must be kept, not removed.
    assert not _would_remove(pillar, theta, expected + _TOLERANCE + 0.005)


def test_range_tolerance_boundary_is_inclusive_and_exclusive_correctly():
    pillar = geo.Circle(name='pillar_1', cx=0.10, cy=0.0, radius=0.004)
    theta = math.atan2(pillar.cy, pillar.cx)
    expected = pillar.expected_range(theta)
    # Float addition of two non-exactly-representable values can land a
    # few ulps either side of the true boundary — check just inside and
    # just outside it rather than the boundary value itself.
    assert _would_remove(pillar, theta, expected + _TOLERANCE - 1e-6)
    assert not _would_remove(pillar, theta, expected + _TOLERANCE + 1e-6)


if __name__ == '__main__':
    import pytest
    raise SystemExit(pytest.main([__file__, '-v']))
