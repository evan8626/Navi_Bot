#!/usr/bin/env python3
"""
Test script for the geometry utilities in navi_bot/utils/geometry.py

Tests that the geometry helpers correctly satisfy mathematical invariants:
- Frame transformations (forward and inverse round-trip)
- Distance and angle calculations
- Point/line/circle/polygon collision primitives
- Path interpolation

Several of these are acceptance tests: they pin down the *intended*
behavior of functions that currently contain latent bugs, and will pass
once those functions are fixed.
"""
import logging
import math
import sys

from navi_bot.utils.geometry import (
    transform_point,
    inverse_transform_point,
    distance,
    angle_between_points,
    normalize_angle,
    point_to_line_distance,
    circle_line_collision,
    circle_circle_collision,
    point_in_polygon,
    interpolate_path,
)

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)


# MARK: Transformations

def test_transform_identity():
    """transform_point with an identity pose must return the point unchanged."""
    passed = True
    logger.info("TEST 1: transform_point identity pose")
    cases = [(0.0, 0.0), (1.0, 2.0), (-3.5, 4.25)]
    for point in cases:
        tx, ty = transform_point(point, (0.0, 0.0, 0.0))
        if not (math.isclose(tx, point[0], abs_tol=1e-9) and math.isclose(ty, point[1], abs_tol=1e-9)):
            logger.warning(f"  FAIL point={point}: got ({tx}, {ty})")
            passed = False
        else:
            logger.info(f"  OK   point={point} unchanged")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_transform_rotation_translation():
    """transform_point must rotate then translate: (1,0) through (2,3,pi/2) -> (2,4)."""
    passed = True
    logger.info("TEST 2: transform_point rotation + translation")
    cases = [
        ((1.0, 0.0), (2.0, 3.0, math.pi / 2), (2.0, 4.0)),
        ((0.0, 1.0), (0.0, 0.0, math.pi / 2), (-1.0, 0.0)),
        ((1.0, 1.0), (1.0, 1.0, 0.0), (2.0, 2.0)),
        ((2.0, 0.0), (0.0, 0.0, math.pi), (-2.0, 0.0)),
    ]
    for point, pose, expected in cases:
        tx, ty = transform_point(point, pose)
        if not (math.isclose(tx, expected[0], abs_tol=1e-9) and math.isclose(ty, expected[1], abs_tol=1e-9)):
            logger.warning(f"  FAIL point={point}, pose={pose}: got ({tx:.4f}, {ty:.4f}), expected {expected}")
            passed = False
        else:
            logger.info(f"  OK   point={point}, pose={pose} -> {expected}")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_inverse_transform_roundtrip():
    """inverse_transform_point(transform_point(p, pose), pose) must return p."""
    passed = True
    logger.info("TEST 3: inverse_transform_point round-trip")
    poses = [(0.0, 0.0, 0.0), (2.0, -1.0, math.pi / 3), (-4.0, 0.5, -2.5)]
    points = [(1.0, 0.0), (-2.0, 3.0), (0.0, 0.0)]
    try:
        for pose in poses:
            for point in points:
                forward = transform_point(point, pose)
                back = inverse_transform_point(forward, pose)
                if not (math.isclose(back[0], point[0], abs_tol=1e-9) and math.isclose(back[1], point[1], abs_tol=1e-9)):
                    logger.warning(f"  FAIL point={point}, pose={pose}: round-trip gave {back}")
                    passed = False
                else:
                    logger.info(f"  OK   point={point}, pose={pose} round-trips")
    except Exception as e:
        logger.error(f"  inverse_transform_point raised {type(e).__name__}: {e}")
        passed = False
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: Distance / Angles

def test_distance():
    """distance must compute Euclidean distance (3-4-5 triangle, zero case)."""
    passed = True
    logger.info("TEST 4: distance Euclidean invariants")
    cases = [((0, 0), (3, 4), 5.0), ((1, 1), (1, 1), 0.0), ((-1, -1), (1, 1), 2 * math.sqrt(2))]
    for p1, p2, expected in cases:
        d = distance(p1, p2)
        if not math.isclose(d, expected, abs_tol=1e-9):
            logger.warning(f"  FAIL {p1}->{p2}: got {d}, expected {expected}")
            passed = False
        else:
            logger.info(f"  OK   {p1}->{p2} = {expected}")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_angle_between_points():
    """angle_between_points must return atan2 bearing from p1 to p2."""
    passed = True
    logger.info("TEST 5: angle_between_points cardinal and diagonal bearings")
    cases = [
        ((0, 0), (1, 0), 0.0),
        ((0, 0), (0, 1), math.pi / 2),
        ((0, 0), (-1, 0), math.pi),
        ((0, 0), (1, 1), math.pi / 4),
        ((2, 2), (2, 1), -math.pi / 2),
    ]
    for p1, p2, expected in cases:
        a = angle_between_points(p1, p2)
        if not math.isclose(a, expected, abs_tol=1e-9):
            logger.warning(f"  FAIL {p1}->{p2}: got {a}, expected {expected}")
            passed = False
        else:
            logger.info(f"  OK   {p1}->{p2} = {expected:.4f} rad")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_normalize_angle():
    """normalize_angle must map any angle into [-pi, pi] preserving direction."""
    passed = True
    logger.info("TEST 6: normalize_angle wraps into [-pi, pi]")
    cases = [(0.0, 0.0), (2 * math.pi, 0.0), (3 * math.pi, math.pi),
             (-3 * math.pi, -math.pi), (math.pi / 2, math.pi / 2), (-9 * math.pi / 4, -math.pi / 4)]
    for angle, expected in cases:
        n = normalize_angle(angle)
        if not (-math.pi - 1e-9 <= n <= math.pi + 1e-9):
            logger.warning(f"  FAIL {angle}: result {n} outside [-pi, pi]")
            passed = False
        elif not math.isclose(n, expected, abs_tol=1e-9):
            logger.warning(f"  FAIL {angle}: got {n}, expected {expected}")
            passed = False
        else:
            logger.info(f"  OK   {angle:.4f} -> {n:.4f}")
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: Collision Primitives

def test_point_to_line_perpendicular():
    """point_to_line_distance must return perpendicular distance to the segment."""
    passed = True
    logger.info("TEST 7: point_to_line_distance perpendicular distance")
    try:
        cases = [
            ((0, 1), (-1, 0), (1, 0), 1.0),    # directly above the middle
            ((0.5, 2), (0, 0), (1, 0), 2.0),   # above, inside segment span
            ((0, 0), (0, -1), (0, 1), 0.0),    # on the segment
        ]
        for point, a, b, expected in cases:
            d = point_to_line_distance(point, a, b)
            if not math.isclose(d, expected, abs_tol=1e-9):
                logger.warning(f"  FAIL point={point}, seg={a}-{b}: got {d}, expected {expected}")
                passed = False
            else:
                logger.info(f"  OK   point={point}, seg={a}-{b} = {expected}")
    except Exception as e:
        logger.error(f"  point_to_line_distance raised {type(e).__name__}: {e}")
        passed = False
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_point_to_line_endpoint_clamp():
    """Distance beyond a segment's end must clamp to the nearest endpoint."""
    passed = True
    logger.info("TEST 8: point_to_line_distance endpoint clamping and degenerate segment")
    try:
        cases = [
            ((3, 0), (0, 0), (1, 0), 2.0),       # beyond the far end -> distance to (1,0)
            ((-2, 0), (0, 0), (1, 0), 2.0),      # before the near end -> distance to (0,0)
            ((1, 1), (0, 0), (0, 0), math.sqrt(2)),  # zero-length segment -> point distance
        ]
        for point, a, b, expected in cases:
            d = point_to_line_distance(point, a, b)
            if not math.isclose(d, expected, abs_tol=1e-9):
                logger.warning(f"  FAIL point={point}, seg={a}-{b}: got {d}, expected {expected}")
                passed = False
            else:
                logger.info(f"  OK   point={point}, seg={a}-{b} = {expected:.4f}")
    except Exception as e:
        logger.error(f"  point_to_line_distance raised {type(e).__name__}: {e}")
        passed = False
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_circle_line_collision():
    """circle_line_collision must detect touching/overlapping, reject distant."""
    passed = True
    logger.info("TEST 9: circle_line_collision detection")
    try:
        cases = [
            ((0, 1), 1.0, (-2, 0), (2, 0), True),    # circle touches the line
            ((0, 3), 1.0, (-2, 0), (2, 0), False),   # circle well above the line
            ((0, 0), 0.5, (-1, 0), (1, 0), True),    # center on the line
        ]
        for center, r, a, b, expected in cases:
            hit = circle_line_collision(center, r, a, b)
            if hit != expected:
                logger.warning(f"  FAIL center={center}, r={r}, seg={a}-{b}: got {hit}, expected {expected}")
                passed = False
            else:
                logger.info(f"  OK   center={center}, r={r}, seg={a}-{b} -> {expected}")
    except Exception as e:
        logger.error(f"  circle_line_collision raised {type(e).__name__}: {e}")
        passed = False
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_circle_circle_collision():
    """circle_circle_collision must detect overlap and tangency, reject separation."""
    passed = True
    logger.info("TEST 10: circle_circle_collision detection")
    cases = [
        ((0, 0), 1.0, (1, 0), 1.0, True),    # overlapping
        ((0, 0), 1.0, (2, 0), 1.0, True),    # exactly tangent
        ((0, 0), 1.0, (5, 0), 1.0, False),   # separated
    ]
    for c1, r1, c2, r2, expected in cases:
        hit = circle_circle_collision(c1, r1, c2, r2)
        if hit != expected:
            logger.warning(f"  FAIL c1={c1} r1={r1}, c2={c2} r2={r2}: got {hit}, expected {expected}")
            passed = False
        else:
            logger.info(f"  OK   c1={c1}, c2={c2} -> {expected}")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_point_in_polygon():
    """point_in_polygon must classify inside/outside for a unit square."""
    passed = True
    logger.info("TEST 11: point_in_polygon unit square classification")
    square = [(0, 0), (1, 0), (1, 1), (0, 1)]
    try:
        cases = [((0.5, 0.5), True), ((1.5, 0.5), False), ((-0.5, 0.5), False), ((0.25, 0.75), True)]
        for point, expected in cases:
            inside = point_in_polygon(point, square)
            if inside != expected:
                logger.warning(f"  FAIL point={point}: got {inside}, expected {expected}")
                passed = False
            else:
                logger.info(f"  OK   point={point} -> {expected}")
    except Exception as e:
        logger.error(f"  point_in_polygon raised {type(e).__name__}: {e}")
        passed = False
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: Path Interpolation

def test_interpolate_path_spacing():
    """interpolate_path must keep endpoints and space points at most `resolution` apart."""
    passed = True
    logger.info("TEST 12: interpolate_path endpoint preservation and spacing")
    try:
        waypoints = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]
        resolution = 0.1
        result = interpolate_path(waypoints, resolution)
        if result[0] != waypoints[0] or result[-1] != waypoints[-1]:
            logger.warning(f"  FAIL endpoints not preserved: {result[0]} ... {result[-1]}")
            passed = False
        for i in range(1, len(result)):
            gap = distance(result[i - 1], result[i])
            if gap > resolution + 1e-6:
                logger.warning(f"  FAIL gap {gap:.4f} between {result[i-1]} and {result[i]} exceeds resolution")
                passed = False
                break
        if len(result) < len(waypoints):
            logger.warning(f"  FAIL interpolation returned fewer points ({len(result)}) than input")
            passed = False
        if passed:
            logger.info(f"  OK   {len(waypoints)} waypoints -> {len(result)} points, max gap <= {resolution}")
    except Exception as e:
        logger.error(f"  interpolate_path raised {type(e).__name__}: {e}")
        passed = False
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_interpolate_path_short_input():
    """interpolate_path must return short inputs (0 or 1 waypoints) unchanged."""
    passed = True
    logger.info("TEST 13: interpolate_path short input passthrough")
    for waypoints in ([], [(2.0, 3.0)]):
        result = interpolate_path(waypoints, 0.1)
        if result != waypoints:
            logger.warning(f"  FAIL input {waypoints}: got {result}")
            passed = False
        else:
            logger.info(f"  OK   input {waypoints} returned unchanged")
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: Main Method

def main():
    tests = [
        test_transform_identity,
        test_transform_rotation_translation,
        test_inverse_transform_roundtrip,
        test_distance,
        test_angle_between_points,
        test_normalize_angle,
        test_point_to_line_perpendicular,
        test_point_to_line_endpoint_clamp,
        test_circle_line_collision,
        test_circle_circle_collision,
        test_point_in_polygon,
        test_interpolate_path_spacing,
        test_interpolate_path_short_input,
    ]

    args = sys.argv[1:]
    if args and args[0] == '--list':
        for i, t in enumerate(tests, 1):
            doc = (t.__doc__ or t.__name__).strip().splitlines()[0]
            print(f"TEST {i}: {doc}")
        return

    selected = tests
    if args:
        try:
            n = int(args[0])
            if not 1 <= n <= len(tests):
                raise ValueError
        except ValueError:
            logger.error(f"Invalid test selector {args[0]!r} — use 1..{len(tests)} or --list")
            sys.exit(2)
        selected = [tests[n - 1]]

    logger.info("Geometry Utilities Test Suite")
    results = [t() for t in selected]

    logger.info(f"Results: {sum(results)}/{len(results)} passed")
    logger.info("All tests complete.")
    sys.exit(0 if all(results) else 1)


if __name__ == '__main__':
    main()
