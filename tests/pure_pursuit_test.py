#!/usr/bin/env python3
"""
Test script for PurePursuitController from pure_pursuit.py

Tests that the Pure Pursuit controller correctly satisfies physical and
geometric invariants for:
- Initialization (parameter clamping)
- find_next_lookahead (waypoint selection geometry)
- pure_pursuit (control output invariants)
"""
import logging
import math
import sys
import numpy as np

from navi_bot.control.pure_pursuit import PurePursuitController, MAX_LOOKAHEAD, MAX_LINEAR_VELOCITY

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)


# MARK: Setup

def setup_controller(lookahead=2.0, linear_velocity=1.0):
    return PurePursuitController(lookahead=lookahead, linear_velocity=linear_velocity)

def straight_path(length=10, step=1):
    """Horizontal path along the x-axis."""
    return [(float(i * step), 0.0) for i in range(length)]

def vertical_path(length=10, step=1):
    """Vertical path along the y-axis."""
    return [(0.0, float(i * step)) for i in range(length)]


# MARK: Initialization

def test_init_lookahead_within_limit():
    """
    TEST 1: Lookahead below MAX_LOOKAHEAD must be stored as-is.
    """
    passed = True
    logger.info("TEST 1: Init — lookahead below max must be stored unchanged")
    for ld in [0.5, 1.0, 2.0, MAX_LOOKAHEAD]:
        c = setup_controller(lookahead=ld)
        if not math.isclose(c.lookahead_distance, ld, abs_tol=1e-9):
            logger.warning(f"  FAIL ld={ld}: stored={c.lookahead_distance}")
            passed = False
        else:
            logger.info(f"  OK   ld={ld}: stored={c.lookahead_distance}")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_init_lookahead_clamped_to_max():
    """
    TEST 2: Lookahead above MAX_LOOKAHEAD must be clamped to MAX_LOOKAHEAD.
    """
    passed = True
    logger.info("TEST 2: Init — lookahead above max must be clamped to MAX_LOOKAHEAD")
    for ld in [MAX_LOOKAHEAD + 0.1, MAX_LOOKAHEAD * 2, 100.0]:
        c = setup_controller(lookahead=ld)
        if not math.isclose(c.lookahead_distance, MAX_LOOKAHEAD, abs_tol=1e-9):
            logger.warning(f"  FAIL ld={ld}: stored={c.lookahead_distance}, expected={MAX_LOOKAHEAD}")
            passed = False
        else:
            logger.info(f"  OK   ld={ld}: clamped to {c.lookahead_distance}")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_init_linear_velocity_within_limit():
    """
    TEST 3: Linear velocity below MAX_LINEAR_VELOCITY must be stored as-is.
    """
    passed = True
    logger.info("TEST 3: Init — linear velocity below max must be stored unchanged")
    for v in [0.1, 0.5, 1.0, MAX_LINEAR_VELOCITY]:
        c = setup_controller(linear_velocity=v)
        if not math.isclose(c.linear_velocity, v, abs_tol=1e-9):
            logger.warning(f"  FAIL v={v}: stored={c.linear_velocity}")
            passed = False
        else:
            logger.info(f"  OK   v={v}: stored={c.linear_velocity}")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_init_linear_velocity_clamped_to_max():
    """
    TEST 4: Linear velocity above MAX_LINEAR_VELOCITY must be clamped.
    """
    passed = True
    logger.info("TEST 4: Init — linear velocity above max must be clamped to MAX_LINEAR_VELOCITY")
    for v in [MAX_LINEAR_VELOCITY + 0.1, 5.0, 100.0]:
        c = setup_controller(linear_velocity=v)
        if not math.isclose(c.linear_velocity, MAX_LINEAR_VELOCITY, abs_tol=1e-9):
            logger.warning(f"  FAIL v={v}: stored={c.linear_velocity}, expected={MAX_LINEAR_VELOCITY}")
            passed = False
        else:
            logger.info(f"  OK   v={v}: clamped to {c.linear_velocity}")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_init_waypoint_starts_at_zero():
    """
    TEST 5: Waypoint index must start at zero on construction.
    """
    passed = False
    logger.info("TEST 5: Init — waypoint index must start at 0")
    c = setup_controller()
    if c.waypoint == 0:
        logger.info(f"  OK   waypoint={c.waypoint}")
        passed = True
    else:
        logger.warning(f"  FAIL waypoint={c.waypoint}, expected 0")
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: find_next_lookahead

def test_lookahead_point_is_at_least_lookahead_distance_away():
    """
    TEST 6: find_next_lookahead geometric invariant.
    The returned point must be at least lookahead_distance from current_pos.
    """
    passed = True
    logger.info("TEST 6: Lookahead — returned point must be >= lookahead_distance from robot")
    path = straight_path(length=20, step=1)
    current_pos = (0.0, 0.0)
    for ld in [1.0, 2.0, 3.0]:
        c = setup_controller(lookahead=ld)
        point = c.find_next_lookahead(path, current_pos)
        if point is None:
            logger.warning(f"  FAIL ld={ld}: returned None")
            passed = False
            continue
        dist = math.hypot(point[0] - current_pos[0], point[1] - current_pos[1])
        if dist < ld - 1e-9:
            logger.warning(f"  FAIL ld={ld}: returned point at dist={dist:.4f}, expected >= {ld}")
            passed = False
        else:
            logger.info(f"  OK   ld={ld}: point={point}, dist={dist:.4f}")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_lookahead_returns_last_point_when_waypoint_exhausted():
    """
    TEST 7: find_next_lookahead path exhaustion invariant.
    When waypoints are exhausted but the robot is still close to path[-1]
    (within the distance threshold), the last point must be returned.
    """
    passed = False
    logger.info("TEST 7: Lookahead — must return last path point when exhausted and robot is nearby")
    path = straight_path(length=5, step=1)  # last point is (4, 0)
    c = setup_controller(lookahead=2.0)
    c.waypoint = len(path)  # Force exhausted state
    # Robot is close to path[-1], well within the threshold, so path[-1] should be returned.
    current_pos = (3.5, 0.0)
    point = c.find_next_lookahead(path, current_pos)
    if point == path[-1]:
        logger.info(f"  OK   returned last point {point}")
        passed = True
    else:
        logger.warning(f"  FAIL returned {point}, expected {path[-1]}")
    logger.info("PASS" if passed else "FAIL")
    return passed



def test_lookahead_waypoint_index_advances():
    """
    TEST 8: find_next_lookahead waypoint advancement invariant.
    After a successful call, the waypoint index must be >= its prior value.
    The controller must never move backward along the path.
    """
    passed = True
    logger.info("TEST 8: Lookahead — waypoint index must never decrease between calls")
    path = straight_path(length=20, step=1)
    c = setup_controller(lookahead=2.0)
    prev_waypoint = c.waypoint
    positions = [(0.0, 0.0), (2.0, 0.0), (4.0, 0.0), (6.0, 0.0)]
    for pos in positions:
        c.find_next_lookahead(path, pos)
        if c.waypoint < prev_waypoint:
            logger.warning(f"  FAIL at pos={pos}: waypoint went from {prev_waypoint} to {c.waypoint}")
            passed = False
        else:
            logger.info(f"  OK   pos={pos}: waypoint={c.waypoint}")
        prev_waypoint = c.waypoint
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_lookahead_larger_distance_returns_farther_point():
    """
    TEST 9: find_next_lookahead monotonicity invariant.
    A larger lookahead distance must return a point at least as far away
    as a smaller lookahead distance, from the same position.
    """
    passed = True
    logger.info("TEST 9: Lookahead — larger lookahead must select a farther point")
    path = straight_path(length=20, step=1)
    current_pos = (0.0, 0.0)
    lookaheads = [1.0, 2.0, 3.0, 4.0]
    dists = []
    for ld in lookaheads:
        c = setup_controller(lookahead=ld)
        point = c.find_next_lookahead(path, current_pos)
        if point is None:
            logger.warning(f"  FAIL ld={ld}: returned None")
            passed = False
            dists.append(-1)
            continue
        dist = math.hypot(point[0] - current_pos[0], point[1] - current_pos[1])
        dists.append(dist)
        logger.info(f"  ld={ld}: selected point at dist={dist:.4f}")
    for i in range(1, len(dists)):
        if dists[i] < dists[i-1] - 1e-9:
            logger.warning(f"  FAIL ld={lookaheads[i]} gave closer point than ld={lookaheads[i-1]}")
            passed = False
    if passed:
        logger.info("  OK   monotonicity holds")
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: pure_pursuit

def test_pure_pursuit_linear_velocity_unchanged():
    """
    TEST 10: pure_pursuit output invariant.
    The returned linear velocity must always equal self.linear_velocity.
    Pure pursuit only commands omega; linear velocity is fixed.
    """
    passed = True
    logger.info("TEST 10: Pure pursuit — linear velocity output must equal configured linear_velocity")
    path = straight_path(length=20, step=1)
    for v in [0.3, 0.5, 1.0]:
        c = setup_controller(lookahead=2.0, linear_velocity=v)
        pose = (0.0, 0.0, 0.0)
        v_out, _ = c.pure_pursuit(path, pose)
        if not math.isclose(v_out, v, abs_tol=1e-9):
            logger.warning(f"  FAIL configured v={v}: returned v={v_out}")
            passed = False
        else:
            logger.info(f"  OK   configured v={v}: returned v={v_out:.4f}")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_pure_pursuit_facing_goal_omega_near_zero():
    """
    TEST 11: pure_pursuit heading invariant.
    When the robot faces directly toward the lookahead point (alpha=0),
    omega must be zero — no steering correction needed.
    """
    passed = False
    logger.info("TEST 11: Pure pursuit — robot facing goal directly must produce near-zero omega")
    # Path along x-axis, robot at origin facing East (theta=0)
    path = straight_path(length=20, step=1)
    c = setup_controller(lookahead=2.0, linear_velocity=1.0)
    pose = (0.0, 0.0, 0.0)
    _, omega = c.pure_pursuit(path, pose)
    if abs(omega) < 1e-6:
        logger.info(f"  OK   omega={omega:.6f} (near zero)")
        passed = True
    else:
        logger.warning(f"  FAIL omega={omega:.6f}, expected near zero")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_pure_pursuit_left_turn_positive_omega():
    """
    TEST 12: pure_pursuit steering direction invariant.
    When the lookahead point is to the left of the robot's heading,
    omega must be positive (left turn).
    """
    passed = False
    logger.info("TEST 12: Pure pursuit — lookahead left of heading must produce positive omega")
    # Path goes upward (north), robot faces East — goal is to the left
    path = vertical_path(length=20, step=1)
    c = setup_controller(lookahead=2.0, linear_velocity=1.0)
    pose = (0.0, 0.0, 0.0)  # Facing East, path goes North
    _, omega = c.pure_pursuit(path, pose)
    if omega > 0:
        logger.info(f"  OK   omega={omega:.4f} (positive, turning left)")
        passed = True
    else:
        logger.warning(f"  FAIL omega={omega:.4f}, expected positive")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_pure_pursuit_right_turn_negative_omega():
    """
    TEST 13: pure_pursuit steering direction invariant.
    When the lookahead point is to the right of the robot's heading,
    omega must be negative (right turn).
    """
    passed = False
    logger.info("TEST 13: Pure pursuit — lookahead right of heading must produce negative omega")
    # Path goes downward (south), robot faces East — goal is to the right
    path = [(0.0, float(-i)) for i in range(20)]
    c = setup_controller(lookahead=2.0, linear_velocity=1.0)
    pose = (0.0, 0.0, 0.0)  # Facing East, path goes South
    _, omega = c.pure_pursuit(path, pose)
    if omega < 0:
        logger.info(f"  OK   omega={omega:.4f} (negative, turning right)")
        passed = True
    else:
        logger.warning(f"  FAIL omega={omega:.4f}, expected negative")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_pure_pursuit_larger_alpha_larger_omega():
    """
    TEST 14: pure_pursuit curvature monotonicity invariant.
    A larger heading error (alpha) must produce a larger magnitude omega.
    The controller must command more aggressive steering for larger errors.
    """
    passed = True
    logger.info("TEST 14: Pure pursuit — larger heading error must produce larger |omega|")
    # Robot at origin. Rotate the lookahead point at a fixed distance
    # by increasing angles to create increasing alpha values.
    ld = 3.0
    angles = [math.pi / 8, math.pi / 4, math.pi / 3]
    prev_omega_mag = 0.0
    for angle in angles:
        # Place a two-point path: start and one lookahead point at `angle`
        target = (ld * math.cos(angle), ld * math.sin(angle))
        path = [(0.0, 0.0), target]
        c = setup_controller(lookahead=ld, linear_velocity=1.0)
        pose = (0.0, 0.0, 0.0)  # Facing East
        _, omega = c.pure_pursuit(path, pose)
        omega_mag = abs(omega)
        if omega_mag < prev_omega_mag - 1e-9:
            logger.warning(f"  FAIL angle={math.degrees(angle):.1f}°: |omega|={omega_mag:.4f} < prev {prev_omega_mag:.4f}")
            passed = False
        else:
            logger.info(f"  OK   angle={math.degrees(angle):.1f}°: |omega|={omega_mag:.4f}")
        prev_omega_mag = omega_mag
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_pure_pursuit_exhausted_path_returns_zero():
    """
    TEST 15: pure_pursuit path exhaustion invariant.
    When find_next_lookahead returns None, pure_pursuit must return (0, 0).
    """
    passed = False
    logger.info("TEST 15: Pure pursuit — exhausted path with robot far from goal must return (0, 0)")
    # 5-point path, last point is (4, 0). Robot is far enough away that
    # distance(current_pos, path[-1]) >= path_len + lookahead_distance, triggering None.
    path = straight_path(length=5, step=1)  # path_len=5, last point=(4,0)
    ld = 1.0
    # distance to goal must exceed path_len + ld = 5 + 1 = 6
    # Robot at (0, 0): distance to (4, 0) = 4 — not far enough.
    # Robot at (-10, 0): distance to (4, 0) = 14 — triggers None.
    c = setup_controller(lookahead=ld, linear_velocity=1.0)
    pose = (-10.0, 0.0, 0.0)
    c.waypoint = len(path)  # Force exhausted state
    result = c.pure_pursuit(path, pose)
    if result == (0, 0):
        logger.info(f"  OK   returned {result}")
        passed = True
    else:
        logger.warning(f"  FAIL returned {result}, expected (0, 0)")
    logger.info("PASS" if passed else "FAIL")
    return passed



def test_pure_pursuit_omega_bounded_by_curvature_formula():
    """
    TEST 16: pure_pursuit curvature formula invariant.
    omega = v * k, where k = 2*sin(alpha) / ld.
    Since |sin(alpha)| <= 1, |omega| must never exceed 2*v / ld.
    """
    passed = True
    logger.info("TEST 16: Pure pursuit — |omega| must never exceed 2*v / lookahead_distance")
    path = straight_path(length=20, step=1)
    for ld, v in [(1.0, 1.0), (2.0, 0.5), (3.0, 1.0)]:
        c = setup_controller(lookahead=ld, linear_velocity=v)
        max_omega = 2 * v / ld
        for theta in [0.0, math.pi/4, math.pi/2, math.pi, -math.pi/2]:
            pose = (0.0, 0.0, theta)
            _, omega = c.pure_pursuit(path, pose)
            if abs(omega) > max_omega + 1e-9:
                logger.warning(f"  FAIL ld={ld}, v={v}, theta={theta:.3f}: |omega|={abs(omega):.4f} > max={max_omega:.4f}")
                passed = False
            else:
                logger.info(f"  OK   ld={ld}, v={v}, theta={theta:.3f}: |omega|={abs(omega):.4f}, max={max_omega:.4f}")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_pure_pursuit_symmetric_paths():
    """
    TEST 17: pure_pursuit symmetry invariant.
    A path mirrored across the robot's heading must produce equal and opposite omega.
    The magnitude of the steering command must be the same for left and right turns
    of the same angle.
    """
    passed = True
    logger.info("TEST 17: Pure pursuit — mirrored paths must produce equal and opposite omega")
    ld = 3.0
    for angle in [math.pi/6, math.pi/4, math.pi/3]:
        target_left  = (ld * math.cos(angle),  ld * math.sin(angle))
        target_right = (ld * math.cos(angle), -ld * math.sin(angle))

        path_left  = [(0.0, 0.0), target_left]
        path_right = [(0.0, 0.0), target_right]

        c_left  = setup_controller(lookahead=ld, linear_velocity=1.0)
        c_right = setup_controller(lookahead=ld, linear_velocity=1.0)

        pose = (0.0, 0.0, 0.0)
        _, omega_left  = c_left.pure_pursuit(path_left,  pose)
        _, omega_right = c_right.pure_pursuit(path_right, pose)

        if not math.isclose(omega_left, -omega_right, abs_tol=1e-9):
            logger.warning(f"  FAIL angle={math.degrees(angle):.1f}°: omega_left={omega_left:.4f}, omega_right={omega_right:.4f}")
            passed = False
        else:
            logger.info(f"  OK   angle={math.degrees(angle):.1f}°: omega_left={omega_left:.4f}, omega_right={omega_right:.4f}")
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: Main

def main():
    tests = [
        test_init_lookahead_within_limit,
        test_init_lookahead_clamped_to_max,
        test_init_linear_velocity_within_limit,
        test_init_linear_velocity_clamped_to_max,
        test_init_waypoint_starts_at_zero,
        test_lookahead_point_is_at_least_lookahead_distance_away,
        test_lookahead_returns_last_point_when_waypoint_exhausted,
        test_lookahead_waypoint_index_advances,
        test_lookahead_larger_distance_returns_farther_point,
        test_pure_pursuit_linear_velocity_unchanged,
        test_pure_pursuit_facing_goal_omega_near_zero,
        test_pure_pursuit_left_turn_positive_omega,
        test_pure_pursuit_right_turn_negative_omega,
        test_pure_pursuit_larger_alpha_larger_omega,
        test_pure_pursuit_exhausted_path_returns_zero,
        test_pure_pursuit_omega_bounded_by_curvature_formula,
        test_pure_pursuit_symmetric_paths,
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

    logger.info("Pure Pursuit Controller Test Suite")
    results = [t() for t in selected]

    logger.info(f"Results: {sum(results)}/{len(results)} passed")
    logger.info("All tests complete.")
    sys.exit(0 if all(results) else 1)


if __name__ == '__main__':
    main()