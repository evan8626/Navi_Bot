#!/usr/bin/env python3
"""
Test script for wheel odometry in navi_bot/sensors/odometry.py

Tests that dead-reckoning odometry correctly:
- Converts encoder ticks to distance
- Integrates straight-line, reverse, and turn-in-place motion
- Estimates linear and angular velocity
- Tracks incremental encoder values between updates
- Keeps heading normalized and supports pose reset

Default robot: wheel_base 0.4 m, wheel_radius 0.1 m, 1000 ticks/rev
(one wheel revolution = 2*pi*0.1 ~= 0.6283 m).
"""
import logging
import math
import sys

from navi_bot.sensors.odometry import WheelOdometry

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)

WHEEL_BASE = 0.4
WHEEL_RADIUS = 0.1
TICKS_PER_REV = 1000
METERS_PER_REV = 2 * math.pi * WHEEL_RADIUS


# MARK: Setup

def setup_odometry():
    return WheelOdometry(wheel_base=WHEEL_BASE, wheel_radius=WHEEL_RADIUS,
                         ticks_per_rev=TICKS_PER_REV)


# MARK: Tick Conversion

def test_ticks_to_distance():
    """ticks_to_distance must map one revolution to 2*pi*r (and scale linearly)."""
    passed = True
    logger.info("TEST 1: ticks_to_distance conversion")
    odom = setup_odometry()
    cases = [
        (TICKS_PER_REV, METERS_PER_REV),
        (TICKS_PER_REV // 2, METERS_PER_REV / 2),
        (0, 0.0),
        (-TICKS_PER_REV, -METERS_PER_REV),
    ]
    for ticks, expected in cases:
        d = odom.ticks_to_distance(ticks)
        if not math.isclose(d, expected, abs_tol=1e-9):
            logger.warning(f"  FAIL {ticks} ticks: got {d}, expected {expected}")
            passed = False
        else:
            logger.info(f"  OK   {ticks} ticks -> {expected:.4f} m")
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: Motion Integration

def test_straight_line():
    """Equal wheel ticks must move the robot straight along its heading."""
    passed = True
    logger.info("TEST 2: straight-line motion")
    odom = setup_odometry()
    x, y, theta, v, omega = odom.update(TICKS_PER_REV, TICKS_PER_REV, 1.0)
    if not math.isclose(x, METERS_PER_REV, abs_tol=1e-9):
        logger.warning(f"  FAIL x={x}, expected {METERS_PER_REV}")
        passed = False
    elif not (math.isclose(y, 0.0, abs_tol=1e-9) and math.isclose(theta, 0.0, abs_tol=1e-9)):
        logger.warning(f"  FAIL y={y}, theta={theta}, expected both 0")
        passed = False
    else:
        logger.info(f"  OK   one revolution forward -> x={x:.4f} m, y=0, theta=0")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_reverse():
    """Equal negative wheel ticks must move the robot backward."""
    passed = True
    logger.info("TEST 3: reverse motion")
    odom = setup_odometry()
    x, y, theta, _, _ = odom.update(-500, -500, 1.0)
    if not (x < 0 and math.isclose(y, 0.0, abs_tol=1e-9) and math.isclose(theta, 0.0, abs_tol=1e-9)):
        logger.warning(f"  FAIL x={x}, y={y}, theta={theta}, expected x<0, y=0, theta=0")
        passed = False
    else:
        logger.info(f"  OK   negative ticks -> x={x:.4f} m (backward)")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_rotate_in_place():
    """Opposite wheel ticks must rotate in place: position fixed, theta = (dr-dl)/L."""
    passed = True
    logger.info("TEST 4: turn-in-place rotation")
    odom = setup_odometry()
    x, y, theta, _, _ = odom.update(-500, 500, 1.0)
    expected_theta = (METERS_PER_REV / 2 - (-METERS_PER_REV / 2)) / WHEEL_BASE
    if not (math.isclose(x, 0.0, abs_tol=1e-9) and math.isclose(y, 0.0, abs_tol=1e-9)):
        logger.warning(f"  FAIL position moved during pure rotation: ({x}, {y})")
        passed = False
    elif not math.isclose(theta, expected_theta, abs_tol=1e-9):
        logger.warning(f"  FAIL theta={theta}, expected {expected_theta}")
        passed = False
    else:
        logger.info(f"  OK   right wheel forward, left back -> theta={theta:.4f} rad (CCW), position fixed")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_arc_curves_toward_faster_wheel():
    """A faster right wheel must curve the robot left (theta > 0, y > 0)."""
    passed = True
    logger.info("TEST 5: arc motion curves toward the slower wheel side")
    odom = setup_odometry()
    x, y, theta, _, _ = odom.update(1000, 1100, 1.0)
    if not (theta > 0 and y > 0 and x > 0):
        logger.warning(f"  FAIL x={x:.4f}, y={y:.4f}, theta={theta:.4f}; expected all positive")
        passed = False
    else:
        logger.info(f"  OK   right wheel faster -> x={x:.4f}, y={y:.4f}, theta={theta:.4f}")
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: Velocity Estimation

def test_velocity_estimates():
    """update must report v = ds/dt and omega = dtheta/dt."""
    passed = True
    logger.info("TEST 6: velocity estimation")
    odom = setup_odometry()
    dt = 0.5
    _, _, _, v, omega = odom.update(TICKS_PER_REV, TICKS_PER_REV, dt)
    expected_v = METERS_PER_REV / dt
    if not (math.isclose(v, expected_v, abs_tol=1e-9) and math.isclose(omega, 0.0, abs_tol=1e-9)):
        logger.warning(f"  FAIL v={v}, omega={omega}, expected v={expected_v}, omega=0")
        passed = False
    else:
        logger.info(f"  OK   v={v:.4f} m/s, omega=0")
    odom2 = setup_odometry()
    _, _, _, v2, omega2 = odom2.update(-500, 500, 0.5)
    expected_omega = (METERS_PER_REV / WHEEL_BASE) / 0.5
    if not (math.isclose(v2, 0.0, abs_tol=1e-9) and math.isclose(omega2, expected_omega, abs_tol=1e-9)):
        logger.warning(f"  FAIL v={v2}, omega={omega2}, expected v=0, omega={expected_omega}")
        passed = False
    else:
        logger.info(f"  OK   pure rotation -> v=0, omega={omega2:.4f} rad/s")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_zero_dt_no_crash():
    """dt=0 must not divide by zero; velocities keep their previous values."""
    passed = True
    logger.info("TEST 7: zero dt is handled safely")
    odom = setup_odometry()
    odom.update(100, 100, 1.0)
    v_before, omega_before = odom.get_velocity()
    try:
        odom.update(200, 200, 0.0)
        v_after, omega_after = odom.get_velocity()
        if not (math.isclose(v_after, v_before, abs_tol=1e-9) and math.isclose(omega_after, omega_before, abs_tol=1e-9)):
            logger.warning(f"  FAIL velocities changed on dt=0: {v_before}->{v_after}")
            passed = False
        else:
            logger.info("  OK   dt=0 update kept previous velocity estimates")
    except ZeroDivisionError:
        logger.error("  update raised ZeroDivisionError on dt=0")
        passed = False
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: State Tracking

def test_incremental_ticks():
    """Encoder values are absolute: repeating the same totals must not move the robot."""
    passed = True
    logger.info("TEST 8: incremental tick tracking")
    odom = setup_odometry()
    odom.update(1000, 1000, 1.0)
    x1, y1, theta1 = odom.get_pose()
    odom.update(1000, 1000, 1.0)  # same absolute tick counts again
    x2, y2, theta2 = odom.get_pose()
    if not (math.isclose(x1, x2, abs_tol=1e-9) and math.isclose(y1, y2, abs_tol=1e-9)
            and math.isclose(theta1, theta2, abs_tol=1e-9)):
        logger.warning(f"  FAIL pose moved with unchanged encoders: ({x1},{y1}) -> ({x2},{y2})")
        passed = False
    else:
        logger.info("  OK   unchanged encoder totals -> no motion")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_theta_stays_normalized():
    """Heading must remain in [-pi, pi] across many full rotations."""
    passed = True
    logger.info("TEST 9: heading normalization across multiple spins")
    odom = setup_odometry()
    ticks = 0
    for _ in range(20):
        ticks += 500
        odom.update(-ticks, ticks, 1.0)
        _, _, theta = odom.get_pose()
        if not (-math.pi - 1e-9 <= theta <= math.pi + 1e-9):
            logger.warning(f"  FAIL theta={theta} outside [-pi, pi]")
            passed = False
            break
    if passed:
        logger.info("  OK   theta stayed within [-pi, pi] over 20 spin updates")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_reset_and_getters():
    """reset must set the pose; get_pose/get_velocity must report current state."""
    passed = True
    logger.info("TEST 10: reset and state getters")
    odom = setup_odometry()
    odom.update(800, 600, 1.0)
    odom.reset(1.0, -2.0, 0.5)
    x, y, theta = odom.get_pose()
    if not (math.isclose(x, 1.0) and math.isclose(y, -2.0) and math.isclose(theta, 0.5)):
        logger.warning(f"  FAIL reset pose ({x}, {y}, {theta}), expected (1.0, -2.0, 0.5)")
        passed = False
    else:
        logger.info("  OK   reset(1.0, -2.0, 0.5) reflected by get_pose")
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: Main Method

def main():
    tests = [
        test_ticks_to_distance,
        test_straight_line,
        test_reverse,
        test_rotate_in_place,
        test_arc_curves_toward_faster_wheel,
        test_velocity_estimates,
        test_zero_dt_no_crash,
        test_incremental_ticks,
        test_theta_stays_normalized,
        test_reset_and_getters,
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

    logger.info("Wheel Odometry Test Suite")
    results = [t() for t in selected]

    logger.info(f"Results: {sum(results)}/{len(results)} passed")
    logger.info("All tests complete.")
    sys.exit(0 if all(results) else 1)


if __name__ == '__main__':
    main()
