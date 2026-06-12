#!/usr/bin/env python3
"""
Test script for DifferentialDriveKinematics from diff_drive_kinematics.py

Tests that the kinematics model correctly satisfies physical and mathematical
invariants for:
- Inverse kinematics (robot velocity -> wheel velocities)
- Forward kinematics (wheel velocities -> robot velocity)
- Pose update (straight-line and arc motion geometry)
- Velocity validation
"""
import logging
import math
import sys
import numpy as np

from navi_bot.control.kinematics import DifferentialDriveKinematics

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)


# MARK: Setup

def setup_robot(wheel_base=0.4, wheel_radius=0.1, max_wheel_speed=10.0):
    return DifferentialDriveKinematics(wheel_base, wheel_radius, max_wheel_speed)

def setup_uncapped_robot():
    """Robot with an effectively infinite speed limit, so clipping never interferes."""
    return setup_robot(max_wheel_speed=1000.0)


# MARK: Inverse Kinematics

def test_ik_average_equals_linear_velocity():
    """
    TEST 1: IK average wheel speed invariant.
    (v_l + v_r) / 2 must always equal the commanded linear velocity v.
    Verified across: forward, pure rotation, reverse+spin, straight, zero.
    """
    passed = True
    logger.info("TEST 1: IK — average of wheel speeds must equal linear velocity v")
    robot = setup_uncapped_robot()
    cases = [(1.0, 0.5), (0.0, 1.0), (-0.5, -1.0), (0.3, 0.0), (0.0, 0.0)]
    for v, omega in cases:
        v_l, v_r = robot.inverse_kinematics(v, omega)
        avg = (v_l + v_r) / 2.0
        if not math.isclose(avg, v, abs_tol=1e-9):
            logger.warning(f"  FAIL v={v}, omega={omega}: avg={avg}, expected={v}")
            passed = False
        else:
            logger.info(f"  OK   v={v}, omega={omega}: avg={avg:.6f}")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_ik_speed_difference_encodes_omega():
    """
    TEST 2: IK wheel speed difference invariant.
    (v_r - v_l) must equal omega * L — this encodes the turning rate.
    Verified across: combined, pure rotation, reverse+spin, straight.
    """
    passed = True
    logger.info("TEST 2: IK — wheel speed difference must equal omega * L")
    robot = setup_uncapped_robot()
    cases = [(1.0, 0.5), (0.0, 2.0), (-0.5, -1.0), (0.3, 0.0)]
    for v, omega in cases:
        v_l, v_r = robot.inverse_kinematics(v, omega)
        diff = v_r - v_l
        expected = omega * robot.L
        if not math.isclose(diff, expected, abs_tol=1e-9):
            logger.warning(f"  FAIL v={v}, omega={omega}: diff={diff}, expected={expected}")
            passed = False
        else:
            logger.info(f"  OK   v={v}, omega={omega}: diff={diff:.6f}, expected={expected:.6f}")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_ik_rotation_symmetry():
    """
    TEST 3: IK rotation symmetry invariant.
    For pure rotation (v=0): wheels must be equal and opposite.
    Flipping the sign of omega must swap the wheel assignments.
    """
    passed = True
    logger.info("TEST 3: IK — pure rotation must produce equal and opposite wheel speeds")
    robot = setup_uncapped_robot()
    for omega in [1.0, -1.0, 2.5, -2.5]:
        v_l_pos, v_r_pos = robot.inverse_kinematics(0.0,  omega)
        v_l_neg, v_r_neg = robot.inverse_kinematics(0.0, -omega)
        wheels_opposite = math.isclose(v_l_pos, -v_r_pos, abs_tol=1e-9)
        signs_swap      = math.isclose(v_l_pos, -v_l_neg, abs_tol=1e-9)
        if not wheels_opposite or not signs_swap:
            logger.warning(f"  FAIL omega={omega}: v_l={v_l_pos}, v_r={v_r_pos}")
            passed = False
        else:
            logger.info(f"  OK   omega={omega}: v_l={v_l_pos:.4f}, v_r={v_r_pos:.4f}")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_ik_straight_line_wheels_equal():
    """
    TEST 4: IK straight-line invariant.
    omega=0 must produce identical left and right wheel speeds.
    """
    passed = True
    logger.info("TEST 4: IK — omega=0 must produce equal wheel speeds")
    robot = setup_uncapped_robot()
    for v in [0.5, 1.0, -0.5]:
        v_l, v_r = robot.inverse_kinematics(v, 0.0)
        if not math.isclose(v_l, v_r, abs_tol=1e-9):
            logger.warning(f"  FAIL v={v}: v_l={v_l}, v_r={v_r}")
            passed = False
        else:
            logger.info(f"  OK   v={v}: v_l={v_l:.4f}, v_r={v_r:.4f}")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_ik_clipping_never_exceeds_limit():
    """
    TEST 5: IK clipping invariant.
    Regardless of commanded speed, wheel outputs must never exceed max_v.
    """
    passed = True
    logger.info("TEST 5: IK — clipped outputs must never exceed max wheel speed")
    robot = setup_robot()
    max_v = robot.max_wheel_speed * robot.r
    cases = [(5.0, 0.0), (0.0, 500.0), (10.0, 100.0)]
    for v, omega in cases:
        v_l, v_r = robot.inverse_kinematics(v, omega)
        if abs(v_l) > max_v + 1e-9 or abs(v_r) > max_v + 1e-9:
            logger.warning(f"  FAIL v={v}, omega={omega}: v_l={v_l}, v_r={v_r}, max={max_v}")
            passed = False
        else:
            logger.info(f"  OK   v={v}, omega={omega}: v_l={v_l:.4f}, v_r={v_r:.4f}, max={max_v}")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_ik_wider_wheelbase_larger_diff():
    """
    TEST 6: IK wheelbase scaling invariant.
    Doubling the wheel base must double the speed difference between wheels.
    """
    passed = True
    logger.info("TEST 6: IK — doubling wheel base must double the wheel speed difference")
    for L in [0.2, 0.4, 1.0]:
        r1 = setup_robot(wheel_base=L,     max_wheel_speed=1000.0)
        r2 = setup_robot(wheel_base=L * 2, max_wheel_speed=1000.0)
        v_l1, v_r1 = r1.inverse_kinematics(1.0, 1.0)
        v_l2, v_r2 = r2.inverse_kinematics(1.0, 1.0)
        diff1 = v_r1 - v_l1
        diff2 = v_r2 - v_l2
        if not math.isclose(diff2, diff1 * 2.0, abs_tol=1e-9):
            logger.warning(f"  FAIL L={L}: diff(L)={diff1}, diff(2L)={diff2}, expected {diff1*2}")
            passed = False
        else:
            logger.info(f"  OK   L={L}: diff(L)={diff1:.4f}, diff(2L)={diff2:.4f}")
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: Forward Kinematics

def test_fk_linear_velocity_is_average():
    """
    TEST 7: FK linear velocity invariant.
    v must equal (v_l + v_r) / 2 — the average of the wheel speeds.
    """
    passed = True
    logger.info("TEST 7: FK — linear velocity must equal average of wheel speeds")
    robot = setup_robot()
    cases = [(1.0, 1.0), (-0.5, -0.5), (0.3, 0.7), (0.0, 0.0), (-1.0, 1.0)]
    for v_l, v_r in cases:
        v, _ = robot.forward_kinematics(v_l, v_r)
        expected = (v_l + v_r) / 2.0
        if not math.isclose(float(v), expected, abs_tol=1e-9):
            logger.warning(f"  FAIL v_l={v_l}, v_r={v_r}: v={v}, expected={expected}")
            passed = False
        else:
            logger.info(f"  OK   v_l={v_l}, v_r={v_r}: v={float(v):.4f}")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_fk_angular_velocity_from_difference():
    """
    TEST 8: FK angular velocity invariant.
    omega must equal (v_r - v_l) / L — the normalised speed difference.
    """
    passed = True
    logger.info("TEST 8: FK — angular velocity must equal (v_r - v_l) / L")
    robot = setup_robot()
    cases = [(1.0, 1.0), (-0.5, -0.5), (0.3, 0.7), (0.0, 0.0), (-1.0, 1.0)]
    for v_l, v_r in cases:
        _, omega = robot.forward_kinematics(v_l, v_r)
        expected = (v_r - v_l) / robot.L
        if not math.isclose(float(omega), expected, abs_tol=1e-9):
            logger.warning(f"  FAIL v_l={v_l}, v_r={v_r}: omega={omega}, expected={expected}")
            passed = False
        else:
            logger.info(f"  OK   v_l={v_l}, v_r={v_r}: omega={float(omega):.4f}")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_fk_ik_roundtrip():
    """
    TEST 9: IK -> FK roundtrip invariant.
    Applying IK then FK must recover the original (v, omega) exactly.
    Uses an uncapped robot so clipping never corrupts the roundtrip.
    """
    passed = True
    logger.info("TEST 9: FK — IK followed by FK must recover original (v, omega)")
    robot = setup_uncapped_robot()
    cases = [(0.5, 0.5), (1.0, 0.0), (0.0, 1.0), (-0.3, 0.8)]
    for v_in, omega_in in cases:
        v_l, v_r = robot.inverse_kinematics(v_in, omega_in)
        v_out, omega_out = robot.forward_kinematics(v_l, v_r)
        v_ok     = math.isclose(float(v_out),     v_in,     abs_tol=1e-9)
        omega_ok = math.isclose(float(omega_out), omega_in, abs_tol=1e-9)
        if not v_ok or not omega_ok:
            logger.warning(f"  FAIL v={v_in}, omega={omega_in}: got v={v_out}, omega={omega_out}")
            passed = False
        else:
            logger.info(f"  OK   v={v_in}, omega={omega_in}: recovered v={float(v_out):.4f}, omega={float(omega_out):.4f}")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_fk_equal_speeds_zero_omega():
    """
    TEST 10: FK no-rotation invariant.
    Equal wheel speeds must never produce angular velocity.
    """
    passed = True
    logger.info("TEST 10: FK — equal wheel speeds must produce zero angular velocity")
    robot = setup_robot()
    for speed in [0.0, 0.5, 1.0, -0.7]:
        _, omega = robot.forward_kinematics(speed, speed)
        if not math.isclose(float(omega), 0.0, abs_tol=1e-9):
            logger.warning(f"  FAIL speed={speed}: omega={omega}, expected 0")
            passed = False
        else:
            logger.info(f"  OK   speed={speed}: omega={float(omega):.6f}")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_fk_opposite_speeds_zero_linear():
    """
    TEST 11: FK no-translation invariant.
    Equal-magnitude opposite wheel speeds must never produce linear velocity.
    """
    passed = True
    logger.info("TEST 11: FK — opposite wheel speeds must produce zero linear velocity")
    robot = setup_robot()
    for speed in [0.5, 1.0, 0.3]:
        v, _ = robot.forward_kinematics(-speed, speed)
        if not math.isclose(float(v), 0.0, abs_tol=1e-9):
            logger.warning(f"  FAIL speed={speed}: v={v}, expected 0")
            passed = False
        else:
            logger.info(f"  OK   speed={speed}: v={float(v):.6f}")
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: Pose Update

def test_pose_straight_displacement_magnitude():
    """
    TEST 12: Pose update — straight-line displacement magnitude invariant.
    |displacement| must equal |v * dt| regardless of heading.
    The heading controls direction only, not distance.
    """
    passed = True
    logger.info("TEST 12: Pose — straight-line displacement magnitude must equal |v * dt|")
    robot = setup_robot()
    v, dt = 1.0, 0.5
    for theta in [0.0, math.pi/4, math.pi/2, math.pi, -math.pi/3]:
        dx, dy, _ = robot.compute_pose_update(v, 0.0, theta, dt)
        dist = math.hypot(dx, dy)
        expected = abs(v * dt)
        if not math.isclose(dist, expected, abs_tol=1e-9):
            logger.warning(f"  FAIL theta={theta:.3f}: dist={dist}, expected={expected}")
            passed = False
        else:
            logger.info(f"  OK   theta={theta:.3f}: dist={dist:.6f}")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_pose_straight_direction_matches_heading():
    """
    TEST 13: Pose update — straight-line direction invariant.
    The angle of the displacement vector must match the robot's heading theta.
    """
    passed = True
    logger.info("TEST 13: Pose — straight-line displacement direction must match heading theta")
    robot = setup_robot()
    for theta in [0.0, math.pi/4, math.pi/2, math.pi]:
        dx, dy, _ = robot.compute_pose_update(1.0, 0.0, theta, 1.0)
        angle = math.atan2(dy, dx)
        if not math.isclose(angle, theta, abs_tol=1e-9):
            logger.warning(f"  FAIL theta={theta:.3f}: displacement angle={angle:.3f}")
            passed = False
        else:
            logger.info(f"  OK   theta={theta:.3f}: displacement angle={angle:.3f}")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_pose_straight_no_rotation():
    """
    TEST 14: Pose update — straight-line dtheta must be zero.
    """
    passed = True
    logger.info("TEST 14: Pose — straight-line motion must produce zero heading change")
    robot = setup_robot()
    _, _, dtheta = robot.compute_pose_update(1.0, 0.0, 0.5, 1.0)
    if not math.isclose(dtheta, 0.0, abs_tol=1e-9):
        logger.warning(f"  FAIL dtheta={dtheta}, expected 0")
        passed = False
    else:
        logger.info(f"  OK   dtheta={dtheta}")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_pose_arc_dtheta_equals_omega_dt():
    """
    TEST 15: Pose update — arc dtheta invariant.
    dtheta must equal omega * dt. This is the definition of angular velocity.
    """
    passed = True
    logger.info("TEST 15: Pose — arc dtheta must equal omega * dt")
    robot = setup_robot()
    cases = [(1.0, 0.5), (-1.0, 0.5), (2.0, 0.25), (0.5, 1.0)]
    for omega, dt in cases:
        _, _, dtheta = robot.compute_pose_update(1.0, omega, 0.0, dt)
        expected = omega * dt
        if not math.isclose(dtheta, expected, abs_tol=1e-9):
            logger.warning(f"  FAIL omega={omega}, dt={dt}: dtheta={dtheta}, expected={expected}")
            passed = False
        else:
            logger.info(f"  OK   omega={omega}, dt={dt}: dtheta={dtheta:.6f}")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_pose_arc_chord_length():
    """
    TEST 16: Pose update — arc chord length invariant.
    For a circular arc, chord = 2 * R * sin(|dtheta| / 2).
    This is pure Euclidean geometry and must hold for any arc.
    """
    passed = True
    logger.info("TEST 16: Pose — arc chord length must equal 2R * sin(|dtheta|/2)")
    robot = setup_robot()
    cases = [(1.0, 1.0), (2.0, 0.5), (1.0, -1.0), (0.5, 2.0)]
    for v, omega in cases:
        dt = 0.3
        dx, dy, dtheta = robot.compute_pose_update(v, omega, 0.0, dt)
        R = abs(v / omega)
        chord_expected = 2 * R * abs(math.sin(dtheta / 2.0))
        chord_actual   = math.hypot(dx, dy)
        if not math.isclose(chord_actual, chord_expected, abs_tol=1e-9):
            logger.warning(f"  FAIL v={v}, omega={omega}: chord={chord_actual}, expected={chord_expected}")
            passed = False
        else:
            logger.info(f"  OK   v={v}, omega={omega}: chord={chord_actual:.6f}")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_pose_arc_heading_independence():
    """
    TEST 17: Pose update — arc heading independence invariant.
    Rotating the initial heading by phi must rotate the displacement
    vector by exactly phi. The arc shape is heading-relative, not absolute.
    """
    passed = True
    logger.info("TEST 17: Pose — rotating initial heading must rotate displacement by same angle")
    robot = setup_robot()
    phi = math.pi / 3
    for omega in [1.0, -1.0, 2.0, -2.0]:
        dx0, dy0, _ = robot.compute_pose_update(1.0, omega, 0.0, 0.5)
        dx1, dy1, _ = robot.compute_pose_update(1.0, omega, phi, 0.5)
        dx0_rot = dx0 * math.cos(phi) - dy0 * math.sin(phi)
        dy0_rot = dx0 * math.sin(phi) + dy0 * math.cos(phi)
        if not math.isclose(dx1, dx0_rot, abs_tol=1e-9) or not math.isclose(dy1, dy0_rot, abs_tol=1e-9):
            logger.warning(f"  FAIL omega={omega}: got ({dx1:.4f},{dy1:.4f}), expected ({dx0_rot:.4f},{dy0_rot:.4f})")
            passed = False
        else:
            logger.info(f"  OK   omega={omega}: displacement rotated correctly")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_pose_zero_velocity_no_movement():
    """
    TEST 18: Pose update — stationary robot must not move.
    """
    passed = True
    logger.info("TEST 18: Pose — zero velocity must produce zero displacement")
    robot = setup_robot()
    dx, dy, dtheta = robot.compute_pose_update(0.0, 0.0, 1.2, 1.0)
    if not (math.isclose(dx, 0.0, abs_tol=1e-9) and
            math.isclose(dy, 0.0, abs_tol=1e-9) and
            math.isclose(dtheta, 0.0, abs_tol=1e-9)):
        logger.warning(f"  FAIL dx={dx}, dy={dy}, dtheta={dtheta}")
        passed = False
    else:
        logger.info(f"  OK   dx={dx}, dy={dy}, dtheta={dtheta}")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_pose_arc_omega_sign_symmetry():
    """
    TEST 19: Pose update — reversing omega must mirror dy and negate dtheta.
    dx must stay the same (for theta=0). Checks left-right arc symmetry.
    """
    passed = True
    logger.info("TEST 19: Pose — negating omega must mirror dy, keep dx, negate dtheta (at theta=0)")
    robot = setup_robot()
    for omega in [1.0, -0.5, 2.0]:
        dx_p, dy_p, dth_p = robot.compute_pose_update(1.0,  omega, 0.0, 0.5)
        dx_n, dy_n, dth_n = robot.compute_pose_update(1.0, -omega, 0.0, 0.5)
        dx_ok  = math.isclose(dx_p,   dx_n,  abs_tol=1e-9)
        dy_ok  = math.isclose(dy_p,  -dy_n,  abs_tol=1e-9)
        dth_ok = math.isclose(dth_p, -dth_n, abs_tol=1e-9)
        if not (dx_ok and dy_ok and dth_ok):
            logger.warning(f"  FAIL omega={omega}: dx_p={dx_p:.4f} dx_n={dx_n:.4f}, dy_p={dy_p:.4f} dy_n={dy_n:.4f}")
            passed = False
        else:
            logger.info(f"  OK   omega={omega}")
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: Validate Velocities

def test_validate_consistent_with_ik():
    """
    TEST 20: validate_velocities must agree with directly checking IK output.
    These two paths must never disagree.
    """
    passed = True
    logger.info("TEST 20: Validate — result must be consistent with checking IK output directly")
    robot = setup_robot()
    max_v = robot.max_wheel_speed * robot.r
    cases = [(0.5, 0.5), (100.0, 0.0), (0.0, 100.0), (0.3, 0.1)]
    for v, omega in cases:
        # Compute raw (pre-clip) wheel speeds to determine ground-truth feasibility.
        raw_v_l = v - (omega * robot.L / 2.0)
        raw_v_r = v + (omega * robot.L / 2.0)
        expected = abs(raw_v_l) <= max_v and abs(raw_v_r) <= max_v
        result   = robot.validate_velocities(v, omega)
        if bool(result) != bool(expected):
            logger.warning(f"  FAIL v={v}, omega={omega}: got {result}, expected {expected}")
            passed = False
        else:
            logger.info(f"  OK   v={v}, omega={omega}: result={result}")
    logger.info("PASS" if passed else "FAIL")
    return passed
 
 
def test_validate_infeasible_command_rejected():
    """
    TEST 21: validate_velocities must return False for physically unreachable speeds.
    v=100 m/s requires each wheel at 100 m/s, far above max_v=1.0 m/s.
    """
    passed = False
    logger.info("TEST 21: Validate — v=100 m/s must be rejected as infeasible (exposes clipping bug)")
    robot = setup_robot()
    result = robot.validate_velocities(100.0, 0.0)
    if result == False:
        logger.info("  OK   v=100 correctly rejected")
        passed = True
    else:
        logger.warning("  FAIL v=100 was incorrectly reported as valid")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_validate_zero_always_valid():
    """
    TEST 22: validate_velocities must return True for a zero velocity command.
    """
    passed = False
    logger.info("TEST 22: Validate — zero velocity must always be feasible")
    robot = setup_robot()
    if robot.validate_velocities(0.0, 0.0):
        logger.info("  OK   (0, 0) correctly accepted")
        passed = True
    else:
        logger.warning("  FAIL (0, 0) was rejected")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_validate_omega_sign_symmetry():
    """
    TEST 23: validate_velocities must be symmetric in the sign of omega.
    The speed limit is applied equally to both wheels.
    """
    passed = True
    logger.info("TEST 23: Validate — feasibility must be symmetric in sign of omega")
    robot = setup_robot()
    for omega in [0.5, 1.0, -0.5, -1.0]:
        pos = robot.validate_velocities(0.0,  omega)
        neg = robot.validate_velocities(0.0, -omega)
        if bool(pos) != bool(neg):
            logger.warning(f"  FAIL omega={omega}: +omega={pos}, -omega={neg}")
            passed = False
        else:
            logger.info(f"  OK   omega={omega}: symmetric ({pos})")
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: Main

def main():
    tests = [
        test_ik_average_equals_linear_velocity,
        test_ik_speed_difference_encodes_omega,
        test_ik_rotation_symmetry,
        test_ik_straight_line_wheels_equal,
        test_ik_clipping_never_exceeds_limit,
        test_ik_wider_wheelbase_larger_diff,
        test_fk_linear_velocity_is_average,
        test_fk_angular_velocity_from_difference,
        test_fk_ik_roundtrip,
        test_fk_equal_speeds_zero_omega,
        test_fk_opposite_speeds_zero_linear,
        test_pose_straight_displacement_magnitude,
        test_pose_straight_direction_matches_heading,
        test_pose_straight_no_rotation,
        test_pose_arc_dtheta_equals_omega_dt,
        test_pose_arc_chord_length,
        test_pose_arc_heading_independence,
        test_pose_zero_velocity_no_movement,
        test_pose_arc_omega_sign_symmetry,
        test_validate_consistent_with_ik,
        test_validate_infeasible_command_rejected,
        test_validate_zero_always_valid,
        test_validate_omega_sign_symmetry,
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

    logger.info("Differential Drive Kinematics Test Suite")
    results = [t() for t in selected]

    logger.info(f"Results: {sum(results)}/{len(results)} passed")
    logger.info("All tests complete.")
    sys.exit(0 if all(results) else 1)


if __name__ == '__main__':
    main()