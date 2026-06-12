#!/usr/bin/env python3
"""
Test script for the PID motion controller in navi_bot/control/motion_controller.py

Tests that the controller correctly:
- Computes P, I, and D terms, output limits, and anti-windup (PIDController)
- Drives toward a goal, slows for sharp turns, and stops inside tolerance
- Rotates in place to meet a goal orientation
- Reports goal-reached status
- Converges to a goal in a closed-loop unicycle simulation
"""
import logging
import math
import sys

from navi_bot.control.motion_controller import PIDController, MotionController

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)


# MARK: PID Controller

def test_pid_proportional():
    """A P-only controller must output kp * error."""
    passed = True
    logger.info("TEST 1: PID proportional term")
    pid = PIDController(kp=2.0, ki=0.0, kd=0.0)
    out = pid.update(1.5, 0.1)
    if not math.isclose(out, 3.0, abs_tol=1e-9):
        logger.warning(f"  FAIL output={out}, expected 3.0")
        passed = False
    else:
        logger.info("  OK   kp=2, error=1.5 -> 3.0")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_pid_integral_accumulates():
    """An I-only controller must accumulate error * dt across updates."""
    passed = True
    logger.info("TEST 2: PID integral accumulation")
    pid = PIDController(kp=0.0, ki=1.0, kd=0.0)
    out1 = pid.update(1.0, 0.1)
    out2 = pid.update(1.0, 0.1)
    if not (math.isclose(out1, 0.1, abs_tol=1e-9) and math.isclose(out2, 0.2, abs_tol=1e-9)):
        logger.warning(f"  FAIL outputs {out1}, {out2}, expected 0.1 then 0.2")
        passed = False
    else:
        logger.info("  OK   integral term grew 0.1 -> 0.2")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_pid_derivative():
    """A D-only controller must output kd * (error - prev_error) / dt."""
    passed = True
    logger.info("TEST 3: PID derivative term")
    pid = PIDController(kp=0.0, ki=0.0, kd=1.0)
    out1 = pid.update(1.0, 0.5)   # (1.0 - 0.0) / 0.5 = 2.0
    out2 = pid.update(1.5, 0.5)   # (1.5 - 1.0) / 0.5 = 1.0
    if not (math.isclose(out1, 2.0, abs_tol=1e-9) and math.isclose(out2, 1.0, abs_tol=1e-9)):
        logger.warning(f"  FAIL outputs {out1}, {out2}, expected 2.0 then 1.0")
        passed = False
    else:
        logger.info("  OK   derivative computed against previous error")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_pid_output_limits():
    """Output must be clipped to the configured limits in both directions."""
    passed = True
    logger.info("TEST 4: PID output saturation")
    pid = PIDController(kp=10.0, ki=0.0, kd=0.0, output_limits=(-1.0, 1.0))
    hi = pid.update(5.0, 0.1)
    lo = pid.update(-5.0, 0.1)
    if not (math.isclose(hi, 1.0, abs_tol=1e-9) and math.isclose(lo, -1.0, abs_tol=1e-9)):
        logger.warning(f"  FAIL outputs {hi}, {lo}, expected +1.0 and -1.0")
        passed = False
    else:
        logger.info("  OK   clipped to [-1, 1]")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_pid_anti_windup():
    """The integral must not keep growing while the output is saturated."""
    passed = True
    logger.info("TEST 5: PID anti-windup under saturation")
    pid = PIDController(kp=10.0, ki=1.0, kd=0.0, output_limits=(-1.0, 1.0))
    for _ in range(10):
        pid.update(5.0, 0.1)  # massively saturated every update
    if abs(pid.integral) > 0.5 + 1e-9:
        logger.warning(f"  FAIL integral wound up to {pid.integral}")
        passed = False
    else:
        logger.info(f"  OK   integral bounded at {pid.integral:.4f} despite saturation")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_pid_zero_dt():
    """dt=0 must not divide by zero (derivative term guards it)."""
    passed = True
    logger.info("TEST 6: PID zero dt safety")
    pid = PIDController(kp=1.0, ki=0.5, kd=2.0)
    try:
        out = pid.update(1.0, 0.0)
        logger.info(f"  OK   dt=0 handled, output={out}")
    except ZeroDivisionError:
        logger.error("  update raised ZeroDivisionError on dt=0")
        passed = False
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_pid_reset():
    """reset must clear the integral and previous error."""
    passed = True
    logger.info("TEST 7: PID reset clears state")
    pid = PIDController(kp=0.0, ki=1.0, kd=1.0)
    pid.update(2.0, 0.1)
    pid.reset()
    if not (math.isclose(pid.integral, 0.0, abs_tol=1e-9) and math.isclose(pid.prev_error, 0.0, abs_tol=1e-9)):
        logger.warning(f"  FAIL integral={pid.integral}, prev_error={pid.prev_error}")
        passed = False
    else:
        logger.info("  OK   integral and prev_error zeroed")
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: Motion Controller

def test_no_goal_zero_command():
    """With no goal set, compute_control must command zero velocities."""
    passed = True
    logger.info("TEST 8: no goal -> zero command")
    mc = MotionController()
    v, omega = mc.compute_control((0.0, 0.0, 0.0), 0.1)
    if not (math.isclose(v, 0.0, abs_tol=1e-9) and math.isclose(omega, 0.0, abs_tol=1e-9)):
        logger.warning(f"  FAIL got ({v}, {omega}), expected (0, 0)")
        passed = False
    else:
        logger.info("  OK   (0.0, 0.0)")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_drive_toward_goal_ahead():
    """A goal straight ahead must command forward motion with ~zero turn rate."""
    passed = True
    logger.info("TEST 9: goal straight ahead -> forward, no turn")
    mc = MotionController()
    mc.set_goal(5.0, 0.0)
    v, omega = mc.compute_control((0.0, 0.0, 0.0), 0.1)
    if v <= 0:
        logger.warning(f"  FAIL v={v}, expected forward motion")
        passed = False
    elif abs(omega) > 1e-6:
        logger.warning(f"  FAIL omega={omega}, expected ~0 when already aligned")
        passed = False
    else:
        logger.info(f"  OK   v={v:.3f}, omega={omega:.6f}")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_sharp_turn_slows_down():
    """A large heading error (> pi/4) must reduce forward speed and command a turn."""
    passed = True
    logger.info("TEST 10: sharp heading error halves forward speed")
    mc_aligned = MotionController()
    mc_aligned.set_goal(5.0, 0.0)
    v_aligned, _ = mc_aligned.compute_control((0.0, 0.0, 0.0), 0.1)

    mc_behind = MotionController()
    mc_behind.set_goal(-5.0, 0.0)  # directly behind the robot
    v_behind, omega_behind = mc_behind.compute_control((0.0, 0.0, 0.0), 0.1)

    if not (v_behind < v_aligned):
        logger.warning(f"  FAIL v_behind={v_behind} not less than v_aligned={v_aligned}")
        passed = False
    elif abs(omega_behind) <= 1e-6:
        logger.warning("  FAIL expected a turn command for a goal behind the robot")
        passed = False
    else:
        logger.info(f"  OK   aligned v={v_aligned:.3f}, behind v={v_behind:.3f} with omega={omega_behind:.3f}")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_stops_inside_tolerance():
    """Within goal tolerance (and no goal heading), the command must be (0, 0)."""
    passed = True
    logger.info("TEST 11: stops inside goal tolerance")
    mc = MotionController()
    mc.set_goal(0.05, 0.0)  # within the 0.1 m tolerance of the origin
    v, omega = mc.compute_control((0.0, 0.0, 0.0), 0.1)
    if not (math.isclose(v, 0.0, abs_tol=1e-9) and math.isclose(omega, 0.0, abs_tol=1e-9)):
        logger.warning(f"  FAIL got ({v}, {omega}), expected (0, 0)")
        passed = False
    else:
        logger.info("  OK   no motion commanded inside tolerance")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_rotates_to_goal_heading():
    """At the goal position with a heading requirement, it must rotate in place."""
    passed = True
    logger.info("TEST 12: rotate in place to goal heading")
    mc = MotionController()
    mc.set_goal(0.0, 0.0, theta=math.pi / 2)
    v, omega = mc.compute_control((0.0, 0.0, 0.0), 0.1)
    if not math.isclose(v, 0.0, abs_tol=1e-9):
        logger.warning(f"  FAIL v={v}, expected 0 while rotating in place")
        passed = False
    elif omega <= 0:
        logger.warning(f"  FAIL omega={omega}, expected positive (CCW toward +pi/2)")
        passed = False
    else:
        logger.info(f"  OK   v=0, omega={omega:.3f}")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_goal_reached_no_goal():
    """is_goal_reached with no goal set must be True."""
    passed = True
    logger.info("TEST 13: is_goal_reached without a goal")
    mc = MotionController()
    if not mc.is_goal_reached((0.0, 0.0, 0.0)):
        logger.warning("  FAIL expected True with no goal")
        passed = False
    else:
        logger.info("  OK   True")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_goal_reached_with_goal():
    """is_goal_reached must be True at the goal and False far away."""
    passed = True
    logger.info("TEST 14: is_goal_reached at and away from the goal")
    mc = MotionController()
    mc.set_goal(1.0, 1.0)
    try:
        far = mc.is_goal_reached((5.0, 5.0, 0.0))
        near = mc.is_goal_reached((1.0, 1.0, 0.0))
        if far:
            logger.warning("  FAIL reported reached while 5+ m away")
            passed = False
        elif not near:
            logger.warning("  FAIL reported not reached while at the goal")
            passed = False
        else:
            logger.info("  OK   far -> False, at goal -> True")
    except Exception as e:
        logger.error(f"  is_goal_reached raised {type(e).__name__}: {e}")
        passed = False
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_normalize_angle():
    """normalize_angle must wrap into [-pi, pi]."""
    passed = True
    logger.info("TEST 15: normalize_angle wrapping")
    mc = MotionController()
    cases = [(0.0, 0.0), (3 * math.pi, math.pi), (-3 * math.pi, -math.pi), (math.pi / 4, math.pi / 4)]
    for angle, expected in cases:
        n = mc.normalize_angle(angle)
        if not math.isclose(n, expected, abs_tol=1e-9):
            logger.warning(f"  FAIL {angle} -> {n}, expected {expected}")
            passed = False
        else:
            logger.info(f"  OK   {angle:.4f} -> {n:.4f}")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_closed_loop_convergence():
    """Closed-loop unicycle simulation must converge to the goal within tolerance."""
    passed = True
    logger.info("TEST 16: closed-loop convergence to goal")
    mc = MotionController()
    start = (0.0, 0.0)
    goal = (2.0, 1.0)
    mc.set_goal(goal[0], goal[1])
    logger.info(f"Start: ({start[0]:.0f}, {start[1]:.0f}), Goal: ({goal[0]:.0f}, {goal[1]:.0f})")

    x, y, theta = start[0], start[1], 0.0
    dt = 0.05
    reached_step = None
    for step in range(800):
        if step % 10 == 0:
            logger.debug(f"pose=({x:.2f},{y:.2f})")
        if math.hypot(goal[0] - x, goal[1] - y) < mc.goal_tolerance:
            reached_step = step
            break
        v, omega = mc.compute_control((x, y, theta), dt)
        x += v * math.cos(theta) * dt
        y += v * math.sin(theta) * dt
        theta = mc.normalize_angle(theta + omega * dt)

    logger.debug(f"pose=({x:.2f},{y:.2f})")
    if reached_step is None:
        logger.warning(f"  FAIL did not converge in 800 steps, final pose ({x:.2f}, {y:.2f})")
        passed = False
    else:
        logger.info(f"  OK   reached goal in {reached_step} steps ({reached_step * dt:.1f} s simulated)")
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: Main Method

def main():
    tests = [
        test_pid_proportional,
        test_pid_integral_accumulates,
        test_pid_derivative,
        test_pid_output_limits,
        test_pid_anti_windup,
        test_pid_zero_dt,
        test_pid_reset,
        test_no_goal_zero_command,
        test_drive_toward_goal_ahead,
        test_sharp_turn_slows_down,
        test_stops_inside_tolerance,
        test_rotates_to_goal_heading,
        test_goal_reached_no_goal,
        test_goal_reached_with_goal,
        test_normalize_angle,
        test_closed_loop_convergence,
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

    logger.info("Motion Controller Test Suite")
    results = [t() for t in selected]

    logger.info(f"Results: {sum(results)}/{len(results)} passed")
    logger.info("All tests complete.")
    sys.exit(0 if all(results) else 1)


if __name__ == '__main__':
    main()
