#!/usr/bin/env python3
"""
Test script for RobotController from robot_controller.py

RobotController is the main control-loop node: it filters a LIDAR scan into an
obstacle set, tracks the latest pose, computes a velocity command, applies a
safety governor (full stop when very close, halve when near), and publishes on
/cmd_vel.

These tests pin the OBSERVABLE contract — sensor filtering, the safety
governor, and command publishing — independent of the internal control law.
TESTs 1-15 predate the follower rebuild and still pass unchanged (point-goal
PID mode). TESTs 16+ cover the PATH mode: the PathFollower's waypoint
bookkeeping, /planned_path adoption, DWA-driven following, arrival reporting
on /nav_status, and the governor backstopping path mode too.
"""
import logging
import math
import sys
from types import SimpleNamespace

from navi_bot.robot_controller import RobotController
from navi_bot.control.path_follower import PathFollower
from navi_bot.mock_ros2 import LaserScan, Pose2D, Path, Point


logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)


# MARK: Setup

def setup_controller():
    """Create a fresh RobotController for each test."""
    return RobotController()


def make_scan(ranges, angle_min=-math.pi, angle_increment=0.1,
              range_min=0.1, range_max=10.0):
    """Build a LaserScan with the given ranges (and sane defaults)."""
    scan = LaserScan()
    scan.ranges = list(ranges)
    scan.angle_min = angle_min
    scan.angle_increment = angle_increment
    scan.range_min = range_min
    scan.range_max = range_max
    return scan


def last_cmd(controller):
    """The most recently published /cmd_vel Twist, or None."""
    return controller.cmd_vel_pub.last_msg


# MARK: Construction

def test_init_state_is_nominal():
    """Construction — battery/miss/obstacle state must start nominal."""
    passed = True
    logger.info("TEST 1: Construction — initial state must be nominal")
    c = setup_controller()
    checks = {
        'battery_level == 100.0': c.battery_level == 100.0,
        'deadline_misses == 0': c.deadline_misses == 0,
        'closest_obstacle_dist == inf': c.closest_obstacle_dist == float('inf'),
        'valid_obstacles == []': c.valid_obstacles == [],
        'obstacle_threshold == 1.5': c.obstacle_threshold == 1.5,
    }
    for name, ok in checks.items():
        if ok:
            logger.info(f"  OK   {name}")
        else:
            logger.warning(f"  FAIL {name}")
            passed = False
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_cmd_vel_publisher_created():
    """Construction — a /cmd_vel publisher must exist."""
    passed = False
    logger.info("TEST 2: Construction — /cmd_vel publisher must exist")
    c = setup_controller()
    pub = getattr(c, 'cmd_vel_pub', None)
    if pub is not None and getattr(pub, 'topic', None) == '/cmd_vel':
        logger.info("  OK   cmd_vel_pub on /cmd_vel")
        passed = True
    else:
        logger.warning(f"  FAIL cmd_vel_pub={pub!r}")
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: LIDAR callback

def test_lidar_populates_close_obstacles():
    """lidar_callback — in-range readings within threshold become obstacles."""
    passed = True
    logger.info("TEST 3: lidar_callback — close in-range readings become obstacles")
    c = setup_controller()
    # index 0: 0.5 (obstacle), 1: 2.0 (in-range but > threshold), 2: 0.8 (obstacle)
    c.lidar_callback(make_scan([0.5, 2.0, 0.8]))
    if len(c.valid_obstacles) != 2:
        logger.warning(f"  FAIL expected 2 obstacles, got {len(c.valid_obstacles)}")
        passed = False
    else:
        logger.info("  OK   2 obstacles within threshold")
    if abs(c.closest_obstacle_dist - 0.5) > 1e-9:
        logger.warning(f"  FAIL closest={c.closest_obstacle_dist}, expected 0.5")
        passed = False
    else:
        logger.info("  OK   closest is 0.5")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_lidar_excludes_beyond_threshold():
    """lidar_callback — in-range readings beyond obstacle_threshold are excluded."""
    passed = False
    logger.info("TEST 4: lidar_callback — readings beyond threshold are excluded")
    c = setup_controller()
    c.lidar_callback(make_scan([2.0, 3.0, 5.0]))  # all in [0.1, 10] but > 1.5
    if c.valid_obstacles == [] and c.closest_obstacle_dist == float('inf'):
        logger.info("  OK   nothing within threshold -> no obstacles")
        passed = True
    else:
        logger.warning(f"  FAIL obstacles={c.valid_obstacles}, closest={c.closest_obstacle_dist}")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_lidar_ignores_out_of_range():
    """lidar_callback — readings outside [range_min, range_max] are ignored."""
    passed = False
    logger.info("TEST 5: lidar_callback — out-of-range readings are ignored")
    c = setup_controller()
    # 0.05 < range_min, 12.0 > range_max, 0.7 valid
    c.lidar_callback(make_scan([0.05, 12.0, 0.7]))
    if len(c.valid_obstacles) == 1 and abs(c.closest_obstacle_dist - 0.7) < 1e-9:
        logger.info("  OK   only the in-range 0.7 reading counted")
        passed = True
    else:
        logger.warning(f"  FAIL obstacles={c.valid_obstacles}, closest={c.closest_obstacle_dist}")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_lidar_resets_each_scan():
    """lidar_callback — obstacle state resets at the start of each scan."""
    passed = False
    logger.info("TEST 6: lidar_callback — state resets between scans")
    c = setup_controller()
    c.lidar_callback(make_scan([0.3, 0.9]))       # cluttered
    c.lidar_callback(make_scan([5.0, 6.0, 7.0]))  # all clear (in-range, > threshold)
    if c.valid_obstacles == [] and c.closest_obstacle_dist == float('inf'):
        logger.info("  OK   prior scan's obstacles cleared")
        passed = True
    else:
        logger.warning(f"  FAIL obstacles={c.valid_obstacles}, closest={c.closest_obstacle_dist}")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_lidar_closest_matches_nearest():
    """lidar_callback — closest dist/angle track the nearest reading."""
    passed = True
    logger.info("TEST 7: lidar_callback — closest tracks the nearest reading")
    c = setup_controller()
    c.lidar_callback(make_scan([0.9, 0.3, 1.2]))  # nearest is index 1 (0.3)
    if abs(c.closest_obstacle_dist - 0.3) > 1e-9:
        logger.warning(f"  FAIL closest={c.closest_obstacle_dist}, expected 0.3")
        passed = False
    else:
        logger.info("  OK   closest dist 0.3")
    expected_angle = -math.pi + 1 * 0.1
    if abs(c.closest_obstacle_angle - expected_angle) > 1e-9:
        logger.warning(f"  FAIL angle={c.closest_obstacle_angle}, expected {expected_angle}")
        passed = False
    else:
        logger.info("  OK   closest angle matches index 1")
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: Pose callback

def test_pose_callback_updates_pose():
    """pose_callback — stores the latest pose."""
    passed = False
    logger.info("TEST 8: pose_callback — updates current_pose")
    c = setup_controller()
    p = Pose2D()
    p.x, p.y, p.theta = 1.0, 2.0, 0.5
    c.pose_callback(p)
    if c.current_pose is p and c.current_pose.x == 1.0 and c.current_pose.y == 2.0:
        logger.info("  OK   current_pose updated")
        passed = True
    else:
        logger.warning(f"  FAIL current_pose=({c.current_pose.x}, {c.current_pose.y})")
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: set_goal

def test_set_goal_delegates_to_motion_controller():
    """set_goal — forwards the goal to the motion controller."""
    passed = False
    logger.info("TEST 9: set_goal — delegates to motion_controller")
    c = setup_controller()
    c.set_goal(3.0, 4.0, 0.0)
    goal = c.motion_controller.current_goal
    if goal == (3.0, 4.0, 0.0):
        logger.info("  OK   motion_controller goal set")
        passed = True
    else:
        logger.warning(f"  FAIL motion_controller.current_goal={goal}")
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: Control loop

def test_control_loop_no_goal_publishes_zero():
    """control_loop — with no goal, publishes a zero command."""
    passed = False
    logger.info("TEST 10: control_loop — no goal publishes zero cmd_vel")
    c = setup_controller()
    c.control_loop()
    cmd = last_cmd(c)
    if cmd is not None and cmd.linear.x == 0.0 and cmd.angular.z == 0.0:
        logger.info("  OK   published (0, 0)")
        passed = True
    elif cmd is None:
        logger.warning("  FAIL nothing published")
    else:
        logger.warning(f"  FAIL cmd=({cmd.linear.x}, {cmd.angular.z})")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_control_loop_publishes_each_call():
    """control_loop — publishes a /cmd_vel every call."""
    passed = False
    logger.info("TEST 11: control_loop — publishes a command each call")
    c = setup_controller()
    before = len(c.cmd_vel_pub.published_msgs)
    c.control_loop()
    after = len(c.cmd_vel_pub.published_msgs)
    if after == before + 1:
        logger.info("  OK   one command published")
        passed = True
    else:
        logger.warning(f"  FAIL published {after - before} messages")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_control_loop_moves_toward_goal_when_clear():
    """control_loop — with a goal and clear path, commands nonzero velocity."""
    passed = False
    logger.info("TEST 12: control_loop — clear path yields a nonzero command")
    c = setup_controller()
    c.set_goal(5.0, 6.0)
    c.closest_obstacle_dist = float('inf')
    c.control_loop()
    cmd = last_cmd(c)
    if cmd is not None and abs(cmd.linear.x) > 0.0:
        logger.info(f"  OK   nonzero forward velocity {cmd.linear.x:.3f}")
        passed = True
    else:
        logger.warning(f"  FAIL linear.x={cmd.linear.x if cmd else None}")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_control_loop_stops_for_very_close_obstacle():
    """control_loop — an obstacle < 0.5 m forces a full stop."""
    passed = False
    logger.info("TEST 13: control_loop — obstacle < 0.5 m stops the robot")
    c = setup_controller()
    c.set_goal(5.0, 6.0)              # would otherwise command motion
    c.closest_obstacle_dist = 0.3
    c.control_loop()
    cmd = last_cmd(c)
    if cmd is not None and cmd.linear.x == 0 and cmd.angular.z == 0:
        logger.info("  OK   stopped for close obstacle")
        passed = True
    else:
        logger.warning(f"  FAIL cmd=({cmd.linear.x}, {cmd.angular.z})")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_control_loop_halves_for_near_obstacle():
    """control_loop — an obstacle in [0.5, 1.0) m halves the command."""
    passed = True
    logger.info("TEST 14: control_loop — obstacle in [0.5,1.0) halves the command")
    # Two fresh controllers with identical goal/pose produce identical raw
    # commands (fresh PID state); only the obstacle distance differs, isolating
    # the 0.5x safety scaling.
    clear = setup_controller()
    clear.set_goal(5.0, 6.0)
    clear.closest_obstacle_dist = float('inf')
    clear.control_loop()
    raw = last_cmd(clear)

    near = setup_controller()
    near.set_goal(5.0, 6.0)
    near.closest_obstacle_dist = 0.75
    near.control_loop()
    scaled = last_cmd(near)

    if abs(scaled.linear.x - 0.5 * raw.linear.x) > 1e-9:
        logger.warning(f"  FAIL linear.x {scaled.linear.x} != 0.5*{raw.linear.x}")
        passed = False
    else:
        logger.info("  OK   linear.x halved")
    if abs(scaled.angular.z - 0.5 * raw.angular.z) > 1e-9:
        logger.warning(f"  FAIL angular.z {scaled.angular.z} != 0.5*{raw.angular.z}")
        passed = False
    else:
        logger.info("  OK   angular.z halved")
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: Shutdown

def test_shutdown_publishes_stop():
    """shutdown — publishes a zero (stop) command."""
    passed = False
    logger.info("TEST 15: shutdown — publishes a stop command")
    c = setup_controller()
    c.shutdown()
    cmd = last_cmd(c)
    if cmd is not None and cmd.linear.x == 0.0 and cmd.angular.z == 0.0:
        logger.info("  OK   stop command published")
        passed = True
    else:
        logger.warning("  FAIL no zero command on shutdown")
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: Path follower (pure logic)

def test_follower_lookahead_targeting():
    """PathFollower — advances past reached waypoints and aims a lookahead
    target, never the nearest waypoint."""
    passed = True
    logger.info("TEST 16: PathFollower — waypoint advance + lookahead targeting")
    f = PathFollower(wp_tol=0.6, goal_tol=0.7, lookahead=1.0)
    f.set_path([(0, 0), (1, 0), (2, 0), (3, 0)])
    t = f.target(0.0, 0.0)
    if t != (1.0, 0.0):
        logger.warning(f"  FAIL from origin expected lookahead target (1,0), got {t}")
        passed = False
    else:
        logger.info("  OK   aims past the co-located waypoint at (1,0)")
    t = f.target(1.5, 0.0)     # waypoints 0,1 now within wp_tol/behind
    if t != (3.0, 0.0):
        logger.warning(f"  FAIL from (1.5,0) expected (3,0) (>=1.0 away), got {t}")
        passed = False
    else:
        logger.info("  OK   advances and aims >= lookahead ahead at (3,0)")
    if f.target(10.0, 10.0) != (3.0, 0.0):
        logger.warning("  FAIL far from path should still aim at final waypoint")
        passed = False
    else:
        logger.info("  OK   final waypoint is the terminal target")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_follower_goal_and_clear():
    """PathFollower — goal detection within goal_tol; clear() drops the path."""
    passed = True
    logger.info("TEST 17: PathFollower — goal detection and clear")
    f = PathFollower(goal_tol=0.7)
    f.set_path([(0, 0), (5, 5)])
    checks = [
        ('has_path after set', f.has_path()),
        ('not at goal from origin', not f.goal_reached(0.0, 0.0)),
        ('at goal within tol', f.goal_reached(4.8, 4.8)),
    ]
    f.clear()
    checks += [
        ('no path after clear', not f.has_path()),
        ('no goal without path', not f.goal_reached(5.0, 5.0)),
        ('no target without path', f.target(0.0, 0.0) is None),
    ]
    for name, ok in checks:
        if ok:
            logger.info(f"  OK   {name}")
        else:
            logger.warning(f"  FAIL {name}")
            passed = False
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: Path mode (follower + DWA)

def make_path_msg(waypoints):
    """Mock /planned_path message: Path with bare Points (as path_planner sends)."""
    msg = Path()
    for x, y in waypoints:
        p = Point()
        p.x, p.y = float(x), float(y)
        msg.poses.append(p)
    return msg


def test_path_callback_adopts_path():
    """path_callback — adopts the path into follower AND DWA's path term."""
    passed = True
    logger.info("TEST 18: path_callback — follower and DWA both receive the path")
    c = setup_controller()
    c.path_callback(make_path_msg([(0, 0), (1, 1), (2, 2)]))
    if not c.follower.has_path():
        logger.warning("  FAIL follower has no path")
        passed = False
    else:
        logger.info("  OK   follower adopted the path")
    if c.dwa.global_path != [(0.0, 0.0), (1.0, 1.0), (2.0, 2.0)]:
        logger.warning(f"  FAIL DWA global_path={c.dwa.global_path}")
        passed = False
    else:
        logger.info("  OK   DWA path-following term fed (anti-limit-cycle)")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_path_callback_real_ros_shape():
    """path_callback — also accepts real nav_msgs/Path (PoseStamped poses)."""
    passed = False
    logger.info("TEST 19: path_callback — nav_msgs/Path shape (pose.position) accepted")
    c = setup_controller()
    stamped = lambda x, y: SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=x, y=y)))
    c.path_callback(SimpleNamespace(poses=[stamped(0.0, 0.0), stamped(3.0, 4.0)]))
    if c.follower.has_path() and c.follower.path[-1] == (3.0, 4.0):
        logger.info("  OK   PoseStamped-shaped path adopted")
        passed = True
    else:
        logger.warning(f"  FAIL follower path={c.follower.path}")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_path_mode_drives_along_path():
    """control_loop — with a path and clear floor, publishes a nonzero command
    (DWA following, not the PID)."""
    passed = False
    logger.info("TEST 20: control_loop — path mode produces a driving command")
    c = setup_controller()
    c.path_callback(make_path_msg([(0, 0), (2, 0), (4, 0), (6, 0)]))
    c.control_loop()
    cmd = last_cmd(c)
    if cmd is not None and (abs(cmd.linear.x) > 0.0 or abs(cmd.angular.z) > 0.0):
        logger.info(f"  OK   driving command ({cmd.linear.x:.3f}, {cmd.angular.z:.3f})")
        passed = True
    else:
        logger.warning(f"  FAIL cmd={(cmd.linear.x, cmd.angular.z) if cmd else None}")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_path_mode_goal_reached_reports_nav_status():
    """control_loop — arrival publishes 'goal_reached' on /nav_status, stops,
    and clears the path (returns to point-goal mode)."""
    passed = True
    logger.info("TEST 21: control_loop — arrival reports /nav_status and clears the path")
    c = setup_controller()
    c.path_callback(make_path_msg([(0, 0), (1, 1)]))
    p = Pose2D()
    p.x, p.y, p.theta = 1.0, 1.0, 0.0     # already at the final waypoint
    c.pose_callback(p)
    c.control_loop()
    cmd = last_cmd(c)
    status = c.nav_status_pub.last_msg
    if status is None or status.data != 'goal_reached':
        logger.warning(f"  FAIL nav_status={getattr(status, 'data', None)}")
        passed = False
    else:
        logger.info("  OK   published nav_status 'goal_reached'")
    if cmd.linear.x != 0.0 or cmd.angular.z != 0.0:
        logger.warning(f"  FAIL expected stop, got ({cmd.linear.x}, {cmd.angular.z})")
        passed = False
    else:
        logger.info("  OK   stop command on arrival")
    if c.follower.has_path():
        logger.warning("  FAIL path not cleared after arrival")
        passed = False
    else:
        logger.info("  OK   path cleared — back to point-goal mode")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_path_mode_governor_still_stops():
    """control_loop — the < 0.5 m safety stop overrides path mode too."""
    passed = False
    logger.info("TEST 22: control_loop — safety governor stops the robot in path mode")
    c = setup_controller()
    c.path_callback(make_path_msg([(0, 0), (2, 0), (4, 0), (6, 0)]))
    c.closest_obstacle_dist = 0.3
    c.control_loop()
    cmd = last_cmd(c)
    if cmd is not None and cmd.linear.x == 0.0 and cmd.angular.z == 0.0:
        logger.info("  OK   full stop despite an active path")
        passed = True
    else:
        logger.warning(f"  FAIL cmd=({cmd.linear.x}, {cmd.angular.z})")
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: Runner

def main():
    tests = [
        test_init_state_is_nominal,
        test_cmd_vel_publisher_created,
        test_lidar_populates_close_obstacles,
        test_lidar_excludes_beyond_threshold,
        test_lidar_ignores_out_of_range,
        test_lidar_resets_each_scan,
        test_lidar_closest_matches_nearest,
        test_pose_callback_updates_pose,
        test_set_goal_delegates_to_motion_controller,
        test_control_loop_no_goal_publishes_zero,
        test_control_loop_publishes_each_call,
        test_control_loop_moves_toward_goal_when_clear,
        test_control_loop_stops_for_very_close_obstacle,
        test_control_loop_halves_for_near_obstacle,
        test_shutdown_publishes_stop,
        test_follower_lookahead_targeting,
        test_follower_goal_and_clear,
        test_path_callback_adopts_path,
        test_path_callback_real_ros_shape,
        test_path_mode_drives_along_path,
        test_path_mode_goal_reached_reports_nav_status,
        test_path_mode_governor_still_stops,
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

    logger.info("Robot Controller Test Suite")
    results = [t() for t in selected]

    logger.info(f"Results: {sum(results)}/{len(results)} passed")
    logger.info("All tests complete.")
    sys.exit(0 if all(results) else 1)


if __name__ == '__main__':
    main()
