#!/usr/bin/env python3
"""
Integration test: global planner (A*) + local planner (DWA)

This is the architecture that actually navigates a dense obstacle field:
A* plans a global route over the occupancy grid, and DWA follows it by
chasing successive waypoints while reactively respecting the robot's
footprint. Neither planner does this job alone — A* ignores robot size,
DWA is myopic — so the integration is the point.

The suite separates the two responsibilities:
- TEST 1 isolates DWA's path-FOLLOWING on a known corner-safe route, so it
  passes independently of A*'s current state.
- TEST 2 is the true end-to-end test (real A* -> real DWA). It first checks
  that A*'s path is robot-safe (no corner cuts, clearance >= the footprint)
  and, if not, fails with a message pointing at the A* corner-cutting fix.
  It goes green once A* produces safe paths AND DWA scoring is fixed.
- TEST 3 documents the footprint geometry that makes corner cuts unfollowable.
"""
import logging
import math
import sys

import numpy as np

from navi_bot.planners.astar import AStarPlanner
from navi_bot.planners.dwa import DWAPlanner

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)

# Robot footprint vs. an obstacle cell center: robot radius + cell half-width.
# A trajectory point closer than this to an obstacle center is in contact.
ROBOT_RADIUS = 0.25
CELL_RADIUS = 0.5
CONTACT = ROBOT_RADIUS + CELL_RADIUS  # 0.75 m


# MARK: Maps

def obstacle_map():
    return np.array([
        [0, 0, 0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 1, 1, 1],
        [0, 0, 1, 0, 0, 1, 0, 0],
        [0, 1, 1, 1, 0, 1, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0],
        [1, 0, 1, 0, 1, 0, 1, 0],
        [1, 0, 1, 0, 1, 0, 1, 0],
        [1, 0, 1, 0, 1, 0, 1, 0],
    ])


# MARK: Path Safety Helpers

def corner_cut_segments(path, grid):
    """Diagonal segments that squeeze past a blocked orthogonal neighbor."""
    cuts = []
    for a, b in zip(path, path[1:]):
        dr, dc = b[0] - a[0], b[1] - a[1]
        if dr != 0 and dc != 0:
            if grid[a[0]][b[1]] != 0 or grid[b[0]][a[1]] != 0:
                cuts.append((a, b))
    return cuts


def min_clearance(path, grid, resolution=0.1):
    """Smallest distance from any point along the path to an obstacle center."""
    obstacles = np.argwhere(grid == 1)
    if obstacles.size == 0:
        return float('inf')
    worst = float('inf')
    for a, b in zip(path, path[1:]):
        steps = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / resolution))
        for s in range(steps + 1):
            t = s / steps
            px = a[0] + t * (b[0] - a[0])
            py = a[1] + t * (b[1] - a[1])
            for ox, oy in obstacles:
                d = math.hypot(px - ox, py - oy)
                if d < worst:
                    worst = d
    return worst


def path_is_robot_safe(path, grid):
    """(ok, reason): a path an inflated robot can actually follow."""
    if path is None:
        return False, "A* returned no path"
    cuts = corner_cut_segments(path, grid)
    if cuts:
        return False, f"A* path corner-cuts at {cuts[0]} (+{len(cuts)-1} more) — fix astar.py corner-cutting"
    clear = min_clearance(path, grid)
    if clear < CONTACT:
        return False, f"A* path clearance {clear:.3f} < robot footprint {CONTACT}"
    return True, ""


# MARK: DWA Waypoint Follower

def follow_path(dwa, grid, start, theta, waypoints,
                max_replans=200, wp_tol=0.6, goal_tol=0.7, lookahead=1.5):
    """
    Drive DWA along a global path by chasing successive waypoints. Logs
    Start/Goal and pose telemetry so the dashboard can draw the route.
    Returns (reached, collided, left_map, replans, final_pose).
    """
    obstacles = np.argwhere(grid == 1)
    if hasattr(dwa, 'set_occupancy_grid'):
        dwa.set_occupancy_grid(grid)
    x, y, th = float(start[0]), float(start[1]), float(theta)
    v, om = 0.0, 0.0
    wp_idx = 0
    collided = left = False
    goal = waypoints[-1]
    logger.info(f"Start: ({start[0]}, {start[1]}), Goal: ({goal[0]}, {goal[1]})")
    logger.info(f"MAP {len(grid)} {len(grid[0])}")
    for _row in grid:
        logger.info("MAPROW " + "".join('1' if int(_c) != 0 else '0' for _c in _row))
    for i in range(max_replans):
        logger.info(f"pose=({x:.2f},{y:.2f})")
        if math.hypot(goal[0] - x, goal[1] - y) <= goal_tol:
            return True, collided, left, i, (x, y)
        # advance past waypoints we've already reached
        while wp_idx < len(waypoints) - 1 and math.hypot(waypoints[wp_idx][0] - x, waypoints[wp_idx][1] - y) <= wp_tol:
            wp_idx += 1
        # Aim at a LOOKAHEAD point further along the path, not the nearest
        # waypoint: a target ~1 cell away gets overshot inside DWA's 2 s
        # rollout, which stalls the robot at direction-change corners.
        target_idx = wp_idx
        while target_idx < len(waypoints) - 1 and math.hypot(waypoints[target_idx][0] - x, waypoints[target_idx][1] - y) < lookahead:
            target_idx += 1
        pair = dwa.plan((x, y, th), (v, om), waypoints[target_idx], obstacles)
        if pair is None:
            logger.warning("DWA returned None mid-route.")
            return False, collided, left, i, (x, y)
        v, om = pair
        for _ in range(5):  # one 0.5 s control period
            x += v * math.cos(th) * 0.1
            y += v * math.sin(th) * 0.1
            th += om * 0.1
            r, c = int(round(x)), int(round(y))
            if not collided and 0 <= r < len(grid) and 0 <= c < len(grid[0]) and grid[r][c] != 0:
                logger.warning(f"Collision at cell ({r}, {c}), pose=({x:.2f},{y:.2f})")
                collided = True
            if not left and not (-0.5 <= x <= len(grid) - 0.5 and -0.5 <= y <= len(grid[0]) - 0.5):
                logger.warning(f"Left the map at pose=({x:.2f},{y:.2f})")
                left = True
    logger.warning(f"Did not reach goal after {max_replans} replans.")
    return False, collided, left, max_replans, (x, y)


# MARK: Tests

def test_dwa_follows_safe_path():
    """DWA must follow a known corner-safe global route to the goal.

    Hand-authored L-route (down free column 0, along free row 4) with 1.0
    clearance everywhere — isolates DWA's path-following from A*. Passes
    independently of A*'s corner-cutting bug.
    """
    passed = True
    logger.info("TEST 1: DWA follows a corner-safe global path")
    grid = obstacle_map()
    safe = [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0),
            (4, 1), (4, 2), (4, 3), (4, 4), (4, 5), (4, 6), (4, 7)]
    ok, reason = path_is_robot_safe(safe, grid)
    if not ok:
        logger.error(f"  test setup error: reference route not safe — {reason}")
        logger.info("FAIL")
        return False
    reached, collided, left, replans, pose = follow_path(
        DWAPlanner(), grid, (0, 0), 0.0, safe)
    if reached and not collided and not left:
        logger.info(f"  PASS: followed the safe route to goal in {replans} replans")
    else:
        why = ("collision" if collided else "left map" if left
               else f"did not reach goal, final pose ({pose[0]:.2f}, {pose[1]:.2f})")
        logger.error(f"  FAIL: {why}")
        passed = False
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_astar_dwa_end_to_end():
    """End-to-end: A* plans, DWA follows, robot reaches goal without collision.

    Acceptance test for the full navigation stack. Requires A* to emit a
    robot-safe path (no corner cuts, clearance >= footprint) AND DWA scoring
    that follows it without stalling. Currently RED — see the failure reason.
    """
    passed = True
    logger.info("TEST 2: A* plans -> DWA follows -> goal (end-to-end)")
    grid = obstacle_map()
    start, goal = (0, 0), (4, 7)

    astar = AStarPlanner()
    astar.set_occupancy_grid(grid)
    path = astar.plan(start, goal)
    logger.info(f"  A* path: {path}")

    ok, reason = path_is_robot_safe(path, grid)
    if not ok:
        logger.error(f"  FAIL: {reason}")
        logger.info("FAIL")
        return False

    reached, collided, left, replans, pose = follow_path(
        DWAPlanner(), grid, start, 0.0, path)
    if reached and not collided and not left:
        logger.info(f"  PASS: A*+DWA reached the goal in {replans} replans")
    else:
        why = ("collision" if collided else "left map" if left
               else f"did not reach goal, final pose ({pose[0]:.2f}, {pose[1]:.2f})")
        logger.error(f"  FAIL: {why}")
        passed = False
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_moving_obstacle():
    """DWA must avoid a moving obstacle that crosses the path, then reach goal.

    A* plans on the static (empty) map — a straight route along row 4. A
    moving hazard descends column 4 and crosses the robot's path mid-route.
    It is invisible to A* (never in the grid); DWA sees it in the obstacle
    set each cycle and must avoid it reactively, then proceed once it passes.
    Asserts the hazard genuinely interfered (came close) yet was never hit.
    """
    logger.info("TEST 4: DWA avoids a moving obstacle crossing the path")
    grid = np.zeros((8, 8), dtype=int)   # open map (no static obstacles)
    start, goal, theta = (4, 0), (4, 7), 0.0
    static = np.argwhere(grid == 1)

    astar = AStarPlanner()
    astar.set_occupancy_grid(grid)
    path = astar.plan(start, goal)
    if path is None:
        logger.error("  FAIL: A* found no path on the open map")
        logger.info("FAIL")
        return False

    def hazard_at(step):
        # descends column 4 from the top, one row every 3 replans
        return (min(7, step // 3), 4)

    dwa = DWAPlanner()
    dwa.set_occupancy_grid(grid)
    x, y, th = float(start[0]), float(start[1]), float(theta)
    v, om = 0.0, 0.0
    wp_idx = 0
    lookahead = 1.5
    max_replans = 160
    collided = False
    max_dev = 0.0          # farthest the robot swerved off the row-4 path
    min_dist = float('inf')
    reached = False
    i = 0

    # "(open map)" tells the dashboard to draw a clear grid for this case,
    # despite "obstacle" appearing elsewhere in the log.
    logger.info(f"Start: ({start[0]}, {start[1]}), Goal: ({goal[0]}, {goal[1]})")
    logger.info(f"MAP {len(grid)} {len(grid[0])}")
    for _row in grid:
        logger.info("MAPROW " + "".join('1' if int(_c) != 0 else '0' for _c in _row))
    for i in range(max_replans):
        hz = hazard_at(i)
        logger.info(f"pose=({x:.2f},{y:.2f})")
        logger.debug(f"hazard at ({hz[0]},{hz[1]})")
        min_dist = min(min_dist, math.hypot(hz[0] - x, hz[1] - y))
        max_dev = max(max_dev, abs(x - start[0]))  # swerve off the straight path
        if math.hypot(goal[0] - x, goal[1] - y) <= 0.7:
            reached = True
            break
        # lookahead target along the global path
        while wp_idx < len(path) - 1 and math.hypot(path[wp_idx][0] - x, path[wp_idx][1] - y) <= 0.6:
            wp_idx += 1
        target_idx = wp_idx
        while target_idx < len(path) - 1 and math.hypot(path[target_idx][0] - x, path[target_idx][1] - y) < lookahead:
            target_idx += 1
        obstacles = np.array([list(hz)]) if static.size == 0 else np.vstack([static, [hz]])
        pair = dwa.plan((x, y, th), (v, om), path[target_idx], obstacles)
        if pair is None:
            v, om = 0.0, 0.0   # hold position this cycle
            continue
        v, om = pair
        for _ in range(5):  # one 0.5 s control period
            x += v * math.cos(th) * 0.1
            y += v * math.sin(th) * 0.1
            th += om * 0.1
            if math.hypot(hz[0] - x, hz[1] - y) < CONTACT:
                collided = True

    if reached and not collided and max_dev > 0.8:
        logger.info(f"  PASS: swerved {max_dev:.2f} off-path around the hazard (closest {min_dist:.2f}), reached goal in {i} replans")
        logger.info("PASS")
        return True
    if collided:
        logger.error(f"  FAIL: collided with the moving hazard (closest {min_dist:.2f})")
    elif not reached:
        logger.error(f"  FAIL: did not reach goal, final pose ({x:.2f}, {y:.2f})")
    else:
        logger.error(f"  FAIL: robot never swerved (max dev {max_dev:.2f}) — hazard didn't interfere")
    logger.info("FAIL")
    return False


def test_corner_cut_path_is_unfollowable():
    """A corner-cutting diagonal must be flagged as below the robot footprint.

    Documents WHY the A* fix matters to DWA: a diagonal squeezing between
    two corner-touching obstacle cells has clearance sqrt(0.5) ~= 0.707,
    below the 0.75 footprint, so no inflated robot can follow it.
    """
    passed = True
    logger.info("TEST 3: corner-cutting paths fall inside the robot footprint")
    grid = np.zeros((8, 8), dtype=int)
    grid[2][2] = 1
    grid[3][3] = 1
    # diagonal (2,3)->(3,2) squeezes between the two corner-touching obstacles
    cutting = [(1, 4), (2, 3), (3, 2), (4, 1)]
    cuts = corner_cut_segments(cutting, grid)
    clear = min_clearance(cutting, grid)
    if not cuts:
        logger.error("  FAIL: corner-cut detector missed the squeeze")
        passed = False
    elif clear >= CONTACT:
        logger.error(f"  FAIL: clearance {clear:.3f} unexpectedly >= footprint {CONTACT}")
        passed = False
    else:
        logger.info(f"  PASS: clearance {clear:.3f} < footprint {CONTACT}; {len(cuts)} corner cut(s) flagged")
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: Main Method

def main():
    tests = [
        test_dwa_follows_safe_path,
        test_astar_dwa_end_to_end,
        test_corner_cut_path_is_unfollowable,
        test_moving_obstacle,
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

    logger.info("A* + DWA Navigation Integration Test Suite")
    results = [t() for t in selected]

    logger.info(f"Results: {sum(results)}/{len(results)} passed")
    logger.info("All tests complete.")
    sys.exit(0 if all(results) else 1)


if __name__ == '__main__':
    main()
