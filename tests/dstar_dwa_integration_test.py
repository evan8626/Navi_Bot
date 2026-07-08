#!/usr/bin/env python3
"""
Integration test: global planner (D* Lite) + local planner (DWA)

The D* Lite counterpart to astar_dwa_integration_test.py. D* Lite plans a
global route; DWA follows it by chasing a lookahead point while reactively
respecting the robot's footprint. D* Lite's distinctive value over A* is
incremental REPLANNING when the map changes, so this suite leans on that:

- TEST 1 is the end-to-end happy path (D* Lite plans -> DWA follows -> goal).
- TEST 2 is the signature scenario: an obstacle appears on the route, D* Lite
  replans incrementally, and DWA follows the updated path to the goal.
- TEST 3 is the inverse: an obstacle is removed, the shortcut opens, and the
  replan exploits it.

These are acceptance tests: they go green once D* Lite emits robot-safe paths
and replans correctly (its suite's TESTs 10-13), and DWA follows with lookahead.
"""
import logging
import math
import sys

import numpy as np

from navi_bot.planners.dstar_lite import DStarLitePlanner
from navi_bot.planners.dwa import DWAPlanner

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)

ROBOT_RADIUS = 0.25
CELL_RADIUS = 0.5
CONTACT = ROBOT_RADIUS + CELL_RADIUS  # 0.75 m


# MARK: Maps

def clear_map():
    return np.zeros((8, 8), dtype=int)


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
    cuts = []
    for a, b in zip(path, path[1:]):
        dr, dc = b[0] - a[0], b[1] - a[1]
        if dr != 0 and dc != 0:
            if grid[a[0]][b[1]] != 0 or grid[b[0]][a[1]] != 0:
                cuts.append((a, b))
    return cuts


def min_clearance(path, grid, resolution=0.1):
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
        return False, "D* Lite returned no path"
    if len(path) < 2:
        return False, f"path too short: {path}"
    cuts = corner_cut_segments(path, grid)
    if cuts:
        return False, f"path corner-cuts at {cuts[0]} (+{len(cuts)-1} more)"
    if any(grid[r][c] != 0 for r, c in path):
        return False, "path passes through an occupied cell"
    clear = min_clearance(path, grid)
    if clear < CONTACT:
        return False, f"path clearance {clear:.3f} < robot footprint {CONTACT}"
    return True, ""


# MARK: DWA Waypoint Follower (lookahead — aim ahead, not at the nearest waypoint)

def follow_path(dwa, grid, start, theta, waypoints,
                max_replans=200, wp_tol=0.6, goal_tol=0.7, lookahead=1.0):
    obstacles = np.argwhere(grid == 1)
    if hasattr(dwa, 'set_occupancy_grid'):
        dwa.set_occupancy_grid(grid)
    dwa.set_global_path(waypoints)  # global path for the anti-limit-cycle path term
    # Start heading along the path's first segment (motion model: x=row uses
    # cos, y=col uses sin -> atan2(dcol, drow)), not the passed default —
    # otherwise the robot starts ~90 deg off an east-west path and spirals.
    th = (math.atan2(waypoints[1][1] - waypoints[0][1], waypoints[1][0] - waypoints[0][0])
          if len(waypoints) >= 2 else float(theta))
    x, y = float(start[0]), float(start[1])
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
        while wp_idx < len(waypoints) - 1 and math.hypot(waypoints[wp_idx][0] - x, waypoints[wp_idx][1] - y) <= wp_tol:
            wp_idx += 1
        target_idx = wp_idx
        while target_idx < len(waypoints) - 1 and math.hypot(waypoints[target_idx][0] - x, waypoints[target_idx][1] - y) < lookahead:
            target_idx += 1
        pair = dwa.plan((x, y, th), (v, om), waypoints[target_idx], obstacles)
        if pair is None:
            logger.warning("DWA returned None mid-route.")
            return False, collided, left, i, (x, y)
        v, om = pair
        for _ in range(5):
            x += v * math.cos(th) * 0.1
            y += v * math.sin(th) * 0.1
            th += om * 0.1
            r, c = int(round(x)), int(round(y))
            if not collided and 0 <= r < len(grid) and 0 <= c < len(grid[0]) and grid[r][c] != 0:
                logger.warning(f"Collision at cell ({r}, {c}), pose=({x:.2f},{y:.2f})")
                collided = True
            if not left and not (-0.5 <= x <= len(grid) - 0.5 and -0.5 <= y <= len(grid[0]) - 0.5):
                left = True
    logger.warning(f"Did not reach goal after {max_replans} replans.")
    return False, collided, left, max_replans, (x, y)


def log_path(path):
    for r, c in path:
        logger.info(f"X: {r}, Y: {c}")


def report(reached, collided, left, pose, replans, what):
    if reached and not collided and not left:
        logger.info(f"  PASS: {what} in {replans} replans")
        logger.info("PASS")
        return True
    why = ("collision" if collided else "left map" if left
           else f"did not reach goal, final pose ({pose[0]:.2f}, {pose[1]:.2f})")
    logger.error(f"  FAIL: {why}")
    logger.info("FAIL")
    return False


# MARK: Tests

def test_dstar_dwa_end_to_end():
    """End-to-end: D* Lite plans, DWA follows, robot reaches the goal."""
    logger.info("TEST 1: D* Lite plans -> DWA follows -> goal (end-to-end)")
    grid = obstacle_map()
    start, goal = (0, 0), (4, 7)

    dstar = DStarLitePlanner()
    dstar.set_occupancy_grid(grid)
    path = dstar.plan(start, goal)
    logger.info(f"  D* Lite path: {path}")
    ok, reason = path_is_robot_safe(path, grid)
    if not ok:
        logger.error(f"  FAIL: {reason}")
        logger.info("FAIL")
        return False
    log_path(path)

    reached, collided, left, replans, pose = follow_path(DWAPlanner(), grid, start, 0.0, path)
    return report(reached, collided, left, pose, replans, "D*+DWA reached the goal")


def test_dstar_replan_after_obstacle_appears():
    """An obstacle appears on the route; D* Lite replans, DWA follows it."""
    logger.info("TEST 2: obstacle appears -> D* Lite replans -> DWA follows")
    start, goal = (2, 0), (2, 7)   # row 2 (interior) so the follower has room on both sides

    dstar = DStarLitePlanner()
    grid_a = clear_map()
    dstar.set_occupancy_grid(grid_a)
    path1 = dstar.plan(start, goal)
    ok, reason = path_is_robot_safe(path1, grid_a)
    if not ok:
        logger.error(f"  FAIL: initial plan not usable — {reason}")
        logger.info("FAIL")
        return False
    logger.info(f"  initial path: {len(path1)} waypoints")

    # Robot has advanced; a wall now blocks the direct route ahead.
    new_start = tuple(path1[min(2, len(path1) - 2)])
    grid_b = clear_map()
    grid_b[1][4] = 1
    grid_b[2][4] = 1
    dstar.set_occupancy_grid(grid_b)
    path2 = dstar.plan(new_start, goal)
    ok, reason = path_is_robot_safe(path2, grid_b)
    if not ok:
        logger.error(f"  FAIL: replan not usable — {reason}")
        logger.info("FAIL")
        return False
    log_path(path2)

    reached, collided, left, replans, pose = follow_path(DWAPlanner(), grid_b, new_start, 0.0, path2)
    return report(reached, collided, left, pose, replans, "replanned around the wall and DWA reached goal")


def test_dstar_replan_after_obstacle_removed():
    """A wall is removed, opening a shortcut; D* Lite replans, DWA follows it."""
    logger.info("TEST 3: obstacle removed -> D* Lite replans shorter -> DWA follows")
    start, goal = (2, 0), (2, 7)   # row 2 (interior) so the driven path doesn't hug the edge

    dstar = DStarLitePlanner()
    walled = clear_map()
    for r in range(0, 7):
        walled[r][3] = 1   # wall down column 3, gap only at the bottom row
    dstar.set_occupancy_grid(walled)
    path1 = dstar.plan(start, goal)
    ok, reason = path_is_robot_safe(path1, walled)
    if not ok:
        logger.error(f"  FAIL: initial (walled) plan not usable — {reason}")
        logger.info("FAIL")
        return False

    dstar.set_occupancy_grid(clear_map())
    path2 = dstar.plan(start, goal)
    ok, reason = path_is_robot_safe(path2, clear_map())
    if not ok:
        logger.error(f"  FAIL: replan after wall removal not usable — {reason}")
        logger.info("FAIL")
        return False
    if not len(path2) < len(path1):
        logger.error(f"  FAIL: replan did not exploit the opened shortcut ({len(path1)} -> {len(path2)} waypoints)")
        logger.info("FAIL")
        return False
    log_path(path2)

    reached, collided, left, replans, pose = follow_path(DWAPlanner(), clear_map(), start, 0.0, path2)
    return report(reached, collided, left, pose, replans, "took the reopened shortcut and DWA reached goal")


# MARK: Moving-hazard helper + full-stack test

def drive_with_hazard(dwa, grid, start, theta, path, hazard_fn,
                      max_replans=200, wp_tol=0.6, goal_tol=0.7, lookahead=1.5,
                      encounter_r=2.0, yield_speed=0.2):
    """
    Drive DWA along `path` while a moving hazard crosses it. The hazard
    (hazard_fn(step) -> (r, c)) is added to the obstacle set each cycle, as a
    perception tracker would feed it to the local planner. Logs grid + pose +
    "hazard at" telemetry so the dashboard animates both robot and hazard.

    Returns a metrics dict including `yielded`: whether the robot slowed below
    `yield_speed` while the hazard was within `encounter_r` (the desired
    behavior — slow/halt for the crossing hazard rather than dart in front).
    """
    static = np.argwhere(grid == 1)
    if hasattr(dwa, 'set_occupancy_grid'):
        
        dwa.set_occupancy_grid(grid)
    dwa.set_global_path(path)  # global path for the anti-limit-cycle path term
    # Start heading along the path so the robot doesn't swing ~90 deg onto it.
    th = (math.atan2(path[1][1] - path[0][1], path[1][0] - path[0][0])
          if len(path) >= 2 else float(theta))
    x, y = float(start[0]), float(start[1])
    v, om = 0.0, 0.0
    wp_idx = 0
    collided = left = reached = False
    min_dist = float('inf')
    min_speed_near = float('inf')
    goal = path[-1]
    logger.info(f"Start: ({start[0]}, {start[1]}), Goal: ({goal[0]}, {goal[1]})")
    logger.info(f"MAP {len(grid)} {len(grid[0])}")
    for _row in grid:
        logger.info("MAPROW " + "".join('1' if int(_c) != 0 else '0' for _c in _row))
    px, py = x, y
    i = 0
    for i in range(max_replans):
        hz = hazard_fn(i)
        # if hz is not None:
        #     g2 = grid.copy(); g2[hz[0], hz[1]] = 1
        #     dstar.set_occupancy_grid(g2)
        #     new = dstar.plan((int(round(x)), int(round(y))), goal)
        #     if path_is_robot_safe(new, g2)[0]:
        #         path, wp_idx = new, 0
        logger.info(f"pose=({x:.2f},{y:.2f})")
        logger.debug(f"hazard at ({hz[0]},{hz[1]})")
        if i >= 1:                                     # forecast the hazard a few steps ahead (dashboard cloud)
            n = min(4, i)
            hp = hazard_fn(i - n)
            vr, vc = (hz[0] - hp[0]) / n, (hz[1] - hp[1]) / n
            logger.debug("HAZARD " + " ".join(f"{hz[0] + vr*k:.2f},{hz[1] + vc*k:.2f}" for k in range(1, 6)))
        hd = math.hypot(hz[0] - x, hz[1] - y)
        min_dist = min(min_dist, hd)
        if i > 0 and hd < encounter_r:
            min_speed_near = min(min_speed_near, math.hypot(x - px, y - py))
        px, py = x, y
        if math.hypot(goal[0] - x, goal[1] - y) <= goal_tol:
            reached = True
            break
        while wp_idx < len(path) - 1 and math.hypot(path[wp_idx][0] - x, path[wp_idx][1] - y) <= wp_tol:
            wp_idx += 1
        target_idx = wp_idx
        while target_idx < len(path) - 1 and math.hypot(path[target_idx][0] - x, path[target_idx][1] - y) < lookahead:
            target_idx += 1
        obstacles = np.array([list(hz)]) if static.size == 0 else np.vstack([static, [hz]])
        pair = dwa.plan((x, y, th), (v, om), path[target_idx], obstacles)
        if pair is None:
            v, om = 0.0, 0.0
            continue
        v, om = pair
        for _ in range(5):
            x += v * math.cos(th) * 0.1
            y += v * math.sin(th) * 0.1
            th += om * 0.1
            r, c = int(round(x)), int(round(y))
            if not collided and 0 <= r < len(grid) and 0 <= c < len(grid[0]) and grid[r][c] != 0:
                collided = True
            if not collided and math.hypot(hz[0] - x, hz[1] - y) < CONTACT:
                collided = True
            if not left and not (-0.5 <= x <= len(grid) - 0.5 and -0.5 <= y <= len(grid[0]) - 0.5):
                left = True
    return {'reached': reached, 'collided': collided, 'left': left,
            'yielded': min_speed_near < yield_speed,
            'min_dist': min_dist, 'min_speed_near': min_speed_near,
            'replans': i, 'pose': (x, y)}


def report_hazard(res, what):
    """Reach, no collision, the hazard genuinely interfered, and the robot
    YIELDED (slowed/halted) rather than darting in front of it."""
    if (res['reached'] and not res['collided'] and not res['left']
            and res['min_dist'] < 2.0 and res['yielded']):
        logger.info(f"  PASS: {what}; yielded to the hazard (min speed {res['min_speed_near']:.2f}, "
                    f"closest {res['min_dist']:.2f}), reached goal in {res['replans']} replans")
        logger.info("PASS")
        return True
    if res['collided']:
        logger.error(f"  FAIL: collided (closest {res['min_dist']:.2f})")
    elif res['left']:
        logger.error("  FAIL: robot left the map")
    elif not res['reached']:
        logger.error(f"  FAIL: did not reach goal, final pose ({res['pose'][0]:.2f}, {res['pose'][1]:.2f})")
    elif res['min_dist'] >= 2.0:
        logger.error(f"  FAIL: hazard never got close (closest {res['min_dist']:.2f}) — not a meaningful test")
    else:
        logger.error(f"  FAIL: robot did not yield (min speed near hazard {res['min_speed_near']:.2f}) "
                     "— it darted past instead of slowing/halting for the crossing hazard")
    logger.info("FAIL")
    return False


def test_dstar_moving_obstacle_with_static():
    """Full-stack realism: D* Lite routes around static obstacles, DWA follows
    AND yields to a moving hazard crossing the route.

    The map has a static block the global planner must avoid; a hazard descends
    a free channel (column 4) and crosses the planned path in the open. The
    robot must route around the static obstacle, then slow/halt for the
    crossing hazard without colliding. Acceptance test for the DWA yield rule.
    """
    logger.info("TEST 4: D* Lite routes around static obstacles + DWA yields to a moving hazard")
    grid = np.array([
        [0, 0, 1, 0, 0, 0, 0, 0],
        [0, 0, 1, 1, 0, 0, 0, 0],
        [0, 0, 1, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 1, 1],
        [0, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 0, 1, 0, 0, 0, 0],
        [0, 1, 0, 1, 0, 0, 0, 0],
        [0, 1, 0, 1, 0, 0, 0, 0],
    ])
    start, goal = (0, 0), (7, 6)
    dstar = DStarLitePlanner()
    dstar.set_occupancy_grid(grid)
    path = dstar.plan(start, goal)
    ok, reason = path_is_robot_safe(path, grid)
    if not ok:
        logger.error(f"  FAIL: D* Lite path not usable — {reason}")
        logger.info("FAIL")
        return False
    log_path(path)
    res = drive_with_hazard(DWAPlanner(), grid, start, 0.0, path,
                            lambda s: (min(7, s // 4), 4))
    return report_hazard(res, "D* Lite routed around static obstacles")


# MARK: Main Method

def main():
    tests = [
        test_dstar_dwa_end_to_end,
        test_dstar_replan_after_obstacle_appears,
        test_dstar_replan_after_obstacle_removed,
        test_dstar_moving_obstacle_with_static,
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

    logger.info("D* Lite + DWA Navigation Integration Test Suite")
    results = [t() for t in selected]

    logger.info(f"Results: {sum(results)}/{len(results)} passed")
    logger.info("All tests complete.")
    sys.exit(0 if all(results) else 1)


if __name__ == '__main__':
    main()
