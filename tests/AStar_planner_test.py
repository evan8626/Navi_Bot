#!/usr/bin/env python3
"""
Test script for A* Path Planner from path_planner.py

Tests that the A* planner correctly:
- Finds and computes a weight based path based on an occupancy grid
- Able to plan a straight path
- Able to plan a diagonal path
- Able to plan a more complex path
"""
import heapq
import logging
import math
import sys

import numpy as np

from navi_bot.planners.astar import AStarPlanner

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)

# MARK: Setup Methods

def setup_AStar():
    """
    Setting up AStar search planning algorithm
    """
    a_star = AStarPlanner()
    return a_star

def test_goal_callback(x, y):
    """
    Create a test goal with x and y coords
    """
    return (x, y)

def test_none_start():
    """Test 1: None start coordinates"""
    passed = False
    logger.info("TEST 1: None start coordinates, expect immediate return")
    a_star = setup_AStar()
    
    start = None
    goal = (0, 7)
    
    a_star.set_occupancy_grid(clear_map())
    tester = a_star.plan(start, goal)
    if tester is None:
        logger.info(f"Start is {start} and tester is {tester}")
        logger.info("PASS")
        passed = True
        return passed
    else:
        logger.warning(f"Start is {start}, but tester returned {tester}")
        logger.warning("FAIL")
        return passed

# MARK: No Start
def test_none_goal():
    """Test 2: None goal coordinates"""
    passed = False
    logger.info("TEST 2: None goal coordinates, expect immediate return")
    a_star = setup_AStar()
    
    start = (0, 0)
    goal = None
    
    a_star.set_occupancy_grid(clear_map())
    tester = a_star.plan(start, goal)
    if tester is None:
        logger.info(f"Goal is {goal} and tester is {tester}")
        logger.info("PASS")
        passed = True
        return passed
    else:
        logger.warning(f"Goal is {goal}, but tester returned {tester}")
        logger.warning("FAIL")
        return passed

# MARK: Invalid Start Coords   
def test_invalid_start():
    """Test 3: Invalid start coordinates"""
    passed = False
    logger.info("TEST 3: Invalid start coordinates, expect immediate return")
    a_star = setup_AStar()
    
    start = (-1, -1) # outside the 8x8 map
    goal = (0, 7)
    
    a_star.set_occupancy_grid(clear_map())
    tester = a_star.plan(start, goal)
    if tester is None:
        logger.info(f"Starting coords are: {start[0]}, {start[1]}")
        logger.info("The plan is_coord_valid function returned False")
        logger.info("PASS")
        passed = True
        return passed
    else:
        logger.warning(f"Starting coords are: {start[0]}, {start[1]}, but tester returned {tester}")
        logger.warning("FAIL")
        return passed

# MARK: Invalid Goal Coords  
def test_invalid_goal():
    """Test 4: Invalid goal coordinates"""
    passed = False
    logger.info("TEST 4: Invalid goal coordinates, expect immediate return")
    a_star = setup_AStar()
    
    start = (0, 0)
    goal = (10, 10) # outside the 8 x 8 map
    
    a_star.set_occupancy_grid(clear_map())
    tester = a_star.plan(start, goal)
    if tester is None:
        logger.info(f"Goal coords are: {goal[0]}, {goal[1]}")
        logger.info("The plan is_coord_valid function returned False")
        logger.info("PASS")
        passed = True
        return passed
    else:
        logger.warning(f"Goal coords are: {goal[0]}, {goal[1]}, but tester returned {tester}")
        logger.warning("FAIL")
        return passed

# MARK: At Goal
def test_already_at_goal():
    """Test 5: Start and goal coordinates are the same"""
    passed = False
    logger.info("TEST 5: Start and Goal are same coordinates.")
    a_star = setup_AStar()

    start = (0, 7)
    goal = (0, 7)

    a_star.set_occupancy_grid(clear_map())
    tester = a_star.plan(start, goal)
    # Contract: at-goal returns a trivial path (no movement) — [start] or [] —
    # NOT None (None is reserved for genuine failure). Accept either trivial
    # form; reject None and any real multi-cell path.
    if tester is not None and len(tester) <= 1:
        logger.info(f"The planner returned a trivial at-goal path: {tester}")
        logger.info("PASS")
        passed = True
        return passed
    elif tester is None:
        logger.warning("Planner returned None for at-goal; expected a trivial path [start] or [].")
        logger.warning("FAIL")
        return passed
    else:
        logger.warning(f"Path still created even though start and goal are the same: {tester}")
        logger.warning("FAIL")
        return passed

# MARK: Blocked Map  
def test_blocked_map():
    """Blocked map, no path available."""
    logger.info("TEST 6: Entire map is blocked.")
    passed = False
    a_star = setup_AStar()
    
    start = (0, 0)
    goal = (0, 7)
    
    a_star.set_occupancy_grid(blocked_map())
    tester = a_star.plan(start, goal)
    if tester is None:
        logger.info("No path created for a fully occupied map.")
        logger.info("PASS")
        passed = True
        return passed
    if tester:
        for coord in tester:
            logger.info(f"X: {coord[0]}, Y: {coord[1]}")
        logger.warning("Fully occupied map still returned a path")
        logger.warning("FAIL")
        return passed

# MARK: Straight Line Clear    
def test_straight_line_clear():
    """Straight across the top of the 8x8 empty map"""
    passed = False
    logger.info("TEST 7: Testing path planner with a straight line path")
    a_star = setup_AStar()
    
    start = (0, 0)
    goal = (0, 7)
    
    a_star.set_occupancy_grid(clear_map())
    tester = a_star.plan(start, goal)
    if tester is None:
        logger.warning("No path found.")
        logger.warning("FAIL")
        return passed
    elif tester and (goal == tester[-1]):
        logger.info("Path found to goal.")
        for coord in tester:
            logger.info(f"X: {coord[0]}, Y: {coord[1]}")
        logger.info("PASS")
        passed = True
        return passed
    return passed

# MARK: Diagonal Clear    
def test_diagonal_clear():
    """Straight diagonal through the middle of the 8x8 empty map"""
    passed = False
    logger.info("TEST 8: Test path planning on straight diagonal on an empty map")
    a_star = setup_AStar()
    
    start = (0, 0)
    goal = (7, 7)
    
    a_star.set_occupancy_grid(clear_map())
    tester = a_star.plan(start, goal)
    if tester is None:
        logger.warning("No path found.")
        logger.warning("FAIL")
        return passed
    elif tester and (goal == tester[-1]):
        logger.info("Path found to goal.")
        for coord in tester:
            logger.info(f"X: {coord[0]}, Y: {coord[1]}")
        logger.info("PASS")
        passed = True
        return passed
    return passed

# MARK: Staggered Clear    
def test_staggered_clear():
    """Staggered x, y across 8x8 map"""
    passed = False
    logger.info("TEST 9: Test path planning on clear map where x, y coords are staggered")
    a_star = setup_AStar()
    
    start = (0, 0)
    goal = (5, 7)
    
    a_star.set_occupancy_grid(clear_map())
    tester = a_star.plan(start, goal)
    if tester is None:
        logger.warning("No path found.")
        logger.warning("FAIL")
        return passed
    elif tester and (goal == tester[-1]):
        logger.info("Path found to goal.")
        for coord in tester:
            logger.info(f"X: {coord[0]}, Y: {coord[1]}")
        logger.info("PASS")
        passed = True
        return passed
    return passed

# MARK: Straight Line Obst   
def test_straight_line_obstacle():
    """Straight line to goal, but obstacle in the way"""
    passed = False
    logger.info("TEST 10: Test path planning on a map with obstacles straight goal")
    a_star = setup_AStar()
    
    start = (0, 0)
    goal = (0, 7)
    
    a_star.set_occupancy_grid(obstacle_map())
    tester = a_star.plan(start, goal)
    if tester is None:
        logger.warning("No path found.")
        logger.warning("FAIL")
        return passed
    elif tester and (goal == tester[-1]):
        logger.info("Path found to goal.")
        for coord in tester:
            logger.info(f"X: {coord[0]}, Y: {coord[1]}")
        logger.info("PASS")
        passed = True
        return passed
    return passed

# MARK: Diagonal Obst   
def test_diagonal_obstacle():
    """Diagonal straight across map, with obstacles in the way"""
    passed = False
    logger.info("TEST 11: Test path planning on a map with obstacles diagonal goal")
    a_star = setup_AStar()
    
    start = (0, 0)
    goal = (7, 7)
    
    a_star.set_occupancy_grid(obstacle_map())
    tester = a_star.plan(start, goal)
    if tester is None:
        logger.warning("No path found.")
        logger.warning("FAIL")
        return passed
    elif tester and (goal == tester[-1]):
        logger.info("Path found to goal.")
        for coord in tester:
            logger.info(f"X: {coord[0]}, Y: {coord[1]}")
        logger.info("PASS")
        passed = True
        return passed
    return passed

# MARK: Staggered Obst  
def test_staggered_obstacle():
    """Testing a more complex goal arrangement on obstacle map"""
    passed = False
    logger.info("TEST 12: Test path planning on a map with obstacles staggered goal")
    a_star = setup_AStar()
    
    start = (0, 0)
    goal = (5, 7)
    
    a_star.set_occupancy_grid(obstacle_map())
    tester = a_star.plan(start, goal)
    if tester is None:
        logger.warning("No path found.")
        logger.warning("FAIL")
        return passed
    elif tester and (goal == tester[-1]):
        logger.info("Path found to goal.")
        for coord in tester:
            logger.info(f"X: {coord[0]}, Y: {coord[1]}")
        logger.info("PASS")
        passed = True
        return passed
    return passed

# MARK: Maps

def clear_map():
    """
    Obstacle-less map
    """
    return np.array([
        [0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0]
    ])
    
def blocked_map():
    """
    Fully occupied map
    """
    return np.array([
        [1, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 1, 1, 1]
    ])
    
def obstacle_map():
    """
    Map with obstacles
    """
    return np.array([
        [0, 0, 0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 1, 1, 1],
        [0, 0, 1, 0, 0, 1, 0, 0],
        [0, 1, 1, 1, 0, 1, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0],
        [1, 0, 1, 0, 1, 0, 1, 0],
        [1, 0, 1, 0, 1, 0, 1, 0],
        [1, 0, 1, 0, 1, 0, 1, 0]
    ])

# MARK: Reference Model
#
# Intended movement model for all grid planners in this project:
#   - 8-connected neighbors
#   - cardinal step cost 1.0, diagonal step cost sqrt(2)
#   - a diagonal move is only legal when BOTH adjacent cardinal cells are
#     free (no corner cutting past obstacle corners)

SQRT2 = math.sqrt(2)


def reference_shortest_cost(grid, start, goal):
    """
    Plain Dijkstra under the intended movement model. Small and obviously
    correct — used as the ground truth for optimality tests.
    Returns the optimal path cost, or None if the goal is unreachable.
    """
    rows, cols = len(grid), len(grid[0])

    def free(r, c):
        return 0 <= r < rows and 0 <= c < cols and grid[r][c] == 0

    if not (free(*start) and free(*goal)):
        return None
    dist = {tuple(start): 0.0}
    pq = [(0.0, tuple(start))]
    while pq:
        d, (r, c) = heapq.heappop(pq)
        if (r, c) == tuple(goal):
            return d
        if d > dist.get((r, c), float('inf')):
            continue
        for dr, dc in ((0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)):
            nr, nc = r + dr, c + dc
            if not free(nr, nc):
                continue
            if dr != 0 and dc != 0 and not (free(r, nc) and free(nr, c)):
                continue  # corner cut — not a legal move
            nd = d + (SQRT2 if dr != 0 and dc != 0 else 1.0)
            if nd < dist.get((nr, nc), float('inf')) - 1e-12:
                dist[(nr, nc)] = nd
                heapq.heappush(pq, (nd, (nr, nc)))
    return None


def path_cost(path):
    """Cost of a path under the intended movement model (steps must be adjacent)."""
    total = 0.0
    for a, b in zip(path, path[1:]):
        dr, dc = b[0] - a[0], b[1] - a[1]
        if (dr, dc) == (0, 0) or abs(dr) > 1 or abs(dc) > 1:
            return None
        total += SQRT2 if dr != 0 and dc != 0 else 1.0
    return total


def validate_path(path, grid, start, goal, forbid_corner_cuts=False):
    """Check a path's structure. Returns (ok, reason)."""
    if not path:
        return False, "path is empty"
    if tuple(path[0]) != tuple(start):
        return False, f"path starts at {path[0]}, not {start}"
    if tuple(path[-1]) != tuple(goal):
        return False, f"path ends at {path[-1]}, not {goal}"
    if grid[path[0][0]][path[0][1]] != 0:
        return False, f"start cell {path[0]} is occupied"
    for a, b in zip(path, path[1:]):
        dr, dc = b[0] - a[0], b[1] - a[1]
        if max(abs(dr), abs(dc)) != 1:
            return False, f"illegal step {a} -> {b}"
        if grid[b[0]][b[1]] != 0:
            return False, f"step onto occupied cell {b}"
        if forbid_corner_cuts and dr != 0 and dc != 0:
            if grid[a[0]][b[1]] != 0 or grid[b[0]][a[1]] != 0:
                return False, f"corner cut {a} -> {b}"
    return True, ""


# MARK: Path Quality

def test_path_validity():
    """Returned paths must start at start, end at goal, and step only between adjacent free cells."""
    passed = True
    logger.info("TEST 13: Path validity — endpoints, adjacency, free cells")
    cases = [
        (clear_map(), (0, 0), (7, 7)),
        (obstacle_map(), (0, 0), (7, 7)),
        (obstacle_map(), (0, 0), (5, 7)),
    ]
    for grid, start, goal in cases:
        a_star = setup_AStar()
        a_star.set_occupancy_grid(grid)
        path = a_star.plan(start, goal)
        if path is None:
            logger.warning(f"  FAIL {start}->{goal}: no path returned")
            passed = False
            continue
        ok, reason = validate_path(path, grid, start, goal)
        if not ok:
            logger.warning(f"  FAIL {start}->{goal}: {reason}")
            passed = False
        else:
            logger.info(f"  OK   {start}->{goal}: {len(path)} waypoints, all steps legal")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_path_optimality():
    """Path cost must equal the reference Dijkstra optimum (no corner cuts).

    Differential test: the reference implements the intended movement
    model — cardinal 1.0, diagonal sqrt(2), diagonals only when both
    adjacent cardinal cells are free. The A* path must be legal under
    that model and cost exactly the optimum.
    """
    passed = True
    logger.info("TEST 14: Path cost matches reference optimum (no corner cutting)")
    cases = [
        (clear_map(), (0, 0), (0, 7)),
        (clear_map(), (0, 0), (7, 7)),
        (clear_map(), (0, 0), (5, 7)),
        (obstacle_map(), (0, 0), (0, 7)),
        (obstacle_map(), (0, 0), (7, 7)),
        (obstacle_map(), (0, 0), (5, 7)),
    ]
    for grid, start, goal in cases:
        a_star = setup_AStar()
        a_star.set_occupancy_grid(grid)
        path = a_star.plan(start, goal)
        ref = reference_shortest_cost(grid, start, goal)
        if path is None:
            logger.warning(f"  FAIL {start}->{goal}: no path (reference says cost {ref})")
            passed = False
            continue
        ok, reason = validate_path(path, grid, start, goal, forbid_corner_cuts=True)
        cost = path_cost(path)
        if not ok:
            logger.warning(f"  FAIL {start}->{goal}: {reason}")
            passed = False
        elif ref is None or cost is None or abs(cost - ref) > 1e-6:
            logger.warning(f"  FAIL {start}->{goal}: path cost {cost}, reference optimum {ref}")
            passed = False
        else:
            logger.info(f"  OK   {start}->{goal}: cost {cost:.4f} == optimum")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_no_corner_cutting_through_pinch():
    """A diagonal must not squeeze between two touching obstacle corners.

    Start (0,0) with (0,1) and (1,0) blocked is sealed in: the only way
    out is the illegal diagonal through the pinch to (1,1). A correct
    planner must report no path.
    """
    passed = True
    logger.info("TEST 15: Corner-cutting through a sealed pinch must find no path")
    grid = clear_map()
    grid[0][1] = 1
    grid[1][0] = 1
    a_star = setup_AStar()
    a_star.set_occupancy_grid(grid)
    path = a_star.plan((0, 0), (7, 7))
    if path is not None:
        logger.warning(f"  FAIL planner escaped a sealed start by cutting the corner: {path[:3]}...")
        passed = False
    else:
        logger.info("  OK   sealed start correctly reported as unreachable")
    logger.info("PASS" if passed else "FAIL")
    return passed


#MARK: Main Method

def main():
    tests = [
        test_none_start,
        test_none_goal,
        test_invalid_start,
        test_invalid_goal,
        test_already_at_goal,
        test_blocked_map,
        test_straight_line_clear,
        test_diagonal_clear,
        test_staggered_clear,
        test_straight_line_obstacle,
        test_diagonal_obstacle,
        test_staggered_obstacle,
        test_path_validity,
        test_path_optimality,
        test_no_corner_cutting_through_pinch,
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

    logger.info("A Star Planning Test Suite")
    results = [t() for t in selected]

    logger.info(f"Results: {sum(results)}/{len(results)} passed")
    logger.info("All tests complete.")
    sys.exit(0 if all(results) else 1)
    
if __name__ == '__main__':
    main()