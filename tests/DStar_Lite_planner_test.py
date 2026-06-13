#!/usr/bin/env python3

import heapq
import logging
import math
import sys

import numpy as np

from navi_bot.planners.dstar_lite import DStarLitePlanner

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)

# MARK: Setup Methods

def setup_DStar_Lite():
    """
    Setting up D Start Lite replanning algorithm
    """
    d_star = DStarLitePlanner()
    return d_star

def test_goal_callback(grid):
    """
    Create a test goal with x and y coords
    """
    free_cells = np.argwhere(grid == 0)
    idx = np.random.randint(0, len(free_cells))
    return tuple(int(v) for v in free_cells[idx])

def get_start_callback(grid):
    """
    Get start position from grid
    """
    free_cells = np.argwhere(grid == 0)
    idx = np.random.randint(0, len(free_cells))
    return tuple(int(v) for v in free_cells[idx])

def path_is_valid(path, grid):
    """
    Check if the path is valid (not None, not empty, and all cells are free)
    """
    if path is None or len(path) == 0:
        return False
    for cell in path:
        row, col = cell
        if row < 0 or row >= grid.shape[0] or col < 0 or col >= grid.shape[1] or grid[row, col] == 1:
            return False
    return True

# MARK: No Start
def test_none_start():
    """
    Test D* Lite with None start position
    Known goal.
    """
    logger.info("TEST 1: None Start Position")
    d_star = setup_DStar_Lite()
    map1 = clear_map()
    start = None
    goal = (7, 7)
    d_star.set_occupancy_grid(map1)
    
    try:
        path = d_star.plan(start, goal)
        logger.info("Test None Start: Passed\n")
        return True
    except Exception as e:
        logger.error(f"Test None Start: Failed with error {e}\n")
        return False

# MARK: No Goal
def test_none_goal():
    """
    Test D* Lite with None goal position
    Known start.
    """
    logger.info("TEST 2: None Goal Position")
    d_star = setup_DStar_Lite()
    map1 = clear_map()
    start = (0, 0)
    goal = None
    d_star.set_occupancy_grid(map1)
    
    try:
        path = d_star.plan(start, goal)
        logger.info("Test None Goal: Passed\n")
        return True
    except Exception as e:
        logger.error(f"Test None Goal: Failed with error {e}\n")
        return False

# MARK: Invalid Start Coords
def test_invalid_start():
    """
    Test D* Lite with invalid start position
    Invalid start, known goal.
    """
    logger.info("TEST 3: Invalid Start Position")
    d_star = setup_DStar_Lite()
    map1 = clear_map()
    start = (-1, -1)  # Invalid coordinates
    goal = (7, 7)
    d_star.set_occupancy_grid(map1)
    
    try:
        path = d_star.plan(start, goal)
        logger.info("Test Invalid Start: Passed\n")
        return True
    except Exception as e:
        logger.error(f"Test Invalid Start: Failed with error {e}\n")
        return False

# MARK: Invalid Goal Coords  
def test_invalid_goal():
    """
    Test D* Lite with invalid goal position
    Known start, invalid goal.
    """
    logger.info("TEST 4: Invalid Goal Position")
    d_start = setup_DStar_Lite()
    map1 = clear_map()
    start = (0, 0)
    goal = (-1, -1)  # Invalid coordinates
    d_start.set_occupancy_grid(map1)

    try:
        path = d_start.plan(start, goal)
        logger.info("Test Invalid Goal: Passed\n")
        return True
    except Exception as e:
        logger.error(f"Test Invalid Goal: Failed with error {e}\n")
        return False

# MARK: At Goal  
def test_already_at_goal():
    """
    Test D* Lite when start and goal are the same
    """
    logger.info("TEST 5: Already at Goal")
    d_start = setup_DStar_Lite()
    map1 = clear_map()
    start = (3, 3)
    goal = (3, 3) 
    d_start.set_occupancy_grid(map1)
    
    try:
        path = d_start.plan(start, goal)
        if path is None: 
            logger.info("Test Already at Goal: Passed\n")
            return True
        else:
            logger.error(f"Test Already at Goal, but got a path of {path} instead\n")
            return False
    except Exception as e:
        logger.error(f"Test Already at Goal: Failed with error {e}\n")
        return False

# MARK: Blocked Map
def test_blocked_map():
    """
    Blocked map, no path available.
    Random start and goal.
    """
    logger.info("TEST 6: Blocked Map")
    d_star = setup_DStar_Lite()
    start = (0, 0)
    goal = (7, 7)
    logger.info(f"Start: {start}, Goal: {goal}")
    map_block = blocked_map()
    d_star.set_occupancy_grid(map_block)
    try:
        path = d_star.plan(start, goal)
        if path is None: 
            logger.info("Test Blocked Map: Passed\n")
            return True
        else:
            logger.error(f"Test Blocked Map, but got a path of {path} instead\n")
            return False
    except Exception as e:
        logger.error(f"Test Blocked Map: Failed with error {e}\n")
        return False

# MARK: Coords on Open   
def test_random_coords_clear_map():
    """
    Clear map with randomly generated start and goal coordinates.
    """
    logger.info("TEST 7: Random Coords on Clear Map")
    d_star = setup_DStar_Lite()
    
    map1 = clear_map()
    
    start = get_start_callback(map1)  #(0, 0)
    goal = test_goal_callback(map1) #(0, 7)
    d_star.set_occupancy_grid(map1)
    try:
        path = d_star.plan(start, goal)
        if path_is_valid(path, map1):
            logger.info(f"Start: {start}, Goal: {goal}")
            for coord in path:
                logger.info(f"X: {coord[0]}, Y: {coord[1]}")
            logger.info("Test Random Coords on Clear Map: Passed\n")
            return True
        else:
            logger.error(f"Test Random Coords on Clear Map, but invalid path returned: {path}\n")
            return False
    except Exception as e:
        logger.error(f"Test Random Coords on Clear Map: Failed with error {e}\n")
        return False

# MARK: Coords on Obstacle    
def test_random_coords_obstacle_map():
    """
    Obstacle map with randomly generated start and goal coordinates.
    """
    logger.info("TEST 8: Random Coords on map with obstacles")
    d_star = setup_DStar_Lite()
    
    map1 = obstacle_map()
    
    start = get_start_callback(map1)  
    goal = test_goal_callback(map1) 
    d_star.set_occupancy_grid(map1)
    
    try:
        path = d_star.plan(start, goal)
        if path_is_valid(path, map1):
            logger.info(f"Start: {start}, Goal: {goal} (obstacle map)")
            for coord in path:
                logger.info(f"X: {coord[0]}, Y: {coord[1]}")
            logger.info("Test Random Coords on Obstacle Map: Passed\n")
            return True
        else:
            logger.error(f"Test Random Coords on Obstacle Map, but invalid path returned: {path}\n")
            return False
    except Exception as e:
        logger.error(f"Test Random Coords on Obstacle Map: Failed with error {e}\n")
        return False

# MARK: Coords on Dynamic  
def test_random_coords_on_moving_map():
    """
    Randomly generated start and goal coords, with changing map
    """
    logger.info("TEST 9: Random Coords on moving map")
    d_star = setup_DStar_Lite()
    
    map1 = obstacle_map()
    map2 = obstacle_map2()
    
    start = get_start_callback(map1)  
    goal = test_goal_callback(map1) 
    d_star.set_occupancy_grid(map1)
    
    try:
        path = d_star.plan(start, goal)
        d_star.set_occupancy_grid(map2)
        if path_is_valid(path, map1):
            logger.info(f"Start: {start}, Goal: {goal} (obstacle map)")
            for coord in path:
                logger.info(f"X: {coord[0]}, Y: {coord[1]}")
            logger.info("Test Random Coords on Moving Map: Passed\n")
            return True
        else:
            logger.error(f"Test Random Coords on Moving Map, but invalid path returned: {path}\n")
            return False
    except Exception as e:
        logger.error(f"Test Random Coords on Moving Map: Failed with error {e}\n")
        return False

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
    
def obstacle_map2():
    """
    Map with obstacles
    """
    return np.array([
        [0, 0, 0, 1, 0, 0, 0, 0],
        [0, 0, 1, 1, 0, 1, 1, 1],
        [0, 0, 0, 0, 0, 1, 0, 0],
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

def test_path_optimality():
    """Path cost must equal the reference Dijkstra optimum (no corner cuts).

    Differential test: diagonals must cost sqrt(2) — not 1.0 — and are
    only legal when both adjacent cardinal cells are free. The heuristic
    must be admissible for 8-connected movement (octile/Chebyshev, not
    Manhattan). Any of those issues shows up here as a cost mismatch or
    an illegal step.
    """
    passed = True
    logger.info("TEST 10: Path cost matches reference optimum (no corner cutting)")
    cases = [
        (clear_map(), (0, 0), (0, 7)),
        (clear_map(), (0, 0), (7, 7)),
        (clear_map(), (0, 0), (5, 7)),
        (obstacle_map(), (0, 0), (0, 7)),
        (obstacle_map(), (0, 0), (7, 7)),
        (obstacle_map(), (0, 0), (5, 7)),
    ]
    for grid, start, goal in cases:
        d_star = setup_DStar_Lite()
        d_star.set_occupancy_grid(grid)
        try:
            path = d_star.plan(start, goal)
        except Exception as e:
            logger.warning(f"  FAIL {start}->{goal}: plan raised {type(e).__name__}: {e}")
            passed = False
            continue
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


# MARK: Replanning

def test_replan_after_obstacle_appears():
    """After a wall appears across the route, a replan must detour around it."""
    passed = True
    logger.info("TEST 11: Replan after an obstacle appears on the path")
    d_star = setup_DStar_Lite()
    grid_a = clear_map()
    start, goal = (0, 0), (0, 7)
    d_star.set_occupancy_grid(grid_a)
    path1 = d_star.plan(start, goal)
    if path1 is None:
        logger.warning("  FAIL initial plan on a clear map returned no path")
        logger.info("FAIL")
        return False
    logger.info(f"  initial path: {len(path1)} waypoints")

    # The robot has advanced two steps; a wall now blocks the direct route
    new_start = tuple(path1[min(2, len(path1) - 2)])
    grid_b = clear_map()
    grid_b[0][4] = 1
    grid_b[1][4] = 1
    d_star.set_occupancy_grid(grid_b)
    path2 = d_star.plan(new_start, goal)
    if path2 is None:
        logger.warning("  FAIL replan returned no path even though a detour exists")
        passed = False
    else:
        ok, reason = validate_path(path2, grid_b, new_start, goal)
        if not ok:
            logger.warning(f"  FAIL replanned path invalid: {reason}")
            passed = False
        elif any(grid_b[r][c] != 0 for r, c in path2):
            logger.warning("  FAIL replanned path passes through the new wall")
            passed = False
        else:
            logger.info(f"  OK   replanned around the wall with {len(path2)} waypoints")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_replan_after_obstacle_removed():
    """When a wall disappears, a replan must exploit the opened shortcut."""
    passed = True
    logger.info("TEST 12: Replan after an obstacle is removed")
    d_star = setup_DStar_Lite()
    start, goal = (0, 0), (0, 7)

    # Wall down column 3, gap only at the bottom row -> long detour
    grid_walled = clear_map()
    for r in range(0, 7):
        grid_walled[r][3] = 1
    d_star.set_occupancy_grid(grid_walled)
    path1 = d_star.plan(start, goal)
    if path1 is None:
        logger.warning("  FAIL no path on the walled map (gap at row 7 exists)")
        logger.info("FAIL")
        return False
    cost1 = path_cost(path1)

    # Wall removed -> direct route should now be much cheaper
    d_star.set_occupancy_grid(clear_map())
    path2 = d_star.plan(start, goal)
    if path2 is None:
        logger.warning("  FAIL no path after the wall was removed")
        passed = False
    else:
        cost2 = path_cost(path2)
        if cost2 is None or cost1 is None or not (cost2 < cost1 - 1.0):
            logger.warning(f"  FAIL replan did not exploit the opening: cost {cost1} -> {cost2}")
            passed = False
        else:
            logger.info(f"  OK   cost dropped {cost1:.2f} -> {cost2:.2f} after wall removal")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_incremental_replan_updates_km():
    """A replan after robot movement + map change must reuse search state (k_m grows).

    This is the defining feature of D* Lite (Koenig & Likhachev 2002):
    when the robot has moved and edge costs change, the planner keeps its
    g/rhs values and accumulates k_m += h(s_last, s_start) so stale queue
    keys stay valid. k_m == 0 after such a replan means the search was
    re-initialized from scratch — correct output, but no longer D* Lite.
    """
    passed = True
    logger.info("TEST 13: Incremental replan must accumulate k_m (search state reuse)")
    d_star = setup_DStar_Lite()
    grid_a = clear_map()
    start, goal = (0, 0), (7, 7)
    d_star.set_occupancy_grid(grid_a)
    path1 = d_star.plan(start, goal)
    if path1 is None or len(path1) < 6:
        logger.warning("  FAIL initial plan missing or too short to stage the scenario")
        logger.info("FAIL")
        return False

    # Robot advances two steps, then discovers an obstacle further along
    new_start = tuple(path1[2])
    blocked = tuple(path1[4])
    grid_b = clear_map()
    grid_b[blocked[0]][blocked[1]] = 1
    d_star.set_occupancy_grid(grid_b)
    path2 = d_star.plan(new_start, goal)

    if path2 is None:
        logger.warning("  FAIL replan returned no path")
        passed = False
    else:
        ok, reason = validate_path(path2, grid_b, new_start, goal)
        if not ok:
            logger.warning(f"  FAIL replanned path invalid: {reason}")
            passed = False
    if passed and d_star.k_m <= 0:
        logger.warning(f"  FAIL k_m is {d_star.k_m} after moving and replanning — "
                       "the search was re-initialized instead of updated incrementally")
        passed = False
    if passed:
        logger.info(f"  OK   replanned incrementally with k_m={d_star.k_m:.4f}")
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: Main Method

def main():
    tests = [
        test_none_start,
        test_none_goal,
        test_invalid_start,
        test_invalid_goal,
        test_already_at_goal,
        test_blocked_map,
        test_random_coords_clear_map,
        test_random_coords_obstacle_map,
        test_random_coords_on_moving_map,
        test_path_optimality,
        test_replan_after_obstacle_appears,
        test_replan_after_obstacle_removed,
        test_incremental_replan_updates_km,
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

    logger.info("D Star Lite Planning Test Suite\n")
    results = [t() for t in selected]

    logger.info(f"Results: {sum(results)}/{len(results)} passed")
    logger.info("All tests complete.")
    sys.exit(0 if all(results) else 1)
    
if __name__ == '__main__':
    main()