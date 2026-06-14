#!/usr/bin/env python3

import numpy as np
import logging
import sys

from navi_bot.planners.dwa import DWAPlanner

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)

# MARK: Setup Methods

def setup_DWA():
    """
    Setting up DWA replanning algorithm
    """
    dwa = DWAPlanner()
    return dwa

def get_start_callback(grid):
    """
    Get start position from grid
    """
    if grid is None:
        return None
    
    free_cells = np.argwhere(grid == 0)
    if not free_cells.size:
        return None
    
    idx = np.random.randint(0, len(free_cells))
    return tuple(int(v) for v in free_cells[idx])

def test_goal_callback(grid):
    """
    Create a test goal with x and y coords
    """
    if grid is None:
        return None
    
    free_cells = np.argwhere(grid == 0)
    if not free_cells.size:
        return None
    
    idx = np.random.randint(0, len(free_cells))
    return tuple(int(v) for v in free_cells[idx])

def dwa_is_valid(best_vel, best_omega, dwa):
    """
    Check if DWA returned valid velocity commands
    """
    if best_vel >= dwa.min_vel_x and best_vel <= dwa.max_vel_x:
        if abs(best_omega) <= dwa.max_vel_theta:
            return True
    return False

def navigate_closed_loop(dwa, start, theta, goal, obstacles, grid=None, max_replans=100, tol=0.7):
    """
    Drive the robot with repeated DWA plans (closed loop) until it reaches
    the goal, re-validating every velocity command against the limits.
    Each chosen command is integrated for one 0.5 s control period before
    replanning. When `grid` is given, every integrated pose is checked
    against it — entering an occupied cell counts as a collision, and the
    robot's center moving past the map's half-cell border (cells span
    +/-0.5 around their integer centers) counts as leaving the map.
    Returns (reached, commands_valid, collided, left_map, replans, final_pose).
    """
    x, y, th = float(start[0]), float(start[1]), float(theta)
    v, om = 0.0, 0.0
    commands_valid = True
    collided = False
    left_map = False
    logger.info(f"Start: ({start[0]}, {start[1]}), Goal: ({goal[0]}, {goal[1]})")
    if grid is not None:  # log the real grid so the dashboard draws the actual obstacles
        logger.info(f"MAP {len(grid)} {len(grid[0])}")
        for _row in grid:
            logger.info("MAPROW " + "".join('1' if int(_c) != 0 else '0' for _c in _row))
    for i in range(max_replans):
        logger.info(f"pose=({x:.2f},{y:.2f})")
        if np.hypot(goal[0] - x, goal[1] - y) <= tol:
            return True, commands_valid, collided, left_map, i, (x, y)
        pair = dwa.plan((x, y, th), (v, om), goal, obstacles)
        if pair is None:
            logger.warning("Planner returned None mid-route.")
            return False, commands_valid, collided, left_map, i, (x, y)
        v, om = pair
        if not dwa_is_valid(v, om, dwa):
            logger.warning(f"Invalid command v={v:.3f}, omega={om:.3f} at replan {i}")
            commands_valid = False
        for _ in range(5):  # integrate one control period (5 x 0.1 s)
            x += v * np.cos(th) * 0.1
            y += v * np.sin(th) * 0.1
            th += om * 0.1
            if grid is not None:
                if not collided:
                    r, c = int(round(x)), int(round(y))
                    if 0 <= r < len(grid) and 0 <= c < len(grid[0]) and grid[r][c] != 0:
                        logger.warning(f"Collision: robot entered occupied cell ({r}, {c}) at pose=({x:.2f},{y:.2f})")
                        collided = True
                if not left_map:
                    if not (-0.5 <= x <= len(grid) - 0.5 and -0.5 <= y <= len(grid[0]) - 0.5):
                        logger.warning(f"Out of bounds: robot center left the map at pose=({x:.2f},{y:.2f})")
                        left_map = True
    logger.warning(f"Did not reach goal after {max_replans} replans.")
    return False, commands_valid, collided, left_map, max_replans, (x, y)

# END SETUP METHODS

# MARK: No Start coords
def test_none_start():
    """
    Test DWA With a None start position
    """
    logger.info("TEST 1: None Start Position")
    dwa = setup_DWA()
    map1 = clear_map()
    
    # current pose
    start = None
    theta = 0.0
    current_pose = ()
    if (start is not None) and (theta is not None):
        current_pose = (start[0], start[1], theta)
    else:
        current_pose = None
        
    # linear and angular velocity
    current_vel = (0.0, 0.0)
    
    # goal
    goal = test_goal_callback(map1)
    
    # obstacles
    obstacles = np.argwhere(map1 == 1)
    
    path = dwa.plan(current_pose, current_vel, goal, obstacles)
    
    if path is None:
        logger.info("PASS: DWA handled None start position")
        return True
    else:
        logger.error("FAIL: DWA returned a path even though start is None")
        return False

# MARK: No Goal coords
def test_none_goal():
    """
    Test DWA With a None goal position
    """
    logger.info("TEST 2: None Goal Position")
    dwa = setup_DWA()
    map1 = clear_map()
    
    # current pose
    start = get_start_callback(map1)
    theta = 0.0
    current_pose = ()
    if (start is not None) and (theta is not None):
        current_pose = (start[0], start[1], theta)
    else:
        current_pose = None
        
    # linear and angular velocity
    current_vel = (0.0, 0.0)
    
    #goal
    goal = None
    
    # obstacles
    obstacles = np.argwhere(map1 == 1)
    
    path = dwa.plan(current_pose, current_vel, goal, obstacles)
    
    if path is None:
        logger.info("PASS: DWA handled None goal position")
        return True
    else:
        logger.error("FAIL: DWA returned a path even though goal is None")
        return False

# MARK: Fully Blocked Map
def test_fully_blocked():
    """
    Test DWA With a fully blocked map
    """
    logger.info("TEST 3: Fully Blocked Map")
    dwa = setup_DWA()
    map1 = blocked_map()
    
    # current pose
    start = get_start_callback(map1)
    theta = 0.0
    current_pose = ()
    if (start is not None) and (theta is not None):
        current_pose = (start[0], start[1], theta)
    else:
        current_pose = None
        
    # linear and angular velocity
    current_vel = (1.0, 0.0)
    
    #goal
    goal = test_goal_callback(map1)
    if goal:
        while goal == start:
            goal = test_goal_callback(map1)
        
    # obstacles
    obstacles = np.argwhere(map1 == 1)
    for obs in obstacles:
        if np.array_equal(obs, start) or np.array_equal(obs, goal):
            logger.warning("Obstacle coincides with start or goal. Skipping obstacle.")
            continue
        
    pair = dwa.plan(current_pose, current_vel, goal, obstacles)
    best_vel = 0.0 
    best_omega = 0.0
    if(pair is not None):
        best_vel, best_omega = pair
        
    if pair is None:
        logger.info("PASS: DWA handled fully blocked map")
        return True
    else:
        logger.error("FAIL: DWA returned a path even though map is fully blocked")
        return False

# MARK: Closed-Loop Empty Map
def test_valid_path():
    """
    Closed-loop navigation to a goal on a clear map — every velocity
    command must stay within limits and the robot must reach the goal.
    """
    logger.info("TEST 4: Closed-Loop Navigation on Clear Map")
    dwa = setup_DWA()
    map1 = clear_map()

    start = get_start_callback(map1)
    theta = 0.0

    goal = test_goal_callback(map1)
    if goal:
        while goal == start:
            goal = test_goal_callback(map1)

    obstacles = np.argwhere(map1 == 1)

    # future API: when DWAPlanner grows set_occupancy_grid (like A*/D* Lite),
    # the planner learns the map bounds without any further test changes
    if hasattr(dwa, 'set_occupancy_grid'):
        dwa.set_occupancy_grid(map1)

    reached, commands_valid, collided, left_map, replans, final_pose = navigate_closed_loop(
        dwa, start, theta, goal, obstacles, grid=map1
    )

    if reached and commands_valid and not collided and not left_map:
        logger.info(f"PASS: reached goal in {replans} replans, stayed on the map, no collisions, all commands within limits")
        return True
    if not commands_valid:
        logger.error("FAIL: DWA produced out-of-limit velocity commands")
    elif collided:
        logger.error("FAIL: robot drove through an obstacle cell en route")
    elif left_map:
        logger.error("FAIL: robot left the map boundary en route")
    else:
        logger.error(f"FAIL: robot did not reach goal, final pose ({final_pose[0]:.2f}, {final_pose[1]:.2f})")
    return False

# MARK: Closed-Loop Single-Obstacle Avoidance
def test_valid_path_obstacles():
    """
    Closed-loop reactive avoidance — DWA's actual job. A single obstacle
    sits just off the straight line from start to goal, so the direct path
    would clip it and the robot must steer around it, keep every command
    within limits, and reach the goal.

    This is deliberately NOT the dense obstacle map: threading a field of
    obstacles needs a global planner (see the A*+DWA / D*+DWA integration
    suites). A bare local planner is only responsible for reacting to
    obstacles near its path, which is what this tests.
    """
    logger.info("TEST 5: Closed-Loop Single-Obstacle Avoidance")
    dwa = setup_DWA()
    map1 = clear_map()
    map1[3][4] = 1   # one obstacle, ~0.7 cell off the (1,1)->(6,6) bearing line

    start = (1, 1)
    theta = 0.0
    goal = (6, 6)

    obstacles = np.argwhere(map1 == 1)

    if hasattr(dwa, 'set_occupancy_grid'):
        dwa.set_occupancy_grid(map1)

    reached, commands_valid, collided, left_map, replans, final_pose = navigate_closed_loop(
        dwa, start, theta, goal, obstacles, grid=map1
    )

    if reached and commands_valid and not collided and not left_map:
        logger.info(f"PASS: steered around the obstacle and reached goal in {replans} replans, all commands within limits")
        return True
    if not commands_valid:
        logger.error("FAIL: DWA produced out-of-limit velocity commands")
    elif collided:
        logger.error("FAIL: robot drove through the obstacle")
    elif left_map:
        logger.error("FAIL: robot left the map boundary en route")
    else:
        logger.error(f"FAIL: robot did not reach goal, final pose ({final_pose[0]:.2f}, {final_pose[1]:.2f})")
    return False
    
# MARK: Closed-Loop Facing away from Goal
def test_valid_path_facing_away():
    """
    Closed-loop navigation on a clear map where the robot starts facing
    away from the goal — it must turn around, keep every command within
    limits, and reach the goal.
    """
    logger.info("TEST 6: Closed-Loop Navigation Facing away from Goal")
    dwa = setup_DWA()
    map1 = clear_map()

    start = get_start_callback(map1)
    theta = 3.1  # ~pi: facing away

    goal = test_goal_callback(map1)
    if goal:
        while goal == start:
            goal = test_goal_callback(map1)

    obstacles = np.argwhere(map1 == 1)

    # future API: when DWAPlanner grows set_occupancy_grid (like A*/D* Lite),
    # the planner learns the map bounds without any further test changes
    if hasattr(dwa, 'set_occupancy_grid'):
        dwa.set_occupancy_grid(map1)

    reached, commands_valid, collided, left_map, replans, final_pose = navigate_closed_loop(
        dwa, start, theta, goal, obstacles, grid=map1
    )

    if reached and commands_valid and not collided and not left_map:
        logger.info(f"PASS: turned around and reached goal in {replans} replans, stayed on the map, no collisions, all commands within limits")
        return True
    if not commands_valid:
        logger.error("FAIL: DWA produced out-of-limit velocity commands")
    elif collided:
        logger.error("FAIL: robot drove through an obstacle cell en route")
    elif left_map:
        logger.error("FAIL: robot left the map boundary en route")
    else:
        logger.error(f"FAIL: robot did not reach goal, final pose ({final_pose[0]:.2f}, {final_pose[1]:.2f})")
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

# MARK: Main Method

def main():
    tests = [
        test_none_start,
        test_none_goal,
        test_fully_blocked,
        test_valid_path,
        test_valid_path_obstacles,
        test_valid_path_facing_away,
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

    logger.info("DWA Planning Test Suite\n")
    results = [t() for t in selected]

    logger.info(f"Results: {sum(results)}/{len(results)} passed")
    logger.info("All tests complete.")
    sys.exit(0 if all(results) else 1)
    
if __name__ == '__main__':
    main()