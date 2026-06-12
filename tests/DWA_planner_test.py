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

def navigate_closed_loop(dwa, start, theta, goal, obstacles, max_replans=60, tol=0.7):
    """
    Drive the robot with repeated DWA plans (closed loop) until it reaches
    the goal, re-validating every velocity command against the limits.
    Each chosen command is integrated for one 0.5 s control period before
    replanning. Returns (reached, commands_valid, replans, final_pose).
    """
    x, y, th = float(start[0]), float(start[1]), float(theta)
    v, om = 0.0, 0.0
    commands_valid = True
    logger.info(f"Start: ({start[0]}, {start[1]}), Goal: ({goal[0]}, {goal[1]})")
    for i in range(max_replans):
        logger.info(f"pose=({x:.2f},{y:.2f})")
        if np.hypot(goal[0] - x, goal[1] - y) <= tol:
            return True, commands_valid, i, (x, y)
        pair = dwa.plan((x, y, th), (v, om), goal, obstacles)
        if pair is None:
            logger.warning("Planner returned None mid-route.")
            return False, commands_valid, i, (x, y)
        v, om = pair
        if not dwa_is_valid(v, om, dwa):
            logger.warning(f"Invalid command v={v:.3f}, omega={om:.3f} at replan {i}")
            commands_valid = False
        for _ in range(5):  # integrate one control period (5 x 0.1 s)
            x += v * np.cos(th) * 0.1
            y += v * np.sin(th) * 0.1
            th += om * 0.1
    logger.warning(f"Did not reach goal after {max_replans} replans.")
    return False, commands_valid, max_replans, (x, y)

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

    reached, commands_valid, replans, final_pose = navigate_closed_loop(
        dwa, start, theta, goal, obstacles
    )

    if reached and commands_valid:
        logger.info(f"PASS: reached goal in {replans} replans, all commands within limits")
        return True
    if not commands_valid:
        logger.error("FAIL: DWA produced out-of-limit velocity commands")
    else:
        logger.error(f"FAIL: robot did not reach goal, final pose ({final_pose[0]:.2f}, {final_pose[1]:.2f})")
    return False

# MARK: Closed-Loop Static Obstacles
def test_valid_path_obstacles():
    """
    Closed-loop navigation on the obstacle map — every velocity command
    must stay within limits and the robot must reach the goal while
    avoiding obstacles.

    Fixed start/goal so the test is deterministic: (0,0) -> (4,7) has a
    clear route down column 0 and along the fully-free row 4, with at
    least one cell of clearance the whole way. Acts as the acceptance
    test for the dwa.py clearance-scoring fix — the current scoring lets
    clearance dominate progress, so the robot stalls near its start.
    """
    logger.info("TEST 5: Closed-Loop Navigation on Obstacle Map")
    dwa = setup_DWA()
    map1 = obstacle_map()

    start = (0, 0)
    theta = 0.0  # facing +row, roughly toward the route down column 0
    goal = (4, 7)

    obstacles = np.argwhere(map1 == 1)

    reached, commands_valid, replans, final_pose = navigate_closed_loop(
        dwa, start, theta, goal, obstacles
    )

    if reached and commands_valid:
        logger.info(f"PASS: reached goal in {replans} replans, all commands within limits")
        return True
    if not commands_valid:
        logger.error("FAIL: DWA produced out-of-limit velocity commands")
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

    reached, commands_valid, replans, final_pose = navigate_closed_loop(
        dwa, start, theta, goal, obstacles
    )

    if reached and commands_valid:
        logger.info(f"PASS: turned around and reached goal in {replans} replans, all commands within limits")
        return True
    if not commands_valid:
        logger.error("FAIL: DWA produced out-of-limit velocity commands")
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