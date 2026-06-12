#!/usr/bin/env python3

import numpy as np
import logging
import sys

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