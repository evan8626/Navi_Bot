#!/usr/bin/env python3
import numpy as np
import logging
from navi_bot.path_planner import DStarLitePlanner

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)

# SET UP METHODS
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
    coordinates = np.random.randint(0, grid.shape[0]-1, size=2)
    x = coordinates[0]
    y = coordinates[1]
    return (x, y)

def get_start_callback(grid):
    """
    Get start position from grid
    """
    coordinates = np.random.randint(0, grid.shape[0]-1, size=2)
    x = coordinates[0]
    y = coordinates[1]
    return (x, y)

# TEST METHODS
def test_none_start():
    """
    Test D* Lite with None start position
    Known goal.
    """
    logger.info("TEST 1: None Start Position")
    d_star = setup_DStar_Lite()
    start = None
    goal = (7, 7)
    d_star.set_occupancy_grid(clear_map())
    
    try:
        path = d_star.plan(start, goal)
        logger.info("Test None Start: Passed\n")
        return True
    except Exception as e:
        logger.error(f"Test None Start: Failed with error {e}\n")
        return False
    
def test_none_goal():
    """
    Test D* Lite with None goal position
    Known start.
    """
    logger.info("TEST 2: None Goal Position")
    d_star = setup_DStar_Lite()
    start = (0, 0)
    goal = None
    d_star.set_occupancy_grid(clear_map())
    
    try:
        path = d_star.plan(start, goal)
        logger.info("Test None Goal: Passed\n")
        return True
    except Exception as e:
        logger.error(f"Test None Goal: Failed with error {e}\n")
        return False

def test_invalid_start():
    """
    Test D* Lite with invalid start position
    Invalid start, known goal.
    """
    logger.info("TEST 3: Invalid Start Position")
    d_star = setup_DStar_Lite()
    start = (-1, -1)  # Invalid coordinates
    goal = (7, 7)
    d_star.set_occupancy_grid(clear_map())
    
    try:
        path = d_star.plan(start, goal)
        logger.info("Test Invalid Start: Passed\n")
        return True
    except Exception as e:
        logger.error(f"Test Invalid Start: Failed with error {e}\n")
        return False
    
def test_invalid_goal():
    """
    Test D* Lite with invalid goal position
    Known start, invalid goal.
    """
    logger.info("TEST 4: Invalid Goal Position")
    d_start = setup_DStar_Lite()
    start = (0, 0)
    goal = (-1, -1)  # Invalid coordinates
    d_start.set_occupancy_grid(clear_map())

    try:
        path = d_start.plan(start, goal)
        logger.info("Test Invalid Goal: Passed\n")
        return True
    except Exception as e:
        logger.error(f"Test Invalid Goal: Failed with error {e}\n")
        return False
    
def test_already_at_goal():
    """
    Test D* Lite when start and goal are the same
    """
    logger.info("TEST 5: Already at Goal")
    d_start = setup_DStar_Lite()
    start = (3, 3)
    goal = (3, 3) 
    d_start.set_occupancy_grid(clear_map())
    
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
    
def test_blocked_map():
    """
    Blocked map, no path available.
    Random start and goal.
    """
    logger.info("TEST 6: Blocked Map")
    d_star = setup_DStar_Lite()
    start = test_goal_callback(blocked_map())
    goal = test_goal_callback(blocked_map())
    logger.info(f"Start: {start}, Goal: {goal}")
    d_star.set_occupancy_grid(blocked_map())
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
    
def main():
    logger.info("D Star Lite Planning Test Suite\n")
    results = []
    
    results.append(test_none_start())
    results.append(test_none_goal())
    results.append(test_invalid_start())
    results.append(test_invalid_goal())
    results.append(test_already_at_goal())
    results.append(test_blocked_map())
    
    logger.info(f"Results: {sum(results)}/{len(results)} passed")
    logger.info("All tests complete.")
    
if __name__ == '__main__':
    main()