#!/usr/bin/env python3

import numpy as np
import logging

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

# END SETUP METHODS

# MARK: Test Methods

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
    logger.info("D Star Lite Planning Test Suite\n")
    results = []
    
    results.append(test_none_start())
    
    logger.info(f"Results: {sum(results)}/{len(results)} passed")
    logger.info("All tests complete.")
    
if __name__ == '__main__':
    main()