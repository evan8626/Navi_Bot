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

def dwa_is_valid(best_vel, best_omega, dwa):
    """
    Check if DWA returned valid velocity commands
    """
    if best_vel >= dwa.min_vel_x and best_vel <= dwa.max_vel_x:
        if abs(best_omega) <= dwa.max_vel_theta:
            return True
    return False

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
    
def test_valid_path():
    """
    Test DWA with a valid start and goal position on a clear map
    """
    logger.info("TEST 3: Valid Path on Clear Map")
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
    current_vel = (1.0, 0.0)
    
    #goal
    goal = test_goal_callback(map1)
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
    
    if dwa_is_valid(best_vel, best_omega, dwa):
        logger.info("PASS: DWA returned valid velocity commands")
        return True
    else:
        logger.error("FAIL: DWA returned an invalid path")
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
    logger.info("D Star Lite Planning Test Suite\n")
    results = []
    
    results.append(test_none_start())
    results.append(test_none_goal())
    results.append(test_valid_path() )
    
    logger.info(f"Results: {sum(results)}/{len(results)} passed")
    logger.info("All tests complete.")
    
if __name__ == '__main__':
    main()