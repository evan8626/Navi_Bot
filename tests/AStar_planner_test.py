#!/usr/bin/env python3
"""
Test script for A* Path Planner from path_planner.py

Tests that the A* planner correctly:
- Finds and computes a weight based path based on an occupancy grid
- Able to plan a straight path
- Able to plan a diagonal path
- Able to plan a more complex path
"""
import logging
import numpy as np
import sys


from navi_bot.path_planner import AStarPlanner

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)

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
    
def test_already_at_goal():
    """Test 5: Start and goal coordinates are the same"""
    passed = False
    logger.info("TEST 5: Start and Goal are same coordinates.")
    a_star = setup_AStar()
    
    start = (0, 7)
    goal = (0, 7) 
    
    a_star.set_occupancy_grid(clear_map())
    tester = a_star.plan(start, goal)
    if tester is None:
        logger.info("The planner immediate returned that robot is at goal")
        logger.info("PASS")
        passed = True
        return passed
    else:
        logger.warning("Path still created even though start and goal are the same.")
        logger.warning("FAIL")
        return passed
    
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
    logger.info("A Star Planning Test Suite")
    results = []
    
    results.append(test_none_start())
    results.append(test_none_goal())
    results.append(test_invalid_start())
    results.append(test_invalid_goal())
    results.append(test_already_at_goal())
    results.append(test_blocked_map())
    results.append(test_straight_line_clear())
    results.append(test_diagonal_clear())
    results.append(test_staggered_clear())
    results.append(test_straight_line_obstacle())
    results.append(test_diagonal_obstacle())
    results.append(test_staggered_obstacle())

    logger.info(f"Results: {sum(results)}/{len(results)} passed")
    logger.info("All tests complete.")
    
if __name__ == '__main__':
    main()