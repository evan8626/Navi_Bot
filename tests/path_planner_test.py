#!/usr/bin/env python3
"""
Test script for PathPlannerNode from path_planner.py

Tests that the PathPlannerNode correctly:
- Initializes with proper default state
- Handles callbacks (goal, map, pose)
- Guards against planning without pose or goal
- Plans paths end-to-end with A* on clear and obstacle maps
- Publishes planned paths correctly
- Resets replanning flag after successful plan
- Tracks deadline misses
"""
import logging
import sys
import numpy as np

from navi_bot.path_planner import PathPlannerNode
from navi_bot.mock_ros2 import Pose2D, OccupancyGrid

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)


# MARK: Setup

def setup_node():
    return PathPlannerNode()

def make_pose_msg(x, y, theta=0.0):
    msg = Pose2D()
    msg.x = x
    msg.y = y
    msg.theta = theta
    return msg

def make_goal_msg(x, y):
    msg = Pose2D()
    msg.x = x
    msg.y = y
    return msg

def make_grid(array):
    """Wrap a numpy array as a mock OccupancyGrid-like object the planners accept."""
    return array

def clear_map():
    return np.array([
        [0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0],
    ])

def blocked_map():
    return np.array([
        [1, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 1, 1, 1],
    ])

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

def fire(node):
    """Run one planning loop tick."""
    node.planning_loop()


# MARK: Initialization

def test_init_default_state():
    """
    TEST 1: PathPlannerNode must initialize with no goal, no path, no pose,
    and replanning_needed=False.
    """
    passed = True
    logger.info("TEST 1: Init — default state must be clean")
    node = setup_node()
    if node.current_goal is not None:
        logger.warning(f"  FAIL current_goal={node.current_goal}, expected None")
        passed = False
    else:
        logger.info("  OK   current_goal=None")
    if node.current_path is not None:
        logger.warning(f"  FAIL current_path={node.current_path}, expected None")
        passed = False
    else:
        logger.info("  OK   current_path=None")
    if node.current_pose is not None:
        logger.warning(f"  FAIL current_pose={node.current_pose}, expected None")
        passed = False
    else:
        logger.info("  OK   current_pose=None")
    if node.replanning_needed:
        logger.warning("  FAIL replanning_needed=True, expected False")
        passed = False
    else:
        logger.info("  OK   replanning_needed=False")
    if node.planning_deadline_misses != 0:
        logger.warning(f"  FAIL deadline_misses={node.planning_deadline_misses}, expected 0")
        passed = False
    else:
        logger.info("  OK   planning_deadline_misses=0")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_init_dstar_not_initialized():
    """
    TEST 2: D* Lite must not be initialized on construction.
    """
    passed = False
    logger.info("TEST 2: Init — dstar_initialized must be False on construction")
    node = setup_node()
    if not node.dstar_initialized:
        logger.info("  OK   dstar_initialized=False")
        passed = True
    else:
        logger.warning("  FAIL dstar_initialized=True")
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: Callbacks

def test_goal_callback_sets_goal():
    """
    TEST 3: goal_callback must store the goal as a tuple (x, y).
    """
    passed = False
    logger.info("TEST 3: Callback — goal_callback must store goal as (x, y)")
    node = setup_node()
    node.goal_callback(make_goal_msg(3.0, 5.0))
    if node.current_goal == (3.0, 5.0):
        logger.info(f"  OK   current_goal={node.current_goal}")
        passed = True
    else:
        logger.warning(f"  FAIL current_goal={node.current_goal}, expected (3.0, 5.0)")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_goal_callback_sets_replanning_needed():
    """
    TEST 4: goal_callback must set replanning_needed=True.
    """
    passed = False
    logger.info("TEST 4: Callback — goal_callback must set replanning_needed=True")
    node = setup_node()
    node.goal_callback(make_goal_msg(3.0, 5.0))
    if node.replanning_needed:
        logger.info("  OK   replanning_needed=True")
        passed = True
    else:
        logger.warning("  FAIL replanning_needed=False after goal_callback")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_goal_callback_resets_dstar():
    """
    TEST 5: goal_callback must reset dstar_initialized to False.
    A new goal means D* Lite must reinitialize.
    """
    passed = False
    logger.info("TEST 5: Callback — goal_callback must reset dstar_initialized to False")
    node = setup_node()
    node.dstar_initialized = True  # Pretend it was previously initialized
    node.goal_callback(make_goal_msg(3.0, 5.0))
    if not node.dstar_initialized:
        logger.info("  OK   dstar_initialized reset to False")
        passed = True
    else:
        logger.warning("  FAIL dstar_initialized still True after new goal")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_pose_callback_stores_pose():
    """
    TEST 6: current_pose_callback must store pose as (x, y, theta).
    """
    passed = False
    logger.info("TEST 6: Callback — current_pose_callback must store (x, y, theta)")
    node = setup_node()
    node.current_pose_callback(make_pose_msg(1.0, 2.0, 0.5))
    if node.current_pose == (1.0, 2.0, 0.5):
        logger.info(f"  OK   current_pose={node.current_pose}")
        passed = True
    else:
        logger.warning(f"  FAIL current_pose={node.current_pose}, expected (1.0, 2.0, 0.5)")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_map_callback_sets_grid():
    """
    TEST 7: map_callback must update both the global and D* Lite planners.
    """
    passed = True
    logger.info("TEST 7: Callback — map_callback must update global and D* planners")
    node = setup_node()
    grid = clear_map()
    node.map_callback(grid)
    if node.global_planner.occupancy_grid is None:
        logger.warning("  FAIL global_planner.occupancy_grid is None after map_callback")
        passed = False
    else:
        logger.info("  OK   global_planner.occupancy_grid set")
    if node.DStar_local_planner.occupancy_grid is None:
        logger.warning("  FAIL DStar_local_planner.occupancy_grid is None after map_callback")
        passed = False
    else:
        logger.info("  OK   DStar_local_planner.occupancy_grid set")
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: Planning Loop Guard Conditions

def test_planning_loop_no_goal_does_nothing():
    """
    TEST 8: planning_loop must return immediately if current_goal is None.
    current_path must remain None.
    """
    passed = False
    logger.info("TEST 8: Guard — planning_loop with no goal must not plan")
    node = setup_node()
    node.current_pose_callback(make_pose_msg(0.0, 0.0))
    node.replanning_needed = True
    node.map_callback(clear_map())
    fire(node)
    if node.current_path is None:
        logger.info("  OK   current_path remains None")
        passed = True
    else:
        logger.warning(f"  FAIL current_path={node.current_path}, expected None")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_planning_loop_no_replanning_does_nothing():
    """
    TEST 9: planning_loop must return immediately if replanning_needed is False.
    """
    passed = False
    logger.info("TEST 9: Guard — planning_loop with replanning_needed=False must not plan")
    node = setup_node()
    node.current_pose_callback(make_pose_msg(0.0, 0.0))
    node.current_goal = (5, 5)
    node.replanning_needed = False
    node.map_callback(clear_map())
    fire(node)
    if node.current_path is None:
        logger.info("  OK   current_path remains None")
        passed = True
    else:
        logger.warning(f"  FAIL current_path={node.current_path}, expected None")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_planning_loop_no_pose_does_nothing():
    """
    TEST 10: planning_loop must return early if current_pose is None.
    """
    passed = False
    logger.info("TEST 10: Guard — planning_loop with no pose must not plan")
    node = setup_node()
    node.goal_callback(make_goal_msg(5.0, 5.0))
    node.map_callback(clear_map())
    # Do NOT set current_pose
    fire(node)
    if node.current_path is None:
        logger.info("  OK   current_path remains None (no pose)")
        passed = True
    else:
        logger.warning(f"  FAIL current_path={node.current_path}, expected None")
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: Planning Integration

def test_planning_finds_path_clear_map():
    """
    TEST 11: planning_loop must find and store a path on a clear map.
    """
    passed = False
    logger.info("TEST 11: Planning — must find a path on a clear map")
    node = setup_node()
    node.map_callback(clear_map())
    node.current_pose_callback(make_pose_msg(0.0, 0.0))
    node.goal_callback(make_goal_msg(0.0, 7.0))
    fire(node)
    if node.current_path is not None and len(node.current_path) > 0:
        logger.info(f"  OK   path found with {len(node.current_path)} waypoints")
        passed = True
    else:
        logger.warning("  FAIL no path found on clear map")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_planning_path_ends_at_goal():
    """
    TEST 12: The last waypoint of the planned path must equal the goal.
    """
    passed = False
    logger.info("TEST 12: Planning — last waypoint must equal the goal")
    node = setup_node()
    node.map_callback(clear_map())
    node.current_pose_callback(make_pose_msg(0.0, 0.0))
    goal = (0, 7)
    node.goal_callback(make_goal_msg(goal[0], goal[1]))
    fire(node)
    if node.current_path is not None and node.current_path[-1] == goal:
        logger.info(f"  OK   last waypoint={node.current_path[-1]}")
        passed = True
    else:
        last = node.current_path[-1] if node.current_path else None
        logger.warning(f"  FAIL last waypoint={last}, expected {goal}")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_planning_path_starts_at_start():
    """
    TEST 13: The first waypoint of the planned path must equal the start position.
    """
    passed = False
    logger.info("TEST 13: Planning — first waypoint must equal the start position")
    node = setup_node()
    node.map_callback(clear_map())
    start = (0, 0)
    node.current_pose_callback(make_pose_msg(start[0], start[1]))
    node.goal_callback(make_goal_msg(0.0, 7.0))
    fire(node)
    if node.current_path is not None and node.current_path[0] == start:
        logger.info(f"  OK   first waypoint={node.current_path[0]}")
        passed = True
    else:
        first = node.current_path[0] if node.current_path else None
        logger.warning(f"  FAIL first waypoint={first}, expected {start}")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_planning_resets_replanning_flag():
    """
    TEST 14: After a successful plan, replanning_needed must be reset to False.
    """
    passed = False
    logger.info("TEST 14: Planning — replanning_needed must be False after successful plan")
    node = setup_node()
    node.map_callback(clear_map())
    node.current_pose_callback(make_pose_msg(0.0, 0.0))
    node.goal_callback(make_goal_msg(0.0, 7.0))
    fire(node)
    if not node.replanning_needed:
        logger.info("  OK   replanning_needed=False")
        passed = True
    else:
        logger.warning("  FAIL replanning_needed still True after successful plan")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_planning_publishes_path():
    """
    TEST 15: After a successful plan, a path message must be published.
    """
    passed = False
    logger.info("TEST 15: Planning — must publish path after successful plan")
    node = setup_node()
    node.map_callback(clear_map())
    node.current_pose_callback(make_pose_msg(0.0, 0.0))
    node.goal_callback(make_goal_msg(0.0, 7.0))
    fire(node)
    last_msg = node.path_pub.last_msg
    if last_msg is not None and len(last_msg.poses) > 0:
        logger.info(f"  OK   published path with {len(last_msg.poses)} poses")
        passed = True
    else:
        logger.warning(f"  FAIL path_pub.last_msg={last_msg}")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_planning_published_path_matches_waypoints():
    """
    TEST 16: The published path poses must match the planned waypoints exactly.
    """
    passed = True
    logger.info("TEST 16: Planning — published poses must match planned waypoints")
    node = setup_node()
    node.map_callback(clear_map())
    node.current_pose_callback(make_pose_msg(0.0, 0.0))
    node.goal_callback(make_goal_msg(0.0, 7.0))
    fire(node)
    if node.current_path is None or node.path_pub.last_msg is None:
        logger.warning("  FAIL no path or published message")
        logger.info("FAIL")
        return False
    poses = node.path_pub.last_msg.poses
    path = node.current_path
    if len(poses) != len(path):
        logger.warning(f"  FAIL pose count {len(poses)} != path length {len(path)}")
        passed = False
    else:
        for i, (pose, waypoint) in enumerate(zip(poses, path)):
            if pose.x != waypoint[0] or pose.y != waypoint[1]:
                logger.warning(f"  FAIL pose[{i}]=({pose.x},{pose.y}) != waypoint={waypoint}")
                passed = False
                break
        else:
            logger.info(f"  OK   all {len(poses)} poses match waypoints")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_planning_finds_path_obstacle_map():
    """
    TEST 17: planning_loop must find a path around obstacles.
    """
    passed = False
    logger.info("TEST 17: Planning — must find a path on an obstacle map")
    node = setup_node()
    node.map_callback(obstacle_map())
    node.current_pose_callback(make_pose_msg(0.0, 0.0))
    node.goal_callback(make_goal_msg(0.0, 7.0))
    fire(node)
    if node.current_path is not None and len(node.current_path) > 0:
        logger.info(f"  OK   path found with {len(node.current_path)} waypoints")
        passed = True
    else:
        logger.warning("  FAIL no path found on obstacle map")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_planning_blocked_map_no_path():
    """
    TEST 18: planning_loop must not store a path on a fully blocked map.
    """
    passed = False
    logger.info("TEST 18: Planning — blocked map must produce no path")
    node = setup_node()
    node.map_callback(blocked_map())
    node.current_pose_callback(make_pose_msg(0.0, 0.0))
    node.goal_callback(make_goal_msg(0.0, 7.0))
    fire(node)
    if node.current_path is None:
        logger.info("  OK   no path on blocked map")
        passed = True
    else:
        logger.warning(f"  FAIL path returned on blocked map: {node.current_path}")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_planning_path_contains_no_obstacles():
    """
    TEST 19: Every waypoint in the planned path must be on a free cell.
    """
    passed = True
    logger.info("TEST 19: Planning — path must not pass through occupied cells")
    node = setup_node()
    grid = obstacle_map()
    node.map_callback(grid)
    node.current_pose_callback(make_pose_msg(0.0, 0.0))
    node.goal_callback(make_goal_msg(0.0, 7.0))
    fire(node)
    if node.current_path is None:
        logger.warning("  FAIL no path returned")
        logger.info("FAIL")
        return False
    for waypoint in node.current_path:
        row, col = int(waypoint[0]), int(waypoint[1])
        if grid[row][col] != 0:
            logger.warning(f"  FAIL waypoint {waypoint} is on an occupied cell")
            passed = False
            break
    else:
        logger.info(f"  OK   all {len(node.current_path)} waypoints are on free cells")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_planning_new_goal_triggers_replan():
    """
    TEST 20: Receiving a new goal after a completed plan must set replanning_needed=True
    and reset dstar_initialized=False.
    """
    passed = True
    logger.info("TEST 20: Planning — new goal after completed plan must trigger replan")
    node = setup_node()
    node.map_callback(clear_map())
    node.current_pose_callback(make_pose_msg(0.0, 0.0))
    node.goal_callback(make_goal_msg(0.0, 7.0))
    fire(node)
    # Now send a new goal
    node.goal_callback(make_goal_msg(7.0, 7.0))
    if not node.replanning_needed:
        logger.warning("  FAIL replanning_needed=False after new goal")
        passed = False
    else:
        logger.info("  OK   replanning_needed=True")
    if node.dstar_initialized:
        logger.warning("  FAIL dstar_initialized=True after new goal")
        passed = False
    else:
        logger.info("  OK   dstar_initialized=False")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_planning_pose_update_used_as_start():
    """
    TEST 21: The planning loop must use the most recent pose as the path start,
    not a stale or hardcoded value.
    """
    passed = False
    logger.info("TEST 21: Planning — must use current_pose as path start")
    node = setup_node()
    node.map_callback(clear_map())
    start = (2, 2)
    node.current_pose_callback(make_pose_msg(start[0], start[1]))
    node.goal_callback(make_goal_msg(2.0, 7.0))
    fire(node)
    if node.current_path is not None and node.current_path[0] == start:
        logger.info(f"  OK   path starts at current_pose {start}")
        passed = True
    else:
        first = node.current_path[0] if node.current_path else None
        logger.warning(f"  FAIL path starts at {first}, expected {start}")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_dstar_initialized_after_plan():
    """
    TEST 22: After a successful plan, dstar_initialized must be True.
    """
    passed = False
    logger.info("TEST 22: D* Lite — dstar_initialized must be True after planning loop runs")
    node = setup_node()
    node.map_callback(clear_map())
    node.current_pose_callback(make_pose_msg(0.0, 0.0))
    node.goal_callback(make_goal_msg(0.0, 7.0))
    fire(node)
    if node.dstar_initialized:
        logger.info("  OK   dstar_initialized=True")
        passed = True
    else:
        logger.warning("  FAIL dstar_initialized=False after plan")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_dstar_start_set_after_plan():
    """
    TEST 23: After a successful plan, dstar_start must be set to the robot's start position.
    """
    passed = False
    logger.info("TEST 23: D* Lite — dstar_start must be set to robot start after planning")
    node = setup_node()
    node.map_callback(clear_map())
    start = (0, 0)
    node.current_pose_callback(make_pose_msg(start[0], start[1]))
    node.goal_callback(make_goal_msg(0.0, 7.0))
    fire(node)
    if node.dstar_start is not None:
        logger.info(f"  OK   dstar_start={node.dstar_start}")
        passed = True
    else:
        logger.warning("  FAIL dstar_start is None after plan")
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: Main

def main():
    tests = [
        test_init_default_state,
        test_init_dstar_not_initialized,
        test_goal_callback_sets_goal,
        test_goal_callback_sets_replanning_needed,
        test_goal_callback_resets_dstar,
        test_pose_callback_stores_pose,
        test_map_callback_sets_grid,
        test_planning_loop_no_goal_does_nothing,
        test_planning_loop_no_replanning_does_nothing,
        test_planning_loop_no_pose_does_nothing,
        test_planning_finds_path_clear_map,
        test_planning_path_ends_at_goal,
        test_planning_path_starts_at_start,
        test_planning_resets_replanning_flag,
        test_planning_publishes_path,
        test_planning_published_path_matches_waypoints,
        test_planning_finds_path_obstacle_map,
        test_planning_blocked_map_no_path,
        test_planning_path_contains_no_obstacles,
        test_planning_new_goal_triggers_replan,
        test_planning_pose_update_used_as_start,
        test_dstar_initialized_after_plan,
        test_dstar_start_set_after_plan,
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

    logger.info("Path Planner Node Test Suite")
    results = [t() for t in selected]

    logger.info(f"Results: {sum(results)}/{len(results)} passed")
    logger.info("All tests complete.")
    sys.exit(0 if all(results) else 1)


if __name__ == '__main__':
    main()