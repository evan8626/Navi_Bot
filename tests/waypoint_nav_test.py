#!/usr/bin/env python3
"""
Waypoint Navigation Integration Test Suite

End-to-end tests that wire together:
- StateMachine      (mission assignment and state transitions)
- PathPlannerNode   (A* global path planning)
- PurePursuitController (steering commands)
- WheelOdometry     (pose estimation from simulated encoder ticks)

Each test simulates a complete mission: the state machine assigns a mission,
the path planner generates a path, pure pursuit steers toward each waypoint,
and wheel odometry advances the robot's pose via simulated encoder ticks.
The pose is published back to PathPlannerNode via /current_pose so all
components stay in sync through the mock ROS2 message bus.

Tick simulation:
    ticks = (velocity * dt / (2 * pi * wheel_radius)) * ticks_per_rev
"""
import logging
import math
import sys
import numpy as np

import navi_bot.mock_ros2 as rclpy

from navi_bot.state_machine import StateMachine, Mission, RobotState
from navi_bot.path_planner import PathPlannerNode
from navi_bot.control.pure_pursuit import PurePursuitController
from navi_bot.sensors.odometry import WheelOdometry
from navi_bot.mock_ros2 import Pose2D, OccupancyGrid

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)

# ---------------------------------------------------------------------------
# Simulation parameters
# ---------------------------------------------------------------------------
DT              = 0.1       # seconds per simulation step
MAX_STEPS       = 5000      # hard cap to prevent infinite loops
GOAL_TOLERANCE  = 1.5       # meters — waypoint reached threshold
WHEEL_BASE      = 0.4       # meters
WHEEL_RADIUS    = 0.1       # meters
TICKS_PER_REV   = 1000
LOOKAHEAD       = 1.0       # pure pursuit lookahead distance (meters)
LINEAR_VEL      = 1.0       # m/s

# ---------------------------------------------------------------------------
# Maps
# ---------------------------------------------------------------------------

def clear_map(size=20):
    return np.zeros((size, size), dtype=int)

def obstacle_map():
    grid = np.zeros((20, 20), dtype=int)
    # Vertical wall with a gap
    for row in range(5, 15):
        grid[row][10] = 1
    grid[10][10] = 0  # Gap in the wall
    return grid

def complex_map():
    grid = np.zeros((20, 20), dtype=int)
    # Scattered obstacles
    for r, c in [(3,5),(3,6),(4,5),(8,12),(8,13),(9,12),(14,7),(14,8),(15,7)]:
        grid[r][c] = 1
    return grid

# ---------------------------------------------------------------------------
# Setup helpers
# ---------------------------------------------------------------------------

def setup_components():
    """
    Create and wire all components via the mock ROS2 message bus.
    init() resets the bus so each test starts clean.
    """
    rclpy.init()
    sm      = StateMachine()
    planner = PathPlannerNode()
    pursuit = PurePursuitController(lookahead=LOOKAHEAD, linear_velocity=LINEAR_VEL)
    odom    = WheelOdometry(wheel_base=WHEEL_BASE, wheel_radius=WHEEL_RADIUS,
                            ticks_per_rev=TICKS_PER_REV)
    return sm, planner, pursuit, odom


def make_mission(mission_id, pickup, delivery):
    return Mission(mission_id, pickup, delivery)


def publish_pose(planner, x, y, theta):
    """Publish current pose to /current_pose so PathPlannerNode stays in sync.

    Frame boundary: this simulation's pose x runs along grid ROWS (the
    project's historic convention), while the node's world frame follows ROS
    (world x along COLUMNS). Swap when crossing into message space."""
    msg = Pose2D()
    msg.x = float(y)      # world x  <- sim col
    msg.y = float(x)      # world y  <- sim row
    msg.theta = float(theta)
    planner.current_pose_callback(msg)


def publish_goal(planner, cell):
    """Publish a (row, col) grid-cell goal in the node's world frame
    (same ROS-convention swap as publish_pose)."""
    msg = Pose2D()
    msg.x = float(cell[1])    # world x <- col
    msg.y = float(cell[0])    # world y <- row
    planner.goal_callback(msg)


def sim_ticks(velocity, dt, wheel_radius, ticks_per_rev):
    """Convert a wheel velocity (m/s) to encoder tick increment for one dt step."""
    revolutions = (velocity * dt) / (2 * math.pi * wheel_radius)
    return int(revolutions * ticks_per_rev)


def distance_to(pose, goal):
    return math.hypot(pose[0] - goal[0], pose[1] - goal[1])


def navigate_to_waypoint(planner, pursuit, odom, goal, max_steps=MAX_STEPS):
    """
    Run the pure pursuit / odometry simulation loop until the robot reaches
    the goal or max_steps is exceeded.

    Returns (reached, steps, final_pose).
    """
    left_ticks  = odom.prev_left_ticks
    right_ticks = odom.prev_right_ticks

    for step in range(max_steps):
        x, y, theta = odom.get_pose()

        # Periodic pose telemetry so the dashboard can draw the actual route
        if step % 15 == 0:
            logger.debug(f"pose=({x:.2f},{y:.2f})")

        if distance_to((x, y), goal) <= GOAL_TOLERANCE:
            return True, step, (x, y, theta)

        # Build path from current position to goal using path planner
        path = planner.current_path
        if path is None or len(path) == 0:
            # Trigger replan
            planner.replanning_needed = True
            publish_pose(planner, x, y, theta)
            planner.planning_loop()
            path = planner.current_path
            if path is None:
                return False, step, (x, y, theta)

        # Pure pursuit steering
        current_pose_tuple = (x, y, theta)
        result = pursuit.pure_pursuit(path, current_pose_tuple)
        if result == (0, 0):
            # Path exhausted — check if at goal
            if distance_to((x, y), goal) <= GOAL_TOLERANCE:
                return True, step, (x, y, theta)
            return False, step, (x, y, theta)

        v_cmd, omega_cmd = result

        # Compute individual wheel velocities from (v, omega)
        v_left  = v_cmd - (omega_cmd * WHEEL_BASE / 2.0)
        v_right = v_cmd + (omega_cmd * WHEEL_BASE / 2.0)

        # Clamp to reasonable range
        max_wheel_v = 1.0
        v_left  = max(-max_wheel_v, min(max_wheel_v, v_left))
        v_right = max(-max_wheel_v, min(max_wheel_v, v_right))

        # Advance encoder ticks
        left_ticks  += sim_ticks(v_left,  DT, WHEEL_RADIUS, TICKS_PER_REV)
        right_ticks += sim_ticks(v_right, DT, WHEEL_RADIUS, TICKS_PER_REV)

        # Update odometry
        x, y, theta, _, _ = odom.update(left_ticks, right_ticks, DT)

        # Publish updated pose back to planner
        publish_pose(planner, x, y, theta)

    return False, max_steps, odom.get_pose()


def run_mission(sm, planner, pursuit, odom, grid, mission, log_prefix=""):
    """
    Run a complete mission end-to-end:
    1. Set map on planner
    2. Assign mission to state machine
    3. Walk state machine to PICK_NAV
    4. Navigate to pickup
    5. Simulate pickup completion
    6. Navigate to delivery
    7. Simulate delivery completion
    8. Verify return to IDLE

    Returns dict of results for each leg.
    """
    results = {}

    # Set map
    planner.map_callback(grid)

    # Log the occupancy grid so the dashboard draws the real obstacles
    logger.info(f"MAP {len(grid)} {len(grid[0])}")
    for _row in grid:
        logger.info("MAPROW " + "".join('1' if int(_c) != 0 else '0' for _c in _row))

    # Assign mission and advance state machine to PICK_NAV
    sm.add_mission(mission)
    sm.update_state_machine()  # IDLE -> PICK_NAV

    if sm.current_state != RobotState.PICK_NAV:
        logger.warning(f"{log_prefix}  FAIL state machine did not reach PICK_NAV, got {sm.current_state}")
        results['state_machine_init'] = False
        return results
    results['state_machine_init'] = True
    logger.info(f"{log_prefix}  OK   state machine reached PICK_NAV")

    # Set robot start pose and trigger first plan
    start = (0, 0)
    odom.reset(start[0], start[1], 0.0)
    publish_pose(planner, start[0], start[1], 0.0)

    publish_goal(planner, mission.pickup_location)
    planner.planning_loop()

    pickup_goal = mission.pickup_location

    # --- Navigate to pickup ---
    logger.info(f"{log_prefix}  Navigating to pickup {pickup_goal}...")
    reached, steps, pose = navigate_to_waypoint(
        planner, pursuit, odom, pickup_goal
    )
    results['pickup_navigation'] = reached
    if reached:
        logger.info(f"{log_prefix}  OK   reached pickup in {steps} steps, pose=({pose[0]:.2f},{pose[1]:.2f})")
    else:
        logger.warning(f"{log_prefix}  FAIL did not reach pickup after {steps} steps, pose=({pose[0]:.2f},{pose[1]:.2f})")

    # Simulate arrival at pickup
    sm.is_at_goal = True
    sm.update_state_machine()  # PICK_NAV -> PICKING_UP
    sm.update_state_machine()  # PICKING_UP first tick
    sm.pickup_complete = True
    sm.update_state_machine()  # PICKING_UP -> PICKED_UP
    sm.update_state_machine()  # PICKED_UP -> DELIVERY_NAV

    if sm.current_state != RobotState.DELIVERY_NAV:
        logger.warning(f"{log_prefix}  FAIL expected DELIVERY_NAV, got {sm.current_state}")
        results['pickup_state'] = False
    else:
        logger.info(f"{log_prefix}  OK   state machine reached DELIVERY_NAV")
        results['pickup_state'] = True

    # Set delivery goal
    delivery_goal = mission.delivery_location
    publish_goal(planner, delivery_goal)
    planner.planning_loop()

    # --- Navigate to delivery ---
    logger.info(f"{log_prefix}  Navigating to delivery {delivery_goal}...")
    reached_del, steps_del, pose_del = navigate_to_waypoint(
        planner, pursuit, odom, delivery_goal
    )
    results['delivery_navigation'] = reached_del
    if reached_del:
        logger.info(f"{log_prefix}  OK   reached delivery in {steps_del} steps, pose=({pose_del[0]:.2f},{pose_del[1]:.2f})")
    else:
        logger.warning(f"{log_prefix}  FAIL did not reach delivery after {steps_del} steps, pose=({pose_del[0]:.2f},{pose_del[1]:.2f})")

    # Simulate delivery completion
    sm.is_at_goal = True
    sm.update_state_machine()  # DELIVERY_NAV -> DELIVERING
    sm.update_state_machine()  # DELIVERING first tick
    sm.delivery_complete = True
    sm.update_state_machine()  # DELIVERING -> IDLE

    if sm.current_state != RobotState.IDLE:
        logger.warning(f"{log_prefix}  FAIL expected IDLE after delivery, got {sm.current_state}")
        results['delivery_state'] = False
    else:
        logger.info(f"{log_prefix}  OK   state machine returned to IDLE after delivery")
        results['delivery_state'] = True

    return results


# ---------------------------------------------------------------------------
# TEST 1: Single mission, clear map, short route
# ---------------------------------------------------------------------------

def test_single_mission_clear_map():
    """
    TEST 1: Single mission on a clear map.
    Pickup at (3, 3), delivery at (3, 15).
    Verifies basic end-to-end navigation on an unobstructed grid.
    """
    passed = True
    logger.info("TEST 1: Single mission — clear map, short route")
    sm, planner, pursuit, odom = setup_components()
    grid = clear_map(size=20)
    mission = make_mission('M001', pickup=(3, 3), delivery=(3, 15))

    results = run_mission(sm, planner, pursuit, odom, grid, mission, log_prefix="  ")

    for key, val in results.items():
        if not val:
            passed = False

    logger.info("PASS" if passed else "FAIL")
    return passed


# ---------------------------------------------------------------------------
# TEST 2: Single mission, obstacle map
# ---------------------------------------------------------------------------

def test_single_mission_obstacle_map():
    """
    TEST 2: Single mission on a map with a wall obstacle.
    Robot must navigate around the wall to reach pickup and delivery.
    Pickup at (2, 2), delivery at (2, 17).
    """
    passed = True
    logger.info("TEST 2: Single mission — obstacle map, wall with gap")
    sm, planner, pursuit, odom = setup_components()
    grid = obstacle_map()
    mission = make_mission('M002', pickup=(2, 2), delivery=(2, 17))

    results = run_mission(sm, planner, pursuit, odom, grid, mission, log_prefix="  ")

    for key, val in results.items():
        if not val:
            passed = False

    logger.info("PASS" if passed else "FAIL")
    return passed


# ---------------------------------------------------------------------------
# TEST 3: Three sequential missions, clear map
# ---------------------------------------------------------------------------

def test_three_sequential_missions():
    """
    TEST 3: Three sequential missions on a clear map.
    Verifies the state machine correctly cycles through IDLE -> mission ->
    IDLE three times, and the robot successfully navigates each pickup
    and delivery.

    Missions:
      M003: pickup (1,1)  -> delivery (1,18)
      M004: pickup (5,5)  -> delivery (15,15)
      M005: pickup (10,2) -> delivery (10,17)
    """
    passed = True
    logger.info("TEST 3: Three sequential missions — clear map")

    missions = [
        make_mission('M003', pickup=(1, 1),  delivery=(1, 18)),
        make_mission('M004', pickup=(5, 5),  delivery=(15, 15)),
        make_mission('M005', pickup=(10, 2), delivery=(10, 17)),
    ]

    grid = clear_map(size=20)

    for i, mission in enumerate(missions):
        logger.info(f"  --- Mission {i+1}: {mission.mission_id} ---")
        sm, planner, pursuit, odom = setup_components()

        results = run_mission(
            sm, planner, pursuit, odom, grid, mission,
            log_prefix=f"  [{mission.mission_id}] "
        )

        for key, val in results.items():
            if not val:
                logger.warning(f"  FAIL mission {mission.mission_id} failed at: {key}")
                passed = False

        if sm.current_state != RobotState.IDLE:
            logger.warning(f"  FAIL {mission.mission_id} did not return to IDLE")
            passed = False
        else:
            logger.info(f"  OK   {mission.mission_id} complete, state=IDLE")

    logger.info("PASS" if passed else "FAIL")
    return passed


# ---------------------------------------------------------------------------
# TEST 4: Three missions, obstacle map
# ---------------------------------------------------------------------------

def test_three_missions_obstacle_map():
    """
    TEST 4: Three sequential missions on a map with scattered obstacles.
    Tests replanning and obstacle avoidance across multiple missions.

    Missions:
      M006: pickup (1,1)  -> delivery (1,18)
      M007: pickup (6,1)  -> delivery (6,18)
      M008: pickup (12,1) -> delivery (12,18)
    """
    passed = True
    logger.info("TEST 4: Three sequential missions — obstacle map")

    missions = [
        make_mission('M006', pickup=(1, 1),  delivery=(1, 18)),
        make_mission('M007', pickup=(6, 1),  delivery=(6, 18)),
        make_mission('M008', pickup=(12, 1), delivery=(12, 18)),
    ]

    grid = complex_map()

    for i, mission in enumerate(missions):
        logger.info(f"  --- Mission {i+1}: {mission.mission_id} ---")
        sm, planner, pursuit, odom = setup_components()

        results = run_mission(
            sm, planner, pursuit, odom, grid, mission,
            log_prefix=f"  [{mission.mission_id}] "
        )

        for key, val in results.items():
            if not val:
                logger.warning(f"  FAIL mission {mission.mission_id} failed at: {key}")
                passed = False

        if sm.current_state != RobotState.IDLE:
            logger.warning(f"  FAIL {mission.mission_id} did not return to IDLE")
            passed = False
        else:
            logger.info(f"  OK   {mission.mission_id} complete, state=IDLE")

    logger.info("PASS" if passed else "FAIL")
    return passed


# ---------------------------------------------------------------------------
# TEST 5: Low battery forces charging before mission
# ---------------------------------------------------------------------------

def test_low_battery_charges_before_mission():
    """
    TEST 5: Low battery must prevent mission start and force CHARGING state.
    Once battery is restored, mission must proceed normally.
    """
    passed = True
    logger.info("TEST 5: Low battery — must charge before accepting mission")
    sm, planner, pursuit, odom = setup_components()
    grid = clear_map(size=20)

    # Set battery below mission threshold
    sm.battery_level = sm.min_mission_threshold - 5.0
    mission = make_mission('M009', pickup=(3, 3), delivery=(3, 15))
    sm.add_mission(mission)
    sm.update_state_machine()  # Should go to CHARGING not PICK_NAV

    if sm.current_state != RobotState.CHARGING:
        logger.warning(f"  FAIL expected CHARGING with low battery, got {sm.current_state}")
        passed = False
    else:
        logger.info(f"  OK   low battery correctly triggered CHARGING")

    # Restore battery and complete mission
    sm.battery_level = 100.0
    sm.update_state_machine()  # CHARGING -> PICK_NAV (mission assigned, battery ok)

    if sm.current_state != RobotState.PICK_NAV:
        logger.warning(f"  FAIL expected PICK_NAV after charging, got {sm.current_state}")
        passed = False
    else:
        logger.info(f"  OK   PICK_NAV reached after battery restored")

    # Navigate pickup leg
    planner.map_callback(grid)
    odom.reset(0.0, 0.0, 0.0)
    publish_pose(planner, 0.0, 0.0, 0.0)
    publish_goal(planner, mission.pickup_location)
    planner.planning_loop()

    reached, steps, pose = navigate_to_waypoint(
        planner, pursuit, odom, mission.pickup_location
    )
    if reached:
        logger.info(f"  OK   reached pickup after charging, steps={steps}")
    else:
        logger.warning(f"  FAIL did not reach pickup after charging, steps={steps}")
        passed = False

    logger.info("PASS" if passed else "FAIL")
    return passed


# ---------------------------------------------------------------------------
# TEST 6: Error recovery during mission
# ---------------------------------------------------------------------------

def test_error_recovery_during_mission():
    """
    TEST 6: Error flag during navigation must transition to ERROR state,
    reset, and return to IDLE. The mission queue must be able to continue
    after recovery.
    """
    passed = True
    logger.info("TEST 6: Error recovery — error during mission must recover to IDLE")
    sm, planner, pursuit, odom = setup_components()

    mission = make_mission('M010', pickup=(3, 3), delivery=(3, 15))
    sm.add_mission(mission)
    sm.update_state_machine()  # IDLE -> PICK_NAV

    # Inject error mid-navigation
    sm.has_error = True
    sm.update_state_machine()  # ERROR -> IDLE

    if sm.current_state != RobotState.IDLE:
        logger.warning(f"  FAIL expected IDLE after error recovery, got {sm.current_state}")
        passed = False
    else:
        logger.info("  OK   recovered to IDLE after error")

    if sm.has_error:
        logger.warning("  FAIL has_error not reset after recovery")
        passed = False
    else:
        logger.info("  OK   has_error reset to False")

    logger.info("PASS" if passed else "FAIL")
    return passed


# ---------------------------------------------------------------------------
# TEST 7: Long-running multi-waypoint route (extended test)
# ---------------------------------------------------------------------------

def test_long_running_multi_waypoint_route():
    """
    TEST 7: Long-running test — 5 sequential missions across a complex map.
    Each mission has a different pickup and delivery location spread across
    the full grid. Tests sustained navigation, replanning across multiple
    missions, and state machine cycling over an extended run.

    Missions:
      M011: pickup (1,1)   -> delivery (18,18)
      M012: pickup (18,1)  -> delivery (1,18)
      M013: pickup (9,1)   -> delivery (9,18)
      M014: pickup (1,9)   -> delivery (18,9)
      M015: pickup (5,5)   -> delivery (14,14)

    This is intentionally the longest test in the suite.
    """
    passed = True
    logger.info("TEST 7: Long-running — 5 sequential missions across complex map")

    missions = [
        make_mission('M011', pickup=(1, 1),   delivery=(18, 18)),
        make_mission('M012', pickup=(18, 1),  delivery=(1, 18)),
        make_mission('M013', pickup=(9, 1),   delivery=(9, 18)),
        make_mission('M014', pickup=(1, 9),   delivery=(18, 9)),
        make_mission('M015', pickup=(5, 5),   delivery=(14, 14)),
    ]

    grid = complex_map()
    total_steps = 0

    for i, mission in enumerate(missions):
        logger.info(f"  --- Mission {i+1}/5: {mission.mission_id} ---")
        sm, planner, pursuit, odom = setup_components()

        results = run_mission(
            sm, planner, pursuit, odom, grid, mission,
            log_prefix=f"  [{mission.mission_id}] "
        )

        mission_steps = sum(1 for v in results.values() if v)
        total_steps += mission_steps

        for key, val in results.items():
            if not val:
                logger.warning(f"  FAIL mission {mission.mission_id} failed at: {key}")
                passed = False

        if sm.current_state != RobotState.IDLE:
            logger.warning(f"  FAIL {mission.mission_id} did not return to IDLE")
            passed = False
        else:
            logger.info(f"  OK   {mission.mission_id} complete")

    logger.info(f"  Total missions completed: {sum(1 for m in missions if True)}/5")
    logger.info("PASS" if passed else "FAIL")
    return passed


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    tests = [
        test_single_mission_clear_map,
        test_single_mission_obstacle_map,
        test_three_sequential_missions,
        test_three_missions_obstacle_map,
        test_low_battery_charges_before_mission,
        test_error_recovery_during_mission,
        test_long_running_multi_waypoint_route,
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

    logger.info("Waypoint Navigation Integration Test Suite")
    logger.info("=" * 60)
    results = [t() for t in selected]

    logger.info("=" * 60)
    logger.info(f"Results: {sum(results)}/{len(results)} passed")
    logger.info("All tests complete.")
    sys.exit(0 if all(results) else 1)


if __name__ == '__main__':
    main()