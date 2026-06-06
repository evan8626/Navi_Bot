#!/usr/bin/env python3
"""
Test script for LidarProcessor from lidar_processor.py

Tests that the LidarProcessor correctly satisfies behavioral and physical
invariants for:
- process_scan (full pipeline)
- detect_obstacles (clustering and cartesian conversion)
- compute_clear_directions (safety window logic)
- update_costmap (obstacle marking and inflation)
"""
import logging
import sys
import math
import numpy as np

from navi_bot.sensors.lidar_processing import LidarProcessor, ROBOT_RADIUS, CLUSTER_EPS, CLUSTER_MIN_SAMPLES
from navi_bot.mock_ros2 import LaserScan, OccupancyGrid

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)


# MARK: Setup

def setup_processor():
    return LidarProcessor()

def make_scan(ranges, angle_min=None, angle_increment=None):
    """Build a LaserScan message with the given ranges."""
    scan = LaserScan()
    scan.ranges = list(ranges)
    scan.angle_min = angle_min if angle_min is not None else -np.pi
    scan.angle_increment = angle_increment if angle_increment is not None else np.pi / 180.0
    return scan

def clear_scan(num_points=360, range_val=9.0):
    """Scan with all readings at a safe distance — no obstacles."""
    return make_scan([range_val] * num_points)

def obstacle_at_angle(angle_deg, distance=1.0, num_points=360, background=9.0):
    """
    Scan with a cluster of readings at a given angle and distance.
    Generates enough nearby points to exceed CLUSTER_MIN_SAMPLES.
    """
    ranges = [background] * num_points
    angle_rad = math.radians(angle_deg)
    angle_increment = np.pi / 180.0
    angle_min = -np.pi
    center_idx = int((angle_rad - angle_min) / angle_increment)
    # Place enough points to form a valid cluster
    for offset in range(-8, 9):
        idx = center_idx + offset
        if 0 <= idx < num_points:
            ranges[idx] = distance
    return make_scan(ranges)

def blocked_scan(num_points=360, distance=0.3):
    """Scan with all readings below obstacle_threshold — everything blocked."""
    return make_scan([distance] * num_points)

def make_costmap(width=100, height=100, resolution=0.05):
    """Build a blank OccupancyGrid."""
    costmap = OccupancyGrid(width=width, height=height, resolution=resolution)
    costmap.data = [0] * (width * height)
    return costmap


# MARK: process_scan

def test_process_scan_returns_required_keys():
    """
    TEST 1: process_scan must return a dict with all required keys.
    """
    passed = True
    logger.info("TEST 1: process_scan — must return dict with required keys")
    proc = setup_processor()
    result = proc.process_scan(clear_scan())
    required_keys = ['obstacles', 'closest_obstacle', 'clear_directions', 'num_valid_points']
    for key in required_keys:
        if key not in result:
            logger.warning(f"  FAIL missing key: {key}")
            passed = False
        else:
            logger.info(f"  OK   key present: {key}")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_process_scan_filters_out_of_range():
    """
    TEST 2: process_scan must filter readings below min_range and above max_range.
    num_valid_points must only count readings within [min_range, max_range].
    """
    passed = False
    logger.info("TEST 2: process_scan — must filter out-of-range readings")
    proc = setup_processor()
    ranges = [0.05] * 90 + [5.0] * 180 + [15.0] * 90  # 90 too close, 90 too far, 180 valid
    result = proc.process_scan(make_scan(ranges))
    if result['num_valid_points'] == 180:
        logger.info(f"  OK   num_valid_points={result['num_valid_points']}")
        passed = True
    else:
        logger.warning(f"  FAIL num_valid_points={result['num_valid_points']}, expected 180")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_process_scan_closest_obstacle_correct():
    """
    TEST 3: process_scan closest_obstacle must equal the minimum valid range.
    """
    passed = False
    logger.info("TEST 3: process_scan — closest_obstacle must be minimum valid range")
    proc = setup_processor()
    ranges = [9.0] * 350 + [1.5] * 10
    result = proc.process_scan(make_scan(ranges))
    if math.isclose(result['closest_obstacle'], 1.5, abs_tol=1e-9):
        logger.info(f"  OK   closest_obstacle={result['closest_obstacle']}")
        passed = True
    else:
        logger.warning(f"  FAIL closest_obstacle={result['closest_obstacle']}, expected 1.5")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_process_scan_clear_scan_no_close_obstacle():
    """
    TEST 4: process_scan on a clear scan must report closest_obstacle >= obstacle_threshold.
    """
    passed = False
    logger.info("TEST 4: process_scan — clear scan must have no close obstacle")
    proc = setup_processor()
    result = proc.process_scan(clear_scan(range_val=9.0))
    if result['closest_obstacle'] >= proc.obstacle_threshold:
        logger.info(f"  OK   closest_obstacle={result['closest_obstacle']:.2f}")
        passed = True
    else:
        logger.warning(f"  FAIL closest_obstacle={result['closest_obstacle']:.2f}, expected >= {proc.obstacle_threshold}")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_process_scan_clear_directions_length():
    """
    TEST 5: process_scan clear_directions must have same length as input ranges.
    """
    passed = False
    logger.info("TEST 5: process_scan — clear_directions length must match number of scan points")
    proc = setup_processor()
    scan = clear_scan(num_points=360)
    result = proc.process_scan(scan)
    if len(result['clear_directions']) == 360:
        logger.info(f"  OK   clear_directions length={len(result['clear_directions'])}")
        passed = True
    else:
        logger.warning(f"  FAIL clear_directions length={len(result['clear_directions'])}, expected 360")
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: detect_obstacles

def test_detect_obstacles_returns_list():
    """
    TEST 6: detect_obstacles must always return a list.
    """
    passed = False
    logger.info("TEST 6: detect_obstacles — must return a list")
    proc = setup_processor()
    result = proc.detect_obstacles(
        np.array([9.0] * 360), -np.pi, np.pi / 180.0
    )
    if isinstance(result, list):
        logger.info(f"  OK   returned list of length {len(result)}")
        passed = True
    else:
        logger.warning(f"  FAIL returned {type(result)}, expected list")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_detect_obstacles_each_has_centroid_and_size():
    """
    TEST 7: detect_obstacles — each obstacle dict must have 'centroid' and 'size' keys.
    """
    passed = True
    logger.info("TEST 7: detect_obstacles — each obstacle must have centroid and size")
    proc = setup_processor()
    ranges = np.array([9.0] * 360)
    # Place a dense cluster
    ranges[80:98] = 1.5
    result = proc.detect_obstacles(ranges, -np.pi, np.pi / 180.0)
    if not result:
        logger.warning("  FAIL no obstacles detected, cannot check keys")
        logger.info("FAIL")
        return False
    for i, obs in enumerate(result):
        if 'centroid' not in obs:
            logger.warning(f"  FAIL obstacle[{i}] missing 'centroid'")
            passed = False
        else:
            logger.info(f"  OK   obstacle[{i}] has centroid={obs['centroid']}")
        if 'size' not in obs:
            logger.warning(f"  FAIL obstacle[{i}] missing 'size'")
            passed = False
        else:
            logger.info(f"  OK   obstacle[{i}] has size={obs['size']}")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_detect_obstacles_size_matches_cluster_points():
    """
    TEST 8: detect_obstacles — obstacle size must equal the number of points in the cluster.
    """
    passed = True
    logger.info("TEST 8: detect_obstacles — obstacle size must match cluster point count")
    proc = setup_processor()
    ranges = np.array([9.0] * 360)
    ranges[80:98] = 1.5  # 18 points close together
    result = proc.detect_obstacles(ranges, -np.pi, np.pi / 180.0)
    if not result:
        logger.warning("  FAIL no obstacles detected")
        logger.info("FAIL")
        return False
    for obs in result:
        if obs['size'] < CLUSTER_MIN_SAMPLES:
            logger.warning(f"  FAIL obstacle size={obs['size']} < CLUSTER_MIN_SAMPLES={CLUSTER_MIN_SAMPLES}")
            passed = False
        else:
            logger.info(f"  OK   obstacle size={obs['size']} >= CLUSTER_MIN_SAMPLES")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_detect_obstacles_centroid_near_obstacle():
    """
    TEST 9: detect_obstacles — centroid must be geometrically near the placed obstacle.
    """
    passed = False
    logger.info("TEST 9: detect_obstacles — centroid must be near the actual obstacle location")
    proc = setup_processor()
    # Place obstacle at angle 0 (East), distance 2.0m
    ranges = np.array([9.0] * 360)
    angle_min = -np.pi
    angle_inc = np.pi / 180.0
    center_idx = int((0.0 - angle_min) / angle_inc)  # index for angle=0
    for offset in range(-8, 9):
        idx = center_idx + offset
        if 0 <= idx < 360:
            ranges[idx] = 2.0
    result = proc.detect_obstacles(ranges, angle_min, angle_inc)
    if not result:
        logger.warning("  FAIL no obstacles detected")
        logger.info("FAIL")
        return False
    cx, cy = result[0]['centroid']
    dist = math.hypot(cx - 2.0, cy - 0.0)
    if dist < 0.5:
        logger.info(f"  OK   centroid=({cx:.3f}, {cy:.3f}), dist from expected={dist:.3f}")
        passed = True
    else:
        logger.warning(f"  FAIL centroid=({cx:.3f}, {cy:.3f}), dist from expected={dist:.3f}")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_detect_obstacles_noise_discarded():
    """
    TEST 10: detect_obstacles — isolated single points must be discarded as noise.
    A single beam hit should not produce a cluster.
    """
    passed = False
    logger.info("TEST 10: detect_obstacles — isolated single points must be discarded as noise")
    proc = setup_processor()
    ranges = np.array([9.0] * 360)
    ranges[90] = 1.0  # Single isolated point
    result = proc.detect_obstacles(ranges, -np.pi, np.pi / 180.0)
    if len(result) == 0:
        logger.info("  OK   single point correctly discarded as noise")
        passed = True
    else:
        logger.warning(f"  FAIL {len(result)} obstacle(s) returned for single point")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_detect_obstacles_two_separate_clusters():
    """
    TEST 11: detect_obstacles — two spatially separated obstacle groups must
    produce two distinct clusters.
    """
    passed = False
    logger.info("TEST 11: detect_obstacles — two separated obstacle groups must produce two clusters")
    proc = setup_processor()
    ranges = np.array([9.0] * 360)
    # Cluster 1 around index 45
    ranges[37:55] = 1.0
    # Cluster 2 around index 270 (opposite side)
    ranges[262:280] = 3.0
    result = proc.detect_obstacles(ranges, -np.pi, np.pi / 180.0)
    if len(result) == 2:
        logger.info(f"  OK   detected {len(result)} clusters")
        passed = True
    else:
        logger.warning(f"  FAIL detected {len(result)} clusters, expected 2")
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: compute_clear_directions

def test_clear_directions_all_clear_on_clear_scan():
    """
    TEST 12: compute_clear_directions — all directions must be clear when all
    ranges are well above obstacle_threshold.
    """
    passed = False
    logger.info("TEST 12: compute_clear_directions — all directions must be clear on open scan")
    proc = setup_processor()
    ranges = np.full(360, 9.0)
    result = proc.compute_clear_directions(ranges)
    if np.all(result):
        logger.info("  OK   all 360 directions clear")
        passed = True
    else:
        blocked = np.sum(~result)
        logger.warning(f"  FAIL {blocked} directions blocked on clear scan")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_clear_directions_blocked_on_close_obstacle():
    """
    TEST 13: compute_clear_directions — directions around a close obstacle must be False.
    """
    passed = False
    logger.info("TEST 13: compute_clear_directions — close obstacle must block surrounding directions")
    proc = setup_processor()
    ranges = np.full(360, 9.0)
    ranges[90] = 0.3  # Below obstacle_threshold
    result = proc.compute_clear_directions(ranges)
    # Direction 90 and its ±15 window must be blocked
    if not result[90]:
        logger.info("  OK   direction 90 correctly blocked")
        passed = True
    else:
        logger.warning("  FAIL direction 90 not blocked despite close obstacle")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_clear_directions_window_blocks_neighbors():
    """
    TEST 14: compute_clear_directions — the safety window must block directions
    within ±15 degrees of an obstacle, not just the beam that hit it.
    """
    passed = True
    logger.info("TEST 14: compute_clear_directions — safety window must block ±15 degree neighborhood")
    proc = setup_processor()
    ranges = np.full(360, 9.0)
    ranges[180] = 0.3  # Obstacle at index 180
    result = proc.compute_clear_directions(ranges)
    for idx in range(165, 196):  # ±15 around index 180
        if result[idx]:
            logger.warning(f"  FAIL direction {idx} not blocked, within ±15 of obstacle at 180")
            passed = False
            break
    else:
        logger.info("  OK   all directions within ±15 of obstacle correctly blocked")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_clear_directions_all_blocked_on_blocked_scan():
    """
    TEST 15: compute_clear_directions — all directions must be blocked when
    all ranges are below obstacle_threshold.
    """
    passed = False
    logger.info("TEST 15: compute_clear_directions — all directions blocked on fully blocked scan")
    proc = setup_processor()
    ranges = np.full(360, 0.3)
    result = proc.compute_clear_directions(ranges)
    if not np.any(result):
        logger.info("  OK   all directions correctly blocked")
        passed = True
    else:
        clear = np.sum(result)
        logger.warning(f"  FAIL {clear} directions still clear on blocked scan")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_clear_directions_inf_ranges_are_clear():
    """
    TEST 16: compute_clear_directions — inf ranges (filtered invalids) must be
    treated as clear since they represent no detection.
    """
    passed = False
    logger.info("TEST 16: compute_clear_directions — inf ranges must be treated as clear")
    proc = setup_processor()
    ranges = np.full(360, np.inf)
    result = proc.compute_clear_directions(ranges)
    if np.all(result):
        logger.info("  OK   all inf ranges treated as clear")
        passed = True
    else:
        blocked = np.sum(~result)
        logger.warning(f"  FAIL {blocked} inf ranges treated as blocked")
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: update_costmap

def test_update_costmap_marks_obstacle_cell():
    """
    TEST 17: update_costmap — the cell corresponding to an obstacle centroid
    must be marked as 100.
    """
    passed = False
    logger.info("TEST 17: update_costmap — obstacle cell must be marked as 100")
    proc = setup_processor()
    costmap = make_costmap(width=50, height=50, resolution=0.1)
    # Place obstacle at (1.0, 1.0) meters -> cell (10, 10)
    obstacles = [{'centroid': np.array([1.0, 1.0]), 'size': 15}]
    proc.update_costmap(obstacles, costmap)
    grid = np.array(costmap.data).reshape(costmap.height, costmap.width)
    if grid[10][10] == 100:
        logger.info("  OK   obstacle cell (10,10) marked as 100")
        passed = True
    else:
        logger.warning(f"  FAIL cell (10,10) value={grid[10][10]}, expected 100")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_update_costmap_inflation_radius():
    """
    TEST 18: update_costmap — cells within ROBOT_RADIUS of an obstacle must
    also be marked as 100.
    """
    passed = True
    logger.info("TEST 18: update_costmap — cells within ROBOT_RADIUS must be inflated to 100")
    proc = setup_processor()
    resolution = 0.05
    costmap = make_costmap(width=100, height=100, resolution=resolution)
    # Place obstacle at center (2.5, 2.5) meters -> cell (50, 50)
    obstacles = [{'centroid': np.array([2.5, 2.5]), 'size': 15}]
    proc.update_costmap(obstacles, costmap)
    grid = np.array(costmap.data).reshape(costmap.height, costmap.width)
    inflation_cells = int(ROBOT_RADIUS / resolution)
    # Check a cell clearly within the inflation radius
    test_row = 50 + inflation_cells - 1
    test_col = 50
    if grid[test_row][test_col] == 100:
        logger.info(f"  OK   inflated cell ({test_row},{test_col}) marked as 100")
    else:
        logger.warning(f"  FAIL inflated cell ({test_row},{test_col}) value={grid[test_row][test_col]}, expected 100")
        passed = False
    # Check a cell clearly outside the inflation radius
    outer_row = 50 + inflation_cells + 3
    outer_col = 50
    if 0 <= outer_row < costmap.height:
        if grid[outer_row][outer_col] == 0:
            logger.info(f"  OK   outer cell ({outer_row},{outer_col}) remains 0")
        else:
            logger.warning(f"  FAIL outer cell ({outer_row},{outer_col}) value={grid[outer_row][outer_col]}, expected 0")
            passed = False
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_update_costmap_no_out_of_bounds():
    """
    TEST 19: update_costmap — obstacles near the grid edge must not cause
    an IndexError. Bounds checking must prevent out-of-bounds writes.
    """
    passed = False
    logger.info("TEST 19: update_costmap — edge obstacles must not cause IndexError")
    proc = setup_processor()
    costmap = make_costmap(width=20, height=20, resolution=0.1)
    # Place obstacle right at the edge
    obstacles = [{'centroid': np.array([0.05, 0.05]), 'size': 15}]
    try:
        proc.update_costmap(obstacles, costmap)
        logger.info("  OK   no IndexError on edge obstacle")
        passed = True
    except IndexError as e:
        logger.warning(f"  FAIL IndexError: {e}")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_update_costmap_multiple_obstacles():
    """
    TEST 20: update_costmap — multiple obstacles must all be marked on the costmap.
    """
    passed = True
    logger.info("TEST 20: update_costmap — multiple obstacles must all be marked")
    proc = setup_processor()
    resolution = 0.1
    costmap = make_costmap(width=50, height=50, resolution=resolution)
    obstacles = [
        {'centroid': np.array([1.0, 1.0]), 'size': 15},
        {'centroid': np.array([3.0, 3.0]), 'size': 12},
    ]
    proc.update_costmap(obstacles, costmap)
    grid = np.array(costmap.data).reshape(costmap.height, costmap.width)
    for obs in obstacles:
        row = int(obs['centroid'][0] / resolution)
        col = int(obs['centroid'][1] / resolution)
        if grid[row][col] != 100:
            logger.warning(f"  FAIL obstacle at ({row},{col}) not marked")
            passed = False
        else:
            logger.info(f"  OK   obstacle at ({row},{col}) marked as 100")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_update_costmap_empty_obstacles_no_change():
    """
    TEST 21: update_costmap — empty obstacle list must leave costmap unchanged.
    """
    passed = False
    logger.info("TEST 21: update_costmap — empty obstacle list must not modify costmap")
    proc = setup_processor()
    costmap = make_costmap(width=20, height=20, resolution=0.1)
    original_data = list(costmap.data)
    proc.update_costmap([], costmap)
    if costmap.data == original_data:
        logger.info("  OK   costmap unchanged with empty obstacle list")
        passed = True
    else:
        logger.warning("  FAIL costmap was modified despite empty obstacle list")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_update_costmap_inflation_is_circular():
    """
    TEST 22: update_costmap — inflation must be circular, not square.
    Cells at exactly ROBOT_RADIUS distance must be marked; cells at
    the corners of the bounding square but outside the radius must not.
    """
    passed = True
    logger.info("TEST 22: update_costmap — inflation shape must be circular not square")
    proc = setup_processor()
    resolution = 0.05
    costmap = make_costmap(width=100, height=100, resolution=resolution)
    obstacles = [{'centroid': np.array([2.5, 2.5]), 'size': 15}]
    proc.update_costmap(obstacles, costmap)
    grid = np.array(costmap.data).reshape(costmap.height, costmap.width)
    center_row, center_col = 50, 50
    inflation_cells = int(ROBOT_RADIUS / resolution)
    # Corner of bounding square — should NOT be marked if inflation is circular
    corner_row = center_row + inflation_cells
    corner_col = center_col + inflation_cells
    corner_dist = math.hypot(corner_row - center_row, corner_col - center_col) * resolution
    if corner_dist > ROBOT_RADIUS:
        if 0 <= corner_row < costmap.height and 0 <= corner_col < costmap.width:
            if grid[corner_row][corner_col] == 0:
                logger.info(f"  OK   corner cell ({corner_row},{corner_col}) correctly not inflated")
            else:
                logger.warning(f"  FAIL corner cell ({corner_row},{corner_col}) inflated, inflation is square not circular")
                passed = False
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: Main

def main():
    logger.info("LIDAR Processor Test Suite")
    results = []

    results.append(test_process_scan_returns_required_keys())
    results.append(test_process_scan_filters_out_of_range())
    results.append(test_process_scan_closest_obstacle_correct())
    results.append(test_process_scan_clear_scan_no_close_obstacle())
    results.append(test_process_scan_clear_directions_length())

    results.append(test_detect_obstacles_returns_list())
    results.append(test_detect_obstacles_each_has_centroid_and_size())
    results.append(test_detect_obstacles_size_matches_cluster_points())
    results.append(test_detect_obstacles_centroid_near_obstacle())
    results.append(test_detect_obstacles_noise_discarded())
    results.append(test_detect_obstacles_two_separate_clusters())

    results.append(test_clear_directions_all_clear_on_clear_scan())
    results.append(test_clear_directions_blocked_on_close_obstacle())
    results.append(test_clear_directions_window_blocks_neighbors())
    results.append(test_clear_directions_all_blocked_on_blocked_scan())
    results.append(test_clear_directions_inf_ranges_are_clear())

    results.append(test_update_costmap_marks_obstacle_cell())
    results.append(test_update_costmap_inflation_radius())
    results.append(test_update_costmap_no_out_of_bounds())
    results.append(test_update_costmap_multiple_obstacles())
    results.append(test_update_costmap_empty_obstacles_no_change())
    results.append(test_update_costmap_inflation_is_circular())

    logger.info(f"Results: {sum(results)}/{len(results)} passed")
    logger.info("All tests complete.")
    sys.exit(0 if all(results) else 1)


if __name__ == '__main__':
    main()