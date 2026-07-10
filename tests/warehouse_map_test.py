#!/usr/bin/env python3
"""
Acceptance tests for the shipped warehouse map artifact (maps/warehouse_simple.pgm)
and the planner stack running on it.

Unlike the other suites (hand-coded 8x8/20x20 grids), these tests exercise the
REAL map file through the same conventions the future map_publisher will use:
read the PGM, flip image->world orientation, binarize with the yaml thresholds
(negate: 0, occupied_thresh: 0.65, free_thresh: 0.196; unknown counts as wall).

- TESTs 1-2 validate the artifact: it loads, has the declared geometry, only
  legal pixel values, sane occupancy, and every station on free floor.
- TESTs 3-4 run A* at full resolution (200x200, 0.05 m/px) on a grid inflated
  by the robot radius — the costmap-inflation idea: at 5 cm cells a raw A*
  path may legally hug a shelf, so the map is fattened by the footprint and
  any path found is robot-safe by construction (asserted independently).
- TEST 5 is the end-to-end drive: the map is block-max downsampled to 20x20
  (0.5 m/cell) and A* + DWA drive pickup -> delivery across the warehouse.
  DWA is fed only obstacles within a perception radius of the robot each
  cycle, mirroring range-limited LIDAR (and keeping the candidate scoring
  cheap). This test logs MAP/MAPROW/Start/pose telemetry for the dashboard.

Regenerate the artifact with:  python maps/generate_map.py
"""
import logging
import math
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / 'maps'))

from generate_map import read_pgm, inflate            # the artifact's own toolkit
from navi_bot.planners.astar import AStarPlanner
from navi_bot.planners.dstar_lite import DStarLitePlanner
from navi_bot.planners.dwa import DWAPlanner

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)

PGM = REPO / 'maps' / 'warehouse_simple.pgm'
RESOLUTION = 0.05          # m per pixel, as declared in warehouse_simple.yaml
ROBOT_RADIUS = 0.25        # m
OCCUPIED_THRESH = 0.65
FREE_THRESH = 0.196

# Station goals in world metres — fixed properties of the generated layout
# (only the random boxes vary with the seed, never the stations).
STATIONS = {
    'charging':   (0.9, 9.0),
    'pickup_1':   (3.0, 0.9),
    'pickup_2':   (5.0, 0.9),
    'pickup_3':   (7.0, 0.9),
    'delivery_1': (3.0, 9.1),
    'delivery_2': (5.0, 9.1),
    'delivery_3': (7.0, 9.1),
}


# MARK: Loading helpers

def load_world_grid():
    """PGM -> world-oriented pixel grid (row 0 = y 0, bottom of the map)."""
    return np.flipud(read_pgm(PGM))


def binarize(grid):
    """map_server semantics: occupied OR unknown -> wall (1), free -> 0."""
    p = (255.0 - grid) / 255.0
    wall = (p > OCCUPIED_THRESH) | ((p >= FREE_THRESH) & (p <= OCCUPIED_THRESH))
    return wall.astype(int)


def station_cell(name):
    x, y = STATIONS[name]
    return int(round(y / RESOLUTION)), int(round(x / RESOLUTION))   # (row, col)


def downsample(bin_grid, factor):
    """Block-max pooling: a coarse cell is occupied if ANY pixel in it is."""
    h, w = bin_grid.shape
    return bin_grid.reshape(h // factor, factor, w // factor, factor).max(axis=(1, 3))


def nearest_free(grid, cell):
    """BFS to the closest free cell (stations near walls can be swallowed
    when the wall's block-max block claims them at coarse resolution)."""
    from collections import deque
    h, w = grid.shape
    seen = {cell}
    q = deque([cell])
    while q:
        r, c = q.popleft()
        if 0 <= r < h and 0 <= c < w and grid[r][c] == 0:
            return (r, c)
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            n = (r + dr, c + dc)
            if n not in seen:
                seen.add(n)
                q.append(n)
    return None


# MARK: Artifact tests

def test_artifact_loads():
    """Artifact — warehouse_simple.pgm exists, is 200x200, and only contains
    the three legal pixel values (0 occupied / 205 unknown / 254 free)."""
    passed = True
    logger.info("TEST 1: Artifact — PGM loads with declared geometry and legal values")
    if not PGM.exists():
        logger.warning(f"  FAIL {PGM} missing — generate it: python maps/generate_map.py")
        logger.info("FAIL")
        return False
    grid = load_world_grid()
    if grid.shape != (200, 200):
        logger.warning(f"  FAIL shape {grid.shape}, expected (200, 200) for 10 m @ 0.05 m/px")
        passed = False
    else:
        logger.info("  OK   200x200 px (10 m x 10 m @ 0.05 m/px)")
    legal = {0, 205, 254}
    values = set(np.unique(grid).tolist())
    if not values <= legal:
        logger.warning(f"  FAIL illegal pixel values {sorted(values - legal)}")
        passed = False
    else:
        logger.info(f"  OK   pixel values {sorted(values)} within occupied/unknown/free")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_semantics_and_stations():
    """Semantics — yaml-threshold binarization yields a bounded, sanely
    occupied map whose border is wall and whose stations are on free floor."""
    passed = True
    logger.info("TEST 2: Semantics — thresholds, walls, and station cells")
    bin_grid = binarize(load_world_grid())
    frac = bin_grid.mean()
    if not 0.05 <= frac <= 0.40:
        logger.warning(f"  FAIL occupied fraction {frac:.1%} outside sane range 5-40%")
        passed = False
    else:
        logger.info(f"  OK   occupied fraction {frac:.1%}")
    border = np.concatenate([bin_grid[0], bin_grid[-1], bin_grid[:, 0], bin_grid[:, -1]])
    if not border.all():
        logger.warning("  FAIL perimeter is not fully walled")
        passed = False
    else:
        logger.info("  OK   perimeter fully walled")
    for name in STATIONS:
        r, c = station_cell(name)
        if bin_grid[r][c] != 0:
            logger.warning(f"  FAIL station {name} at (row {r}, col {c}) is not free")
            passed = False
    if passed:
        logger.info(f"  OK   all {len(STATIONS)} stations on free floor")
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: Full-resolution A* (inflated costmap)

def plan_inflated(start_name, goal_name):
    """A* between two stations on the robot-radius-inflated full-res grid.
    Returns (path, inflated_grid, raw_grid)."""
    raw = binarize(load_world_grid())
    radius_px = int(np.ceil(ROBOT_RADIUS / RESOLUTION))
    inflated = inflate(raw.astype(bool), radius_px).astype(int)
    astar = AStarPlanner()
    astar.set_occupancy_grid(inflated)
    path = astar.plan(station_cell(start_name), station_cell(goal_name))
    return path, inflated, raw


def check_full_res_route(start_name, goal_name):
    passed = True
    path, inflated, raw = plan_inflated(start_name, goal_name)
    if not path:
        logger.warning(f"  FAIL no A* path {start_name} -> {goal_name}")
        logger.info("FAIL")
        return False
    logger.info(f"  OK   A* path found: {len(path)} waypoints "
                f"({len(path) * RESOLUTION:.1f} m of cells)")
    # Independent safety assertion: every path cell keeps >= ROBOT_RADIUS of
    # clearance from the RAW map (i.e., is outside the inflated obstacle set).
    hugging = [cell for cell in path if inflated[cell[0]][cell[1]] != 0]
    if hugging:
        logger.warning(f"  FAIL {len(hugging)} path cells inside the inflated footprint margin")
        passed = False
    else:
        logger.info(f"  OK   whole path keeps >= {ROBOT_RADIUS} m clearance from obstacles")
    if any(raw[r][c] != 0 for r, c in path):
        logger.warning("  FAIL path crosses an occupied/unknown cell")
        passed = False
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_astar_pickup_to_delivery():
    """Full-res A* — pickup_1 -> delivery_1 on the inflated 200x200 map."""
    logger.info("TEST 3: Full-res A* — pickup_1 -> delivery_1 (robot-radius inflated)")
    return check_full_res_route('pickup_1', 'delivery_1')


def test_astar_pickup_to_charging():
    """Full-res A* — pickup_3 -> charging across the warehouse diagonal."""
    logger.info("TEST 4: Full-res A* — pickup_3 -> charging (robot-radius inflated)")
    return check_full_res_route('pickup_3', 'charging')


# MARK: Coarse end-to-end drive (dashboard telemetry)

def drive(grid, path, perception_radius=3.5, max_replans=200,
          wp_tol=0.6, goal_tol=0.7, lookahead=1.0):
    """DWA follows the global path across the coarse warehouse. Obstacles are
    fed range-limited (like LIDAR): only cells within perception_radius of the
    robot reach the local planner each cycle. Logs dashboard telemetry."""
    dwa = DWAPlanner()
    dwa.set_occupancy_grid(grid)
    dwa.set_global_path(path)   # global path for the anti-limit-cycle path term
    all_obstacles = np.argwhere(np.asarray(grid) != 0)

    th = (math.atan2(path[1][1] - path[0][1], path[1][0] - path[0][0])
          if len(path) >= 2 else 0.0)
    x, y = float(path[0][0]), float(path[0][1])
    v, om = 0.0, 0.0
    wp_idx = 0
    collided = left = False
    goal = path[-1]
    logger.info(f"Start: ({path[0][0]}, {path[0][1]}), Goal: ({goal[0]}, {goal[1]})")
    logger.info(f"MAP {len(grid)} {len(grid[0])}")
    for _row in grid:
        logger.info("MAPROW " + "".join('1' if int(_c) != 0 else '0' for _c in _row))
    for i in range(max_replans):
        logger.info(f"pose=({x:.2f},{y:.2f})")
        if math.hypot(goal[0] - x, goal[1] - y) <= goal_tol:
            return True, collided, left, i
        while wp_idx < len(path) - 1 and math.hypot(path[wp_idx][0] - x, path[wp_idx][1] - y) <= wp_tol:
            wp_idx += 1
        target_idx = wp_idx
        while target_idx < len(path) - 1 and math.hypot(path[target_idx][0] - x, path[target_idx][1] - y) < lookahead:
            target_idx += 1
        d = np.hypot(all_obstacles[:, 0] - x, all_obstacles[:, 1] - y)
        nearby = all_obstacles[d <= perception_radius]
        pair = dwa.plan((x, y, th), (v, om), path[target_idx], nearby)
        if pair is None:
            logger.warning("DWA returned None mid-route.")
            return False, collided, left, i
        v, om = pair
        for _ in range(5):   # one 0.5 s control period
            x += v * math.cos(th) * 0.1
            y += v * math.sin(th) * 0.1
            th += om * 0.1
            r, c = int(round(x)), int(round(y))
            if not collided and 0 <= r < len(grid) and 0 <= c < len(grid[0]) and grid[r][c] != 0:
                logger.warning(f"Collision at cell ({r}, {c}), pose=({x:.2f},{y:.2f})")
                collided = True
            if not left and not (-0.5 <= x <= len(grid) - 0.5 and -0.5 <= y <= len(grid[0]) - 0.5):
                left = True
    logger.warning(f"Did not reach goal after {max_replans} replans.")
    return False, collided, left, max_replans


def crossing(planner, start_name, goal_name):
    """Shared body for the coarse end-to-end drives: the global `planner`
    routes station -> station on the 20x20 downsample, DWA follows."""
    coarse = downsample(binarize(load_world_grid()), 10)   # 200/10 -> 20x20

    def coarse_station(name):
        r, c = station_cell(name)
        return nearest_free(coarse, (r // 10, c // 10))

    start = coarse_station(start_name)
    goal = coarse_station(goal_name)
    if start is None or goal is None:
        logger.warning("  FAIL no free coarse cell near a station")
        logger.info("FAIL")
        return False

    planner.set_occupancy_grid(coarse)
    path = planner.plan(start, goal)
    if not path or len(path) < 2:
        logger.warning(f"  FAIL no coarse route {start} -> {goal}")
        logger.info("FAIL")
        return False
    logger.info(f"  OK   coarse route {start} -> {goal}: {len(path)} waypoints")

    reached, collided, left, replans = drive(coarse, path)
    if reached and not collided and not left:
        logger.info(f"  OK   crossed the warehouse in {replans} replans, no collision")
        logger.info("PASS")
        return True
    why = ("collision" if collided else "left map" if left else "did not reach goal")
    logger.warning(f"  FAIL {why} (replans={replans})")
    logger.info("FAIL")
    return False


def test_warehouse_crossing():
    """End-to-end — A* + DWA drive pickup_1 -> delivery_2 across the coarse
    (20x20, 0.5 m/cell) warehouse with range-limited perception."""
    logger.info("TEST 5: End-to-end — A* + DWA cross the warehouse (20x20 downsample)")
    return crossing(AStarPlanner(), 'pickup_1', 'delivery_2')


def test_warehouse_crossing_dstar():
    """End-to-end — D* Lite + DWA drive pickup_3 -> delivery_1 diagonally
    across the coarse warehouse with range-limited perception."""
    logger.info("TEST 6: End-to-end — D* Lite + DWA cross the warehouse (20x20 downsample)")
    return crossing(DStarLitePlanner(), 'pickup_3', 'delivery_1')


# MARK: Moving-hazard drives (yield behavior on the warehouse floor)

CONTACT = 0.75   # robot footprint + cell half-width, as in the integration suites


def drive_with_hazard(grid, path, hazard_fn, perception_radius=3.5,
                      max_replans=200, wp_tol=0.6, goal_tol=0.7, lookahead=1.0,
                      encounter_r=2.0, yield_speed=0.2):
    """Like drive(), but a moving hazard (hazard_fn(step) -> (r, c)) crosses
    the route. The hazard is fed to DWA every cycle as a perception tracker
    would (it is never in the static map, so it is classified as a mover and
    velocity-predicted); static obstacles stay range-limited. Returns metrics
    incl. `yielded`: robot slowed below yield_speed while the hazard was
    within encounter_r — the desired behavior instead of darting in front."""
    dwa = DWAPlanner()
    dwa.set_occupancy_grid(grid)
    dwa.set_global_path(path)   # global path for the anti-limit-cycle path term
    all_obstacles = np.argwhere(np.asarray(grid) != 0)

    th = (math.atan2(path[1][1] - path[0][1], path[1][0] - path[0][0])
          if len(path) >= 2 else 0.0)
    x, y = float(path[0][0]), float(path[0][1])
    v, om = 0.0, 0.0
    px, py = x, y
    wp_idx = 0
    collided = left = reached = False
    min_dist = float('inf')
    min_speed_near = float('inf')
    goal = path[-1]
    logger.info(f"Start: ({path[0][0]}, {path[0][1]}), Goal: ({goal[0]}, {goal[1]})")
    logger.info(f"MAP {len(grid)} {len(grid[0])}")
    for _row in grid:
        logger.info("MAPROW " + "".join('1' if int(_c) != 0 else '0' for _c in _row))
    replans = 0
    for i in range(max_replans):
        replans = i
        hz = hazard_fn(i)
        logger.info(f"pose=({x:.2f},{y:.2f})")
        logger.debug(f"hazard at ({hz[0]},{hz[1]})")
        if i >= 1:                                     # forecast for the dashboard cloud
            n = min(4, i)
            hp = hazard_fn(i - n)
            vr, vc = (hz[0] - hp[0]) / n, (hz[1] - hp[1]) / n
            logger.debug("HAZPRED " + " ".join(f"{hz[0] + vr*k:.2f},{hz[1] + vc*k:.2f}" for k in range(1, 6)))
        hd = math.hypot(hz[0] - x, hz[1] - y)
        min_dist = min(min_dist, hd)
        if i > 0 and hd < encounter_r:
            min_speed_near = min(min_speed_near, math.hypot(x - px, y - py))
        px, py = x, y
        if math.hypot(goal[0] - x, goal[1] - y) <= goal_tol:
            reached = True
            break
        while wp_idx < len(path) - 1 and math.hypot(path[wp_idx][0] - x, path[wp_idx][1] - y) <= wp_tol:
            wp_idx += 1
        target_idx = wp_idx
        while target_idx < len(path) - 1 and math.hypot(path[target_idx][0] - x, path[target_idx][1] - y) < lookahead:
            target_idx += 1
        d = np.hypot(all_obstacles[:, 0] - x, all_obstacles[:, 1] - y)
        nearby = all_obstacles[d <= perception_radius]
        obstacles = np.vstack([nearby, [hz]]) if nearby.size else np.array([list(hz)])
        pair = dwa.plan((x, y, th), (v, om), path[target_idx], obstacles)
        if pair is None:
            v, om = 0.0, 0.0   # hold position this cycle
            continue
        v, om = pair
        for _ in range(5):   # one 0.5 s control period
            x += v * math.cos(th) * 0.1
            y += v * math.sin(th) * 0.1
            th += om * 0.1
            r, c = int(round(x)), int(round(y))
            if not collided and 0 <= r < len(grid) and 0 <= c < len(grid[0]) and grid[r][c] != 0:
                logger.warning(f"Collision at cell ({r}, {c}), pose=({x:.2f},{y:.2f})")
                collided = True
            if not collided and math.hypot(hz[0] - x, hz[1] - y) < CONTACT:
                logger.warning(f"Collision with hazard at ({hz[0]},{hz[1]}), pose=({x:.2f},{y:.2f})")
                collided = True
            if not left and not (-0.5 <= x <= len(grid) - 0.5 and -0.5 <= y <= len(grid[0]) - 0.5):
                left = True
    return {'reached': reached, 'collided': collided, 'left': left,
            'min_dist': min_dist, 'min_speed_near': min_speed_near,
            'yielded': min_speed_near < yield_speed, 'replans': replans}


def hazard_crossing(planner, hazard_fn):
    """Shared body for the moving-hazard drives: robot runs the central aisle
    pickup_2 -> delivery_2 while the hazard walks across open row 8."""
    coarse = downsample(binarize(load_world_grid()), 10)

    def coarse_station(name):
        r, c = station_cell(name)
        return nearest_free(coarse, (r // 10, c // 10))

    start, goal = coarse_station('pickup_2'), coarse_station('delivery_2')
    planner.set_occupancy_grid(coarse)
    path = planner.plan(start, goal)
    if not path or len(path) < 2:
        logger.warning(f"  FAIL no coarse route {start} -> {goal}")
        logger.info("FAIL")
        return False
    logger.info(f"  OK   coarse route {start} -> {goal}: {len(path)} waypoints")

    res = drive_with_hazard(coarse, path, hazard_fn)
    if (res['reached'] and not res['collided'] and not res['left']
            and res['min_dist'] < 2.0 and res['yielded']):
        logger.info(f"  OK   yielded to the hazard (min speed {res['min_speed_near']:.2f}, "
                    f"closest {res['min_dist']:.2f}), reached goal in {res['replans']} replans")
        logger.info("PASS")
        return True
    why = ("collision" if res['collided'] else "left map" if res['left']
           else "never encountered the hazard" if res['min_dist'] >= 2.0
           else "did not yield" if not res['yielded'] else "did not reach goal")
    logger.warning(f"  FAIL {why} (min_dist={res['min_dist']:.2f}, "
                   f"min_speed_near={res['min_speed_near']:.2f}, replans={res['replans']})")
    logger.info("FAIL")
    return False


def test_hazard_crossing_astar():
    """Moving hazard — A* + DWA run the aisle while a worker crosses row 8
    left-to-right; the robot must yield, not collide, and still arrive."""
    logger.info("TEST 7: Moving hazard — A* + DWA yield to a worker crossing the aisle")
    return hazard_crossing(AStarPlanner(), lambda i: (8, min(4 + i // 3, 16)))


def test_hazard_crossing_dstar():
    """Moving hazard — D* Lite + DWA run the aisle while a worker crosses
    row 8 right-to-left; the robot must yield, not collide, and still arrive."""
    logger.info("TEST 8: Moving hazard — D* Lite + DWA yield to a worker crossing the aisle")
    return hazard_crossing(DStarLitePlanner(), lambda i: (8, max(16 - i // 3, 4)))


# MARK: Runner

def main():
    tests = [
        test_artifact_loads,
        test_semantics_and_stations,
        test_astar_pickup_to_delivery,
        test_astar_pickup_to_charging,
        test_warehouse_crossing,
        test_warehouse_crossing_dstar,
        test_hazard_crossing_astar,
        test_hazard_crossing_dstar,
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

    logger.info("Warehouse Map Test Suite")
    results = [t() for t in selected]

    logger.info(f"Results: {sum(results)}/{len(results)} passed")
    logger.info("All tests complete.")
    sys.exit(0 if all(results) else 1)


if __name__ == '__main__':
    main()
