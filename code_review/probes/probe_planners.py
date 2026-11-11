"""Reproduction probes for the planner algorithms (A*, D* Lite, DWA)."""

import math
import random
import statistics
import time
from collections import deque

from crlib.probe_api import probe, reproduced, not_reproduced

ASTAR_DEPS = ["navi_bot/planners/astar.py"]
DSTAR_DEPS = ["navi_bot/planners/dstar_lite.py"]
DWA_DEPS = ["navi_bot/planners/dwa.py", "navi_bot/utils/geometry.py"]

MOVES = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]


def _open_grid(rows, cols):
    import numpy as np

    return np.zeros((rows, cols), dtype=int)


def _astar(grid):
    from navi_bot.planners.astar import AStarPlanner

    planner = AStarPlanner()
    planner.set_occupancy_grid(grid)
    return planner


def _dstar(grid):
    from navi_bot.planners.dstar_lite import DStarLitePlanner

    planner = DStarLitePlanner()
    planner.set_occupancy_grid(grid)
    return planner


def _fewest_moves(grid, start, goal):
    """Reference optimum for D* Lite's own cost model: every move costs 1,
    8-connected, a diagonal is forbidden when either orthogonal cell is blocked."""
    rows, cols = len(grid), len(grid[0])
    seen = {start: 0}
    queue = deque([start])
    while queue:
        cell = queue.popleft()
        if cell == goal:
            return seen[cell]
        for d_row, d_col in MOVES:
            row, col = cell[0] + d_row, cell[1] + d_col
            if not (0 <= row < rows and 0 <= col < cols) or grid[row][col] != 0:
                continue
            if d_row and d_col and (
                grid[cell[0] + d_row][cell[1]] != 0 or grid[cell[0]][cell[1] + d_col] != 0
            ):
                continue
            if (row, col) not in seen:
                seen[(row, col)] = seen[cell] + 1
                queue.append((row, col))
    return None


def _random_case(rng, size, density):
    import numpy as np

    grid = np.array(
        [[1 if rng.random() < density else 0 for _ in range(size)] for _ in range(size)]
    )
    free = [(r, c) for r in range(size) for c in range(size) if grid[r][c] == 0]
    start, goal = rng.sample(free, 2)
    return grid, start, goal


# ----------------------------------------------------------------------- A*

@probe("AS-001", deps=ASTAR_DEPS)
def astar_answers_before_checking_for_a_map():
    from navi_bot.planners.astar import AStarPlanner

    result = AStarPlanner().plan((3, 3), (3, 3))
    if result is None:
        return not_reproduced("plan() returns None when no occupancy grid has been set")
    return reproduced(
        f"with no occupancy grid set, plan((3, 3), (3, 3)) returns {result} - a 'valid path' - "
        "because the grid-is-None check comes after the already-at-goal shortcut"
    )


@probe("AS-002", deps=ASTAR_DEPS)
def astar_plans_out_of_an_occupied_start_cell():
    grid = _open_grid(6, 6)
    grid[2][2] = 1
    path = _astar(grid).plan((2, 2), (5, 5))
    if path is None:
        return not_reproduced("plan() rejects a start cell that is occupied")
    return reproduced(
        f"start (2, 2) is an occupied cell, yet plan() returns a {len(path)}-waypoint path "
        f"starting inside the obstacle: {path}. The goal cell is checked for occupancy "
        "(implicitly) but the start never is"
    )


@probe("AS-003", deps=ASTAR_DEPS)
def astar_fails_silently_on_list_coordinates():
    grid = _open_grid(6, 6)
    planner = _astar(grid)
    reference = planner.plan((0, 0), (4, 4))
    try:
        as_list = planner.plan((0, 0), [4, 4])
        outcome = f"returns {as_list}"
    except Exception as exc:  # noqa: BLE001 - how it fails is the finding
        as_list, outcome = None, f"raises {type(exc).__name__}: {exc}"
    if as_list == reference:
        return not_reproduced("list and tuple coordinates give the same path")
    return reproduced(
        f"plan((0, 0), (4, 4)) finds a {len(reference)}-waypoint path, but the same goal passed "
        f"as a list, plan((0, 0), [4, 4]), {outcome} - `visited == goal` never matches a tuple "
        "against a list, so the whole grid is searched and 'no path' is reported"
    )


@probe("AS-004", deps=ASTAR_DEPS)
def astar_unreachable_goal_costs_a_full_search():
    size = 200
    grid = _open_grid(size, size)
    grid[150][150] = 1
    planner = _astar(grid)
    started = time.perf_counter()
    result = planner.plan((10, 10), (150, 150))
    elapsed_ms = (time.perf_counter() - started) * 1000
    if result is not None or elapsed_ms < 50:
        return not_reproduced(f"occupied goal handled in {elapsed_ms:.0f} ms (result {result})")
    return reproduced(
        f"asking for an occupied goal cell on a {size}x{size} grid takes {elapsed_ms:.0f} ms: A* "
        "expands every reachable cell before returning None, although one look at the goal "
        "cell would answer immediately",
        elapsed_ms=round(elapsed_ms),
    )


# ------------------------------------------------------------------ D* Lite

@probe("DS-001", deps=DSTAR_DEPS)
def dstar_returns_longer_paths_than_optimal():
    rng = random.Random(7)
    trials, worse, missed, example = 300, 0, 0, None
    for _ in range(trials):
        grid, start, goal = _random_case(rng, size=9, density=0.25)
        best = _fewest_moves(grid, start, goal)
        if best is None:
            continue
        path = _dstar(grid).plan(start, goal)
        if path is None:
            missed += 1
            example = example or (grid, start, goal, None, best)
        elif len(path) - 1 > best:
            worse += 1
            if example is None or example[3] is None:
                example = (grid, start, goal, len(path) - 1, best)
    if not worse and not missed:
        return not_reproduced(f"D* Lite matched the optimum on all {trials} random 9x9 grids")
    grid, start, goal, got, best = example
    rows = ["".join("#" if cell else "." for cell in row) for row in grid]
    return reproduced(
        f"on {trials} seeded random 9x9 grids, D* Lite returned a longer-than-optimal path "
        f"{worse} times and no path {missed} times when one exists. Example: start {start}, "
        f"goal {goal}: D* Lite {got} moves, optimum {best} moves under its own cost model "
        "(every move costs 1). The Manhattan heuristic over-estimates diagonal moves "
        "(h = 2 for a move that costs 1), which breaks the consistency D* Lite relies on",
        worse=worse,
        missed=missed,
        grid=rows,
    )


@probe("DS-002", deps=DSTAR_DEPS)
def dstar_misses_changes_made_to_the_same_grid_object():
    grid = _open_grid(8, 8)
    planner = _dstar(grid)
    start, goal = (0, 0), (7, 7)
    first = planner.plan(start, goal)
    blocked = first[len(first) // 2]
    grid[blocked[0]][blocked[1]] = 1               # caller updates its grid in place
    planner.set_occupancy_grid(grid)
    updated = planner.plan(start, goal)
    fresh = _dstar(grid.copy()).plan(start, goal)
    same = (updated is None) == (fresh is None) and (updated is None or len(updated) == len(fresh))
    if same:
        return not_reproduced("an in-place grid edit is picked up on the next plan()")
    describe = "None" if updated is None else f"{len(updated) - 1} moves"
    return reproduced(
        f"cell {blocked} on the planned path was blocked by editing the grid array in place and "
        f"passing it to set_occupancy_grid() again. The incremental plan() now returns "
        f"{describe}; "
        f"a fresh planner on the same grid returns {len(fresh) - 1} moves. previous_grid and "
        "occupancy_grid are the same object, so edge_changed() sees no difference and the "
        "stored g/rhs values are never repaired",
        updated=None if updated is None else len(updated) - 1,
        fresh=len(fresh) - 1,
    )


@probe("DS-003", deps=DSTAR_DEPS)
def dstar_loses_changes_when_the_map_updates_twice_between_plans():
    start, goal = (0, 0), (7, 7)
    base = _open_grid(8, 8)
    planner = _dstar(base)
    first = planner.plan(start, goal)
    blocked = first[len(first) // 2]
    second = base.copy()
    second[blocked[0]][blocked[1]] = 1             # map update 1: obstacle on the path
    third = second.copy()
    third[7][0] = 1                                # map update 2: unrelated cell
    planner.set_occupancy_grid(second)
    planner.set_occupancy_grid(third)              # arrives before the next plan()
    updated = planner.plan(start, goal)
    fresh = _dstar(third.copy()).plan(start, goal)
    same = (updated is None) == (fresh is None) and (updated is None or len(updated) == len(fresh))
    if same:
        return not_reproduced("two map updates between plans are both accounted for")
    describe = "None" if updated is None else f"{len(updated) - 1} moves"
    return reproduced(
        f"two map updates arrived between plan() calls (first blocks {blocked} on the path, "
        f"second changes an unrelated cell). plan() returned {describe}; a fresh planner returns "
        f"{len(fresh) - 1} moves. Only the last two grids are diffed, so the obstacle from the "
        "first update is never propagated. /map callbacks and the planning timer are not "
        "synchronised, so this ordering is normal in the node",
        updated=None if updated is None else len(updated) - 1,
        fresh=len(fresh) - 1,
    )


@probe("DS-004", deps=DSTAR_DEPS)
def dstar_crashes_when_the_map_grows():
    planner = _dstar(_open_grid(5, 5))
    planner.plan((0, 0), (4, 4))
    planner.set_occupancy_grid(_open_grid(8, 8))
    try:
        planner.plan((0, 0), (4, 4))
    except IndexError as exc:
        return reproduced(
            f"after a 5x5 map is replaced by an 8x8 one, plan() raises IndexError ({exc}): "
            "edge_changed() walks the new grid's dimensions while indexing the old grid"
        )
    return not_reproduced("a map of a different size is handled")


def _queue_removal_ms(queue_type, size, removals=200):
    """Milliseconds to remove ``removals`` entries from a queue holding ``size``."""
    queue = queue_type()
    for index in range(size):
        queue.Insert((index, index), (float(index), 0.0))
    started = time.perf_counter()
    for index in range(removals):
        queue.Remove((index * (size // removals), index * (size // removals)))
    return (time.perf_counter() - started) * 1000


@probe("DS-005", deps=DSTAR_DEPS)
def dstar_queue_removal_is_linear_in_queue_size():
    from navi_bot.planners.dstar_lite import DStarLitePlanner

    queue_type = type(DStarLitePlanner().U)
    small, large = 2_000, 20_000
    small_ms = _queue_removal_ms(queue_type, small)
    large_ms = _queue_removal_ms(queue_type, large)
    growth = large_ms / max(small_ms, 1e-9)
    if growth < 4:
        return not_reproduced(f"200 removals: {small_ms:.1f} ms at {small} entries, "
                              f"{large_ms:.1f} ms at {large}")
    return reproduced(
        f"200 Remove() calls take {small_ms:.0f} ms on a queue of {small:,} entries and "
        f"{large_ms:.0f} ms on {large:,} ({growth:.0f}x for 10x the size): each call rebuilds the "
        "whole list and re-heapifies it. update_vertex() calls Remove() for every neighbour of "
        "every expanded cell, so replans on the 200x200 map are dominated by queue rebuilding",
        small_ms=round(small_ms, 1),
        large_ms=round(large_ms, 1),
    )


def _replan_after_blocking_path(grid, start, goal):
    """Plan, block one mid-path cell in a FRESH array, replan one step along.

    Returns (incremental result, optimum on the new grid) or None when the
    scenario cannot be built (no initial path long enough to block).
    """
    planner = _dstar(grid)
    first = planner.plan(start, goal)
    if not first or len(first) < 4:
        return None
    blocked = first[len(first) // 2]
    changed = grid.copy()
    changed[blocked[0]][blocked[1]] = 1
    planner.set_occupancy_grid(changed)
    return planner.plan(first[1], goal), _fewest_moves(changed, first[1], goal)


def _count_failed_replans(seed=3, cases=200):
    """(failures, trials): seeded single-obstacle replans that return None for a reachable goal."""
    rng = random.Random(seed)
    trials = failures = 0
    for _ in range(cases):
        grid, start, goal = _random_case(rng, size=10, density=0.2)
        outcome = _replan_after_blocking_path(grid, start, goal)
        if outcome is None or outcome[1] is None:
            continue
        trials += 1
        failures += outcome[0] is None
    return failures, trials


@probe("DS-006", deps=DSTAR_DEPS)
def dstar_replan_reports_no_path_when_a_detour_exists():
    from navi_bot.planners import dstar_lite

    simple, optimum = _replan_after_blocking_path(_open_grid(6, 6), (0, 0), (5, 5))
    failures, trials = _count_failed_replans()
    if simple is not None and failures == 0:
        return not_reproduced(f"replans succeeded: open 6x6 case and {trials} random cases")

    shipped_heuristic = dstar_lite.heuristic_backward
    dstar_lite.heuristic_backward = lambda a, b: max(abs(a[0] - b[0]), abs(a[1] - b[1]))
    try:
        control_failures, control_trials = _count_failed_replans()
    finally:
        dstar_lite.heuristic_backward = shipped_heuristic
    return reproduced(
        "open 6x6 grid, path (0,0) -> (5,5) runs down the diagonal; block cell (3,3) and replan "
        f"from (1,1): plan() returns {simple if simple is None else f'{len(simple) - 1} moves'} "
        f"although a {optimum}-move detour exists. On seeded random 10x10 grids, {failures} of "
        f"{trials} single-obstacle replans return None for a reachable goal. Control run: with "
        "the heuristic swapped (inside this probe only) for max(|dr|, |dc|), which matches "
        f"unit-cost diagonals, {control_failures} of {control_trials} fail",
        failures=failures,
        trials=trials,
        control_failures=control_failures,
    )


# ---------------------------------------------------------------------- DWA

def _dwa_plan_ms(obstacle_count, repeats=3):
    from navi_bot.planners.dwa import DWAPlanner

    planner = DWAPlanner()
    planner.set_occupancy_grid(_open_grid(40, 40))
    planner.set_global_path([(float(x), 20.0) for x in range(5, 36, 2)])
    ring = [(20.0 + 6 * math.cos(2 * math.pi * k / max(obstacle_count, 1)),
             20.0 + 6 * math.sin(2 * math.pi * k / max(obstacle_count, 1)))
            for k in range(obstacle_count)]
    durations = []
    for _ in range(repeats):
        started = time.perf_counter()
        planner.plan((10.0, 20.0, 0.0), (0.5, 0.0), (20.0, 20.0), ring)
        durations.append((time.perf_counter() - started) * 1000)
    return statistics.median(durations)


@probe("DW-001", deps=DWA_DEPS)
def dwa_plan_is_far_slower_than_the_control_period():
    control_period_ms = 20.0
    timings = {
        count: _dwa_plan_ms(count, repeats=3 if count < 50 else 1) for count in (0, 20, 100)
    }
    if timings[20] <= control_period_ms:
        return not_reproduced(f"plan() takes {timings[20]:.1f} ms with 20 obstacles")
    return reproduced(
        "one DWAPlanner.plan() call takes "
        + ", ".join(f"{ms:.0f} ms with {count} obstacles" for count, ms in timings.items())
        + f" (800 candidate trajectories x 21 poses, each checked against every obstacle in "
        f"pure Python). robot_controller calls it inside a {control_period_ms:.0f} ms "
        "control loop",
        timings={str(count): round(ms) for count, ms in timings.items()},
    )


@probe("DW-003", deps=DWA_DEPS)
def dwa_cannot_move_once_inside_its_contact_radius():
    from navi_bot.planners import dwa

    gap = 0.80                                         # centre distance; touching would be 0.75
    touching = dwa.ROBOT_RADIUS + dwa.CELL_RADIUS
    frozen = []
    for label, obstacle in (("behind", (10.0 - gap, 10.0)), ("beside", (10.0, 10.0 + gap)),
                            ("ahead", (10.0 + gap, 10.0))):
        planner = dwa.DWAPlanner()
        planner.set_occupancy_grid(_open_grid(20, 20))
        command = planner.plan((10.0, 10.0, 0.0), (0.0, 0.0), (15.0, 10.0), [obstacle])
        if command is not None and float(command[0]) == 0.0 and float(command[1]) == 0.0:
            frozen.append(label)
    if not frozen:
        return not_reproduced(f"DWA still commands motion with an obstacle {gap} away")
    return reproduced(
        f"with one obstacle {gap:.2f} from the robot's centre (not touching - contact is at "
        f"{touching:.2f}) plan() returns (0, 0) whether the obstacle is {', '.join(frozen)}, even "
        f"with the goal straight ahead. Every rollout starts at the current pose, which is "
        f"already "
        f"inside CONTACT = {dwa.CONTACT:.3f}, so all 800 candidates are rejected - including "
        "reversing or turning away. Once anything gets that close the planner can never leave",
        frozen=frozen,
        contact=dwa.CONTACT,
    )


@probe("DW-002", deps=DWA_DEPS)
def dwa_assumes_a_fixed_half_second_between_calls():
    import inspect

    from navi_bot.planners.dwa import DWAPlanner

    planner = DWAPlanner()
    params = [name for name in inspect.signature(planner.plan).parameters
              if any(key in name.lower() for key in ("dt", "period", "time", "stamp", "elapsed"))]
    attrs = [name for name in vars(planner)
             if any(key in name.lower() for key in ("dt", "period", "rate", "hz"))]
    if params or attrs:
        return not_reproduced(f"the call period is configurable ({params or attrs})")
    planner.set_occupancy_grid(_open_grid(30, 30))
    mover_speed = 1.0                                  # cells per second
    real_dt = 0.1                                      # caller's true period: 10 Hz
    for step in range(5):
        position = (15.0, 10.0 + mover_speed * real_dt * step)
        tracked = planner.track_movers([position], 0.5)   # what plan() passes internally
    estimate = float(math.hypot(*tracked[0][1]))
    return reproduced(
        "plan() takes no time step and hard-codes control_period = 0.5 s for both the dynamic "
        "window and mover tracking. A mover crossing at 1.00 cell/s, observed at a true 10 Hz, "
        f"is estimated at {estimate:.2f} cell/s ({mover_speed / max(estimate, 1e-9):.0f}x too "
        f"slow), "
        "so its predicted positions lag the real hazard; at robot_controller's 50 Hz the same "
        "scan is also seen 5 times in a row, driving the estimate towards zero",
        estimate=round(estimate, 3),
    )
