"""Reproduction probes for navi_bot/path_planner.py (the planning node)."""

import contextlib
import time
from types import SimpleNamespace

from crlib.probe_api import probe, reproduced, not_reproduced

DEPS = [
    "navi_bot/path_planner.py",
    "navi_bot/planners/astar.py",
    "navi_bot/planners/dstar_lite.py",
    "navi_bot/utils/geometry.py",
    "navi_bot/mock_ros2.py",
    "navi_bot/ros_compat.py",
]
MAP_DEPS = DEPS + [
    "navi_bot/map_publisher.py", "maps/warehouse_simple.pgm", "maps/warehouse_simple.yaml"
]

ROBOT_RADIUS_M = 0.25
PICKUP_1 = (3.0, 0.9)       # station coordinates printed by maps/generate_map.py
PICKUP_3 = (7.0, 0.9)
DELIVERY_1 = (3.0, 9.1)
CHARGING = (0.9, 9.0)


def _pose(x, y, theta=0.0):
    from navi_bot.mock_ros2 import Pose2D

    msg = Pose2D()
    msg.x, msg.y, msg.theta = x, y, theta
    return msg


def _map_msg(grid, resolution=1.0, origin=(0.0, 0.0)):
    """A nav_msgs/OccupancyGrid-shaped message (dimensions under .info)."""
    height, width = len(grid), len(grid[0])
    flat = [100 if cell else 0 for row in grid for cell in row]
    info = SimpleNamespace(
        resolution=resolution, width=width, height=height,
        origin=SimpleNamespace(position=SimpleNamespace(x=origin[0], y=origin[1])),
    )
    return SimpleNamespace(info=info, data=flat)


def _fresh_node():
    from navi_bot import mock_ros2
    from navi_bot.path_planner import PathPlannerNode

    mock_ros2.init()
    return PathPlannerNode()


def _count_plans(node):
    """Wrap the global planner so every plan() call is counted."""
    calls = {"count": 0, "seconds": []}
    original = node.global_planner.plan

    def counted(start, goal):
        started = time.perf_counter()
        try:
            return original(start, goal)
        finally:
            calls["count"] += 1
            calls["seconds"].append(time.perf_counter() - started)

    node.global_planner.plan = counted
    return calls


def _warehouse_nodes():
    """A PathPlannerNode that has received the real warehouse map."""
    from navi_bot import mock_ros2
    from navi_bot.map_publisher import MapPublisherNode
    from navi_bot.path_planner import PathPlannerNode

    mock_ros2.init()
    planner = PathPlannerNode()
    publisher = MapPublisherNode()
    return planner, publisher


@probe("PP-001", deps=DEPS)
def goal_before_map_crashes_the_node():
    node = _fresh_node()
    node.current_pose_callback(_pose(0.5, 0.5))
    node.goal_callback(_pose(3.5, 3.5))
    try:
        node.planning_loop()
    except Exception as exc:  # noqa: BLE001 - any exception here is the defect
        return reproduced(
            f"planning_loop() raised {type(exc).__name__}: {exc} - the D* Lite section indexes "
            "an occupancy grid that is still None. Under rclpy an exception in a timer callback "
            "ends spin(), so the node dies",
            exception=type(exc).__name__,
        )
    return not_reproduced("planning_loop() returned without raising when no map had arrived")


@probe("PP-002", deps=DEPS)
def goal_received_before_map_keeps_the_wrong_cell():
    from navi_bot.utils.geometry import world_to_grid

    node = _fresh_node()
    resolution = 0.05
    goal_xy = (5.0, 5.0)
    node.goal_callback(_pose(*goal_xy))
    node.map_callback(_map_msg([[0] * 200 for _ in range(200)], resolution=resolution))
    expected = world_to_grid(goal_xy[0], goal_xy[1], (0.0, 0.0), resolution)
    actual = tuple(node.current_goal)
    if actual == expected:
        return not_reproduced(f"goal cell is {actual}, matching the map resolution")
    return reproduced(
        f"goal {goal_xy} m was converted with the pre-map default resolution (1.0 m/cell) to "
        f"cell {actual}; with the {resolution} m map it is cell {expected}, and the stored cell "
        f"is never recomputed - the robot would be sent to "
        f"({actual[1] * resolution:.2f}, {actual[0] * resolution:.2f}) m",
        actual=actual,
        expected=expected,
    )


@probe("PP-003", deps=DEPS)
def map_change_does_not_trigger_a_replan():
    node = _fresh_node()
    size = 12
    open_grid = [[0] * size for _ in range(size)]
    node.map_callback(_map_msg(open_grid))
    node.current_pose_callback(_pose(1.5, 1.5))
    node.goal_callback(_pose(10.5, 1.5))
    node.planning_loop()
    paths_before = len(node.path_pub.published_msgs)
    if paths_before == 0:
        raise RuntimeError("no initial path was published")

    walled = [row[:] for row in open_grid]
    for row in range(size - 2):
        walled[row][6] = 1          # a wall straight across the published path
    node.map_callback(_map_msg(walled))
    calls = _count_plans(node)
    for _ in range(5):
        node.planning_loop()
    replanned = len(node.path_pub.published_msgs) > paths_before
    if replanned:
        return not_reproduced("a new path was published after the map changed")
    return reproduced(
        f"a wall was added across the published path and planning_loop() ran 5 more times: "
        f"{calls['count']} planner calls, no new path published. Only a new goal sets "
        "replanning_needed, so the robot keeps following a path through the new obstacle",
        planner_calls=calls["count"],
    )


@probe("PP-004", deps=DEPS)
def dstar_lite_work_is_computed_then_discarded():
    from navi_bot.planners.dstar_lite import DStarLitePlanner

    node = _fresh_node()
    dstar = next(
        (value for value in vars(node).values() if isinstance(value, DStarLitePlanner)), None
    )
    if dstar is None:
        return not_reproduced("the node no longer owns a D* Lite planner")
    usage = {"compute_seconds": 0.0, "plan_calls": 0}
    original_compute, original_plan = dstar.compute_shortest_path, dstar.plan

    def timed_compute(*args, **kwargs):
        started = time.perf_counter()
        try:
            return original_compute(*args, **kwargs)
        finally:
            usage["compute_seconds"] += time.perf_counter() - started

    def counted_plan(*args, **kwargs):
        usage["plan_calls"] += 1
        return original_plan(*args, **kwargs)

    dstar.compute_shortest_path, dstar.plan = timed_compute, counted_plan
    size = 40
    node.map_callback(_map_msg([[0] * size for _ in range(size)]))
    node.current_pose_callback(_pose(1.5, 1.5))
    node.goal_callback(_pose(38.5, 38.5))
    started = time.perf_counter()
    node.planning_loop()
    total = time.perf_counter() - started
    publishers = sorted(pub.topic for pub in node._publishers)
    if usage["plan_calls"] or usage["compute_seconds"] == 0.0:
        return not_reproduced(
            f"D* Lite plan() calls: {usage['plan_calls']}, compute time "
            f"{usage['compute_seconds'] * 1000:.0f} ms"
        )
    share = 100 * usage["compute_seconds"] / total
    return reproduced(
        f"on a {size}x{size} grid D* Lite's compute_shortest_path() took "
        f"{usage['compute_seconds'] * 1000:.0f} ms of the {total * 1000:.0f} ms planning cycle "
        f"({share:.0f}%), "
        f"yet its plan()/path extraction is never called and the node's only publisher is "
        f"{publishers}: the A* path is published and the D* Lite result is thrown away",
        compute_ms=round(usage["compute_seconds"] * 1000),
        total_ms=round(total * 1000),
    )


def _gap_to_occupied_cells(point_xy, centres_xy, half_cell):
    """Smallest distance (m) from a point to the edge of any occupied cell."""
    import numpy as np

    dx = np.maximum(np.abs(centres_xy[:, 0] - point_xy[0]) - half_cell, 0.0)
    dy = np.maximum(np.abs(centres_xy[:, 1] - point_xy[1]) - half_cell, 0.0)
    return float(np.hypot(dx, dy).min())


@probe("PP-005", deps=MAP_DEPS)
def path_on_real_map_hugs_obstacles():
    import numpy as np

    routes = {
        "pickup_1 -> delivery_1": (PICKUP_1, DELIVERY_1),
        "pickup_1 -> charging": (PICKUP_1, CHARGING),
        "pickup_3 -> charging": (PICKUP_3, CHARGING),
    }
    worst_route, worst_gap, shape = None, float("inf"), None
    for name, (start, goal) in routes.items():
        planner, publisher = _warehouse_nodes()
        planner.current_pose_callback(_pose(*start))
        planner.goal_callback(_pose(*goal))
        planner.planning_loop()
        if not planner.path_pub.published_msgs:
            raise RuntimeError(f"no path published for {name}")
        resolution = publisher.spec["resolution"]
        shape = publisher.occupancy.shape
        occupied = np.argwhere(publisher.occupancy != 0)        # (row, col) of non-free cells
        centres_xy = (occupied[:, ::-1] + 0.5) * resolution     # world (x, y) cell centres
        gap = min(_gap_to_occupied_cells((p.x, p.y), centres_xy, resolution / 2)
                  for p in planner.path_pub.published_msgs[-1].poses)
        if gap < worst_gap:
            worst_route, worst_gap = name, gap
    if worst_gap >= ROBOT_RADIUS_M:
        return not_reproduced(f"closest approach of any planned path to an obstacle is "
                              f"{worst_gap:.2f} m")
    return reproduced(
        f"A* runs on the raw {shape[1]}x{shape[0]} map with no inflation: the planned "
        f"{worst_route} path comes within {worst_gap * 100:.1f} cm of an occupied cell, while the "
        f"robot's radius is {ROBOT_RADIUS_M * 100:.0f} cm - the published centre-line cannot be "
        f"driven",
        clearance_m=round(worst_gap, 3),
        route=worst_route,
    )


@probe("PP-006", deps=MAP_DEPS)
def planning_exceeds_its_own_time_budget():
    planner, _publisher = _warehouse_nodes()
    planner.current_pose_callback(_pose(*PICKUP_1))
    planner.goal_callback(_pose(*DELIVERY_1))
    started = time.perf_counter()
    planner.planning_loop()
    elapsed_ms = (time.perf_counter() - started) * 1000
    budget_ms = planner.planning_time_budget * 1000
    if elapsed_ms <= budget_ms:
        return not_reproduced(f"planning took {elapsed_ms:.0f} ms within the {budget_ms:.0f} ms "
                              f"budget")
    return reproduced(
        f"one planning_loop() on the real warehouse map took {elapsed_ms:.0f} ms against the "
        f"node's own {budget_ms:.0f} ms budget ({elapsed_ms / budget_ms:.1f}x over). The loop "
        "runs on a 100 ms timer in the node's only thread, so every other callback waits",
        elapsed_ms=round(elapsed_ms),
        budget_ms=round(budget_ms),
    )


@probe("PP-007", deps=DEPS)
def unreachable_goal_is_replanned_forever():
    node = _fresh_node()
    size = 60
    grid = [[0] * size for _ in range(size)]
    grid[50][50] = 1                                   # the goal cell itself is occupied
    node.map_callback(_map_msg(grid))
    node.current_pose_callback(_pose(1.5, 1.5))
    node.goal_callback(_pose(50.5, 50.5))
    calls = _count_plans(node)
    loops = 5
    for _ in range(loops):
        with contextlib.suppress(Exception):   # PP-001 covers crashes; count calls here
            node.planning_loop()
    if calls["count"] < loops:
        return not_reproduced(f"{calls['count']} planner calls over {loops} loops for an "
                              f"unreachable goal")
    mean_ms = 1000 * sum(calls["seconds"]) / len(calls["seconds"])
    return reproduced(
        f"with an unreachable goal, A* ran on every one of {loops} planning loops "
        f"({mean_ms:.0f} ms each on a {size}x{size} grid - an exhaustive search every time) "
        "and logs a warning each time; there is no back-off, failure report, or give-up",
        planner_calls=calls["count"],
        mean_ms=round(mean_ms, 1),
    )


@probe("PP-008", deps=DEPS + ["navi_bot/state_machine.py"])
def state_machine_goal_spam_forces_constant_replanning():
    from navi_bot import mock_ros2
    from navi_bot.path_planner import PathPlannerNode
    from navi_bot.state_machine import Mission, StateMachine

    mock_ros2.init()
    planner = PathPlannerNode()
    machine = StateMachine()
    planner.map_callback(_map_msg([[0] * 20 for _ in range(20)]))
    planner.current_pose_callback(_pose(1.5, 1.5))
    machine.add_mission(Mission("m1", (15.5, 15.5), (3.5, 15.5)))
    calls = _count_plans(planner)
    ticks = 10
    for _ in range(ticks):
        machine.update_state_machine()       # 5 Hz state machine tick
        planner.planning_loop()              # 10 Hz planner tick (twice per SM tick)
        planner.planning_loop()
    paths = len(planner.path_pub.published_msgs)
    if calls["count"] <= 2:
        return not_reproduced(f"{calls['count']} plans for one unchanged goal over {ticks} ticks")
    return reproduced(
        f"one unchanged pickup goal produced {calls['count']} full A* plans and {paths} published "
        f"paths in {ticks} state-machine ticks (2 s): every tick re-sends the goal, which resets "
        "replanning_needed and re-initialises D* Lite. Each new path also resets the follower's "
        "progress in robot_controller",
        plans=calls["count"],
        paths=paths,
    )
