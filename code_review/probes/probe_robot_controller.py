"""Reproduction probes for navi_bot/robot_controller.py and node wiring."""

import ast
import importlib
import math
import statistics
import time
from pathlib import Path

from crlib.probe_api import probe, reproduced, not_reproduced

REPO_ROOT = Path(__file__).resolve().parents[2]
DEPS = [
    "navi_bot/robot_controller.py",
    "navi_bot/planners/dwa.py",
    "navi_bot/control/path_follower.py",
    "navi_bot/control/motion_controller.py",
    "navi_bot/utils/geometry.py",
    "navi_bot/mock_ros2.py",
    "navi_bot/ros_compat.py",
]
NODE_DEPS = DEPS + [
    "setup.py", "navi_bot/path_planner.py", "navi_bot/state_machine.py",
    "navi_bot/map_publisher.py",
]
MAP_DEPS = DEPS + [
    "navi_bot/map_publisher.py", "maps/warehouse_simple.pgm", "maps/warehouse_simple.yaml"
]

STRAIGHT_PATH = [(float(x), 0.0) for x in range(0, 13, 2)]


def _fresh_controller():
    from navi_bot import mock_ros2
    from navi_bot.robot_controller import RobotController

    mock_ros2.init()
    return RobotController()


def _scan(readings, increment_deg=1.0):
    """LaserScan from {angle_deg: range_m}; every other beam reads 5 m (clear)."""
    from navi_bot.mock_ros2 import LaserScan

    count = int(round(360 / increment_deg))
    scan = LaserScan()
    scan.angle_min = -math.pi
    scan.angle_increment = math.radians(increment_deg)
    scan.ranges = [5.0] * count
    for angle_deg, distance in readings.items():
        scan.ranges[int(round((angle_deg + 180) / increment_deg)) % count] = distance
    return scan


def _last_cmd(controller):
    msg = controller.cmd_vel_pub.last_msg
    return msg.linear.x, msg.angular.z


def _console_script_modules():
    """Module names behind setup.py's console_scripts entry points."""
    tree = ast.parse((REPO_ROOT / "setup.py").read_text(encoding="utf-8"))
    modules = []
    for node in ast.walk(tree):
        if (isinstance(node, ast.Constant) and isinstance(node.value, str)
                and ":main" in node.value):
            modules.append(node.value.split("=")[1].split(":")[0].strip())
    return modules


def _instantiate_all_nodes():
    """One instance of every Node subclass reachable from the console scripts."""
    from navi_bot import mock_ros2

    mock_ros2.init()
    nodes = []
    for module_name in _console_script_modules():
        module = importlib.import_module(module_name)
        for value in vars(module).values():
            is_node = isinstance(value, type) and issubclass(value, mock_ros2.Node)
            if is_node and value is not mock_ros2.Node and value.__module__ == module_name:
                nodes.append(value())
    return nodes


@probe("SYS-001", deps=NODE_DEPS)
def subscribed_topics_that_nobody_publishes():
    nodes = _instantiate_all_nodes()
    published = {pub.topic for node in nodes for pub in node._publishers}
    orphans = {}
    for node in nodes:
        for sub in node._subscriptions:
            if sub.topic not in published:
                orphans.setdefault(sub.topic, []).append(node.node_name)
    if not orphans:
        return not_reproduced(f"every subscribed topic has a publisher among {len(nodes)} nodes")
    listing = "; ".join(
        f"{topic} (wanted by {', '.join(names)})" for topic, names in sorted(orphans.items())
    )
    return reproduced(
        f"across the {len(nodes)} launchable nodes, no node publishes: {listing}. Without a pose "
        "source the planner logs 'Current pose unknown' forever and the robot cannot move",
        orphans=orphans,
    )


@probe("SYS-002", deps=NODE_DEPS)
def pose_topic_names_disagree():
    nodes = _instantiate_all_nodes()
    pose_topics = {}
    for node in nodes:
        for sub in node._subscriptions:
            if sub.msg_type.__name__ == "Pose2D" and "goal" not in sub.topic:
                pose_topics.setdefault(sub.topic, []).append(node.node_name)
    if len(pose_topics) <= 1:
        return not_reproduced(f"robot pose is consumed on a single topic: {sorted(pose_topics)}")
    listing = ", ".join(
        f"{names[0]} listens on {topic}" for topic, names in sorted(pose_topics.items())
    )
    return reproduced(
        f"the robot's pose is expected on {len(pose_topics)} different topics: {listing}. A "
        f"single "
        "odometry publisher can satisfy only one of them",
        topics=pose_topics,
    )


@probe("RC-001", deps=DEPS)
def control_loop_misses_its_deadline_in_path_mode():
    controller = _fresh_controller()
    controller.path_callback(STRAIGHT_PATH)
    controller.lidar_callback(_scan({angle: 1.2 for angle in range(-60, 61, 6)}))
    cycles = 6
    durations = []
    for _ in range(cycles):
        started = time.perf_counter()
        controller.control_loop()
        durations.append(time.perf_counter() - started)
    median_ms = statistics.median(durations) * 1000
    period_ms = controller.control_period * 1000
    if median_ms <= period_ms:
        return not_reproduced(f"median control_loop() time {median_ms:.1f} ms within the "
                              f"{period_ms:.0f} ms period")
    return reproduced(
        f"with a path and {len(controller.valid_obstacles)} LIDAR returns in view, control_loop() "
        f"takes a median {median_ms:.0f} ms against its {period_ms:.0f} ms period "
        f"({median_ms / period_ms:.1f}x over; {controller.deadline_misses}/{cycles} cycles "
        f"counted as "
        "misses). The 50 Hz loop cannot run at 50 Hz while DWA is evaluated inside it",
        median_ms=round(median_ms, 1),
        misses=controller.deadline_misses,
    )


@probe("RC-002", deps=DEPS)
def obstacle_behind_the_robot_freezes_it():
    controller = _fresh_controller()
    controller.path_callback(STRAIGHT_PATH)             # goal is straight ahead (+x)
    controller.lidar_callback(_scan({180: 0.4}))        # one return 0.4 m directly behind
    commands = []
    for _ in range(3):
        controller.control_loop()
        commands.append(_last_cmd(controller))
    if any(abs(v) > 0 or abs(omega) > 0 for v, omega in commands):
        return not_reproduced(f"robot still commands motion with an obstacle behind it: "
                              f"{commands[-1]}")
    return reproduced(
        "a single LIDAR return 0.4 m directly BEHIND the robot, with a clear path ahead, yields "
        f"cmd_vel {commands[-1]} on every cycle. The governor uses the closest return in any "
        "direction and zeroes rotation too, so the robot can neither drive away nor turn: once "
        "anything is within 0.5 m it stays stopped until that object leaves on its own",
        commands=commands,
    )


@probe("RC-003", deps=DEPS)
def dwa_called_at_50hz_ignores_the_acceleration_limit():
    controller = _fresh_controller()
    controller.path_callback(STRAIGHT_PATH)
    speeds = [0.0]
    for _ in range(5):
        controller.control_loop()
        speeds.append(_last_cmd(controller)[0])
    period = controller.control_period
    worst = max(abs(b - a) for a, b in zip(speeds, speeds[1:])) / period
    limit = controller.dwa.max_accel_x
    if worst <= 1.5 * limit:
        return not_reproduced(f"largest commanded acceleration {worst:.2f} m/s^2 vs limit "
                              f"{limit} m/s^2")
    return reproduced(
        f"commanded speed over consecutive {period * 1000:.0f} ms cycles: "
        f"{', '.join(f'{v:.2f}' for v in speeds)} m/s - an acceleration of {worst:.1f} m/s^2 "
        f"against DWA's max_accel_x of {limit} m/s^2 ({worst / limit:.0f}x). plan() sizes its "
        "dynamic window for a hard-coded 0.5 s control period, but the controller calls it every "
        "20 ms",
        speeds=[round(v, 3) for v in speeds],
        accel=round(worst, 2),
    )


@probe("RC-004", deps=DEPS)
def perception_range_is_shorter_than_the_planner_needs():
    from navi_bot.planners import dwa

    controller = _fresh_controller()
    seen = controller.obstacle_threshold
    needed = dwa.YIELD_RANGE
    if seen >= needed:
        return not_reproduced(f"obstacle_threshold {seen} m covers YIELD_RANGE {needed} m")
    controller.lidar_callback(_scan({0: 2.0}))          # something 2 m dead ahead
    return reproduced(
        f"robot_controller forwards only returns closer than obstacle_threshold = {seen} m, but "
        f"DWA starts yielding at YIELD_RANGE = {needed} and rolls trajectories out 2 s "
        f"(2 m at top speed). An obstacle 2.0 m dead ahead reaches DWA as "
        f"{len(controller.world_obstacles())} obstacles, so the graduated slow-down between "
        f"{needed} and {seen} never happens and braking starts late",
        obstacle_threshold=seen,
        yield_range=needed,
    )


@probe("RC-005", deps=MAP_DEPS)
def static_shelf_is_classified_as_a_moving_obstacle():
    from navi_bot import mock_ros2
    from navi_bot.map_publisher import MapPublisherNode
    from navi_bot.robot_controller import RobotController
    from navi_bot.utils.geometry import world_to_grid

    import numpy as np

    mock_ros2.init()
    controller = RobotController()
    publisher = MapPublisherNode()
    resolution = publisher.spec["resolution"]
    origin = publisher.spec["origin"]
    occupied = np.argwhere(publisher.occupancy != 0)
    movers = 0
    for row, col in occupied:
        centre_xy = (origin[0] + (col + 0.5) * resolution, origin[1] + (row + 0.5) * resolution)
        movers += bool(controller.dwa.is_dynamic_obstacle(centre_xy))
    shelf_xy = (9.0, 5.0)                               # inside the right-hand shelving band
    shelf_cell = world_to_grid(shelf_xy[0], shelf_xy[1], origin, resolution)
    outside_xy = (15.0, 15.0)                           # 5 m beyond the 10 m x 10 m map
    thinks_inside = controller.dwa.is_pose_inbounds(*outside_xy)
    if movers == 0 and not thinks_inside:
        return not_reproduced("world-frame points are classified consistently with the map")
    share = 100 * movers / len(occupied)
    return reproduced(
        f"the map is {resolution} m/cell and the path, pose and LIDAR points are in world metres, "
        f"but DWA rounds those metres straight to (row, col) indices. Of the map's "
        f"{len(occupied):,} "
        f"occupied cells, {movers:,} ({share:.0f}%) are classified as MOVING obstacles when their "
        f"own world position is passed in - e.g. the shelf at {shelf_xy} m is map cell "
        f"{shelf_cell} but is looked up as cell (9, 5). A pose at {outside_xy} m, 5 m outside the "
        f"map, is reported {'in bounds' if thinks_inside else 'out of bounds'}. Scale (x20), axis "
        "order (x is the column in the map) and the half-cell offset all differ",
        movers=movers,
        occupied=int(len(occupied)),
        outside_pose_in_bounds=thinks_inside,
    )


@probe("RC-006", deps=DEPS)
def stale_scan_is_trusted_forever():
    real_clocks = (time.time, time.monotonic, time.perf_counter)
    controller = _fresh_controller()
    controller.path_callback(STRAIGHT_PATH)
    controller.lidar_callback(_scan({}))                # one all-clear scan, then silence
    jump = 30.0
    try:
        time.time = lambda: real_clocks[0]() + jump
        time.monotonic = lambda: real_clocks[1]() + jump
        time.perf_counter = lambda: real_clocks[2]() + jump
        controller.control_loop()
    finally:
        time.time, time.monotonic, time.perf_counter = real_clocks
    speed, turn = _last_cmd(controller)
    if speed == 0.0 and turn == 0.0:
        return not_reproduced("the controller stops when its last scan is 30 s old")
    return reproduced(
        f"30 s after the last LIDAR scan (system clocks advanced, no new /scan or /pose) the "
        f"controller still commands v = {speed:.2f} m/s from that one old scan. There is no "
        "timestamp or watchdog on /scan or /pose, and before the first scan arrives the robot "
        "treats the world as empty",
        v=round(speed, 3),
    )
