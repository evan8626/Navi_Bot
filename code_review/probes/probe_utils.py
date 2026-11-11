"""Reproduction probes for navi_bot/utils, the ROS mock and the map publisher."""

import asyncio
import contextlib
import sys
import tempfile
import time
import types
import warnings
from pathlib import Path

from crlib.probe_api import probe, reproduced, not_reproduced

GEOMETRY_DEPS = ["navi_bot/utils/geometry.py"]
PROFILER_DEPS = ["navi_bot/utils/profiler.py"]
WEBSOCKET_DEPS = ["navi_bot/utils/websocket.py"]
MOCK_DEPS = ["navi_bot/mock_ros2.py"]
MAP_DEPS = ["navi_bot/map_publisher.py"]


@probe("GE-001", deps=GEOMETRY_DEPS)
def normalize_angle_time_grows_with_the_angle():
    from navi_bot.utils.geometry import normalize_angle

    angle = 1.0e6
    started = time.perf_counter()
    normalize_angle(angle)
    elapsed_ms = (time.perf_counter() - started) * 1000
    if elapsed_ms < 2.0:
        return not_reproduced(f"normalize_angle({angle:g}) takes {elapsed_ms:.3f} ms")
    return reproduced(
        f"normalize_angle({angle:g}) takes {elapsed_ms:.0f} ms: it subtracts 2*pi one turn at a "
        "time, so the cost is proportional to the angle and an infinite input (one bad sensor "
        "value) never returns. It is called inside the DWA scoring loop and the motion "
        "controller keeps its own copy of the same loop",
        elapsed_ms=round(elapsed_ms, 1),
    )


@probe("GE-002", deps=GEOMETRY_DEPS)
def interpolate_path_leaves_gaps_wider_than_requested():
    import math

    from navi_bot.utils.geometry import interpolate_path

    resolution = 0.1
    points = interpolate_path([(0.0, 0.0), (0.19, 0.0)], resolution=resolution)
    widest = max(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:]))
    if widest <= resolution * 1.001:
        return not_reproduced(f"largest gap {widest:.3f} m at resolution {resolution} m")
    return reproduced(
        f"interpolating a 0.19 m segment at resolution {resolution} m returns {len(points)} "
        f"points "
        f"with a {widest:.2f} m gap - nearly twice the requested spacing. int(dist / resolution) "
        "rounds the segment count down, so spacing can approach 2x the resolution",
        widest=round(widest, 3),
    )


@probe("PR-001", deps=PROFILER_DEPS)
def profiler_miss_rate_exceeds_one_hundred_percent():
    from navi_bot.utils.profiler import PerformanceProfiler

    profiler = PerformanceProfiler(history_size=10)
    for _ in range(40):
        profiler.start_timing("loop")
        profiler.stop_timing("loop", deadline=0.0)    # every run misses a zero deadline
    stats = profiler.get_statistics("loop")
    rate = stats["deadline_miss_rate"]
    if rate <= 1.0:
        return not_reproduced(f"miss rate {rate:.2f} after 40 misses with a 10-sample history")
    return reproduced(
        f"40 runs that all miss their deadline, with history_size = 10, report "
        f"deadline_miss_rate = {rate:.1f} ({rate * 100:.0f}%): the miss counter covers the whole "
        f"run ({stats['deadline_misses']}) but is divided by the capped history length "
        f"({stats['count']})",
        rate=rate,
    )


@probe("PR-002", deps=PROFILER_DEPS)
def profile_decorator_leaks_on_exceptions_and_hides_the_function():
    from navi_bot.utils import profiler

    @profiler.profile("probe_task")
    def explode():
        """Docstring that should survive decoration."""
        raise ValueError("boom")

    with contextlib.suppress(ValueError):
        explode()
    leaked = "probe_task" in profiler.global_profiler.active_timers
    profiler.global_profiler.active_timers.pop("probe_task", None)
    renamed = explode.__name__ != "explode"
    if not leaked and not renamed:
        return not_reproduced("the decorator records failed calls and preserves the function's "
                              "identity")
    problems = []
    if leaked:
        problems.append("a call that raises leaves its timer in active_timers and records nothing")
    if renamed:
        problems.append(f"the decorated function's __name__ becomes {explode.__name__!r} and its "
                        f"docstring is lost")
    return reproduced("@profile: " + "; ".join(problems), leaked=leaked, renamed=renamed)


@probe("WS-002", deps=WEBSOCKET_DEPS)
def websocket_broadcast_fails_on_current_python():
    stub = types.ModuleType("websockets")
    stub.serve = lambda *args, **kwargs: None
    had_real = "websockets" in sys.modules
    if not had_real:
        sys.modules["websockets"] = stub
    try:
        sys.modules.pop("navi_bot.utils.websocket", None)
        from navi_bot.utils.websocket import WebSocketServer

        class FakeClient:
            def __init__(self):
                self.sent = []

            async def send(self, message):
                self.sent.append(message)

        server, client = WebSocketServer(), FakeClient()
        server.clients.add(client)
        warnings.simplefilter("ignore", RuntimeWarning)   # the un-awaited send() coroutine
        try:
            asyncio.run(server.broadcast({"type": "pose", "row": 1, "col": 2}))
        except TypeError as exc:
            return reproduced(
                f"broadcast() raises TypeError on Python {sys.version_info.major}."
                f"{sys.version_info.minor}: {exc} asyncio.wait() stopped accepting bare "
                "coroutines in Python 3.11, so no telemetry frame can ever be sent"
            )
        return not_reproduced(f"broadcast() delivered {len(client.sent)} message(s)")
    finally:
        sys.modules.pop("navi_bot.utils.websocket", None)
        if not had_real:
            sys.modules.pop("websockets", None)


@probe("MK-001", deps=MOCK_DEPS)
def destroyed_mock_node_keeps_receiving_messages():
    from navi_bot import mock_ros2

    mock_ros2.init()
    received = []
    listener = mock_ros2.Node("listener")
    listener.create_subscription(mock_ros2.String, "/probe_topic", received.append, 10)
    listener.destroy_node()
    talker = mock_ros2.Node("talker")
    talker.create_publisher(mock_ros2.String, "/probe_topic", 10).publish(
        mock_ros2.String(data="hi")
    )
    if not received:
        return not_reproduced("a destroyed node's subscriptions are removed from the bus")
    return reproduced(
        "a node that has been destroy_node()-ed still receives messages: its callbacks stay "
        "registered on the module-global bus until the next init(). Nodes left over from one "
        "test keep reacting during the next unless every test calls init() first"
    )


@probe("MK-002", deps=MOCK_DEPS)
def mock_logger_is_missing_rclpy_logger_methods():
    from navi_bot import mock_ros2

    logger = mock_ros2.Node("probe").get_logger()
    missing = [name for name in ("debug", "fatal") if not hasattr(logger, name)]
    if not missing:
        return not_reproduced("the mock logger provides debug() and fatal()")
    return reproduced(
        f"the mock logger has no {' / '.join(missing)}() method, which rclpy's logger does: the "
        "first node that logs at that level works on the robot and raises AttributeError in "
        "every test",
        missing=missing,
    )


@probe("MK-003", deps=MOCK_DEPS)
def mock_occupancy_grid_shape_differs_from_the_real_message():
    from navi_bot import mock_ros2

    grid = mock_ros2.OccupancyGrid()
    if hasattr(grid, "info"):
        return not_reproduced("the mock OccupancyGrid carries its dimensions under .info")
    flat = [name for name in ("width", "height", "resolution") if hasattr(grid, name)]
    return reproduced(
        f"mock OccupancyGrid exposes {', '.join(flat)} directly and has no .info; "
        "nav_msgs/OccupancyGrid has them only under .info (plus .info.origin). Code written "
        "against the mock (lidar_processing.update_costmap) cannot take the real message, and "
        "the nodes need hasattr(msg, 'info') branches to cope"
    )


@probe("MP-001", deps=MAP_DEPS)
def map_yaml_parser_keeps_the_quotes_around_a_quoted_image_name():
    from navi_bot.map_publisher import parse_map_yaml

    with tempfile.TemporaryDirectory() as folder:
        yaml_path = Path(folder) / "map.yaml"
        yaml_path.write_text('image: "warehouse.pgm"\nresolution: 0.05\n', encoding="utf-8")
        image = parse_map_yaml(yaml_path)["image"]
    if image == "warehouse.pgm":
        return not_reproduced("quoted image names are unquoted")
    return reproduced(
        f"`image: \"warehouse.pgm\"` - valid YAML that map_server and most map tools write - is "
        f"parsed as the literal file name {image!r}, quotes included, so the node then fails "
        "with FileNotFoundError. The hand-rolled parser also ignores the `mode` key and the "
        "origin yaw"
    )
