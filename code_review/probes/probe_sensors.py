"""Reproduction probes for navi_bot/sensors (LIDAR, odometry, IMU)."""

import math
from types import SimpleNamespace

from crlib.probe_api import probe, reproduced, not_reproduced

LIDAR_DEPS = ["navi_bot/sensors/lidar_processing.py", "navi_bot/utils/geometry.py",
              "navi_bot/mock_ros2.py", "navi_bot/ros_compat.py"]
ODOM_DEPS = ["navi_bot/sensors/odometry.py", "navi_bot/utils/geometry.py"]
IMU_DEPS = ["navi_bot/sensors/imu_processor.py"]

BEAMS = 360
FAR = 20.0                      # beyond max range: "nothing seen"


def _scan(ranges):
    from navi_bot.mock_ros2 import LaserScan

    scan = LaserScan()
    scan.angle_min = -math.pi
    scan.angle_increment = math.radians(1.0)
    scan.ranges = list(ranges)
    return scan


def _box_scan(distance, width):
    """360-beam scan of a flat box face of ``width`` centred dead ahead."""
    ranges = [FAR] * BEAMS
    half_angle = math.atan2(width / 2, distance)
    for index in range(BEAMS):
        angle = -math.pi + math.radians(index)
        if abs(angle) <= half_angle:
            ranges[index] = distance / math.cos(angle)
    return ranges


def _clusters(ranges):
    from navi_bot.sensors.lidar_processing import LidarProcessor

    return LidarProcessor().process_scan(_scan(ranges))["obstacles"]


@probe("LD-001", deps=LIDAR_DEPS)
def thin_and_distant_obstacles_are_invisible():
    cases = {
        "a 0.12 m post at 1.0 m": _box_scan(1.0, 0.12),
        "a 0.30 m box at 2.5 m": _box_scan(2.5, 0.30),
        "a 1.00 m pallet at 5.0 m": _box_scan(5.0, 1.00),
    }
    missed = {}
    for name, ranges in cases.items():
        hits = sum(1 for value in ranges if value < FAR)
        if not _clusters(ranges):
            missed[name] = hits
    if not missed:
        return not_reproduced("every test obstacle produced a cluster")
    listing = "; ".join(f"{name} ({hits} beams hit it)" for name, hits in missed.items())
    return reproduced(
        f"process_scan() reports NO obstacle for: {listing}. DBSCAN needs 10 returns within "
        "0.3 m of each other; at 1 degree per beam that excludes anything narrower than ten "
        "beams and everything beyond roughly 3.4 m, where neighbouring returns are too sparse",
        missed=missed,
    )


@probe("LD-002", deps=LIDAR_DEPS)
def a_wall_collapses_to_a_single_point():
    from navi_bot.mock_ros2 import OccupancyGrid
    from navi_bot.sensors.lidar_processing import LidarProcessor

    wall_length, distance = 4.0, 1.5
    ranges = _box_scan(distance, wall_length)
    processor = LidarProcessor()
    obstacles = processor.process_scan(_scan(ranges))["obstacles"]
    if len(obstacles) != 1:
        raise RuntimeError(f"expected one cluster for the wall, got {len(obstacles)}")
    resolution, cells = 0.05, 200
    costmap = OccupancyGrid(width=cells, height=cells, resolution=resolution)
    processor.update_costmap([{"centroid": (5.0, 5.0), "size": obstacles[0]["size"]}], costmap)
    marked = sum(1 for value in costmap.data if value)
    wall_cells = int(wall_length / resolution)
    widest = max(sum(1 for value in costmap.data[row * cells:(row + 1) * cells] if value)
                 for row in range(cells))
    if widest >= wall_cells * 0.8:
        return not_reproduced(f"the costmap footprint spans {widest} cells of the "
                              f"{wall_cells}-cell wall")
    return reproduced(
        f"a {wall_length:.0f} m wall {distance} m ahead ({obstacles[0]['size']} returns) is "
        f"reduced "
        f"to one centroid, and update_costmap() stamps a single disc {widest * resolution:.2f} m "
        f"wide ({marked} cells) for it. The wall is {wall_cells} cells long, so "
        f"{100 * (1 - widest / wall_cells):.0f}% of it never reaches the costmap",
        widest_cells=widest,
        wall_cells=wall_cells,
    )


@probe("LD-003", deps=LIDAR_DEPS)
def costmap_axes_are_swapped_relative_to_the_project_convention():
    from navi_bot.mock_ros2 import OccupancyGrid
    from navi_bot.sensors.lidar_processing import LidarProcessor
    from navi_bot.utils.geometry import world_to_grid

    resolution, cells = 0.05, 100
    costmap = OccupancyGrid(width=cells, height=cells, resolution=resolution)
    point = (3.0, 1.0)                                # x = 3 m, y = 1 m
    LidarProcessor().update_costmap([{"centroid": point, "size": 20}], costmap)
    expected = world_to_grid(point[0], point[1], (0.0, 0.0), resolution)
    marked = [(index // cells, index % cells) for index, value in enumerate(costmap.data) if value]
    centre = (
        round(sum(r for r, _ in marked) / len(marked)), round(
            sum(c for _, c in marked) / len(marked)
        )
    )
    if abs(centre[0] - expected[0]) <= 1 and abs(centre[1] - expected[1]) <= 1:
        return not_reproduced(f"obstacle at {point} lands at cell {centre}, as world_to_grid says")
    return reproduced(
        f"an obstacle at x = {point[0]} m, y = {point[1]} m is written around cell (row "
        f"{centre[0]}, "
        f"col {centre[1]}); utils.geometry.world_to_grid - used by path_planner and the map - "
        f"puts "
        f"that point at (row {expected[0]}, col {expected[1]}). update_costmap() maps x to the "
        f"row "
        "and y to the column, so LIDAR obstacles land mirrored across the diagonal of the map. "
        "It also uses robot-frame points with no pose transform or map origin",
        written=centre,
        expected=expected,
    )


@probe("LD-004", deps=LIDAR_DEPS)
def costmap_update_rejects_a_real_occupancy_grid():
    from navi_bot.sensors.lidar_processing import LidarProcessor

    info = SimpleNamespace(width=50, height=50, resolution=0.05)
    real_shaped = SimpleNamespace(info=info, data=[0] * 2500)   # nav_msgs/OccupancyGrid layout
    try:
        LidarProcessor().update_costmap([{"centroid": (1.0, 1.0), "size": 12}], real_shaped)
    except AttributeError as exc:
        return reproduced(
            f"update_costmap() on a message shaped like nav_msgs/OccupancyGrid raises "
            f"AttributeError ({exc}): it reads .height/.width/.resolution from the message "
            "itself, which only the mock provides; the real message keeps them under .info"
        )
    return not_reproduced("update_costmap() accepts dimensions under .info")


@probe("LD-005", deps=LIDAR_DEPS)
def clear_directions_do_not_wrap_around_the_scan():
    from navi_bot.sensors.lidar_processing import LidarProcessor

    ranges = [5.0] * BEAMS
    for index in range(350, 360):                     # obstacle 0.3 m away at the scan's end
        ranges[index] = 0.3
    clear = LidarProcessor().process_scan(_scan(ranges))["clear_directions"]
    wrongly_clear = [index for index in range(0, 6) if clear[index]]
    if not wrongly_clear:
        return not_reproduced("directions next to the seam see the obstacle on the other side")
    return reproduced(
        f"an obstacle 0.3 m away fills beams 350-359; beams {wrongly_clear} - angularly 1 to "
        "6 degrees from it across the +/-180 degree seam - are reported CLEAR. The +/-15-beam "
        "window is cut off at the array ends instead of wrapping, so a 360-degree scan has a "
        "blind spot directly behind the robot",
        wrongly_clear=wrongly_clear,
    )


@probe("LD-006", deps=LIDAR_DEPS)
def scan_match_does_not_recover_a_known_transform():
    import numpy as np

    from navi_bot.sensors.lidar_processing import LidarProcessor

    wall_a = [(x, 2.0) for x in np.linspace(-2.0, 2.0, 60)]
    wall_b = [(2.0, y) for y in np.linspace(-1.5, 2.0, 50)]
    reference = np.array(wall_a + wall_b)             # an L-shaped corner
    true_dx, true_dy, true_dtheta = 0.20, -0.10, math.radians(5.0)
    cos_t, sin_t = math.cos(true_dtheta), math.sin(true_dtheta)
    rotation = np.array([[cos_t, -sin_t], [sin_t, cos_t]])
    current = (reference - np.array([true_dx, true_dy])) @ rotation    # R^-1 (ref - t)
    (dx, dy, dtheta), _error = LidarProcessor().scan_match(current, reference)
    position_error = math.hypot(dx - true_dx, dy - true_dy)
    angle_error = abs(math.degrees(dtheta - true_dtheta))
    if position_error < 0.03 and angle_error < 1.0:
        return not_reproduced(f"recovered the transform within {position_error * 100:.1f} cm / "
                              f"{angle_error:.1f} deg")
    return reproduced(
        f"current scan = reference moved by (0.20 m, -0.10 m, 5.0 deg). scan_match() returns "
        f"({dx:+.3f} m, {dy:+.3f} m, {math.degrees(dtheta):+.1f} deg): off by "
        f"{position_error * 100:.0f} cm and {angle_error:.1f} deg. tests/scan_matching_test.py "
        "currently passes 3 of 12 cases for the same reason",
        position_error_cm=round(position_error * 100, 1),
        angle_error_deg=round(angle_error, 2),
    )


@probe("OD-001", deps=ODOM_DEPS)
def odometry_integrates_arcs_with_the_wrong_heading():
    from navi_bot.sensors.odometry import WheelOdometry

    radius, steps = 1.0, 8                            # a quarter circle in 8 updates
    odometry = WheelOdometry()
    step_angle = (math.pi / 2) / steps
    arc_left = (radius - odometry.wheel_base / 2) * step_angle
    arc_right = (radius + odometry.wheel_base / 2) * step_angle
    ticks_per_metre = odometry.ticks_per_rev / (2 * math.pi * odometry.wheel_radius)
    left = right = 0.0
    for _ in range(steps):
        left += arc_left * ticks_per_metre
        right += arc_right * ticks_per_metre
        x, y, _theta, _v, _omega = odometry.update(left, right, 0.1)
    error = math.hypot(x - radius, y - radius)        # exact end point is (R, R)
    chord = 2 * radius * math.sin(step_angle / 2)
    mid_x = sum(chord * math.cos((k + 0.5) * step_angle) for k in range(steps))
    mid_y = sum(chord * math.sin((k + 0.5) * step_angle) for k in range(steps))
    midpoint_error = math.hypot(mid_x - radius, mid_y - radius)
    if error < 5 * max(midpoint_error, 1e-6) and error < 0.01:
        return not_reproduced(f"quarter-circle end point error {error * 1000:.1f} mm")
    return reproduced(
        f"a 1 m-radius quarter circle integrated in {steps} updates should end at (1.000, 1.000); "
        f"odometry gives ({x:.3f}, {y:.3f}) - {error * 100:.1f} cm off after only 1.57 m of "
        f"travel. "
        "update() adds the full rotation to theta first and then translates along that END "
        "heading; using the mid-step heading the same data ends within "
        f"{midpoint_error * 1000:.2f} mm. kinematics.compute_pose_update uses the exact arc, so "
        "the project's two motion models disagree",
        error_cm=round(error * 100, 2),
    )


@probe("OD-002", deps=ODOM_DEPS)
def first_odometry_update_jumps_by_the_absolute_tick_count():
    from navi_bot.sensors.odometry import WheelOdometry

    odometry = WheelOdometry()
    x, y, _theta, v, _omega = odometry.update(50_000, 50_000, 0.02)
    if abs(x) < 0.01:
        return not_reproduced("the first update takes the encoder reading as its baseline")
    return reproduced(
        f"the first update with encoders already at 50,000 ticks (controller restarted while the "
        f"MCU kept counting) moves the pose to x = {x:.1f} m and reports v = {v:.0f} m/s: "
        "previous ticks are assumed to be 0 instead of being seeded from the first reading. "
        "Fixed-width counter wrap-around is not handled either",
        x=round(float(x), 2),
        v=round(float(v), 1),
    )


@probe("IM-001", deps=IMU_DEPS)
def imu_processor_cannot_be_constructed():
    from navi_bot.sensors.imu_processor import ImuProcessor

    try:
        ImuProcessor()
    except TypeError as exc:
        return reproduced(
            f"ImuProcessor() raises TypeError: {exc}. Nothing in the class can run until this is "
            "fixed (tests/imu_test.py: 0 of 9 pass on this line)"
        )
    return not_reproduced("ImuProcessor() constructs")
