"""Reproduction probes for navi_bot/control (kinematics, PID, followers)."""

import math

from crlib.probe_api import probe, reproduced, not_reproduced

KIN_DEPS = ["navi_bot/control/kinematics.py"]
PID_DEPS = ["navi_bot/control/motion_controller.py"]
FOLLOWER_DEPS = ["navi_bot/control/path_follower.py"]
PURSUIT_DEPS = ["navi_bot/control/pure_pursuit.py", "navi_bot/utils/geometry.py"]


def _drive_unicycle(controller, path, pose, steps=600, dt=0.05):
    """Closed-loop pure-pursuit run on an ideal unicycle; returns the final pose."""
    x, y, theta = pose
    for _ in range(steps):
        v, omega = controller.pure_pursuit(path, (x, y, theta))
        x += v * math.cos(theta) * dt
        y += v * math.sin(theta) * dt
        theta += omega * dt
    return x, y, theta


@probe("KN-001", deps=KIN_DEPS)
def wheel_saturation_changes_the_turn_radius():
    from navi_bot.control.kinematics import DifferentialDriveKinematics

    kin = DifferentialDriveKinematics()
    v_cmd, omega_cmd = 1.0, 2.0
    v_out, omega_out = kin.forward_kinematics(*kin.inverse_kinematics(v_cmd, omega_cmd))
    curvature_cmd, curvature_out = omega_cmd / v_cmd, omega_out / v_out
    if abs(curvature_cmd - curvature_out) < 1e-6:
        return not_reproduced("a saturated command keeps its commanded curvature")
    return reproduced(
        f"command (v = {v_cmd}, omega = {omega_cmd}) saturates the outer wheel; clipping each "
        f"wheel on its own yields (v = {v_out:.2f}, omega = {omega_out:.2f}): the turn radius "
        f"changes from {1 / curvature_cmd:.2f} m to {1 / curvature_out:.2f} m, so the robot "
        "leaves the arc the planner chose instead of following it more slowly",
        radius_cmd=round(1 / curvature_cmd, 3),
        radius_out=round(1 / curvature_out, 3),
    )


@probe("MC-001", deps=PID_DEPS)
def first_pid_update_has_a_derivative_kick():
    from navi_bot.control.motion_controller import MotionController

    controller = MotionController()
    controller.set_goal(0.5, 0.0)
    dt = 0.02
    first, _ = controller.compute_control((0.0, 0.0, 0.0), dt)
    second, _ = controller.compute_control((0.0, 0.0, 0.0), dt)
    if first <= 1.5 * second:
        return not_reproduced(f"first command {first:.2f} m/s vs steady {second:.2f} m/s")
    return reproduced(
        f"goal 0.5 m straight ahead: the first command is {first:.2f} m/s (saturated), the next "
        f"is {second:.2f} m/s for the identical error. prev_error starts at 0, so the first "
        "derivative is error/dt = 25 m/s - a kick on every fresh controller and every new goal",
        first=round(float(first), 3),
        second=round(float(second), 3),
    )


@probe("MC-002", deps=PID_DEPS)
def new_goal_inherits_old_pid_state():
    from navi_bot.control.motion_controller import MotionController

    controller = MotionController()
    dt = 0.02
    controller.set_goal(4.0, 0.0)
    for _ in range(50):
        controller.compute_control((0.0, 0.0, 0.0), dt)
    controller.set_goal(1.0, 0.0)                     # nearer goal, still straight ahead
    carried, _ = controller.compute_control((0.0, 0.0, 0.0), dt)
    fresh_controller = MotionController()
    fresh_controller.set_goal(1.0, 0.0)
    fresh_controller.compute_control((0.0, 0.0, 0.0), dt)
    steady, _ = fresh_controller.compute_control((0.0, 0.0, 0.0), dt)
    if carried > 0:
        return not_reproduced(f"first command after a goal change is {carried:.2f} m/s forward")
    return reproduced(
        f"after tracking a goal 4 m ahead, switching to a goal 1 m ahead (same direction) makes "
        f"the first command {carried:.2f} m/s - full REVERSE - where a settled controller gives "
        f"{steady:+.2f} m/s. set_goal() keeps the old prev_error and integral, so the drop in "
        "distance error reads as a huge negative derivative",
        carried=round(float(carried), 3),
        steady=round(float(steady), 3),
    )


@probe("PF-001", deps=FOLLOWER_DEPS)
def follower_can_target_a_waypoint_behind_the_robot():
    from navi_bot.control.path_follower import PathFollower

    follower = PathFollower()
    follower.set_path([(float(x), 0.0) for x in range(0, 11)])
    robot = (5.0, 1.0)                                # halfway along, 1.0 off to the side
    target = follower.target(*robot)
    if target[0] >= robot[0]:
        return not_reproduced(f"target {target} is ahead of the robot at {robot}")
    return reproduced(
        f"robot at {robot}, halfway along a straight 10-unit path but 1.0 to one side (as after "
        f"swerving round an obstacle): target() returns {target}, the START of the path, "
        f"{robot[0] - target[0]:.0f} units behind. Progress only advances when the robot passes "
        f"within wp_tol ({follower.wp_tol}) of each waypoint in order, so one missed waypoint "
        "sends the robot back for it",
        target=target,
    )


@probe("PF-002", deps=FOLLOWER_DEPS)
def follower_declares_a_loop_path_complete_at_the_start():
    from navi_bot.control.path_follower import PathFollower

    follower = PathFollower()
    loop = [(0.0, 0.0), (5.0, 0.0), (5.0, 5.0), (0.0, 5.0), (0.0, 0.5)]
    follower.set_path(loop)
    if not follower.goal_reached(0.0, 0.0):
        return not_reproduced("a path that ends near its start is not complete at the start")
    return reproduced(
        "for a 20-unit loop whose last waypoint is 0.5 from its first, goal_reached() is True "
        "at the starting pose before the robot has moved: arrival checks only the distance to "
        "the final waypoint, not progress along the path"
    )


@probe("PU-001", deps=PURSUIT_DEPS)
def pure_pursuit_stops_short_of_the_goal():
    from navi_bot.control.pure_pursuit import PurePursuitController

    controller = PurePursuitController()
    path = [(float(x), 0.0) for x in range(0, 11)]
    x, y, _ = _drive_unicycle(controller, path, (0.0, 0.0, 0.0))
    remaining = math.hypot(path[-1][0] - x, path[-1][1] - y)
    if remaining < 0.2:
        return not_reproduced(f"robot finishes {remaining:.2f} m from the last waypoint")
    return reproduced(
        f"simulated unicycle on a straight 10 m path: the controller stops at ({x:.2f}, {y:.2f}), "
        f"{remaining:.2f} m short of the goal at {path[-1]}, and commands (0, 0) from then on. "
        "Once every remaining waypoint is nearer than the lookahead distance "
        f"({controller.lookahead_distance} m) no lookahead point is found and the method falls "
        "off the end returning None",
        stop_x=round(x, 2),
        remaining=round(remaining, 2),
    )


@probe("PU-002", deps=PURSUIT_DEPS)
def pure_pursuit_curvature_ignores_the_real_lookahead_distance():
    from navi_bot.control.pure_pursuit import PurePursuitController

    controller = PurePursuitController()
    target = (5.0, 5.0)
    v, omega = controller.pure_pursuit([(0.0, 0.0), target], (0.0, 0.0, 0.0))
    distance = math.hypot(*target)
    alpha = math.atan2(target[1], target[0])
    exact = 2 * math.sin(alpha) / distance            # arc through the chosen point
    commanded = float(omega) / float(v)
    if abs(commanded - exact) / exact < 0.05:
        return not_reproduced("commanded curvature matches the arc through the lookahead point")
    return reproduced(
        f"the chosen lookahead point {target} is {distance:.2f} m away, so the arc through it has "
        f"curvature {exact:.3f} 1/m; the controller commands {commanded:.3f} 1/m "
        f"({commanded / exact:.1f}x too tight) because it divides by the nominal lookahead "
        f"({controller.lookahead_distance} m) instead of the actual distance to the point",
        commanded=round(commanded, 3),
        exact=round(exact, 3),
    )


@probe("PU-004", deps=PURSUIT_DEPS)
def pure_pursuit_chases_a_waypoint_behind_the_robot():
    from navi_bot.control.pure_pursuit import PurePursuitController

    controller = PurePursuitController()
    path = [(float(x), 0.0) for x in range(0, 11)]
    robot = (6.0, 0.0)
    chosen = controller.find_next_lookahead(path, robot)
    if chosen is not None and chosen[0] > robot[0]:
        return not_reproduced(f"lookahead point {chosen} is ahead of the robot at {robot}")
    v, omega = PurePursuitController().pure_pursuit(path, (robot[0], robot[1], 0.0))
    return reproduced(
        f"a controller that starts with the robot at {robot}, 6 m along a straight 10 m path, "
        f"picks {chosen} as its lookahead point: the search takes the FIRST waypoint at least "
        "one lookahead away in list order, and the path's start qualifies. The point is directly "
        f"behind (alpha = pi, sin = 0), so the command is v = {float(v):.1f} m/s with "
        f"omega = {float(omega):.2f}: full speed straight ahead, 'tracking' a point it is leaving",
        chosen=chosen,
    )


@probe("PU-003", deps=PURSUIT_DEPS)
def pure_pursuit_keeps_its_waypoint_index_across_paths():
    from navi_bot.control.pure_pursuit import PurePursuitController

    controller = PurePursuitController()
    first_path = [(float(x), 0.0) for x in range(0, 11)]
    _drive_unicycle(controller, first_path, (0.0, 0.0, 0.0), steps=120)
    carried_index = controller.waypoint
    second_path = [(0.0, float(y)) for y in range(0, 11)]     # a brand-new path
    chosen = controller.find_next_lookahead(second_path, (0.0, 0.0))
    expected = PurePursuitController().find_next_lookahead(second_path, (0.0, 0.0))
    if chosen == expected:
        return not_reproduced("a new path starts from its own beginning")
    return reproduced(
        f"after following one path the controller's internal index is {carried_index}; handed a "
        f"new path it starts searching from that index and aims at {chosen} instead of "
        f"{expected}. The path is an argument on every call but progress is hidden state with "
        "no reset",
        carried_index=carried_index,
        chosen=chosen,
        expected=expected,
    )
