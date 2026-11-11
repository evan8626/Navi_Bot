"""Reproduction probes for navi_bot/state_machine.py.

Every probe drives the real ``StateMachine`` on the mock ROS backend through
its public inputs (``add_mission``, ``/nav_status`` messages, timer callbacks)
and inspects what it published.
"""

from crlib.probe_api import probe, reproduced, not_reproduced

DEPS = ["navi_bot/state_machine.py", "navi_bot/mock_ros2.py", "navi_bot/ros_compat.py"]

PICKUP = (2.0, 1.0)
DELIVERY = (2.0, 8.0)


def _fresh_state_machine():
    from navi_bot import mock_ros2
    from navi_bot.state_machine import StateMachine

    mock_ros2.init()
    return StateMachine()


def _mission(name="m1"):
    from navi_bot.state_machine import Mission

    return Mission(name, PICKUP, DELIVERY)


def _arrive(sm):
    """Tell the state machine the robot reached its goal, as robot_controller does."""
    from navi_bot.mock_ros2 import String

    sm.nav_status_callback(String(data="goal_reached"))


def _live_timers(sm, skip):
    """Timers created after the first ``skip`` that have not been cancelled."""
    timers = []
    for timer in sm._timers[skip:]:
        is_canceled = getattr(timer, "is_canceled", None)
        if callable(is_canceled) and is_canceled():
            continue
        timers.append(timer)
    return timers


def _fire(timers):
    for timer in timers:
        timer.callback()


def _run_until(sm, state_name, max_ticks=12):
    for _ in range(max_ticks):
        if sm.current_state.name == state_name:
            return True
        sm.update_state_machine()
    return sm.current_state.name == state_name


def _drive_to_delivery_nav(sm):
    """Take one mission from IDLE to DELIVERY_NAV using public inputs only."""
    base_timers = len(sm._timers)
    if not _run_until(sm, "PICK_NAV"):
        return False
    sm.update_state_machine()
    _arrive(sm)
    if not _run_until(sm, "PICKING_UP"):
        return False
    sm.update_state_machine()
    _fire(_live_timers(sm, base_timers))
    return _run_until(sm, "DELIVERY_NAV")


def _complete_mission(sm):
    """Run one mission to completion; returns True when back in IDLE."""
    if not _drive_to_delivery_nav(sm):
        return False
    base_timers = len(sm._timers)
    _arrive(sm)
    if not _run_until(sm, "DELIVERING"):
        return False
    sm.update_state_machine()
    _fire(_live_timers(sm, base_timers))
    return _run_until(sm, "IDLE")


@probe("SM-001", deps=DEPS)
def delivery_goal_is_never_published():
    sm = _fresh_state_machine()
    sm.add_mission(_mission())
    if not _drive_to_delivery_nav(sm):
        raise RuntimeError(f"could not reach DELIVERY_NAV (stuck in {sm.current_state.name})")
    for _ in range(10):
        sm.update_state_machine()
    goals = sorted({(goal.x, goal.y) for goal in sm.goal_pub.published_msgs})
    if DELIVERY in goals:
        return not_reproduced(f"delivery goal {DELIVERY} was published on /goal_pose", goals=goals)
    return reproduced(
        f"after 10 ticks in DELIVERY_NAV the only goals ever published on /goal_pose are {goals}; "
        f"the delivery location {DELIVERY} never appears, so the robot is never sent there",
        goals=goals,
    )


@probe("SM-002", deps=DEPS)
def pickup_goal_is_republished_every_tick():
    sm = _fresh_state_machine()
    sm.add_mission(_mission())
    _run_until(sm, "PICK_NAV")
    before = len(sm.goal_pub.published_msgs)
    ticks = 10
    for _ in range(ticks):
        sm.update_state_machine()
    published = len(sm.goal_pub.published_msgs) - before
    if published <= 1:
        return not_reproduced(f"{published} goal message(s) over {ticks} ticks in PICK_NAV")
    return reproduced(
        f"{published} identical pickup goals published in {ticks} ticks (one every 0.2 s); "
        "each one makes path_planner discard its plan and replan from scratch",
        published=published,
        ticks=ticks,
    )


@probe("SM-003", deps=DEPS)
def stale_periodic_timer_completes_next_pickup_early():
    sm = _fresh_state_machine()
    base_timers = len(sm._timers)
    sm.add_mission(_mission("m1"))
    if not _complete_mission(sm):
        raise RuntimeError(f"mission 1 did not complete (stuck in {sm.current_state.name})")
    stale = _live_timers(sm, base_timers)

    sm.add_mission(_mission("m2"))
    _run_until(sm, "PICK_NAV")
    sm.update_state_machine()
    _arrive(sm)
    _run_until(sm, "PICKING_UP")
    sm.update_state_machine()          # mission 2 starts its own 2 s pickup timer
    _fire(stale)                       # rclpy timers are periodic: mission 1's fire again
    sm.update_state_machine()
    if sm.current_state.name == "PICKING_UP":
        return not_reproduced(
            "mission 2 keeps waiting for its own pickup timer", timers=len(sm._timers)
        )
    return reproduced(
        f"{len(stale)} timer(s) from mission 1 were never cancelled; when they fire again "
        f"(rclpy timers repeat) mission 2's pickup ends immediately - state is now "
        f"{sm.current_state.name} without mission 2's own timer having fired. "
        f"{len(sm._timers) - base_timers} pickup/delivery timers exist so far and none was "
        f"cancelled",
        stale_timers=len(stale),
    )


@probe("SM-004", deps=DEPS)
def battery_exactly_at_max_threshold_enters_error():
    from navi_bot.state_machine import RobotState

    sm = _fresh_state_machine()
    sm.current_state = RobotState.CHARGING
    sm.battery_level = sm.max_threshold
    sm.update_state_machine()
    if sm.current_state is RobotState.ERROR:
        return reproduced(
            f"CHARGING with battery == max_charge_threshold ({sm.max_threshold}) and no mission "
            "matches none of the '<' / '>' branches and falls through to ERROR"
        )
    return not_reproduced(f"state after the tick is {sm.current_state.name}")


@probe("SM-005", deps=DEPS)
def error_silently_drops_the_mission():
    from navi_bot.mock_ros2 import String

    sm = _fresh_state_machine()
    first, second = _mission("m1"), _mission("m2")
    sm.add_mission(first)
    sm.add_mission(second)
    _run_until(sm, "PICK_NAV")
    sm.nav_status_callback(String(data="error"))
    for _ in range(3):
        sm.update_state_machine()
    states = [msg.data for msg in sm.state_pub.published_msgs]
    kept = first in sm.mission_queue or sm.current_mission is first
    kept = kept or any(isinstance(value, (list, tuple, set)) and first in value
                       for value in vars(sm).values())
    if kept and "ERROR" in states:
        return not_reproduced(
            "the failed mission is retained and ERROR was published", states=states
        )
    return reproduced(
        f"after a nav 'error' during mission m1 the published states are {states} "
        f"({'ERROR was never published' if 'ERROR' not in states else 'ERROR published'}); "
        f"m1 is {'retained' if kept else 'gone - not requeued, not recorded'} and the robot "
        f"moved straight on to {getattr(sm.current_mission, 'mission_id', None)}",
        states=states,
    )


@probe("SM-006", deps=DEPS)
def delivered_mission_runs_again_after_charging():
    sm = _fresh_state_machine()
    first, second = _mission("m1"), _mission("m2")
    sm.add_mission(first)
    if not _complete_mission(sm):
        raise RuntimeError(f"mission 1 did not complete (stuck in {sm.current_state.name})")
    sm.add_mission(second)
    sm.battery_level = sm.min_mission_threshold - 10.0
    _run_until(sm, "CHARGING")
    sm.battery_level = 80.0
    sm.update_state_machine()
    if sm.current_mission is first and sm.current_state.name == "PICK_NAV":
        return reproduced(
            "after charging, the state machine went to PICK_NAV with mission m1, which was "
            "already delivered (status 'delivered' is treated as runnable and current_mission "
            "is never cleared); m2 is still waiting in the queue"
        )
    return not_reproduced(
        f"after charging: state {sm.current_state.name}, mission "
        f"{getattr(sm.current_mission, 'mission_id', None)}"
    )


@probe("SM-007", deps=DEPS)
def idle_logs_two_lines_every_tick():
    sm = _fresh_state_machine()
    lines = []
    sm._logger.info = lines.append
    ticks = 25
    for _ in range(ticks):
        sm.update_state_machine()
    if len(lines) < ticks:
        return not_reproduced(f"{len(lines)} log lines over {ticks} idle ticks")
    return reproduced(
        f"{len(lines)} INFO lines in {ticks} idle ticks (5 s of wall time at 5 Hz) - "
        f"about {len(lines) * 12 * 60:,} lines per hour while the robot does nothing",
        lines=len(lines),
    )
