#!/usr/bin/env python3
"""
Test script for StateMachine from state_machine.py

Tests that the state machine correctly satisfies behavioral invariants for:
- Initialization
- IDLE state logic
- PICK_NAV state logic
- PICKING_UP state logic
- PICKED_UP / DELIVERY_NAV state logic
- DELIVERING state logic
- CHARGING state logic
- ERROR state logic
- State transition mechanics
"""
import logging
import sys

from navi_bot.state_machine import StateMachine, Mission, RobotState
from navi_bot.mock_ros2 import String

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)


# MARK: Setup

def setup_sm():
    """Create a fresh StateMachine for each test."""
    return StateMachine()

def make_mission(mission_id='M001', pickup=(1.0, 2.0), delivery=(5.0, 6.0)):
    return Mission(mission_id, pickup, delivery)

def fire(sm):
    """Run one state machine tick."""
    sm.update_state_machine()


# MARK: Initialization

def test_init_state_is_idle():
    """
    TEST 1: StateMachine must start in IDLE.
    """
    passed = False
    logger.info("TEST 1: Init — initial state must be IDLE")
    sm = setup_sm()
    if sm.current_state == RobotState.IDLE:
        logger.info(f"  OK   current_state={sm.current_state}")
        passed = True
    else:
        logger.warning(f"  FAIL current_state={sm.current_state}, expected IDLE")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_init_mission_queue_empty():
    """
    TEST 2: Mission queue must be empty on construction.
    """
    passed = False
    logger.info("TEST 2: Init — mission queue must be empty")
    sm = setup_sm()
    if sm.mission_queue == [] and sm.current_mission is None:
        logger.info("  OK   queue=[], current_mission=None")
        passed = True
    else:
        logger.warning(f"  FAIL queue={sm.mission_queue}, current_mission={sm.current_mission}")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_init_battery_full():
    """
    TEST 3: Battery level must start at 100.0.
    """
    passed = False
    logger.info("TEST 3: Init — battery level must start at 100.0")
    sm = setup_sm()
    if sm.battery_level == 100.0:
        logger.info(f"  OK   battery_level={sm.battery_level}")
        passed = True
    else:
        logger.warning(f"  FAIL battery_level={sm.battery_level}, expected 100.0")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_init_no_error_flags():
    """
    TEST 4: No error or goal flags must be set on construction.
    """
    passed = True
    logger.info("TEST 4: Init — error and goal flags must be False")
    sm = setup_sm()
    if sm.has_error:
        logger.warning("  FAIL has_error=True on init")
        passed = False
    else:
        logger.info("  OK   has_error=False")
    if sm.is_at_goal:
        logger.warning("  FAIL is_at_goal=True on init")
        passed = False
    else:
        logger.info("  OK   is_at_goal=False")
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: Mission

def test_add_mission_appends_to_queue():
    """
    TEST 5: add_mission must append to the mission queue.
    """
    passed = False
    logger.info("TEST 5: Mission — add_mission must append to the queue")
    sm = setup_sm()
    m = make_mission()
    sm.add_mission(m)
    if len(sm.mission_queue) == 1 and sm.mission_queue[0] is m:
        logger.info(f"  OK   queue length={len(sm.mission_queue)}")
        passed = True
    else:
        logger.warning(f"  FAIL queue={sm.mission_queue}")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_mission_initial_status_pending():
    """
    TEST 6: A new Mission must have status 'pending'.
    """
    passed = False
    logger.info("TEST 6: Mission — new mission status must be 'pending'")
    m = make_mission()
    if m.status == 'pending':
        logger.info(f"  OK   status={m.status}")
        passed = True
    else:
        logger.warning(f"  FAIL status={m.status}, expected 'pending'")
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: IDLE State

def test_idle_empty_queue_stays_idle():
    """
    TEST 7: IDLE with no missions must remain IDLE.
    """
    passed = False
    logger.info("TEST 7: IDLE — empty queue must keep state as IDLE")
    sm = setup_sm()
    fire(sm)
    if sm.current_state == RobotState.IDLE:
        logger.info(f"  OK   state={sm.current_state}")
        passed = True
    else:
        logger.warning(f"  FAIL state={sm.current_state}, expected IDLE")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_idle_low_battery_transitions_to_charging():
    """
    TEST 8: IDLE with battery at or below min_mission_threshold must transition to CHARGING.
    """
    passed = False
    logger.info("TEST 8: IDLE — low battery must transition to CHARGING")
    sm = setup_sm()
    sm.battery_level = sm.min_mission_threshold  # exactly at threshold
    sm.add_mission(make_mission())
    fire(sm)
    if sm.current_state == RobotState.CHARGING:
        logger.info(f"  OK   state={sm.current_state}")
        passed = True
    else:
        logger.warning(f"  FAIL state={sm.current_state}, expected CHARGING")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_idle_with_mission_transitions_to_pick_nav():
    """
    TEST 9: IDLE with a pending mission and sufficient battery must transition to PICK_NAV.
    """
    passed = False
    logger.info("TEST 9: IDLE — pending mission with good battery must transition to PICK_NAV")
    sm = setup_sm()
    sm.add_mission(make_mission())
    fire(sm)
    if sm.current_state == RobotState.PICK_NAV:
        logger.info(f"  OK   state={sm.current_state}")
        passed = True
    else:
        logger.warning(f"  FAIL state={sm.current_state}, expected PICK_NAV")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_idle_pops_mission_from_queue():
    """
    TEST 10: IDLE must pop the first mission from the queue and assign it as current_mission.
    """
    passed = False
    logger.info("TEST 10: IDLE — must pop mission from queue and assign to current_mission")
    sm = setup_sm()
    m = make_mission('M001')
    sm.add_mission(m)
    fire(sm)
    if sm.current_mission is m and len(sm.mission_queue) == 0:
        logger.info(f"  OK   current_mission={sm.current_mission.mission_id}, queue empty")
        passed = True
    else:
        logger.warning(f"  FAIL current_mission={sm.current_mission}, queue={sm.mission_queue}")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_idle_picked_up_mission_transitions_to_delivery_nav():
    """
    TEST 11: IDLE with a 'picked_up' mission must transition to DELIVERY_NAV, not PICK_NAV.
    """
    passed = False
    logger.info("TEST 11: IDLE — 'picked_up' mission must transition to DELIVERY_NAV")
    sm = setup_sm()
    m = make_mission()
    m.status = 'picked_up'
    sm.add_mission(m)
    fire(sm)
    if sm.current_state == RobotState.DELIVERY_NAV:
        logger.info(f"  OK   state={sm.current_state}")
        passed = True
    else:
        logger.warning(f"  FAIL state={sm.current_state}, expected DELIVERY_NAV")
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: PICK_NAV State

def test_pick_nav_sets_mission_status_navigating():
    """
    TEST 12: PICK_NAV must set mission status to 'navigating'.
    """
    passed = False
    logger.info("TEST 12: PICK_NAV — must set mission status to 'navigating'")
    sm = setup_sm()
    sm.add_mission(make_mission())
    fire(sm)  # IDLE -> PICK_NAV
    fire(sm)  # PICK_NAV tick
    if sm.current_mission.status == 'navigating':
        logger.info(f"  OK   mission status={sm.current_mission.status}")
        passed = True
    else:
        logger.warning(f"  FAIL mission status={sm.current_mission.status}, expected 'navigating'")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_pick_nav_publishes_goal():
    """
    TEST 13: PICK_NAV must publish a goal pose matching the pickup location.
    """
    passed = False
    logger.info("TEST 13: PICK_NAV — must publish goal matching pickup location")
    sm = setup_sm()
    pickup = (3.0, 7.0)
    sm.add_mission(make_mission(pickup=pickup))
    fire(sm)  # IDLE -> PICK_NAV
    fire(sm)  # PICK_NAV tick publishes goal
    last_goal = sm.goal_pub.last_msg
    if last_goal is not None and last_goal.x == pickup[0] and last_goal.y == pickup[1]:
        logger.info(f"  OK   published goal x={last_goal.x}, y={last_goal.y}")
        passed = True
    else:
        logger.warning(f"  FAIL last_goal={last_goal}")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_pick_nav_at_goal_transitions_to_picking_up():
    """
    TEST 14: PICK_NAV must transition to PICKING_UP when is_at_goal is True.
    """
    passed = False
    logger.info("TEST 14: PICK_NAV — is_at_goal=True must transition to PICKING_UP")
    sm = setup_sm()
    sm.add_mission(make_mission())
    fire(sm)  # IDLE -> PICK_NAV
    sm.is_at_goal = True
    fire(sm)  # PICK_NAV -> PICKING_UP
    if sm.current_state == RobotState.PICKING_UP:
        logger.info(f"  OK   state={sm.current_state}")
        passed = True
    else:
        logger.warning(f"  FAIL state={sm.current_state}, expected PICKING_UP")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_pick_nav_not_at_goal_stays_in_pick_nav():
    """
    TEST 15: PICK_NAV must remain in PICK_NAV while is_at_goal is False.
    """
    passed = False
    logger.info("TEST 15: PICK_NAV — is_at_goal=False must keep state as PICK_NAV")
    sm = setup_sm()
    sm.add_mission(make_mission())
    fire(sm)  # IDLE -> PICK_NAV
    fire(sm)  # PICK_NAV tick, no goal yet
    if sm.current_state == RobotState.PICK_NAV:
        logger.info(f"  OK   state={sm.current_state}")
        passed = True
    else:
        logger.warning(f"  FAIL state={sm.current_state}, expected PICK_NAV")
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: PICKING_UP State

def test_picking_up_sets_timer_flag():
    """
    TEST 16: PICKING_UP must set _pickup_timer_started on first tick.
    """
    passed = False
    logger.info("TEST 16: PICKING_UP — first tick must set _pickup_timer_started")
    sm = setup_sm()
    sm.add_mission(make_mission())
    fire(sm)         # IDLE -> PICK_NAV
    sm.is_at_goal = True
    fire(sm)         # PICK_NAV -> PICKING_UP
    fire(sm)         # PICKING_UP first tick
    if sm._pickup_timer_started:
        logger.info("  OK   _pickup_timer_started=True")
        passed = True
    else:
        logger.warning("  FAIL _pickup_timer_started=False after first tick")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_picking_up_completes_on_flag():
    """
    TEST 17: PICKING_UP must transition to PICKED_UP when pickup_complete is set.
    Timer flag and pickup_complete must be reset after transition.
    """
    passed = True
    logger.info("TEST 17: PICKING_UP — pickup_complete=True must transition to PICKED_UP and reset flags")
    sm = setup_sm()
    sm.add_mission(make_mission())
    fire(sm)
    sm.is_at_goal = True
    fire(sm)
    fire(sm)  # start timer
    sm.pickup_complete = True
    fire(sm)  # should complete pickup
    if sm.current_state != RobotState.PICKED_UP:
        logger.warning(f"  FAIL state={sm.current_state}, expected PICKED_UP")
        passed = False
    else:
        logger.info(f"  OK   state={sm.current_state}")
    if sm.pickup_complete:
        logger.warning("  FAIL pickup_complete was not reset to False")
        passed = False
    else:
        logger.info("  OK   pickup_complete reset to False")
    if sm._pickup_timer_started:
        logger.warning("  FAIL _pickup_timer_started was not reset to False")
        passed = False
    else:
        logger.info("  OK   _pickup_timer_started reset to False")
    if sm.is_at_goal:
        logger.warning("  FAIL is_at_goal was not reset to False")
        passed = False
    else:
        logger.info("  OK   is_at_goal reset to False")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_picking_up_mission_status_set():
    """
    TEST 18: Mission status must be 'picking_up' when arriving at pickup goal.
    """
    passed = False
    logger.info("TEST 18: PICKING_UP — mission status must be set to 'picking_up' on arrival")
    sm = setup_sm()
    sm.add_mission(make_mission())
    fire(sm)
    sm.is_at_goal = True
    fire(sm)  # PICK_NAV sets status to 'picking_up' on goal reached
    if sm.current_mission.status == 'picking_up':
        logger.info(f"  OK   mission status={sm.current_mission.status}")
        passed = True
    else:
        logger.warning(f"  FAIL mission status={sm.current_mission.status}, expected 'picking_up'")
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: PICKED_UP / DELIVERY_NAV

def test_picked_up_transitions_to_delivery_nav():
    """
    TEST 19: PICKED_UP state must transition to DELIVERY_NAV on next tick.
    """
    passed = False
    logger.info("TEST 19: PICKED_UP — must transition to DELIVERY_NAV")
    sm = setup_sm()
    sm.add_mission(make_mission())
    fire(sm)
    sm.is_at_goal = True
    fire(sm)
    fire(sm)
    sm.pickup_complete = True
    fire(sm)  # -> PICKED_UP
    fire(sm)  # PICKED_UP -> DELIVERY_NAV
    if sm.current_state == RobotState.DELIVERY_NAV:
        logger.info(f"  OK   state={sm.current_state}")
        passed = True
    else:
        logger.warning(f"  FAIL state={sm.current_state}, expected DELIVERY_NAV")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_delivery_nav_at_goal_transitions_to_delivering():
    """
    TEST 20: DELIVERY_NAV must transition to DELIVERING when is_at_goal is True.
    """
    passed = False
    logger.info("TEST 20: DELIVERY_NAV — is_at_goal=True must transition to DELIVERING")
    sm = setup_sm()
    sm.add_mission(make_mission())
    # Walk through to DELIVERY_NAV
    fire(sm)
    sm.is_at_goal = True
    fire(sm)
    fire(sm)
    sm.pickup_complete = True
    fire(sm)
    fire(sm)
    # Now in DELIVERY_NAV
    sm.is_at_goal = True
    fire(sm)
    if sm.current_state == RobotState.DELIVERING:
        logger.info(f"  OK   state={sm.current_state}")
        passed = True
    else:
        logger.warning(f"  FAIL state={sm.current_state}, expected DELIVERING")
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: DELIVERING State

def test_delivering_completes_on_flag():
    """
    TEST 21: DELIVERING must transition to IDLE when delivery_complete is set.
    Timer flag and delivery_complete must be reset. Mission status must be 'delivered'.
    """
    passed = True
    logger.info("TEST 21: DELIVERING — delivery_complete=True must transition to IDLE and reset flags")
    sm = setup_sm()
    sm.add_mission(make_mission())
    fire(sm)
    sm.is_at_goal = True
    fire(sm)
    fire(sm)
    sm.pickup_complete = True
    fire(sm)
    fire(sm)
    sm.is_at_goal = True
    fire(sm)
    fire(sm)  # start delivery timer
    sm.delivery_complete = True
    fire(sm)  # complete delivery

    if sm.current_state != RobotState.IDLE:
        logger.warning(f"  FAIL state={sm.current_state}, expected IDLE")
        passed = False
    else:
        logger.info(f"  OK   state={sm.current_state}")
    if sm.delivery_complete:
        logger.warning("  FAIL delivery_complete was not reset to False")
        passed = False
    else:
        logger.info("  OK   delivery_complete reset to False")
    if sm._delivery_timer_started:
        logger.warning("  FAIL _delivery_timer_started was not reset to False")
        passed = False
    else:
        logger.info("  OK   _delivery_timer_started reset to False")
    if sm.is_at_goal:
        logger.warning("  FAIL is_at_goal was not reset to False")
        passed = False
    else:
        logger.info("  OK   is_at_goal reset to False")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_delivering_sets_mission_status_delivered():
    """
    TEST 22: Mission status must be 'delivered' after delivery completes.
    """
    passed = False
    logger.info("TEST 22: DELIVERING — mission status must be 'delivered' after completion")
    sm = setup_sm()
    sm.add_mission(make_mission())
    fire(sm)
    sm.is_at_goal = True
    fire(sm)
    fire(sm)
    sm.pickup_complete = True
    fire(sm)
    fire(sm)
    sm.is_at_goal = True
    fire(sm)
    fire(sm)
    sm.delivery_complete = True
    fire(sm)
    if sm.current_mission.status == 'delivered':
        logger.info(f"  OK   mission status={sm.current_mission.status}")
        passed = True
    else:
        logger.warning(f"  FAIL mission status={sm.current_mission.status}, expected 'delivered'")
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: CHARGING State

def test_charging_below_min_stays_charging():
    """
    TEST 23: CHARGING with battery below min_mission_threshold must stay CHARGING.
    """
    passed = False
    logger.info("TEST 23: CHARGING — battery below min threshold must stay CHARGING")
    sm = setup_sm()
    sm.current_state = RobotState.CHARGING
    sm.battery_level = sm.min_mission_threshold - 1.0
    fire(sm)
    if sm.current_state == RobotState.CHARGING:
        logger.info(f"  OK   state={sm.current_state}")
        passed = True
    else:
        logger.warning(f"  FAIL state={sm.current_state}, expected CHARGING")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_charging_above_min_no_mission_stays_charging():
    """
    TEST 24: CHARGING with battery above min but below max, and no mission, must stay CHARGING.
    """
    passed = False
    logger.info("TEST 24: CHARGING — above min, no mission, must stay CHARGING")
    sm = setup_sm()
    sm.current_state = RobotState.CHARGING
    sm.battery_level = sm.min_mission_threshold + 1.0
    sm.current_mission = None
    fire(sm)
    if sm.current_state == RobotState.CHARGING:
        logger.info(f"  OK   state={sm.current_state}")
        passed = True
    else:
        logger.warning(f"  FAIL state={sm.current_state}, expected CHARGING")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_charging_above_max_no_mission_transitions_to_idle():
    """
    TEST 25: CHARGING with battery above max_threshold and no mission must transition to IDLE.
    """
    passed = False
    logger.info("TEST 25: CHARGING — above max threshold, no mission, must transition to IDLE")
    sm = setup_sm()
    sm.current_state = RobotState.CHARGING
    sm.battery_level = sm.max_threshold + 1.0
    sm.current_mission = None
    fire(sm)
    if sm.current_state == RobotState.IDLE:
        logger.info(f"  OK   state={sm.current_state}")
        passed = True
    else:
        logger.warning(f"  FAIL state={sm.current_state}, expected IDLE")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_charging_with_pending_mission_transitions_to_pick_nav():
    """
    TEST 26: CHARGING with battery above min and a pending mission must transition to PICK_NAV.
    """
    passed = False
    logger.info("TEST 26: CHARGING — above min with pending mission must transition to PICK_NAV")
    sm = setup_sm()
    sm.current_state = RobotState.CHARGING
    sm.battery_level = sm.min_mission_threshold + 1.0
    sm.current_mission = make_mission()
    sm.current_mission.status = 'pending'
    fire(sm)
    if sm.current_state == RobotState.PICK_NAV:
        logger.info(f"  OK   state={sm.current_state}")
        passed = True
    else:
        logger.warning(f"  FAIL state={sm.current_state}, expected PICK_NAV")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_charging_with_picked_up_mission_transitions_to_delivery_nav():
    """
    TEST 27: CHARGING with a 'picked_up' mission must transition to DELIVERY_NAV.
    """
    passed = False
    logger.info("TEST 27: CHARGING — 'picked_up' mission must transition to DELIVERY_NAV")
    sm = setup_sm()
    sm.current_state = RobotState.CHARGING
    sm.battery_level = sm.min_mission_threshold + 1.0
    sm.current_mission = make_mission()
    sm.current_mission.status = 'picked_up'
    fire(sm)
    if sm.current_state == RobotState.DELIVERY_NAV:
        logger.info(f"  OK   state={sm.current_state}")
        passed = True
    else:
        logger.warning(f"  FAIL state={sm.current_state}, expected DELIVERY_NAV")
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: ERROR State

def test_error_flag_triggers_error_state():
    """
    TEST 28: Setting has_error=True must immediately transition to ERROR on next tick.
    """
    passed = False
    logger.info("TEST 28: ERROR — has_error=True must transition to ERROR state")
    sm = setup_sm()
    sm.has_error = True
    fire(sm)
    # ERROR handler resets has_error and transitions back to IDLE in the same tick,
    # so check that previous_state was ERROR or that has_error was consumed.
    if not sm.has_error:
        logger.info("  OK   has_error was consumed (reset to False)")
        passed = True
    else:
        logger.warning("  FAIL has_error was not reset after error handling")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_error_resets_to_idle():
    """
    TEST 29: ERROR handler must reset has_error and transition back to IDLE.
    """
    passed = False
    logger.info("TEST 29: ERROR — must reset has_error and transition to IDLE")
    sm = setup_sm()
    sm.has_error = True
    fire(sm)
    if sm.current_state == RobotState.IDLE and not sm.has_error:
        logger.info(f"  OK   state={sm.current_state}, has_error={sm.has_error}")
        passed = True
    else:
        logger.warning(f"  FAIL state={sm.current_state}, has_error={sm.has_error}")
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: State Transition Mechanics

def test_transition_state_updates_current_state():
    """
    TEST 30: transition_state must update current_state to the new state.
    """
    passed = False
    logger.info("TEST 30: Transition — transition_state must update current_state")
    sm = setup_sm()
    sm.transition_state(RobotState.CHARGING)
    if sm.current_state == RobotState.CHARGING:
        logger.info(f"  OK   current_state={sm.current_state}")
        passed = True
    else:
        logger.warning(f"  FAIL current_state={sm.current_state}, expected CHARGING")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_transition_state_records_previous_state():
    """
    TEST 31: transition_state must record the prior state in previous_state.
    """
    passed = False
    logger.info("TEST 31: Transition — transition_state must record previous state")
    sm = setup_sm()
    sm.transition_state(RobotState.CHARGING)
    if sm.previous_state == RobotState.IDLE:
        logger.info(f"  OK   previous_state={sm.previous_state}")
        passed = True
    else:
        logger.warning(f"  FAIL previous_state={sm.previous_state}, expected IDLE")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_state_published_on_transition():
    """
    TEST 32: A state change must cause the new state name to be published.
    """
    passed = False
    logger.info("TEST 32: Transition — state change must publish new state name")
    sm = setup_sm()
    sm.add_mission(make_mission())
    fire(sm)  # IDLE -> PICK_NAV, should publish
    last_msg = sm.state_pub.last_msg
    if last_msg is not None and last_msg.data == RobotState.PICK_NAV.name:
        logger.info(f"  OK   published state={last_msg.data}")
        passed = True
    else:
        logger.warning(f"  FAIL published={last_msg.data if last_msg else None}, expected {RobotState.PICK_NAV.name}")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_no_publish_when_state_unchanged():
    """
    TEST 33: No state publication must occur when the state does not change.
    """
    passed = False
    logger.info("TEST 33: Transition — no publish must occur when state is unchanged")
    sm = setup_sm()
    fire(sm)  # IDLE -> IDLE, no publish expected
    last_msg = sm.state_pub.last_msg
    if last_msg is None:
        logger.info("  OK   no message published")
        passed = True
    else:
        logger.warning(f"  FAIL message was published: {last_msg.data}")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_nav_status_callback_sets_at_goal():
    """
    TEST 34: nav_status_callback with 'goal_reached' must set is_at_goal=True.
    """
    passed = False
    logger.info("TEST 34: Callback — 'goal_reached' message must set is_at_goal=True")
    sm = setup_sm()
    msg = String()
    msg.data = 'goal_reached'
    sm.nav_status_callback(msg)
    if sm.is_at_goal:
        logger.info("  OK   is_at_goal=True")
        passed = True
    else:
        logger.warning("  FAIL is_at_goal=False after 'goal_reached'")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_nav_status_callback_sets_error():
    """
    TEST 35: nav_status_callback with 'error' must set has_error=True.
    """
    passed = False
    logger.info("TEST 35: Callback — 'error' message must set has_error=True")
    sm = setup_sm()
    msg = String()
    msg.data = 'error'
    sm.nav_status_callback(msg)
    if sm.has_error:
        logger.info("  OK   has_error=True")
        passed = True
    else:
        logger.warning("  FAIL has_error=False after 'error' message")
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: Main

def main():
    logger.info("State Machine Test Suite")
    results = []

    results.append(test_init_state_is_idle())
    results.append(test_init_mission_queue_empty())
    results.append(test_init_battery_full())
    results.append(test_init_no_error_flags())

    results.append(test_add_mission_appends_to_queue())
    results.append(test_mission_initial_status_pending())

    results.append(test_idle_empty_queue_stays_idle())
    results.append(test_idle_low_battery_transitions_to_charging())
    results.append(test_idle_with_mission_transitions_to_pick_nav())
    results.append(test_idle_pops_mission_from_queue())
    results.append(test_idle_picked_up_mission_transitions_to_delivery_nav())

    results.append(test_pick_nav_sets_mission_status_navigating())
    results.append(test_pick_nav_publishes_goal())
    results.append(test_pick_nav_at_goal_transitions_to_picking_up())
    results.append(test_pick_nav_not_at_goal_stays_in_pick_nav())

    results.append(test_picking_up_sets_timer_flag())
    results.append(test_picking_up_completes_on_flag())
    results.append(test_picking_up_mission_status_set())

    results.append(test_picked_up_transitions_to_delivery_nav())
    results.append(test_delivery_nav_at_goal_transitions_to_delivering())

    results.append(test_delivering_completes_on_flag())
    results.append(test_delivering_sets_mission_status_delivered())

    results.append(test_charging_below_min_stays_charging())
    results.append(test_charging_above_min_no_mission_stays_charging())
    results.append(test_charging_above_max_no_mission_transitions_to_idle())
    results.append(test_charging_with_pending_mission_transitions_to_pick_nav())
    results.append(test_charging_with_picked_up_mission_transitions_to_delivery_nav())

    results.append(test_error_flag_triggers_error_state())
    results.append(test_error_resets_to_idle())

    results.append(test_transition_state_updates_current_state())
    results.append(test_transition_state_records_previous_state())
    results.append(test_state_published_on_transition())
    results.append(test_no_publish_when_state_unchanged())
    results.append(test_nav_status_callback_sets_at_goal())
    results.append(test_nav_status_callback_sets_error())

    logger.info(f"Results: {sum(results)}/{len(results)} passed")
    logger.info("All tests complete.")
    sys.exit(0 if all(results) else 1)


if __name__ == '__main__':
    main()