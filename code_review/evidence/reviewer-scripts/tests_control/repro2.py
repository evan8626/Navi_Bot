import math, sys, io, contextlib, time
buf = io.StringIO()
with contextlib.redirect_stdout(buf):
    from navi_bot.robot_controller import RobotController
    from navi_bot.state_machine import StateMachine, Mission, RobotState
    from navi_bot.mock_ros2 import LaserScan, Pose2D, String, Path, Point
    import navi_bot.mock_ros2 as m

def q(f, *a):
    with contextlib.redirect_stdout(io.StringIO()):
        return f(*a)

def scan(ranges, angle_min=-math.pi, inc=0.1):
    s = LaserScan(); s.ranges = ranges; s.angle_min = angle_min; s.angle_increment = inc
    return s

print("== governor: obstacle directly BEHIND (angle -pi) freezes robot permanently")
c = q(RobotController)
c.set_goal(5.0, 0.0)
q(c.lidar_callback, scan([0.4] + [5.0] * 62))  # index 0 at angle -pi => behind
print("closest angle", c.closest_obstacle_angle)
for i in range(3):
    q(c.control_loop)
    print("cmd", c.cmd_vel_pub.last_msg.linear.x, c.cmd_vel_pub.last_msg.angular.z)

print("== stale scan: no scan for a long time, last value persists")
c = q(RobotController); c.set_goal(5.0, 0.0)
q(c.lidar_callback, scan([5.0] * 63))
print("closest after clear scan", c.closest_obstacle_dist, "(no timestamp attr:", hasattr(c, 'last_scan_time'), ")")

print("== DWA path mode timing inside 20 ms period")
def path_msg(wps):
    msg = Path()
    for x, y in wps:
        p = Point(); p.x, p.y = float(x), float(y); msg.poses.append(p)
    return msg
c = q(RobotController)
q(c.path_callback, path_msg([(0, 0), (2, 0), (4, 0), (6, 0)]))
# put several obstacles 1.2 m away so they enter the DWA obstacle list but governor does not act
rng = [1.2 if i % 3 == 0 else 5.0 for i in range(63)]
q(c.lidar_callback, scan(rng))
ts = []
for i in range(20):
    t0 = time.perf_counter(); q(c.control_loop); ts.append((time.perf_counter() - t0) * 1000)
ts.sort()
print(f"control_loop ms: median={ts[10]:.2f} max={ts[-1]:.2f} deadline_misses={c.deadline_misses} obstacles={len(c.valid_obstacles)}")

print("== state machine: CHARGING at exactly max_threshold, no mission")
sm = q(StateMachine)
sm.current_state = RobotState.CHARGING
sm.battery_level = sm.max_threshold
q(sm.update_state_machine)
print("state after tick at battery == max_threshold:", sm.current_state)

print("== state machine: full mission through public inputs only (nav_status + timer callbacks)")
q(m.init)
sm = q(StateMachine)
nav = q(lambda: m.Node('fake_nav'))
navpub = q(nav.create_publisher, String, '/nav_status', 10)
sm.add_mission(Mission('M1', (1.0, 2.0), (5.0, 6.0))) if False else q(sm.add_mission, Mission('M1', (1.0, 2.0), (5.0, 6.0)))
def tick(n=1):
    for _ in range(n): q(sm.update_state_machine)
def fire_timers():
    # emulate rclpy: timers are PERIODIC; fire every timer created after construction
    for t in sm._timers[1:]:
        q(t.callback)
tick(2)
print("state", sm.current_state.name, "goals published", [(g.x, g.y) for g in sm.goal_pub.published_msgs][-1:], "count", len(sm.goal_pub.published_msgs))
tick(5)
print("after 5 more ticks in PICK_NAV goal publish count:", len(sm.goal_pub.published_msgs))
q(navpub.publish, String(data='goal_reached'))
tick(2)
print("state", sm.current_state.name)
fire_timers(); tick(2)
print("state", sm.current_state.name)
n_goals = len(sm.goal_pub.published_msgs)
tick(10)
print("DELIVERY_NAV: new goals published while in DELIVERY_NAV:", len(sm.goal_pub.published_msgs) - n_goals,
      "last goal", (sm.goal_pub.last_msg.x, sm.goal_pub.last_msg.y), "delivery_location", sm.current_mission.delivery_location)

print("== periodic timer: re-fires during 2nd mission's PICKING_UP")
q(navpub.publish, String(data='goal_reached')); tick(2)  # -> DELIVERING
fire_timers(); tick(1)  # deliver
print("state", sm.current_state.name, "timers created so far", len(sm._timers) - 1)
q(sm.add_mission, Mission('M2', (3.0, 3.0), (7.0, 7.0)))
tick(1)
q(navpub.publish, String(data='goal_reached')); tick(2)
print("M2 state", sm.current_state.name)
# a real periodic pickup timer from M1 keeps firing -> fire OLD timers only (not the new one)
for t in sm._timers[1:-1]:
    q(t.callback)
tick(1)
print("M2 after OLD M1 timers fired (no 2 s wait for M2's own timer):", sm.current_state.name)

print("== error mid-mission drops in-flight mission")
q(m.init)
sm = q(StateMachine)
q(sm.add_mission, Mission('A', (1, 1), (2, 2)))
q(sm.add_mission, Mission('B', (3, 3), (4, 4)))
tick(2)
print("current", sm.current_mission.mission_id, sm.current_state.name)
q(sm.nav_status_callback, String(data='error'))
tick(1)
print("after error tick: state", sm.current_state.name, "published", [x.data for x in sm.state_pub.published_msgs])
tick(1)
print("next tick: current mission", sm.current_mission.mission_id, "queue", [x.mission_id for x in sm.mission_queue], "A status kept?")
