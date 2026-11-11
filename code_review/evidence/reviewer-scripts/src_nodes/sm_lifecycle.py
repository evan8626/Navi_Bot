"""Drive StateMachine through two missions with a simulated PERIODIC timer
executor (rclpy semantics: timers fire every period until cancelled)."""
import sys, io, contextlib
sys.path.insert(0, '.')
import navi_bot.mock_ros2 as m
from navi_bot.mock_ros2 import String

# --- periodic timer simulation -------------------------------------------
CLOCK = [0.0]
ALL_TIMERS = []
class SimTimer:
    def __init__(self, period, cb):
        self.period, self.callback, self.next = period, cb, CLOCK[0] + period
        self.cancelled = False
    def cancel(self): self.cancelled = True
_orig = m.Node.create_timer
def create_timer(self, period, cb):
    t = SimTimer(period, cb); ALL_TIMERS.append(t); self._timers.append(t); return t
m.Node.create_timer = create_timer

def advance(seconds, dt=0.01):
    end = CLOCK[0] + seconds
    while CLOCK[0] < end - 1e-9:
        CLOCK[0] = round(CLOCK[0] + dt, 6)
        for t in list(ALL_TIMERS):
            if not t.cancelled and CLOCK[0] >= t.next - 1e-9:
                t.next += t.period
                t.callback()

from navi_bot.state_machine import StateMachine, Mission, RobotState
m.init()
buf = io.StringIO()
with contextlib.redirect_stdout(buf):
    sm = StateMachine()
nav = m.MockPublisher(String, '/nav_status', 10)
transitions = []
_orig_tr = sm.transition_state
def tr(state):
    if state != sm.current_state:
        transitions.append((round(CLOCK[0], 2), sm.current_state.name, state.name))
    _orig_tr(state)
sm.transition_state = tr

goals = []
_gp = sm.goal_pub.publish
def gp(msg):
    goals.append((round(CLOCK[0], 2), sm.current_state.name, msg.x, msg.y)); _gp(msg)
sm.goal_pub.publish = gp

with contextlib.redirect_stdout(buf):
    # --- idle log volume
    advance(1.0)
idle_lines = buf.getvalue().count('Mission queue is empty')
print('idle: "Mission queue is empty" lines in 1.0 s =', idle_lines)

with contextlib.redirect_stdout(buf):
    sm.add_mission(Mission('m1', (3.0, 0.9), (3.0, 9.1)))
    advance(1.0)                       # PICK_NAV for ~1 s
    print_goals_after_1s = len(goals)
    nav.publish(String(data='goal_reached'))
    advance(0.25)
    state_after_reach = sm.current_state.name
    advance(5.0)
print('goal msgs published during ~1s of PICK_NAV:', print_goals_after_1s)
print('goals published (t,state,x,y):', goals[:3], '... total', len(goals))
print('state 5s after pickup goal_reached:', sm.current_state.name)
print('transitions m1:', transitions)
print('delivery-leg goal ever published?', any(g[1] in ('DELIVERY_NAV', 'PICKED_UP') for g in goals))
# Simulate controller reaching the delivery point anyway
with contextlib.redirect_stdout(buf):
    nav.publish(String(data='goal_reached'))
    advance(3.0)
print('after 2nd goal_reached + 3 s:', sm.current_state.name, 'mission status', sm.current_mission.status)
n_timers = len([t for t in ALL_TIMERS if not t.cancelled])
print('live timers after mission 1:', n_timers, [t.period for t in ALL_TIMERS])
print('flags after mission 1: pickup_complete', sm.pickup_complete, 'delivery_complete', sm.delivery_complete)
print('current_mission after delivery:', sm.current_mission.mission_id, sm.current_mission.status)

# --- mission 2: stale flags from periodic timers
transitions.clear(); goals.clear()
with contextlib.redirect_stdout(buf):
    sm.add_mission(Mission('m2', (5.0, 0.9), (5.0, 9.1)))
    advance(0.4)
    nav.publish(String(data='goal_reached'))
    t_reach = CLOCK[0]
    # step finely and record when PICKING_UP -> PICKED_UP happens
    advance(0.4)
print('m2 transitions:', transitions)
pu = [t for t in transitions if t[2] == 'PICKED_UP']
if pu:
    print(f'm2 pickup finished {pu[0][0]-t_reach:.2f}s after arrival (expected 2.0 s dwell)')
print('timers now:', len(ALL_TIMERS))
