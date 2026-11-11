import sys, logging, builtins
sys.path.insert(0, 'tests'); logging.disable(logging.CRITICAL)
_p = builtins.print; builtins.print = lambda *a, **k: None
import numpy as np, waypoint_nav_test as w
grid = w.complex_map()
cur = {'m': None}
orig_run = w.run_mission
def rm(sm, pl, pu, od, g, mission, log_prefix=""):
    cur['m'] = mission.mission_id; return orig_run(sm, pl, pu, od, g, mission, log_prefix)
w.run_mission = rm
orig_update = w.WheelOdometry.update
def upd(self, l, r, dt):
    out = orig_update(self, l, r, dt)
    x, y = out[0], out[1]
    if 0 <= round(x) < 20 and 0 <= round(y) < 20 and grid[round(x)][round(y)]:
        _p(f"{cur['m']}: pose ({x:.2f},{y:.2f}) inside occupied cell ({round(x)},{round(y)})")
    return out
w.WheelOdometry.update = upd
_p("result", w.test_long_running_multi_waypoint_route())
# fresh pursuit per leg: does the bug disappear?
