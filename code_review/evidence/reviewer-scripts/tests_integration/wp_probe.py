"""Probe waypoint_nav_test: trajectory clearance, goal-pose traffic, pursuit index carry-over."""
import sys, math, logging
sys.path.insert(0, 'tests'); logging.disable(logging.CRITICAL)
import builtins
_p = builtins.print
builtins.print = lambda *a, **k: None
import numpy as np
import waypoint_nav_test as w
from navi_bot.path_planner import PathPlannerNode

grid_ref = {}
traj = []
orig_update = w.WheelOdometry.update
def upd(self, l, r, dt):
    out = orig_update(self, l, r, dt)
    traj.append((out[0], out[1]))
    return out
w.WheelOdometry.update = upd

goals = []
orig_goal = PathPlannerNode.goal_callback
def gcb(self, msg):
    goals.append((msg.x, msg.y, self))
    return orig_goal(self, msg)
PathPlannerNode.goal_callback = gcb

orig_pp = w.PurePursuitController.pure_pursuit
starts = []
def pp(self, path, pose):
    if not starts or starts[-1][1] is not path:
        starts.append((self.waypoint, path, len(path)))
    return orig_pp(self, path, pose)
w.PurePursuitController.pure_pursuit = pp

for idx, (fn, grid) in enumerate([(w.test_single_mission_obstacle_map, w.obstacle_map()),
                                  (w.test_three_missions_obstacle_map, w.complex_map()),
                                  (w.test_long_running_multi_waypoint_route, w.complex_map())]):
    traj.clear(); goals.clear(); starts.clear()
    res = fn()
    obs = np.argwhere(grid != 0)
    mind = min(float(np.min(np.hypot(obs[:, 0] - x, obs[:, 1] - y))) for x, y in traj)
    inwall = sum(1 for x, y in traj if 0 <= round(x) < 20 and 0 <= round(y) < 20 and grid[round(x)][round(y)])
    _p(f"{fn.__name__}: passed={res} steps={len(traj)} min_clear_to_obstacle_centre={mind:.3f} poses_in_occupied_cell={inwall}")
    _p("  goal_callback msgs (x,y):", [(g[0], g[1]) for g in goals])
    _p("  pursuit.waypoint at first use of each new path (idx, path_len):", [(s[0], s[2]) for s in starts])
