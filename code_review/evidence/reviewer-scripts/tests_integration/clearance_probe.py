"""Instrument DWAPlanner.plan to reconstruct the driven sub-step trajectory and
measure min clearance to static obstacle centres and to the hazard.
Usage: python clearance_probe.py <suite_module> <test_index> [noyield]"""
import sys, math, logging, importlib
sys.path.insert(0, 'tests')
logging.disable(logging.CRITICAL)
import numpy as np
import navi_bot.planners.dwa as dwa_mod

suite, idx = sys.argv[1], int(sys.argv[2])
if len(sys.argv) > 3 and sys.argv[3] == 'noyield':
    dwa_mod.YIELD_RANGE = 0.0
mod = importlib.import_module(suite)

stats = {'static': float('inf'), 'hazard': float('inf')}
orig = dwa_mod.DWAPlanner.plan
def plan(self, pose, vel, goal, obstacles):
    cmd = orig(self, pose, vel, goal, obstacles)
    grid = self.occupancy_grid
    stat = np.argwhere(np.asarray(grid) != 0) if grid is not None else np.zeros((0, 2))
    obs = np.asarray(obstacles, dtype=float).reshape(-1, 2)
    movers = [o for o in obs if grid is None or not (0 <= int(round(o[0])) < len(grid) and 0 <= int(round(o[1])) < len(grid[0])) or grid[int(round(o[0]))][int(round(o[1]))] == 0]
    if cmd is not None:
        x, y, th = pose
        v, om = cmd
        for _ in range(5):
            x += v * math.cos(th) * 0.1; y += v * math.sin(th) * 0.1; th += om * 0.1
            if len(stat):
                stats['static'] = min(stats['static'], float(np.min(np.hypot(stat[:, 0] - x, stat[:, 1] - y))))
            for m in movers:
                stats['hazard'] = min(stats['hazard'], math.hypot(m[0] - x, m[1] - y))
    return cmd
dwa_mod.DWAPlanner.plan = plan
for name in dir(mod):
    pass
tests = None
import types
src = mod.main.__code__
# rebuild test list by calling main's list via --list parsing is awkward; recreate order from source
order = [n for n in mod.main.__code__.co_names if n.startswith('test_')]
fn = getattr(mod, order[idx - 1])
res = fn()
print(f"{suite} #{idx} {fn.__name__}: passed={res} min_static_clear={stats['static']:.3f} min_hazard_clear={stats['hazard']:.3f}")
