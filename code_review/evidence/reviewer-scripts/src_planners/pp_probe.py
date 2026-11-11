import sys, time, math, logging, io, contextlib
sys.path.insert(0, '.')
logging.disable(logging.CRITICAL)
import numpy as np
from navi_bot.ros_compat import USING_REAL_ROS
import navi_bot.mock_ros2 as m
from navi_bot.path_planner import PathPlannerNode
from navi_bot.map_publisher import parse_map_yaml, read_pgm, classify
from types import SimpleNamespace
print('USING_REAL_ROS', USING_REAL_ROS)

spec = parse_map_yaml('maps/warehouse_simple.yaml')
occ = classify(read_pgm('maps/' + spec['image']), spec)
grid = ((occ >= 50) | (occ == -1)).astype(int)
print('map shape', grid.shape, 'occupied cells', grid.sum(), 'res', spec['resolution'])

def map_msg():
    h, w = occ.shape
    return SimpleNamespace(data=[int(v) for v in occ.ravel()], info=SimpleNamespace(resolution=spec['resolution'], width=w, height=h, origin=SimpleNamespace(position=SimpleNamespace(x=spec['origin'][0], y=spec['origin'][1]))))

def pose(x, y, th=0.0):
    p = m.Pose2D(); p.x, p.y, p.theta = x, y, th; return p

quiet = contextlib.redirect_stdout(io.StringIO())

print('\n== goal arrives BEFORE first map')
m.init()
with quiet:
    n = PathPlannerNode()
    n.goal_callback(pose(8.0, 8.0))
print('current_goal stored (cell) =', n.current_goal, ' (expected at 0.05 m/cell: (160,160))')
with quiet:
    n.map_callback(map_msg()); n.current_pose_callback(pose(1.0, 1.0))
print('after map: current_goal still', n.current_goal)
try:
    with quiet: n.planning_loop()
    print('path', n.current_path)
except Exception as e:
    print('planning_loop raised', repr(e))

print('\n== goal + pose before any map (planning_loop with no grid)')
m.init()
with quiet:
    n = PathPlannerNode(); n.goal_callback(pose(3, 3)); n.current_pose_callback(pose(1, 1))
try:
    with quiet: n.planning_loop()
    print('no exception; current_path', n.current_path)
except Exception as e:
    print('planning_loop raised', type(e).__name__, e)

print('\n== real warehouse map: A* timing + clearance, and the D* section time')
pairs = [((1.0, 1.0), (8.0, 8.0)), ((0.5, 0.5), (9.5, 9.5)), ((2.0, 1.0), (2.0, 8.0)), ((5.0, 1.0), (5.0, 8.0)), ((0.5, 0.5), (5.0, 8.0))]
occupied = np.argwhere(grid == 1)
occ_xy = np.stack([(occupied[:,1]+0.5)*spec['resolution'], (occupied[:,0]+0.5)*spec['resolution']], axis=1)
for (sx, sy), (gx, gy) in pairs:
    m.init()
    with quiet:
        n = PathPlannerNode(); n.map_callback(map_msg())
        n.current_pose_callback(pose(sx, sy)); n.goal_callback(pose(gx, gy))
    s = (int(sy/0.05), int(sx/0.05)); g = n.current_goal
    t0 = time.perf_counter(); path = n.global_planner.plan(s, g); ta = time.perf_counter() - t0
    t0 = time.perf_counter()
    with quiet: n.planning_loop()
    tl = time.perf_counter() - t0
    if path:
        wp = np.array([((c+0.5)*0.05, (r+0.5)*0.05) for r, c in path])
        dmin = min(np.min(np.hypot(occ_xy[:,0]-x, occ_xy[:,1]-y)) for x, y in wp) - 0.025  # to cell edge (approx)
        print(f'({sx},{sy})->({gx},{gy}): A* {1000*ta:.0f} ms, len {len(path)}, min centre-to-occupied-cell-edge clearance {dmin:.3f} m; whole planning_loop {1000*tl:.0f} ms; deadline_misses={n.planning_deadline_misses}')
    else:
        print(f'({sx},{sy})->({gx},{gy}): A* None in {1000*ta:.0f} ms (start occ={grid[s]}, goal occ={grid[g] if 0<=g[0]<200 and 0<=g[1]<200 else "oob"}); planning_loop {1000*tl:.0f} ms')

print('\n== unreachable goal: work repeated each tick')
m.init()
with quiet:
    n = PathPlannerNode(); n.map_callback(map_msg())
    n.current_pose_callback(pose(1.0, 1.0))
# pick an occupied goal cell
r, c = occupied[len(occupied)//2]
with quiet: n.goal_callback(pose((c+0.5)*0.05, (r+0.5)*0.05))
for k in range(3):
    t0 = time.perf_counter()
    with quiet: n.planning_loop()
    print(f'tick {k}: {1000*(time.perf_counter()-t0):.0f} ms, replanning_needed={n.replanning_needed}, deadline_misses={n.planning_deadline_misses}')

print('\n== map update after path: does it trigger replan?')
m.init()
with quiet:
    n = PathPlannerNode(); n.map_callback(map_msg()); n.current_pose_callback(pose(1.0,1.0)); n.goal_callback(pose(8.0,8.0)); n.planning_loop()
print('replanning_needed after plan:', n.replanning_needed)
with quiet: n.map_callback(map_msg())
print('replanning_needed after new map:', n.replanning_needed)

print('\n== D* result usage: after planning_loop, is DStar output referenced anywhere?')
import inspect, navi_bot.path_planner as pp
src = inspect.getsource(pp)
print('DStar_local_planner.plan( used:', 'DStar_local_planner.plan(' in src, '; DWA_local_planner used beyond ctor:', src.count('DWA_local_planner'))
