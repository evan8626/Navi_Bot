import sys, time, math, logging, random
sys.path.insert(0, '.')
logging.disable(logging.CRITICAL)
import numpy as np
from navi_bot.planners.dwa import DWAPlanner, CONTACT
from navi_bot.map_publisher import parse_map_yaml, read_pgm, classify
from navi_bot.planners.astar import AStarPlanner

print('CONTACT =', CONTACT)
print('\n== dynamic window per call vs real 20 ms period')
d = DWAPlanner()
print('window from rest:', d.compute_dynamic_window((0.0, 0.0), 0.5))
d.set_global_path([(0, 0), (0, 50)])
vel = (0.0, 0.0); pose = (5.0, 5.0, math.pi/2); vs = []
for k in range(5):
    cmd = d.plan(pose, vel, (5.0, 40.0), [])
    vs.append(cmd); vel = cmd
print('5 successive plan() calls feeding back last cmd (as robot_controller does):', [(round(a,3), round(b,3)) for a, b in vs])
print(f'implied accel if calls are 20 ms apart: {(vs[0][0]-0)/0.02:.1f} m/s^2 (limit {d.max_accel_x})')

print('\n== window when current v exceeds static limit')
print('compute_dynamic_window((2.0,0),0.5) ->', d.compute_dynamic_window((2.0, 0.0), 0.5))

print('\n== track_movers velocity: same mover moving +1 unit per real 0.1 s, plan() at 50 Hz')
d = DWAPlanner()
truth = 10.0  # units/s
pos = np.array([10.0, 10.0]); out = None
for k in range(10):           # 10 calls at 20 ms -> scan updates every 5th call (10 Hz LIDAR)
    if k % 5 == 0 and k: pos = pos + np.array([1.0, 0.0])
    out = d.track_movers([pos.copy()], 0.5)
print('estimated vel', out[0][1], ' true', [truth, 0.0])

print('\n== track_movers: point cloud association (two movers swap nearest)')
d = DWAPlanner()
d.track_movers([[0, 0], [0, 1]], 0.5)
print(d.track_movers([[0, 1], [0, 2]], 0.5))

print('\n== is_dynamic_obstacle / is_pose_inbounds with production world-metre inputs on the warehouse map')
spec = parse_map_yaml('maps/warehouse_simple.yaml')
occ = classify(read_pgm('maps/' + spec['image']), spec)
grid = ((occ >= 50) | (occ == -1)).astype(int)
d = DWAPlanner(); d.set_occupancy_grid(grid)
occupied = np.argwhere(grid == 1)
wall_world = [((c+0.5)*0.05, (r+0.5)*0.05) for r, c in occupied]
dyn = sum(d.is_dynamic_obstacle(p) for p in wall_world)
print(f'{len(wall_world)} static wall points given in world metres: {dyn} classified as DYNAMIC movers ({100*dyn/len(wall_world):.0f}%)')
print('is_pose_inbounds(-0.2, 5.0) [off-map in metres]:', d.is_pose_inbounds(-0.2, 5.0), '; (50.0, 50.0) [40 m outside 10 m map]:', d.is_pose_inbounds(50.0, 50.0))

print('\n== production-frame plan(): robot on an A* path on the warehouse map, LIDAR-like wall points within 1.5 m')
a = AStarPlanner(); a.set_occupancy_grid(grid)
path = a.plan((20, 20), (160, 160))
wpath = [((c+0.5)*0.05, (r+0.5)*0.05) for r, c in path]
d = DWAPlanner(); d.set_occupancy_grid(grid); d.set_global_path(wpath)
zero = 0; tested = 0
for i in range(5, len(wpath)-20, 15):
    x, y = wpath[i]; nx, ny = wpath[i+5]
    th = math.atan2(ny-y, nx-x)
    obs = [p for p in wall_world if math.hypot(p[0]-x, p[1]-y) <= 1.5]
    cmd = d.plan((x, y, th), (0.3, 0.0), wpath[min(i+20, len(wpath)-1)], obs)
    tested += 1; zero += (cmd == (0.0, 0.0))
    mind = min((math.hypot(p[0]-x, p[1]-y) for p in obs), default=float('inf'))
    print(f'  wp {i}: nearest wall {mind:.2f} m, n_obs={len(obs)}, cmd={tuple(round(float(v),3) for v in cmd)}')
print(f'  (0,0) returned at {zero}/{tested} path poses')

print('\n== timing plan() vs 20 ms period')
for nobs in (0, 20, 100):
    random.seed(0)
    d = DWAPlanner(); d.set_global_path([(0, 0), (0, 10), (10, 10)])
    obs = [(random.uniform(3, 9), random.uniform(3, 9)) for _ in range(nobs)]
    t0 = time.perf_counter(); d.plan((1.0, 1.0, 0.0), (0.2, 0.0), (9.0, 9.0), obs); t = time.perf_counter() - t0
    print(f'  {nobs} obstacles, 3-pt path: {1000*t:.0f} ms')
d = DWAPlanner(); d.set_global_path(wpath)
t0 = time.perf_counter(); d.plan((wpath[5][0], wpath[5][1], 0.8), (0.2, 0.0), wpath[30], []); print(f'  0 obstacles, {len(wpath)}-pt warehouse path: {1000*(time.perf_counter()-t0):.0f} ms')

print('\n== path_cost progress granularity')
d = DWAPlanner(); d.set_global_path([(0, 0), (0, 10)])
print('  2-pt path, points (0,1) and (0,9):', d.path_cost((0, 1)), d.path_cost((0, 9)))
d.set_global_path([(0, 0), (0, 1), (0, 10)])
print('  path (0,0)-(0,1)-(0,10): (0,1.2)->', d.path_cost((0, 1.2)), ' (0,9)->', d.path_cost((0, 9)))
d.set_global_path([(0,0),(0,5),(1,5),(1,0)])  # hairpin
print('  hairpin path: point (0.4, 1) ->', d.path_cost((0.4, 1)), ' point (0.6,1) ->', d.path_cost((0.6, 1)))

print('\n== yield rule: stationary unknown obstacle straight ahead (vel 0)')
d = DWAPlanner()
d.track_movers([], 0.5)
cmd = d.plan((0.0+5, 5.0, 0.0), (0.5, 0.0), (5.0+8, 5.0), [(5.0+2.0, 5.0+3.0)])
print('  off-axis unknown point at 3.6 m ->', cmd)
