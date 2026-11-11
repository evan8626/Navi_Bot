import sys, math, warnings
sys.path.insert(0, '.')
warnings.simplefilter('ignore')
import numpy as np
from navi_bot.sensors.lidar_processing import LidarProcessor, CLUSTER_EPS, CLUSTER_MIN_SAMPLES
from navi_bot.mock_ros2 import LaserScan, OccupancyGrid
from navi_bot.utils.geometry import world_to_grid

lp = LidarProcessor()
inc = math.pi / 180

def scan(fn, n=360):
    s = LaserScan(); s.angle_min = -math.pi; s.angle_increment = inc
    s.ranges = [fn(-math.pi + i*inc) for i in range(n)]
    return s

print('=== DBSCAN at 1 deg ===')
# Point spacing at range r is ~ r*inc. A point needs >= 9 neighbours within eps=0.3 (min_samples=10 incl. itself)
for r in (1, 2, 3, 5, 8.6, 9, 10):
    spacing = r*inc
    n_in_eps = 2*int(CLUSTER_EPS/spacing) + 1
    print(f'range {r:>4} m: spacing {spacing:.3f} m, points within eps along an arc ~{n_in_eps} -> core possible: {n_in_eps >= CLUSTER_MIN_SAMPLES}')
# width of an object at range r needed to have 10 returns
for r in (1.0, 2.0, 5.0):
    print(f'  at {r} m a cluster needs >= {CLUSTER_MIN_SAMPLES} returns => object >= {(CLUSTER_MIN_SAMPLES-1)*r*inc:.2f} m wide (a 0.3 m pole/leg is invisible)')
for r, w in ((1.0, 0.3), (2.0, 0.3), (1.0, 0.1), (3.0, 0.5)):
    half = math.atan2(w/2, r)
    res = lp.process_scan(scan(lambda a: r if abs(a) <= half else 20.0))
    print(f'object {w} m wide at {r} m -> clusters {len(res["obstacles"])}  (returns {int(2*half/inc)+1})')
# far wall: range 9.5 m, 60 deg wide
res = lp.process_scan(scan(lambda a: 9.5 if abs(a) < math.radians(30) else 20.0))
print('60 deg wide wall at 9.5 m -> clusters', len(res['obstacles']))
# reading exactly at max_range (valid) dropped by polar_to_cartesian
res = lp.process_scan(scan(lambda a: 10.0 if abs(a) < math.radians(30) else 20.0))
print('valid returns at exactly range_max=10.0:', res['num_valid_points'], '-> clusters', len(res['obstacles']))
# long wall -> single centroid off the wall
res = lp.process_scan(scan(lambda a: (1.0/math.cos(a)) if abs(a) < math.radians(60) else 20.0))
c = res['obstacles'][0]['centroid']
print('straight wall x=1.0 spanning +-60 deg -> clusters', len(res['obstacles']), 'centroid', np.round(c, 3), '(wall endpoints at y=+-1.73)')
# U-shaped / ring around robot: centroid at robot origin
res = lp.process_scan(scan(lambda a: 1.0))
print('ring at 1 m all round -> centroid', np.round(res['obstacles'][0]['centroid'], 3), 'size', res['obstacles'][0]['size'])

print('=== clear directions ===')
s = scan(lambda a: 0.3 if abs(abs(a) - math.pi) < math.radians(5) else 8.0)   # obstacle straddling +-pi (behind)
res = lp.process_scan(s)
cd = res['clear_directions']
print('obstacle within 5 deg of the +-pi seam: index 0 (=-pi) clear?', cd[0], ' index 20 clear?', cd[20], ' index 345 clear?', cd[345],
      ' index 340?', cd[340])
# index 14 should be blocked since obstacle at index 0..4 within 15
print('  index 10 blocked?', not cd[10], ' index 350 blocked?', not cd[350], ' (window does not wrap: 350 sees 335..359 only, 355..359 blocked)')
s.angle_increment = 2*math.pi/720; s.ranges = [8.0]*720
s.ranges[360] = 0.3   # one return dead-ahead
cd = lp.process_scan(s)['clear_directions']
print('0.5 deg scan, single return ahead: blocked span =', (~cd).sum()*0.5, 'deg (code intends +-15 deg = 31 deg)')

print('=== costmap ===')
cm = OccupancyGrid(width=100, height=100, resolution=0.05)
lp.update_costmap([{'centroid': np.array([1.0, 2.0])}], cm)   # robot-frame point x=1, y=2
g = np.array(cm.data).reshape(100, 100)
rows, cols = np.nonzero(g)
print('centroid (x=1.0, y=2.0) marked around row', int(rows.mean()), 'col', int(cols.mean()),
      '; utils.geometry.world_to_grid gives', world_to_grid(1.0, 2.0, (0, 0), 0.05))
cm = OccupancyGrid(width=100, height=100, resolution=0.05)
lp.update_costmap([{'centroid': np.array([-1.0, 1.0])}], cm)
g = np.array(cm.data).reshape(100, 100); rows, cols = np.nonzero(g)
print('robot-frame obstacle BEHIND (x=-1, y=1): cells marked', len(rows), '(row', int(-1.0/0.05), 'out of range, silently dropped)')
cm = OccupancyGrid(width=100, height=100, resolution=0.05)
lp.update_costmap([{'centroid': np.array([-0.04, 1.0])}], cm)
g = np.array(cm.data).reshape(100, 100); rows, cols = np.nonzero(g)
print('x=-0.04 -> int() truncates toward 0 => row', int(-0.04/0.05), '(floor would be -1); marked cells', len(rows))
lp.update_costmap([], cm)
print('update with no obstacles keeps old marks (never clears):', int(np.count_nonzero(cm.data)))
