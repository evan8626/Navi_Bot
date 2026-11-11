import math
import numpy as np
from navi_bot.sensors.lidar_processing import LidarProcessor
from navi_bot.mock_ros2 import LaserScan, OccupancyGrid

p = LidarProcessor()
INC = np.pi / 180.0

def scan_with(width_idx, dist, start=170):
    r = np.full(360, np.inf)
    r[start:start + width_idx] = dist
    return r

print("== DBSCAN detection vs range for a 10-beam-wide (10 deg) arc ==")
for d in [1.0, 2.0, 3.0, 3.4, 3.5, 4.0, 6.0, 9.0]:
    res = p.detect_obstacles(scan_with(10, d), -np.pi, INC)
    print(f"  range {d:4.1f} m, 10 beams -> {len(res)} clusters")

print("== 10 cm leg at 1 m (subtends ~5.7 deg -> 6 beams) ==")
for beams in [6, 9, 10, 11]:
    res = p.detect_obstacles(scan_with(beams, 1.0), -np.pi, INC)
    print(f"  {beams} beams at 1.0 m -> {len(res)} clusters")

print("== wide obstacle 100 beams at 4 m (a wall) ==")
res = p.detect_obstacles(scan_with(100, 4.0), -np.pi, INC)
print("  ->", len(res))
print("== straight wall x=2.0, y in [-3,3] (beams hitting it) -> single centroid? ==")
r = np.full(360, np.inf)
for i in range(360):
    a = -np.pi + i * INC
    if math.cos(a) > 1e-6:
        rr = 2.0 / math.cos(a)
        if rr <= 3.0:
            r[i] = rr
res = p.detect_obstacles(r, -np.pi, INC)
for c in res:
    print("  centroid", np.round(c['centroid'], 3), "size", c['size'])

print("== background ring at 9.0 m (test fixture) forms a cluster? ==")
print("  ", len(p.detect_obstacles(np.full(360, 9.0), -np.pi, INC)))

print("== compute_clear_directions wrap-around ==")
r = np.full(360, 9.0); r[359] = 0.3
cd = p.compute_clear_directions(r)
print("  obstacle at idx 359: idx0 clear?", bool(cd[0]), " idx 5 clear?", bool(cd[5]), " idx 345 clear?", bool(cd[345]))
r = np.full(360, 9.0); r[0] = 0.3
cd = p.compute_clear_directions(r)
print("  obstacle at idx 0: idx359 clear?", bool(cd[359]), " idx 15 clear?", bool(cd[15]))
print("== 0.5 deg scan (720 beams): window is only +-7.5 deg ==")
r = np.full(720, 9.0); r[360] = 0.3
cd = p.compute_clear_directions(r)
blocked = np.where(~cd)[0]
print("  blocked span", blocked.min(), blocked.max(), "->", (blocked.max()-blocked.min())*0.5, "deg total")

print("== below-min-range readings treated as clear ==")
s = LaserScan(); s.ranges = [0.05]*360; s.angle_min = -np.pi; s.angle_increment = INC
out = p.process_scan(s)
print("  all readings 0.05 m -> closest", out['closest_obstacle'], " all clear?", bool(np.all(out['clear_directions'])))

print("== update_costmap axis & frame ==")
cm = OccupancyGrid(width=50, height=50, resolution=0.1); cm.data = [0]*2500
p.update_costmap([{'centroid': np.array([1.0, 3.0]), 'size': 10}], cm)
g = np.array(cm.data).reshape(50, 50)
print("  obstacle (x=1.0,y=3.0): cell[row10,col30]=", g[10, 30], " cell[row30,col10]=", g[30, 10])
cm = OccupancyGrid(width=50, height=50, resolution=0.1); cm.data = [0]*2500
p.update_costmap([{'centroid': np.array([-1.0, 1.0]), 'size': 10}], cm)
print("  obstacle behind robot (x=-1.0,y=1.0) -> cells marked:", int(np.sum(np.array(cm.data) == 100)))
cm = OccupancyGrid(width=50, height=50, resolution=0.1); cm.data = [0]*2500
p.update_costmap([{'centroid': np.array([-0.05, 1.0]), 'size': 10}], cm)
g = np.array(cm.data).reshape(50, 50)
print("  obstacle x=-0.05 (int() truncation) -> row0 col10 marked:", g[0, 10])
print("  int(0.25/0.05) =", int(0.25/0.05), " 0.25/0.05 =", 0.25/0.05, " int(2.5/0.05) =", int(2.5/0.05))
