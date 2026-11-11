"""Frames/units between path_planner (world metres), robot_controller and DWA
(cell-indexed grid) with the real warehouse map at 0.05 m/cell; DWA timing."""
import sys, io, contextlib, time, math
sys.path.insert(0, '.')
import numpy as np
import navi_bot.mock_ros2 as m
from navi_bot.map_publisher import parse_map_yaml, read_pgm, classify
from navi_bot.ros_compat import occupancy_to_grid
from navi_bot.utils.geometry import world_to_grid, grid_to_world
from navi_bot.planners.dwa import DWAPlanner
from navi_bot.planners.astar import AStarPlanner

spec = parse_map_yaml('maps/warehouse_simple.yaml')
occ = classify(read_pgm('maps/' + spec['image']), spec)
grid = ((occ >= 50) | (occ == -1)).astype(int)
res = spec['resolution']
print('grid shape', grid.shape, 'res', res)

# A known-occupied world point: the bottom perimeter wall at world (x=5.0, y=0.05)
wx, wy = 5.0, 0.025
r, c = world_to_grid(wx, wy, (0, 0), res)
print(f'world ({wx},{wy}) -> cell (row {r}, col {c}) occupied={grid[r][c]}')
dwa = DWAPlanner(); dwa.set_occupancy_grid(grid)
print('dwa.is_dynamic_obstacle((x=5.0,y=0.025)) ->', dwa.is_dynamic_obstacle((wx, wy)),
      '  (DWA reads it as row', int(round(wx)), 'col', int(round(wy)), '-> grid', grid[int(round(wx))][int(round(wy))], ')')
# left shelf band at x in [0.5,1.5], y in [2,4]
wx, wy = 1.0, 3.0
r, c = world_to_grid(wx, wy, (0, 0), res)
print(f'shelf world ({wx},{wy}) -> cell ({r},{c}) occupied={grid[r][c]}; dwa says dynamic={dwa.is_dynamic_obstacle((wx, wy))}')

# A free world point being read as static:
free_pts = [(x/10, y/10) for x in range(1, 100) for y in range(1, 100)]
wrong_static = [(x, y) for x, y in free_pts
                if grid[world_to_grid(x, y, (0, 0), res)] == 0 and not dwa.is_dynamic_obstacle((x, y))]
print('free world points DWA classifies as static wall (sample):', wrong_static[:3], 'count', len(wrong_static))

# Path in world metres as path_planner publishes it, timing a DWA plan call
ap = AStarPlanner(); ap.set_occupancy_grid(grid)
start_w, goal_w = (3.0, 2.0), (7.0, 2.0)
s = world_to_grid(*start_w, (0, 0), res); g = world_to_grid(*goal_w, (0, 0), res)
path = ap.plan(s, g)
world_path = [grid_to_world(p[0], p[1], (0, 0), res) for p in path]
print('A* path cells', len(path), 'world waypoints', len(world_path))
dwa.set_global_path(world_path)
pose = (3.0, 2.0, 0.0)
times = []
for _ in range(5):
    t0 = time.perf_counter(); cmd = dwa.plan(pose, (0.0, 0.0), (3.9, 2.0), []); times.append(time.perf_counter() - t0)
print('DWA plan() wall time ms (no obstacles, world-metre path):', [round(t*1000, 1) for t in times], 'cmd', cmd)
obs = [(3.0 + 1.2*math.cos(a), 2.0 + 1.2*math.sin(a)) for a in np.linspace(-1, 1, 40)]
times = []
for _ in range(3):
    t0 = time.perf_counter(); cmd = dwa.plan(pose, (0.0, 0.0), (3.9, 2.0), obs); times.append(time.perf_counter() - t0)
print('DWA plan() wall time ms (40 LIDAR obstacles):', [round(t*1000, 1) for t in times])
# is_pose_inbounds treats x as row-extent and y as col-extent
print('is_pose_inbounds(5.0, 2.0) on 200x200 grid:', dwa.is_pose_inbounds(5.0, 2.0),
      '; a world point off the map (x=12 m, y=12 m):', dwa.is_pose_inbounds(12.0, 12.0))
