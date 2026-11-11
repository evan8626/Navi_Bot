import sys, math, time, threading
sys.path.insert(0, '.')
import numpy as np
from navi_bot.utils.geometry import normalize_angle, point_in_polygon, world_to_grid, grid_to_world, interpolate_path
from navi_bot.control.path_follower import PathFollower

print('== normalize_angle')
t0 = time.perf_counter(); r = normalize_angle(1e7); print('1e7 ->', r, f'{1000*(time.perf_counter()-t0):.0f} ms')
print('3*pi ->', normalize_angle(3*math.pi), ' -pi ->', normalize_angle(-math.pi), ' pi ->', normalize_angle(math.pi))
print('nan ->', normalize_angle(float('nan')))
res = {}
def run(): res['v'] = normalize_angle(float('inf'))
th = threading.Thread(target=run, daemon=True); th.start(); th.join(2.0)
print('inf -> still running after 2 s (infinite loop):', th.is_alive())

print('\n== point_in_polygon')
sq = [(0,0),(2,0),(2,2),(0,2)]
for p in [(1,1),(0,1),(2,1),(1,0),(1,2),(0,0),(2,2),(2,0),(0,2)]:
    print(' ', p, point_in_polygon(p, sq))
# xinters unbound: need first edge horizontal with y>min and y<=max -> impossible for horizontal (min==max). Try degenerate ordering
try:
    print('  horizontal-first poly (y on edge level):', point_in_polygon((5, 0), [(0,0),(4,0),(4,4)]))
except Exception as e: print('  exception', repr(e))
try:
    print('  empty polygon:', point_in_polygon((0,0), []))
except Exception as e: print('  empty polygon ->', repr(e))
# stale xinters: horizontal edge after a sloped edge reuses old xinters
print('  stale-xinters probe:', point_in_polygon((3, 1), [(0,0),(4,2),(1,1)]))

print('\n== world_to_grid / grid_to_world')
o = (0.0, 0.0); res_ = 0.05
for x in [0.05, 0.1, 0.15, 0.3, 0.35, 0.7, 1.15, -0.05, -0.0001]:
    r, c = world_to_grid(x, 0.0, o, res_)
    print(f'  x={x}: col={c}  expected floor={math.floor(round(x/res_, 9))}')
print('  round trip cell (7,3):', world_to_grid(*grid_to_world(7, 3, o, res_), o, res_))
bad = [(k) for k in range(2000) if world_to_grid(k*0.05, 0, o, 0.05)[1] != k]
print(f'  exact cell-border values k*0.05 mapping to k-1: {len(bad)} of 2000; first few {bad[:8]}')
print('  resolution 0 ->', end=' ')
try: print(world_to_grid(1, 1, o, 0))
except Exception as e: print(repr(e))

print('\n== interpolate_path')
w = [(0, 0)]
out = interpolate_path(w); print('  single wp returns same object:', out is w)
print('  resolution 0 ->', end=' ')
try: print(interpolate_path([(0,0),(1,0)], 0))
except Exception as e: print(repr(e))
print('  resolution -0.1 ->', interpolate_path([(0,0),(1,0)], -0.1))
print('  dist 0.25 res 0.1 ->', interpolate_path([(0,0),(0.25,0)], 0.1))
print('  duplicate wp ->', interpolate_path([(0,0),(0,0),(0.3,0)], 0.1))

print('\n== PathFollower: target behind robot')
f = PathFollower()   # wp_tol 0.6 lookahead 1.0
f.set_path([(0,0),(1,0),(2,0),(3,0),(4,0),(5,0)])
x, y = 3.0, 0.9   # robot drifted sideways 0.9 near wp (3,0); first wps (0,0)..(2,0) are > wp_tol away
print('  robot at (3.0, 0.9), heading +x: target ->', f.target(x, y), ' wp_idx', f.wp_idx)
f2 = PathFollower(); f2.set_path([(0,0),(1,0),(2,0),(3,0),(4,0),(5,0)])
print('  robot jumps to (4.5, 0) (e.g. pushed past): target ->', f2.target(4.5, 0.0))
f3 = PathFollower(); f3.set_path([(0,0),(5,0),(5,0.5),(0.3,0.5)])
print('  loop path end near start: goal_reached at start (0,0):', f3.goal_reached(0.0, 0.0))
print('  in production metres (0.05 m A* waypoints): wp_tol=0.6 spans', int(0.6/0.05), 'waypoints; lookahead 1.0 m =', int(1.0/0.05), 'waypoints; goal_tol 0.7 m vs config goal_distance_tolerance 0.1 m')
