import sys, math, random, time, heapq, logging
sys.path.insert(0, '.')
logging.disable(logging.CRITICAL)
from navi_bot.planners.astar import AStarPlanner
import numpy as np

SQ = math.sqrt(2)
DIRS = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]

def dijkstra(grid, s, g):
    R, C = len(grid), len(grid[0])
    if grid[s[0]][s[1]] or grid[g[0]][g[1]]:
        return None
    dist = {s: 0.0}; pq = [(0.0, s)]
    while pq:
        d, u = heapq.heappop(pq)
        if d > dist.get(u, 1e18): continue
        if u == g: return d
        for dr, dc in DIRS:
            r, c = u[0]+dr, u[1]+dc
            if not (0 <= r < R and 0 <= c < C) or grid[r][c]: continue
            if dr and dc:
                if grid[u[0]+dr][u[1]] or grid[u[0]][u[1]+dc]: continue
                w = SQ
            else: w = 1.0
            nd = d + w
            if nd < dist.get((r,c), 1e18):
                dist[(r,c)] = nd; heapq.heappush(pq, (nd, (r,c)))
    return None

def pathcost(p):
    return sum(math.hypot(p[i+1][0]-p[i][0], p[i+1][1]-p[i][1]) for i in range(len(p)-1))

def section(t): print('\n==', t)

a = AStarPlanner()
section('validation order')
print('no grid, plan((0,0),(3,3)) ->', a.plan((0,0),(3,3)))
print('no grid, start==goal ->', a.plan((5,5),(5,5)))
g = [[0,0,0],[0,1,0],[0,0,0]]
a.set_occupancy_grid(g)
print('occupied start (1,1) ->', a.plan((1,1),(0,0)))
print('occupied goal (1,1) ->', a.plan((0,0),(1,1)))
print('occupied start==goal (1,1) ->', a.plan((1,1),(1,1)))
print('list start [0,0] ->', end=' ')
try: print(a.plan([0,0],(2,2)))
except Exception as e: print(type(e).__name__, e)
print('list goal [2,2] ->', end=' ')
try: print(a.plan((0,0),[2,2]))
except Exception as e: print(type(e).__name__, e)
print('float start (0.0,0.0) ->', end=' ')
try: print(a.plan((0.0,0.0),(2,2)))
except Exception as e: print(type(e).__name__, e)
print('float start (0.4,0.0) ->', end=' ')
try: print(a.plan((0.4,0.0),(2,2)))
except Exception as e: print(type(e).__name__, e)
print('numpy int start ->', a.plan((np.int64(0),np.int64(0)),(2,2)))
a.set_occupancy_grid([])
print('empty grid [] ->', end=' ')
try: print(a.plan((0,0),(0,0)), a.plan((0,0),(1,1)))
except Exception as e: print(type(e).__name__, e)
a.set_occupancy_grid([[0,0,0],[0]])
print('ragged grid ->', end=' ')
try: print(a.plan((0,0),(1,2)))
except Exception as e: print(type(e).__name__, e)
a.set_occupancy_grid(np.zeros((3,3)))
print('negative index via valid check: is_coord_valid(-1,0)=', a.is_coord_valid(-1,0))

section('optimality fuzz vs Dijkstra')
random.seed(1)
bad = 0; reach_mismatch = 0; N = 3000
for t in range(N):
    R, C = random.randint(2, 12), random.randint(2, 12)
    p = random.random()*0.45
    grid = [[1 if random.random() < p else 0 for _ in range(C)] for _ in range(R)]
    s = (random.randrange(R), random.randrange(C)); gl = (random.randrange(R), random.randrange(C))
    grid[s[0]][s[1]] = 0; grid[gl[0]][gl[1]] = 0
    a = AStarPlanner(); a.set_occupancy_grid(grid)
    path = a.plan(s, gl); ref = dijkstra(grid, s, gl)
    if (path is None) != (ref is None): reach_mismatch += 1; continue
    if path and abs(pathcost(path) - ref) > 1e-9: bad += 1
print(f'{N} grids: cost mismatches={bad}, reachability mismatches={reach_mismatch}')

section('timing 200x200')
grid = np.zeros((200,200), dtype=int)
grid[:, 100] = 1  # full wall -> unreachable
a = AStarPlanner(); a.set_occupancy_grid(grid)
t0 = time.perf_counter(); r = a.plan((0,0),(199,199)); t1 = time.perf_counter()
print(f'unreachable numpy 200x200: result={r} time={1000*(t1-t0):.1f} ms')
a.set_occupancy_grid(grid.tolist())
t0 = time.perf_counter(); r = a.plan((0,0),(199,199)); t1 = time.perf_counter()
print(f'unreachable list 200x200: result={r} time={1000*(t1-t0):.1f} ms')
grid2 = np.zeros((200,200), dtype=int)
a.set_occupancy_grid(grid2)
t0 = time.perf_counter(); r = a.plan((0,0),(199,199)); t1 = time.perf_counter()
print(f'open numpy 200x200 corner-corner: len={len(r)} time={1000*(t1-t0):.1f} ms')
