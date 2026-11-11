import sys, math, random, time, logging, copy
from collections import deque
sys.path.insert(0, '.')
logging.disable(logging.CRITICAL)
from navi_bot.planners.dstar_lite import DStarLitePlanner, U
import numpy as np

DIRS = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
def bfs(grid, s, g):
    R, C = len(grid), len(grid[0])
    if grid[s[0]][s[1]] or grid[g[0]][g[1]]: return None
    d = {s: 0}; q = deque([s])
    while q:
        u = q.popleft()
        if u == g: return d[u]
        for dr, dc in DIRS:
            r, c = u[0]+dr, u[1]+dc
            if not (0 <= r < R and 0 <= c < C) or grid[r][c]: continue
            if dr and dc and (grid[u[0]+dr][u[1]] or grid[u[0]][u[1]+dc]): continue
            if (r,c) not in d: d[(r,c)] = d[u]+1; q.append((r,c))
    return None

def valid_path(grid, p, s, g):
    if p[0] != s or p[-1] != g: return False
    for a, b in zip(p, p[1:]):
        dr, dc = b[0]-a[0], b[1]-a[1]
        if max(abs(dr),abs(dc)) != 1 or grid[b[0]][b[1]]: return False
        if dr and dc and (grid[a[0]+dr][a[1]] or grid[a[0]][a[1]+dc]): return False
    return True

def rnd_grid(R, C, p):
    return [[1 if random.random() < p else 0 for _ in range(C)] for _ in range(R)]

print('== (a) single-shot fuzz vs BFS (same cost model)')
random.seed(7)
stats = dict(n=0, longer=0, none_when_exists=0, exc=0, invalid=0)
ex_longer = None; ex_none = None; ex_exc = None
for t in range(3000):
    R, C = random.randint(2, 9), random.randint(2, 9)
    grid = rnd_grid(R, C, random.random()*0.4)
    s = (random.randrange(R), random.randrange(C)); gl = (random.randrange(R), random.randrange(C))
    grid[s[0]][s[1]] = 0; grid[gl[0]][gl[1]] = 0
    if s == gl: continue
    stats['n'] += 1
    d = DStarLitePlanner(); d.set_occupancy_grid(grid)
    ref = bfs(grid, s, gl)
    try:
        p = d.plan(s, gl)
    except Exception as e:
        stats['exc'] += 1; ex_exc = ex_exc or (grid, s, gl, repr(e)); continue
    if p is None:
        if ref is not None:
            stats['none_when_exists'] += 1
            if ex_none is None or R*C < len(ex_none[0])*len(ex_none[0][0]): ex_none = (grid, s, gl, ref)
        continue
    if not valid_path(grid, p, s, gl): stats['invalid'] += 1; continue
    if len(p)-1 > ref:
        stats['longer'] += 1
        if ex_longer is None or R*C < len(ex_longer[0])*len(ex_longer[0][0]): ex_longer = (grid, s, gl, ref, p)
print(stats)
print('min longer example:', ex_longer)
print('min none example:', ex_none)
print('exc example:', ex_exc)

print('\n== heuristic admissibility: open 5x5, (0,0)->(4,4)')
d = DStarLitePlanner(); d.set_occupancy_grid([[0]*5 for _ in range(5)])
p = d.plan((0,0),(4,4)); print('path', p, 'true cost', bfs([[0]*5 for _ in range(5)],(0,0),(4,4)), 'Manhattan h', 8)

print('\n== (b) incremental fuzz: plan, change grid, move start, plan  vs fresh planner')
random.seed(11)
inc = dict(n=0, reach_mismatch=0, cost_mismatch=0, exc=0, fresh_wrong=0)
ex_inc = None
for t in range(1500):
    R, C = random.randint(3, 9), random.randint(3, 9)
    g1 = rnd_grid(R, C, 0.25)
    s = (random.randrange(R), random.randrange(C)); gl = (random.randrange(R), random.randrange(C))
    if s == gl: continue
    g1[s[0]][s[1]] = 0; g1[gl[0]][gl[1]] = 0
    d = DStarLitePlanner(); d.set_occupancy_grid(g1)
    try: p1 = d.plan(s, gl)
    except Exception: continue
    if not p1 or len(p1) < 2: continue
    s2 = p1[1]
    g2 = copy.deepcopy(g1)
    for _ in range(random.randint(1, 4)):
        r, c = random.randrange(R), random.randrange(C)
        g2[r][c] ^= 1
    g2[s2[0]][s2[1]] = 0; g2[gl[0]][gl[1]] = 0
    if s2 == gl: continue
    inc['n'] += 1
    d.set_occupancy_grid(g2)
    ref = bfs(g2, s2, gl)
    try: p2 = d.plan(s2, gl)
    except Exception as e: inc['exc'] += 1; ex_inc = ex_inc or ('exc', g1, g2, s, s2, gl, repr(e)); continue
    if (p2 is None) != (ref is None):
        inc['reach_mismatch'] += 1; ex_inc = ex_inc or ('reach', g1, g2, s, s2, gl, ref, p2); continue
    if p2 and (len(p2)-1 != ref or not valid_path(g2, p2, s2, gl)):
        inc['cost_mismatch'] += 1
print(inc); print('example:', ex_inc)

print('\n== same object mutated in place')
g = [[0]*5 for _ in range(5)]
d = DStarLitePlanner(); d.set_occupancy_grid(g)
p1 = d.plan((0,2),(4,2)); print('p1', p1)
g[2][1]=g[2][2]=g[2][3]=1  # wall across middle, mutate in place
d.set_occupancy_grid(g)
print('previous_grid is occupancy_grid:', d.previous_grid is d.occupancy_grid)
try: p2 = d.plan((1,2),(4,2)); print('p2 (after in-place wall)', p2, 'valid?', p2 and valid_path(g,p2,(1,2),(4,2)), 'bfs', bfs(g,(1,2),(4,2)))
except Exception as e: print('p2 exc', repr(e))
fresh = DStarLitePlanner(); fresh.set_occupancy_grid(g); print('fresh', fresh.plan((1,2),(4,2)))

print('\n== set_occupancy_grid twice between plans')
g0 = [[0]*5 for _ in range(5)]
d = DStarLitePlanner(); d.set_occupancy_grid(g0)
print('p1', d.plan((0,2),(4,2)))
g1 = copy.deepcopy(g0); g1[2] = [0,1,1,1,1]
d.set_occupancy_grid(g1)
d.set_occupancy_grid(copy.deepcopy(g1))   # e.g. a second identical map message
p2 = d.plan((0,2),(4,2)); print('p2', p2, 'valid?', bool(p2) and valid_path(g1,p2,(0,2),(4,2)), 'bfs', bfs(g1,(0,2),(4,2)))

print('\n== obstacle removed (shortcut opens)')
g0 = [[0]*5 for _ in range(5)]; g0[2] = [1,1,1,1,0]
d = DStarLitePlanner(); d.set_occupancy_grid(g0)
p1 = d.plan((0,0),(4,0)); print('p1', p1, len(p1)-1)
g1 = copy.deepcopy(g0); g1[2][0] = 0
d.set_occupancy_grid(g1)
p2 = d.plan((0,0),(4,0)); print('p2', p2, len(p2)-1 if p2 else None, 'bfs', bfs(g1,(0,0),(4,0)))

print('\n== stale diff re-processed: edge_changed after a later plan with no grid change')
print('edges reported on 3rd plan without new grid:', len(d.edge_changed()))

print('\n== grid shape change')
d = DStarLitePlanner(); d.set_occupancy_grid([[0]*5 for _ in range(5)])
d.plan((0,0),(4,4))
d.set_occupancy_grid([[0]*8 for _ in range(8)])
try: print('bigger grid plan', d.plan((0,0),(4,4)))
except Exception as e: print('bigger grid exc', repr(e))
d = DStarLitePlanner(); d.set_occupancy_grid([[0]*8 for _ in range(8)])
d.plan((0,0),(4,4))
d.set_occupancy_grid([[0]*5 for _ in range(5)])
try: print('smaller grid plan', d.plan((0,0),(4,4)))
except Exception as e: print('smaller grid exc', repr(e))

print('\n== start becomes occupied / goal becomes occupied')
d = DStarLitePlanner(); g = [[0]*5 for _ in range(5)]; d.set_occupancy_grid(g)
d.plan((0,0),(4,4)); g2 = copy.deepcopy(g); g2[0][0]=1; d.set_occupancy_grid(g2)
print('start occupied ->', d.plan((0,0),(4,4)))
g3 = copy.deepcopy(g); g3[4][4]=1; d.set_occupancy_grid(g3)
print('goal occupied ->', d.plan((0,0),(4,4)))

print('\n== (c) U queue + call-order hazards')
q = U(); print('empty Top', q.Top(), 'TopKey', q.TopKey())
d = DStarLitePlanner(); d.set_occupancy_grid([[0]*3 for _ in range(3)])
for fn, args in [('update_vertex', ((1,1),)), ('calculate_key', ((1,1),(0,0)))]:
    try: print(fn, '->', getattr(d, fn)(*args))
    except Exception as e: print(fn, 'before init ->', repr(e))
# empty-queue with inconsistent start
d = DStarLitePlanner(); d.set_occupancy_grid([[0]*3 for _ in range(3)])
d.d_star_initialize((0,0),(2,2)); d.U = U(); d.g_values[(0,0)] = 5.0
try: d.compute_shortest_path((0,0),(2,2),None); print('no crash')
except Exception as e: print('compute_shortest_path on empty U with g!=rhs ->', repr(e))

print('\n== (e) timing 200x200')
for desc, mk in [('open', lambda: np.zeros((200,200),dtype=int))]:
    G = mk(); d = DStarLitePlanner(); d.set_occupancy_grid(G)
    t0=time.perf_counter(); p = d.plan((0,0),(199,199)); t1=time.perf_counter()
    print(f'initial plan open 200x200: len={len(p) if p else None} {1000*(t1-t0):.0f} ms, U size={len(d.U.elements)}, g entries={len(d.g_values)}')
    G2 = G.copy(); G2[50:60, 40:60] = 1
    d.set_occupancy_grid(G2)
    t0=time.perf_counter(); p = d.plan(p[1],(199,199)); t1=time.perf_counter()
    print(f'replan after 10x20 block: len={len(p) if p else None} {1000*(t1-t0):.0f} ms')
    t0=time.perf_counter(); p = d.plan(p[1],(199,199)); t1=time.perf_counter()
    print(f'replan with no new grid (stale diff): {1000*(t1-t0):.0f} ms')
G = np.zeros((60,60),dtype=int); d = DStarLitePlanner(); d.set_occupancy_grid(G)
t0=time.perf_counter(); d.plan((0,0),(59,59)); print(f'60x60 initial {1000*(time.perf_counter()-t0):.0f} ms')
