import sys, time, random, copy, logging
sys.path.insert(0, '.')
sys.path.insert(0, r'C:\Users\evan8\AppData\Local\Temp\claude\738ef417e18052f3\0a4a6808-43fe-4936-9d58-6cdc90f47f1a\scratchpad\agents\src_planners')
logging.disable(logging.CRITICAL)
import numpy as np
import navi_bot.planners.dstar_lite as dl
from dstar_probe_lib import bfs, valid_path, rnd_grid

def replan_case():
    G = np.zeros((200,200),dtype=int); d = dl.DStarLitePlanner(); d.set_occupancy_grid(G)
    p = d.plan((0,0),(199,199))
    G2 = G.copy(); G2[50:60, 40:60] = 1
    d.set_occupancy_grid(G2)
    s = p[1]
    t0 = time.perf_counter(); p2 = d.plan(s,(199,199)); dt = time.perf_counter()-t0
    print(f'  replan: {"None" if p2 is None else len(p2)-1} in {dt*1000:.0f} ms; g(start)={d.g_values.get(s)} rhs(start)={d.rhs_values.get(s)} bfs={bfs(G2.tolist(), s, (199,199))}')
    print(f'  TopKey={d.U.TopKey()} key(start)={d.calculate_key(s, s)}')
    return d, s

def inc_fuzz(seed=11, n=1500):
    random.seed(seed); bad = 0; tot = 0
    for t in range(n):
        R, C = random.randint(3, 9), random.randint(3, 9)
        g1 = rnd_grid(R, C, 0.25)
        s = (random.randrange(R), random.randrange(C)); gl = (random.randrange(R), random.randrange(C))
        if s == gl: continue
        g1[s[0]][s[1]] = 0; g1[gl[0]][gl[1]] = 0
        d = dl.DStarLitePlanner(); d.set_occupancy_grid(g1)
        p1 = d.plan(s, gl)
        if not p1 or len(p1) < 2: continue
        s2 = p1[1]; g2 = copy.deepcopy(g1)
        for _ in range(random.randint(1, 4)):
            r, c = random.randrange(R), random.randrange(C); g2[r][c] ^= 1
        g2[s2[0]][s2[1]] = 0; g2[gl[0]][gl[1]] = 0
        if s2 == gl: continue
        tot += 1
        d.set_occupancy_grid(g2); ref = bfs(g2, s2, gl); p2 = d.plan(s2, gl)
        if (p2 is None) != (ref is None) or (p2 and len(p2)-1 != ref): bad += 1
    return bad, tot

def single_fuzz(seed=7, n=3000):
    random.seed(seed); bad = 0; tot = 0
    for t in range(n):
        R, C = random.randint(2, 9), random.randint(2, 9)
        grid = rnd_grid(R, C, random.random()*0.4)
        s = (random.randrange(R), random.randrange(C)); gl = (random.randrange(R), random.randrange(C))
        grid[s[0]][s[1]] = 0; grid[gl[0]][gl[1]] = 0
        if s == gl: continue
        tot += 1
        d = dl.DStarLitePlanner(); d.set_occupancy_grid(grid)
        p = d.plan(s, gl); ref = bfs(grid, s, gl)
        if (p is None) != (ref is None) or (p and len(p)-1 != ref): bad += 1
    return bad, tot

print('== with shipped Manhattan heuristic')
replan_case()
print('  incremental fuzz (bad, total):', inc_fuzz())
print('  single fuzz (bad, total):', single_fuzz())

print('== with Chebyshev heuristic monkeypatched (admissible for unit-cost 8-connected)')
dl.heuristic_backward = lambda a, b: max(abs(a[0]-b[0]), abs(a[1]-b[1]))
replan_case()
print('  incremental fuzz (bad, total):', inc_fuzz())
print('  single fuzz (bad, total):', single_fuzz())

print('== minimal single-shot counterexample re-run (shipped heuristic)')
import importlib; importlib.reload(dl)
grid = [[0,0,0,0,0],[0,0,0,0,0],[0,1,0,0,0],[0,1,0,0,0],[0,0,0,1,0],[0,0,0,0,0],[1,0,1,0,0]]
d = dl.DStarLitePlanner(); d.set_occupancy_grid(grid)
p = d.plan((0,0),(6,3)); print('  D* path cost', len(p)-1, p, ' BFS optimum', bfs(grid,(0,0),(6,3)))
