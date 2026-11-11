import random
from collections import deque
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
