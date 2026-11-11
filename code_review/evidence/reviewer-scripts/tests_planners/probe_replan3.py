import sys, logging, random
sys.path.insert(0, r"C:\Users\evan8\Desktop\Navi Bot"); sys.path.insert(0, r"C:\Users\evan8\Desktop\Navi Bot\tests")
logging.disable(logging.CRITICAL)
import numpy as np
import DStar_Lite_planner_test as T
from navi_bot.planners.dstar_lite import DStarLitePlanner
rng=random.Random(7); N=10
nomove=0; cnt=0; single=0; single_fail=0
for trial in range(300):
    g=np.array([[1 if rng.random()<0.2 else 0 for _ in range(N)] for _ in range(N)])
    free=[tuple(map(int,c)) for c in np.argwhere(g==0)]
    s=rng.choice(free); t=rng.choice(free)
    if s==t: continue
    d=DStarLitePlanner(); d.set_occupancy_grid(g); p=d.plan(s,t)
    if not p: continue
    g2=g.copy()
    for _ in range(6):
        r,c=rng.randrange(N),rng.randrange(N)
        if (r,c) not in (s,t): g2[r][c]=1-g2[r][c]
    d.set_occupancy_grid(g2); p2=d.plan(s,t)   # no robot movement
    if T.reference_shortest_cost(g2,s,t) is not None:
        cnt+=1
        if p2 is None: nomove+=1
# single-cell additions only (one obstacle added)
for trial in range(400):
    g=np.array([[1 if rng.random()<0.2 else 0 for _ in range(N)] for _ in range(N)])
    free=[tuple(map(int,c)) for c in np.argwhere(g==0)]
    s=rng.choice(free); t=rng.choice(free)
    if s==t: continue
    d=DStarLitePlanner(); d.set_occupancy_grid(g); p=d.plan(s,t)
    if not p or len(p)<3: continue
    g2=g.copy(); g2[p[len(p)//2]]=1 if p[len(p)//2] not in (s,t) else 0
    d.set_occupancy_grid(g2); p2=d.plan(s,t)
    if T.reference_shortest_cost(g2,s,t) is not None:
        single+=1
        if p2 is None: single_fail+=1
print("no-move replans reachable:",cnt,"returned None:",nomove)
print("single obstacle on path, reachable:",single,"returned None:",single_fail)
