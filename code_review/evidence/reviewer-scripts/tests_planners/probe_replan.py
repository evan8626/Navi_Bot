import sys, logging, random
sys.path.insert(0, r"C:\Users\evan8\Desktop\Navi Bot"); sys.path.insert(0, r"C:\Users\evan8\Desktop\Navi Bot\tests")
logging.disable(logging.CRITICAL)
import numpy as np
import DStar_Lite_planner_test as T
from navi_bot.planners.dstar_lite import DStarLitePlanner
rng=random.Random(7); N=10
stats=dict(trials=0, none_but_reachable=0, path_but_unreachable=0, invalid=0, worse_than_fresh=0, exc=0)
ex=[]
for trial in range(300):
    g=np.array([[1 if rng.random()<0.2 else 0 for _ in range(N)] for _ in range(N)])
    free=[tuple(map(int,c)) for c in np.argwhere(g==0)]
    s=rng.choice(free); t=rng.choice(free)
    if s==t: continue
    d=DStarLitePlanner(); d.set_occupancy_grid(g); p=d.plan(s,t)
    if not p or len(p)<3: continue
    s2=p[1]
    g2=g.copy()
    for _ in range(6):
        r,c=rng.randrange(N),rng.randrange(N)
        if (r,c) not in (s2,t): g2[r][c]=1-g2[r][c]
    stats['trials']+=1
    d.set_occupancy_grid(g2)
    try: p2=d.plan(s2,t)
    except Exception as e: stats['exc']+=1; ex.append(repr(e)); continue
    f=DStarLitePlanner(); f.set_occupancy_grid(g2); pf=f.plan(s2,t)
    ref=T.reference_shortest_cost(g2,s2,t)
    if ref is None:
        if p2 is not None: stats['path_but_unreachable']+=1
        continue
    if p2 is None: stats['none_but_reachable']+=1; ex.append(('none',trial)); continue
    ok,r=T.validate_path(p2,g2,s2,t,True)
    if not ok: stats['invalid']+=1; ex.append((r,trial)); continue
    if pf and T.path_cost(p2)>T.path_cost(pf)+1e-6: stats['worse_than_fresh']+=1
print(stats); print(ex[:6])
