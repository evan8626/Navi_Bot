import sys, logging, random
sys.path.insert(0, r"C:\Users\evan8\Desktop\Navi Bot")
sys.path.insert(0, r"C:\Users\evan8\Desktop\Navi Bot\tests")
logging.disable(logging.CRITICAL)
import numpy as np
import DStar_Lite_planner_test as T
from navi_bot.planners.dstar_lite import DStarLitePlanner
from navi_bot.planners.astar import AStarPlanner
rng = random.Random(1)
bad=0; badA=0; n=0; fails=[]
for trial in range(300):
    g = np.array([[1 if rng.random()<0.2 else 0 for _ in range(8)] for _ in range(8)])
    free=[tuple(map(int,c)) for c in np.argwhere(g==0)]
    s=rng.choice(free); t=rng.choice(free)
    if s==t: continue
    ref=T.reference_shortest_cost(g,s,t)
    d=DStarLitePlanner(); d.set_occupancy_grid(g); p=d.plan(s,t)
    a=AStarPlanner(); a.set_occupancy_grid(g); pa=a.plan(s,t)
    n+=1
    if ref is None:
        if p is not None: bad+=1; fails.append(("dstar path when unreachable",s,t))
        continue
    if p is None: bad+=1; fails.append(("dstar none",s,t,ref)); continue
    ok,r=T.validate_path(p,g,s,t,True); c=T.path_cost(p)
    if not ok or abs(c-ref)>1e-6: bad+=1; fails.append((r,s,t,c,ref))
    ca=T.path_cost(pa) if pa else None
    if ca is None or abs(ca-ref)>1e-6: badA+=1
print("trials",n,"dstar mismatches",bad,"astar mismatches",badA)
for f in fails[:5]: print(f)
