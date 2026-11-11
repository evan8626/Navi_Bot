import sys, logging, random
sys.path.insert(0, r"C:\Users\evan8\Desktop\Navi Bot"); sys.path.insert(0, r"C:\Users\evan8\Desktop\Navi Bot\tests")
logging.disable(logging.CRITICAL)
import numpy as np
import DStar_Lite_planner_test as T
from navi_bot.planners.dstar_lite import DStarLitePlanner
rng=random.Random(7); N=10; shown=0; cnt=0; fresh_ok=0
for trial in range(300):
    g=np.array([[1 if rng.random()<0.2 else 0 for _ in range(N)] for _ in range(N)])
    free=[tuple(map(int,c)) for c in np.argwhere(g==0)]
    s=rng.choice(free); t=rng.choice(free)
    if s==t: continue
    d=DStarLitePlanner(); d.set_occupancy_grid(g); p=d.plan(s,t)
    if not p or len(p)<3: continue
    s2=p[1]; g2=g.copy()
    for _ in range(6):
        r,c=rng.randrange(N),rng.randrange(N)
        if (r,c) not in (s2,t): g2[r][c]=1-g2[r][c]
    d.set_occupancy_grid(g2); p2=d.plan(s2,t)
    ref=T.reference_shortest_cost(g2,s2,t)
    if ref is not None and p2 is None:
        cnt+=1
        f=DStarLitePlanner(); f.set_occupancy_grid(g2)
        if f.plan(s2,t): fresh_ok+=1
        # also: same planner, unchanged start (no move)
        if shown<1:
            shown+=1; print("trial",trial,"s",s,"->",s2,"goal",t,"ref",ref); print(g); print(g2)
print("incremental None while reachable:",cnt,"fresh planner finds path in",fresh_ok)
