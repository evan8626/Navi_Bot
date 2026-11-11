import sys, logging
sys.path.insert(0, r"C:\Users\evan8\Desktop\Navi Bot"); sys.path.insert(0, r"C:\Users\evan8\Desktop\Navi Bot\tests")
import numpy as np
import DWA_planner_test as W
logging.disable(logging.CRITICAL)
from navi_bot.planners.dwa import DWAPlanner
from navi_bot.planners.dstar_lite import DStarLitePlanner
from navi_bot.planners.astar import AStarPlanner
m=W.blocked_map()
print("DWA blocked: start from callback =", W.get_start_callback(m), "goal =", W.test_goal_callback(m))
d=DWAPlanner(); obs=np.argwhere(m==1)
print("DWA real blocked plan (pose (3,3,0), goal (6,6)) ->", d.plan((3.0,3.0,0.0),(0.0,0.0),(6,6),obs))
# D* None-start style tests: would they pass if plan returned garbage? show plan return values
import DStar_Lite_planner_test as D
p=DStarLitePlanner(); p.set_occupancy_grid(D.clear_map())
print("D* invalid start returns", p.plan((-1,-1),(7,7)))
# D* stale previous_grid: set grid in place mutation
g=D.clear_map(); p=DStarLitePlanner(); p.set_occupancy_grid(g); p1=p.plan((0,0),(0,7)); print("p1",p1)
g[0][3]=1; g[1][3]=1; p.set_occupancy_grid(g); p2=p.plan((0,0),(0,7)); print("in-place mutated grid replan", p2, "edges changed detected:", p.previous_grid is p.occupancy_grid)
# A* with list goal
a=AStarPlanner(); a.set_occupancy_grid(D.clear_map()); print("A* list goal ->", a.plan((0,0),[0,7]))
try: print("A* list start ->", a.plan([0,0],(0,7)))
except Exception as e: print("A* list start raises", type(e).__name__, e)
# A* occupied start
gg=D.clear_map(); gg[0][0]=1; print("A* occupied start ->", a.plan((0,0),(0,3)) if a.set_occupancy_grid(gg) is None else None)
