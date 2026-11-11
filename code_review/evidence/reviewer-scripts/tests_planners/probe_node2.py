import sys, logging, io, contextlib
sys.path.insert(0, r"C:\Users\evan8\Desktop\Navi Bot"); sys.path.insert(0, r"C:\Users\evan8\Desktop\Navi Bot\tests")
from types import SimpleNamespace as NS
import numpy as np
logging.disable(logging.CRITICAL)
import path_planner_test as P
from navi_bot.path_planner import PathPlannerNode
from navi_bot.mock_ros2 import OccupancyGrid
def msg(res=0.5, w=20, h=20, ox=0.0, oy=0.0):
    return NS(info=NS(resolution=res, width=w, height=h, origin=NS(position=NS(x=ox,y=oy))), data=[0]*(w*h))
with contextlib.redirect_stdout(io.StringIO()):
    a=PathPlannerNode(); a.goal_callback(P.make_goal_msg(3.0,5.0)); a.map_callback(msg())
    b=PathPlannerNode(); b.map_callback(msg()); b.goal_callback(P.make_goal_msg(3.0,5.0))
print("goal BEFORE map -> cell", a.current_goal, "; goal AFTER map -> cell", b.current_goal)
with contextlib.redirect_stdout(io.StringIO()):
    c=PathPlannerNode(); c.map_callback(OccupancyGrid(width=8,height=8,resolution=1.0))
g=c.global_planner.occupancy_grid; print("mock OccupancyGrid -> grid ndim", np.ndim(g), "dtype", getattr(g,'dtype',None))
with contextlib.redirect_stdout(io.StringIO()):
    P.pose_at_cell(c,0,0); P.goal_at_cell(c,0,7)
    try: c.planning_loop(); r="ok"
    except Exception as e: r=f"{type(e).__name__}: {e}"
print("planning with mock OccupancyGrid ->", r)
