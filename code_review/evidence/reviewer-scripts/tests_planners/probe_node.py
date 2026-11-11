import sys, logging
sys.path.insert(0, r"C:\Users\evan8\Desktop\Navi Bot"); sys.path.insert(0, r"C:\Users\evan8\Desktop\Navi Bot\tests")
import numpy as np
logging.disable(logging.CRITICAL)
import path_planner_test as P
from navi_bot.path_planner import PathPlannerNode
from navi_bot.mock_ros2 import OccupancyGrid
import navi_bot.ros_compat as rc
print("backend Node:", rc.Node.__module__)
og=OccupancyGrid(); print("OccupancyGrid attrs:", [a for a in dir(og) if not a.startswith('_')])
# 1. goal + pose before any map
n=PathPlannerNode(); P.pose_at_cell(n,0,0); P.goal_at_cell(n,0,7)
try: P.fire(n); print("no-map fire ok; path", n.current_path, "dstar_init", n.dstar_initialized)
except Exception as e: print("no-map fire raises", type(e).__name__, e)
# 2. goal before map with non-unit resolution
n=PathPlannerNode(); n.goal_callback(P.make_goal_msg(3.0,5.0)); goal_before=n.current_goal
try:
    og=OccupancyGrid(); og.info.resolution=0.5; og.info.width=20; og.info.height=20
    og.info.origin.position.x=0.0; og.info.origin.position.y=0.0; og.data=[0]*400
    n.map_callback(og)
    n2=PathPlannerNode(); n2.map_callback(og); n2.goal_callback(P.make_goal_msg(3.0,5.0))
    print("goal-before-map cell", goal_before, "vs goal-after-map cell", n2.current_goal, "node1 still", n.current_goal)
except Exception as e: print("occgrid probe err", type(e).__name__, e)
# 3. map change after plan does not trigger replan
n=PathPlannerNode(); n.map_callback(P.clear_map()); P.pose_at_cell(n,0,0); P.goal_at_cell(n,0,7); P.fire(n)
g=P.clear_map(); g[0][3]=1; n.map_callback(g); print("after map change replanning_needed=", n.replanning_needed); P.fire(n); print("path still through blocked cell:", (0,3) in n.current_path)
# 4. D* result: is anything produced/published from D*? 
n=PathPlannerNode(); n.map_callback(P.obstacle_map()); P.pose_at_cell(n,0,0); P.goal_at_cell(n,0,7); P.fire(n)
print("dstar g(start)=", n.DStar_local_planner.g_values.get(n.dstar_start), "dstar_last", n.dstar_last)
# 5. Test 23 would pass with wrong start
n=PathPlannerNode(); n.map_callback(P.clear_map()); P.pose_at_cell(n,2,5); P.goal_at_cell(n,0,7); P.fire(n); print("dstar_start", n.dstar_start, "path[0]", n.current_path[0])
# 6. axis swap in tests 13/21: make_pose_msg(row,col) used as (x,y)
n=PathPlannerNode(); n.map_callback(P.clear_map()); n.current_pose_callback(P.make_pose_msg(2,5)); n.goal_callback(P.make_goal_msg(2.0,7.0)); P.fire(n); print("make_pose_msg(2,5) start cell ->", n.current_path[0])
# 7. deadline: blocked map — A* fails, D* g inf -> early return before deadline accounting
import time
n=PathPlannerNode(); n.map_callback(P.blocked_map()); P.pose_at_cell(n,0,0); P.goal_at_cell(n,0,7); n.planning_time_budget=-1; P.fire(n); print("deadline misses with budget -1 on no-path:", n.planning_deadline_misses)
n=PathPlannerNode(); n.map_callback(P.clear_map()); P.pose_at_cell(n,0,0); P.goal_at_cell(n,0,7); n.planning_time_budget=-1; P.fire(n); print("deadline misses with budget -1 on success:", n.planning_deadline_misses)
