import sys
sys.path.insert(0, r"C:\Users\evan8\Desktop\Navi Bot"); sys.path.insert(0, r"C:\Users\evan8\Desktop\Navi Bot\tests")
import AStar_planner_test as A
from navi_bot.planners.astar import AStarPlanner
AStarPlanner.plan = lambda self, s, g: None   # break the planner completely
test_straight_line_clear = A.test_straight_line_clear
test_path_optimality = A.test_path_optimality
