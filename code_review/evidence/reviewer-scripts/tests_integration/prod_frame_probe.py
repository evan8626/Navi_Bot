"""Production wiring probe on the mock bus: MapPublisher -> PathPlanner -> RobotController.
Shows the frame the controller's DWA actually receives and how DWA classifies a known wall."""
import sys, logging, builtins, math
logging.disable(logging.CRITICAL)
_p = builtins.print; builtins.print = lambda *a, **k: None
import numpy as np
import navi_bot.mock_ros2 as m
from navi_bot.ros_compat import Pose2D
m.init()
from navi_bot.path_planner import PathPlannerNode
from navi_bot.robot_controller import RobotController
from navi_bot.map_publisher import MapPublisherNode
from navi_bot.state_machine import StateMachine, Mission

pp = PathPlannerNode(); rc = RobotController(); sm = StateMachine()
pub = MapPublisherNode()          # publishes on construction; subscribers exist -> delivered
_p("planner grid shape:", np.asarray(pp.global_planner.occupancy_grid).shape,
   "res:", pp.map_resolution, "controller DWA grid shape:", np.asarray(rc.dwa.occupancy_grid).shape)

# robot at pickup_1 world (3.0, 0.9); goal delivery_1 world (3.0, 9.1)
pose = Pose2D(); pose.x, pose.y = 3.0, 0.9
pp.current_pose_callback(pose); rc.pose_callback(pose)
g = Pose2D(); g.x, g.y = 3.0, 9.1
pp.goal_callback(g); pp.planning_loop()
_p("controller follower path first 3 / last:", rc.follower.path[:3] if rc.follower.path else None,
   rc.follower.path[-1] if rc.follower.path else None)

# DWA with the world-metre pose the controller would use:
_p("DWA is_pose_inbounds(world 3.0, 0.9):", rc.dwa.is_pose_inbounds(3.0, 0.9))
# A LIDAR return on the bottom perimeter wall at world (3.0, 0.05) (grid row 1, col 60)
_p("grid[1][60] (wall at world (3.0,0.05)):", rc.dwa.occupancy_grid[1][60],
   "DWA classifies world point (3.0,0.05) as dynamic mover:", rc.dwa.is_dynamic_obstacle((3.0, 0.05)))

# State machine goal publication: Mission pickup in (row, col)? world?
mis = Mission('X', (18, 60), (182, 60))      # grid-cell convention used by waypoint_nav_test
sm.add_mission(mis); sm.update_state_machine(); sm.update_state_machine()
_p("state machine published /goal_pose x,y =", (sm.goal_pub.last_msg.x, sm.goal_pub.last_msg.y),
   "-> planner current_goal (row,col) =", pp.current_goal)

# One control cycle: obstacle-free; what does DWA return?
cmd_hist = []
for k in range(5):
    rc.control_loop(); cmd_hist.append(rc.last_cmd)
_p("first 5 control cycles (20 ms each) cmd:", [tuple(round(c, 3) for c in x) for x in cmd_hist])
