#!/usr/bin/env python3
"""
Path Planning Node

Implements A* global planning, D* for local, and also DWA for local as a learning experience.
Must meet real-time constraints for safety-critical navigation.
"""

# FOR USE WITH ACTUAL ROS2 INSTALL
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist, Pose2D
# from sensor_msgs.msg import LaserSCan
# from std_msgs.msg import Float32, String

# FOR USE WITH MOCK ROS2
from navi_bot.mock_ros2 import Node, Twist, Pose2D, Point, Path, OccupancyGrid
import navi_bot.mock_ros2 as rclpy
from navi_bot.utils.geometry import distance, angle_between_points, normalize_angle
from navi_bot.planners.astar import AStarPlanner, heuristic_forward
from navi_bot.planners.dstar_lite import DStarLitePlanner, heuristic_backward
from navi_bot.planners.dwa import DWAPlanner
import math
import numpy as np
import time
import logging
import heapq

logger = logging.getLogger(__name__)
              
class PathPlannerNode(Node):
    """
    ROS2 node for path planning.
    
    Runs global planner when goal changes, local planner at high frequency.
    """
    def __init__(self):
        super().__init__('path_planner')
        
        # Planners
        self.global_planner = AStarPlanner()
        self.DWA_local_planner = DWAPlanner()
        self.DStar_local_planner = DStarLitePlanner()
        
        # Planning state
        self.current_path = None
        self.current_goal = None
        self.replanning_needed = False
        
        # Timing
        self.planning_time_budget = 0.050 # 50ms 
        self.planning_deadline_misses = 0
        
        # Subscribers
        self.goal_sub = self.create_subscription(Pose2D, '/goal_pose', self.goal_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        
        # Planning timer (10Hz for global replan checks)
        self.plan_timer = self.create_timer(0.1, self.planning_loop)
        
        # Path publisher
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        
        # D Star initial parameters
        self.dstar_start = None
        self.dstar_last = None
        self.dstar_initialized = False
        
        self.get_logger().info("Path Planner initialized")
        
    def goal_callback(self, msg):
        """Handle new goal"""
        self.current_goal = (msg.x, msg.y)
        self.replanning_needed = True
        self.dstar_initialized = False
        self.get_logger().info(f"New goal received: ({msg.x:.2f}, {msg.y:.2f})")
        
    def map_callback(self, msg):
        """Handle map updates."""
        self.global_planner.set_occupancy_grid(msg)
        self.DStar_local_planner.set_occupancy_grid(msg)
        
    def planning_loop(self):
        """
        Main planning loop.
        
        Checks if replanning is needed and executes global planner.
        """
        if not self.replanning_needed or self.current_goal is None:
            return
        
        planning_start = time.perf_counter()
        
        # TODO: Get current robot position
        start = (0.0, 0.0)
        
        path = self.global_planner.plan(start, self.current_goal)
        
        if path:
            self.current_path = path
            self.replanning_needed = False
            self.get_logger().info(f"Path planned with {len(path)} waypoints")
            path_msg = Path()
            for waypoint in path:
                p = Point()
                p.x = waypoint[0]
                p.y = waypoint[1]
                path_msg.poses.append(p)
                
            self.path_pub.publish(path_msg)
        else:
            self.get_logger().warn("No path found to goal")
            
        # D Star Lite local replanning
        if not self.dstar_initialized and self.current_goal is not None:
            self.DStar_local_planner.d_star_initialize(self.dstar_start, self.current_goal)
            self.dstar_start = start
            self.dstar_last = self.current_goal
            self.dstar_initialized = True
            
        if self.DStar_local_planner.g_values.get(self.dstar_start, float('inf')) == float('inf'):
            logger.warning("No path to goal exists.")
            return
        
        if self.dstar_initialized and self.dstar_start == self.current_goal:
            logger.info("Already at goal, no path needed.")
            return
        
        if self.dstar_initialized and self.dstar_start != self.current_goal:
            changed_edges = self.DStar_local_planner.edge_changed()
            for edge in changed_edges:
                c_old = self.DStar_local_planner.cost(edge[0], edge[1], self.DStar_local_planner.previous_grid)
                c_new = self.DStar_local_planner.cost(edge[0], edge[1], self.DStar_local_planner.occupancy_grid)
                if c_old > c_new:
                    if edge[0] != self.current_goal:
                        self.DStar_local_planner.rhs_values[edge[0]] = min(self.DStar_local_planner.rhs_values.get(edge[0], float('inf')), c_new + self.DStar_local_planner.g_values.get(edge[0], float('inf')))
                    self.DStar_local_planner.U.Update(edge[0], self.DStar_local_planner.calculate_key(edge[0], self.dstar_start))
                elif self.DStar_local_planner.rhs_values.get(edge[0], float('inf')) == c_old + self.DStar_local_planner.g_values.get(edge[1], float('inf')):
                    if edge[0] != self.current_goal:
                        self.DStar_local_planner.rhs_values[edge[1]] = min(self.DStar_local_planner.rhs_values.get(edge[1], float('inf')), min([self.DStar_local_planner.cost(edge[1], s, self.DStar_local_planner.occupancy_grid) + self.DStar_local_planner.g_values.get(s, float('inf')) for s in self.DStar_local_planner.get_successors(edge[1])]))
                    self.DStar_local_planner.U.Update(edge[1], self.DStar_local_planner.calculate_key(edge[1], self.dstar_start))
                
            self.DStar_local_planner.compute_shortest_path(self.dstar_start, self.current_goal, self.DStar_local_planner.occupancy_grid)
            s_start = min(self.DStar_local_planner.get_successors(self.dstar_start), key=lambda s: self.DStar_local_planner.cost(self.dstar_start, s, self.DStar_local_planner.occupancy_grid) + self.DStar_local_planner.g_values.get(s, float('inf')))
            self.dstar_last = self.dstar_start
            self.dstar_start = s_start
            self.DStar_local_planner.k_m += heuristic_backward(self.dstar_last, self.dstar_start)
            
     
        planning_time = time.perf_counter() - planning_start
        if planning_time > self.planning_time_budget:
            self.planning_deadline_misses += 1
            self.get_logger().warn(f"Planning deadline miss! Took {planning_time*1000:.2f}ms")
   
def main(args=None):
    rclpy.init(args=args)
    planner = PathPlannerNode()

    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        pass
    finally:
        planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()