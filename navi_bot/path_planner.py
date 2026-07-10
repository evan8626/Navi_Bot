#!/usr/bin/env python3
"""
Path Planning Node

Implements A* global planning, D* for local, and also DWA for local as a learning experience.
Must meet real-time constraints for safety-critical navigation.
"""

# ROS 2 types via the compat shim: real rclpy when a ROS 2 env is active,
# otherwise the in-process mock_ros2 (so the tests run without ROS 2 installed).
from navi_bot.ros_compat import (
    Node, Twist, Pose2D, Point, Path, OccupancyGrid, rclpy, latched_map_qos,
    make_path_msg,
)
from navi_bot.utils.geometry import (
    distance, angle_between_points, normalize_angle, world_to_grid, grid_to_world,
)
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
        self.current_pose = None
        self.replanning_needed = False

        # Map geometry for the world<->grid boundary. Defaults make one cell
        # equal one world unit at the origin until the first real map arrives.
        self.map_resolution = 1.0
        self.map_origin = (0.0, 0.0)
        
        # Timing
        self.planning_time_budget = 0.050 # 50ms 
        self.planning_deadline_misses = 0
        
        # Subscribers
        self.goal_sub = self.create_subscription(Pose2D, '/goal_pose', self.goal_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, latched_map_qos())
        self.current_pose_sub = self.create_subscription(Pose2D, '/current_pose', self.current_pose_callback, 10)
        
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
        self.current_goal = world_to_grid(msg.x, msg.y, self.map_origin, self.map_resolution)
        self.replanning_needed = True
        self.dstar_initialized = False
        self.get_logger().info(f"New goal received: ({msg.x:.2f}, {msg.y:.2f})")
        
    def map_callback(self, msg):
        """Convert map message to 2D array planners consume"""
        if hasattr(msg, 'info'):
            raw = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
            grid = ((raw >= 50) | (raw == -1)).astype(int)
            self.map_resolution = msg.info.resolution
            self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        else:
            grid = np.asarray(msg)
        self.global_planner.set_occupancy_grid(grid)
        self.DStar_local_planner.set_occupancy_grid(grid)
    
    def current_pose_callback(self, msg):
        """Handle current pose updates."""
        self.current_pose = (float(msg.x), float(msg.y), float(msg.theta))

    def planning_loop(self):
        """
        Main planning loop.
        
        Checks if replanning is needed and executes global planner.
        """
        if not self.replanning_needed or self.current_goal is None:
            return
        
        planning_start = time.perf_counter()
        
        if (self.current_pose is None):
            self.get_logger().warning("Current pose unknown, cannot plan")
            return
        start = world_to_grid(self.current_pose[0], self.current_pose[1],
                              self.map_origin, self.map_resolution)
        
        path = self.global_planner.plan(start, self.current_goal)
        
        if path:
            self.current_path = path
            self.replanning_needed = False
            self.get_logger().info(f"Path planned with {len(path)} waypoints")
            # World cell-centres, packed per-backend (real nav_msgs/Path needs
            # PoseStamped poses; the mock keeps bare Points).
            world_path = [grid_to_world(wp[0], wp[1], self.map_origin, self.map_resolution)
                          for wp in path]
            self.path_pub.publish(make_path_msg(world_path))
        else:
            self.get_logger().warning("No path found to goal")
            
        # D Star Lite local replanning
        if not self.dstar_initialized and self.current_goal is not None:
            self.DStar_local_planner.d_star_initialize(start, self.current_goal)
            self.DStar_local_planner.compute_shortest_path(start, self.current_goal, self.DStar_local_planner.occupancy_grid)
            self.dstar_start = start
            self.dstar_last = self.current_goal
            self.dstar_initialized = True
            
        if self.DStar_local_planner.g_values.get(self.dstar_start, float('inf')) == float('inf'):
            logger.warning("No path to goal exists.")
            return
        
        if self.dstar_initialized and self.dstar_start == self.current_goal:
            logger.info("Already at goal, no path needed.")
            return
            
        planning_time = time.perf_counter() - planning_start
        if planning_time > self.planning_time_budget:
            self.planning_deadline_misses += 1
            self.get_logger().warning(f"Planning deadline miss! Took {planning_time*1000:.2f}ms")
   
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