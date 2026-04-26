#!/usr/bin/env python3

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
import math
import numpy as np
import time
import logging
import heapq

logger = logging.getLogger(__name__)

class AStarPlanner:
    """
    A* path planner on occupancy grid
    """
    def __init__(self, grid_resolution=0.05):
        self.grid_resolution = grid_resolution
        self.occupancy_grid = None
        
    def set_occupancy_grid(self, grid):
        """Update the occupancy grid."""
        self.occupancy_grid = grid
        
    def is_coord_valid(self, row, col):
        """Checks for existant row/col, and row/col values 0 or greater"""
        if row is None:
            logger.info(f"row is {row}")
            return False
        elif col is None:
            logger.info(f"col is {col}")
            return False
        elif (row >= 0) and (col >= 0):
            if self.occupancy_grid is None:
                return True
            max_rows = len(self.occupancy_grid)
            max_cols = len(self.occupancy_grid[0])
            if (max_rows > row) and (max_cols > col):
                return True
            else:
                return False
        else:
            return False
        
    def plan(self, start, goal):
        """
        Plan a path from start to goal.

        Args:
            start (tuple): (x, y) in world coordinates
            goal (tuple): (x, y) in world coordinates
            
        Returns:
            List of tuples (x, y) waypoints, or None if no path found
        """
        if start is None:
            logger.warning("Starting cell is None. Cannot create path.")
            return None
        elif goal is None:
            logger.warning("Goal doesn't exist, no path to calculate.")
            return None
        elif self.is_coord_valid(start[0], start[1]) is False:
            logger.warning("Invalid start coordinates")
            logger.warning(f"Start coords are X: {start[0]}, Y: {start[1]}")
            return None
        elif self.is_coord_valid(goal[0], goal[1]) is False:
            logger.warning("Invalid goal coordinates")
            logger.warning(f"Goal coords are X: {goal[0]}, Y: {goal[1]}")
            return None
        elif heuristic_forward(start, goal) == 0:
            logger.info("Already at goal. No path needed.")
            return None
            
        # h = estimated cost to get from node n to goal (heuristic)
        # g = cost to reach node n from start node 
        # f = total estimated cost of path through node n
        h = heuristic_forward(start, goal)
        g = 0.0
        f = g + h
        open_list = [] # priority queue
        closed_list = set() # set of visited coords
        heapq.heappush(open_list, (f, start))
        cost_dict = {}
        parent_dict = {}
        cost_dict[start] = 0
        parent_dict[start] = None
        g_value = 0.0
        directions = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
        if self.occupancy_grid is None:
            logger.warning("No occupancy grid available.")
            return None
        
        while len(open_list) > 0:
            p = heapq.heappop(open_list)
            visited_coord = p[1]
            closed_list.add(visited_coord)
            
            for dir in directions:
                new_i = dir[0] + visited_coord[0]
                new_j = dir[1] + visited_coord[1]
                if abs(dir[0]) == 1 and abs(dir[1]) == 1: g_value = np.sqrt(2)
                else: g_value = 1.0
                new_coord = (new_i, new_j)
                
                if not self.is_coord_valid(new_i, new_j):
                    continue
                if new_coord in closed_list:
                    continue
                if self.occupancy_grid[new_i][new_j] != 0:
                    # cell is occupied so skip it
                    continue
                
                if new_coord == goal:
                    parent_dict[goal] = visited_coord
                    path = []
                    current = goal
                    
                    while current is not None:
                        path.append(current)
                        current = parent_dict[current]
                    path.reverse()
                    return path
                else:
                    h = heuristic_forward(new_coord, goal)
                    g = cost_dict[visited_coord] + g_value
                    f = g + h
                    if new_coord in cost_dict:
                        if g < cost_dict[new_coord]:
                            cost_dict[new_coord] = g
                            parent_dict[new_coord] = visited_coord
                            heapq.heappush(open_list, (f, new_coord))
                    else:
                        cost_dict[new_coord] = g
                        parent_dict[new_coord] = visited_coord
                        heapq.heappush(open_list, (f, new_coord))
                        
        return None

def heuristic_forward(pos, goal):
    """Euclidian distance heuristic."""
    return np.sqrt((pos[0] - goal[0])**2 + (pos[1] - goal[1])**2)