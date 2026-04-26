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

class U:
    """Priority queue for D* Lite"""
    def __init__(self):
        self.elements = []
    
    def Update(self, item, priority):
        self.Remove(item)
        heapq.heappush(self.elements, (priority, item))
    
    def Remove(self, item):
        self.elements = [(p, i) for p, i in self.elements if i != item]
        heapq.heapify(self.elements)
    
    def Insert(self, item, priority):
        heapq.heappush(self.elements, (priority, item))
        
    def Pop(self):
        return heapq.heappop(self.elements)[1]
    
    def Top(self):
        return self.elements[0][1] if self.elements else None
    
    def TopKey(self):
        return self.elements[0][0] if self.elements else (float('inf'), float('inf'))
                                    
class DStarLitePlanner:
    """
    D* Lite path planner for local planning
    """
    def __init__(self, grid_resolution=0.05):
        self.grid_resolution = grid_resolution
        self.occupancy_grid = None
        self.previous_grid = None
        self.k_m = 0
        self.U = U()
        self.g_values = {}
        self.rhs_values = {}
        
    def d_star_initialize(self, start, goal):
        self.k_m = 0
        self.U = U()
        self.g_values = {}
        self.rhs_values = {}
        self.g_values[goal] = float('inf')
        self.rhs_values[goal] = 0.0
        self.U.Insert(goal, self.calculate_key(goal, start))
        
    def set_occupancy_grid(self, grid):
        """Update the occupancy grid."""
        if self.occupancy_grid is None:
            self.previous_grid = None
        else:
            self.previous_grid = self.occupancy_grid
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
            if (max_rows <= row) or (max_cols <= col):
                return False
            if (self.occupancy_grid[row][col] != 0):
                # cell is occupied
                logger.info(f"Cell at row {row}, col {col} is occupied.")
                return False
            return True
        else:
            return False
        
    def calculate_key(self, node, start):
        """Calculate the key for a node based on its g and rhs values."""
        g = self.g_values.get(node, float('inf'))
        rhs = self.rhs_values.get(node, float('inf'))
        h = heuristic_backward(node, start)
        k1 = min(g, rhs) + h
        k2 = min(g, rhs)
        return (k1, k2)
    
    def cost(self, a, b, grid = None):
        """Cost of moving from a to b. Returns inf if not tranversable."""
        if grid is None:
            grid = self.occupancy_grid
        if not self.is_coord_valid(a[0], a[1]) or not self.is_coord_valid(b[0], b[1]):
            return float('inf')
        if grid[a[0]][a[1]] != 0 or grid[b[0]][b[1]] != 0:
            logger.info(f"Cost from {a} to {b} is infinite due to occupancy.")
            return float('inf')
        return 1.0

    def get_successors(self, node):
        directions = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
        successors = []
        for dir in directions:
            new_i = dir[0] + node[0]
            new_j = dir[1] + node[1]
            if not self.is_coord_valid(new_i, new_j): continue
            new_coord = (new_i, new_j)
            successors.append(new_coord)
        return successors

    def compute_shortest_path(self, start, goal, grid):
        pos = start
        while (self.U.TopKey() < self.calculate_key(pos, start)) or (self.rhs_values.get(pos, float('inf')) != self.g_values.get(pos, float('inf'))):
            u = self.U.Top()
            k_old = self.U.TopKey()
            k_new = self.calculate_key(u, start)
            if k_old < k_new:
                self.U.Update(u, k_new)
            elif self.g_values.get(u, float('inf')) > self.rhs_values.get(u, float('inf')):
                self.g_values[u] = self.rhs_values[u]
                self.U.Remove(u)
                for s in self.get_successors(u):
                    new_rhs = self.cost(s, u) + self.g_values[u]
                    if new_rhs < self.rhs_values.get(s, float('inf')):
                        if s != goal:
                            self.rhs_values[s] = min(self.rhs_values.get(s, float('inf')), self.cost(s, u, self.occupancy_grid) + self.g_values.get(u, float('inf')))
                        self.U.Update(s, self.calculate_key(s, start))
            else:
                g_old = self.g_values.get(u, float('inf'))
                self.g_values[u] = float('inf')
                for s in self.get_successors(u) + [u]:
                    if self.rhs_values.get(s, float('inf')) == self.cost(s, u, self.occupancy_grid) + g_old:
                        if s != goal:
                            self.rhs_values[s] = min(self.rhs_values.get(s, float('inf')), min([self.cost(s, sp, self.occupancy_grid) + self.g_values.get(sp, float('inf')) for sp in self.get_successors(s)]))
                    self.U.Update(s, self.calculate_key(s, start))
        return None
    
    def edge_changed(self):
        changed_edges = []
        for i in range(len(self.occupancy_grid)):
            for j in range(len(self.occupancy_grid[0])):
                if self.previous_grid is not None and self.occupancy_grid[i][j] != self.previous_grid[i][j]:
                    neighbors = self.get_successors((i, j))
                    for neighbor in neighbors:
                        changed_edges.append(((i, j), neighbor))
                        changed_edges.append((neighbor, (i, j)))
        return changed_edges
    
    def plan(self, start, goal):
        
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
        elif heuristic_backward(start, goal) == 0:
            logger.info("Already at goal. No path needed.")
            return None
        logger.info(f"Planning path from {start} to {goal} using D* Lite.")
        path = []
        self.d_star_initialize(start, goal)
        self.calculate_key(goal, start)
        self.compute_shortest_path(start, goal, self.occupancy_grid)
        edges_changed = self.edge_changed()
        
        if edges_changed:
            self.compute_shortest_path(start, goal, self.occupancy_grid)
        
        visited = set()
        while start != goal:
            if start in visited:
                logger.warning(f"Already visited {start} in path extraction.")
                return None
            visited.add(start)
            if self.g_values.get(start, float('inf')) == float('inf'):
                logger.warning("No path to goal exists.")
                return None
            path.append(start)
            successors = self.get_successors(start)
            if not successors:
                logger.warning(f"No successors available. Path is blocked at {start}.")
                return None
            s_start = min(self.get_successors(start), key=lambda s: self.cost(start, s, self.occupancy_grid) + self.g_values.get(s, float('inf')))
            start = s_start
        path.append(goal)
        return path

def heuristic_backward(pos, start):
    """Manhattan distance heuristic."""
    return abs(pos[0] - start[0]) + abs(pos[1] - start[1])