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
        elif heuristic(start, goal) == 0:
            logger.info("Already at goal. No path needed.")
            return None
            
        # h = estimated cost to get from node n to goal (heuristic)
        # g = cost to reach node n from start node 
        # f = total estimated cost of path through node n
        h = heuristic(start, goal)
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
                    h = heuristic(new_coord, goal)
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
                                    

class DStarPlanner:
    """
    D* path planner for local planning
    """
    # TODO: Implement D*
    pass

class DWAPlanner:
    """
    Dynamic Window Approach for local planning.

    Samples velocity commands within dynamic window and scores them based on:
    - Heading: Alignment with goal
    - Clearance: Distance to obstacles
    - Velocity: Preference for higher speeds
    """
    def __init__(self):
        # Robot constraints
        self.max_vel_x = 1.0       # m/s
        self.min_vel_x = -0.5      # m/s
        self.max_vel_theta = 2.0   # rad/s
        self.max_accel_x = 0.5     # m/s^2
        self.max_accel_theta = 1.5 # rad/s^2
        
        # Scoring weights
        self.heading_weight = 1.0
        self.clearance_weight = 2.0
        self.velocity_weight = 0.5
        
    def plan(self, current_pose, current_vel, goal, obstacles):
        """
        Compute optimal velocity command.

        Args:
            current_pose (tuple): (x, y, theta) current coordinates and heading
            current_vel (tuple): (v, omega) current linear and angular velocities
            goal (tuple): (x, y) goal position
            obstacles (List): obstacle points
        """
        time_horizon = 2.0
        step_time = 0.1
        control_period = 0.5
        num_steps = int(time_horizon / step_time)
        
        v_min, v_max, omega_min, omega_max, = self.compute_dynamic_window(current_vel, control_period)
        kinematics = {}
        
        for v in np.linspace(v_min, v_max, 20):
            for omega in np.linspace(omega_min, omega_max, 40):
                x, y, theta = current_pose[0], current_pose[1], current_pose[2]
                positional_info = []
                positional_info.append((x, y, theta))
                for i in range(num_steps):
                    new_x = x + v*np.cos(theta)*step_time
                    new_y = y + v*np.sin(theta)*step_time
                    new_theta = theta + omega*step_time
                    vel = (v, omega)
                    
                    x = new_x
                    y = new_y
                    theta = new_theta
                    positional_info.append((x, y, theta))
                    
                score = self.score_trajectory(vel, goal, obstacles, positional_info)
                kinematics.update({vel: score})
                
        pair = max(kinematics, key=kinematics.get)
        
        best_v, best_omega = pair
        return (best_v, best_omega)
    
    def compute_dynamic_window(self, current_vel, dt):
        """
        Compute dynamic window of feasible velocities

        Args:
            current_vel (tuple): (v, omega) current velocities
            dt (float): simulation step time
        
        Returns:
            (v_min, v_max, omega_min, omega_max) feasible velocity bounds
        """
        current_v = current_vel[0]
        current_omega = current_vel[1]
        
        #V_s
        v_min_s = self.min_vel_x
        v_max_s = self.max_vel_x
        omega_min_s = -self.max_vel_theta
        omega_max_s = self.max_vel_theta
        
        #V_d
        v_min_d = current_v - self.max_accel_x*dt
        v_max_d = current_v + self.max_accel_x*dt
        omega_min_d = current_omega - self.max_accel_theta*dt
        omega_max_d = current_omega + self.max_accel_theta*dt
        
        v_min = max(v_min_s, v_min_d)
        v_max = min(v_max_s, v_max_d)
        omega_min = max(omega_min_s, omega_min_d)
        omega_max = min(omega_max_s, omega_max_d)
        
        return(v_min, v_max, omega_min, omega_max)
    
    def score_trajectory(self, vel, goal, obstacles, pos):
        """Score a velocity command"""
        if vel is None:
            logger.warning("There is no velocity. Cannot score trajectory.")
            return None
        elif goal is None:
            logger.warning("There is no goal. Cannot score trajectory.")
            return None
        elif obstacles is None:
            logger.warning("Provided obstacles list is None. Cannot score trajectory.")
            return None
        elif pos is None:
            logger.warning("There is no positional information. Cannot score trajectory.")
            return None
        
        angle = 0.0
        clearance_list = []
        for p in pos:
            x, y, theta = p
            coord = (x, y)
            angle = angle_between_points(coord, goal)
            for obstacle in obstacles:
                clear = heuristic(coord, obstacle)
                clearance_list.append(clear)
        _, _, final_theta = pos[-1]
        heading = angle_between_points((pos[-1][0], pos[-1][1]), goal) - final_theta
        heading = normalize_angle(heading)
        heading = abs(heading)
        clearance = min(clearance_list)
        score = -(self.heading_weight * heading) + (self.clearance_weight * clearance) + (self.velocity_weight * vel[0])
        return score
              
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
        self.DStar_local_planner = DStarPlanner()
        
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
        
        self.get_logger().info("Path Planner initialized")
        
    def goal_callback(self, msg):
        """Handle new goal"""
        self.current_goal = (msg.x, msg.y)
        self.replanning_needed = True
        self.get_logger().info(f"New goal received: ({msg.x:.2f}, {msg.y:.2f})")
        
    def map_callback(self, msg):
        """Handle map updates."""
        self.global_planner.set_occupancy_grid(msg)
        
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
            # TODO: Publish path
        else:
            self.get_logger().warn("No path found to goal")
            
        planning_time = time.perf_counter() - planning_start
        if planning_time > self.planning_time_budget:
            self.planning_deadline_misses += 1
            self.get_logger().warn(f"Planning deadline miss! Took {planning_time*1000:.2f}ms")

def heuristic(pos, goal):
    """Euclidian distance heuristic."""
    return np.sqrt((pos[0] - goal[0])**2 + (pos[1] - goal[1])**2)
    
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