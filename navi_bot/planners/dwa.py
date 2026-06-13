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

ROBOT_RADIUS = 0.25 # meters
CELL_RADIUS = 0.5 # meters
GRID_RESOLUTION = 0.05 # meters

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
        self.speed_cost_gain = 1.0
        self.obstacle_cost_gain = 0.5
        
        self.grid_resolution = GRID_RESOLUTION
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
        
    def is_pose_inbounds(self, x, y):
        """Check if the pose is within the bounds of the occupancy grid."""
        if self.occupancy_grid is None:
            return True
        rows = len(self.occupancy_grid)
        cols = len(self.occupancy_grid[0])
        return (-0.5 + ROBOT_RADIUS <= x <= rows - 0.5 - ROBOT_RADIUS and -0.5 + ROBOT_RADIUS <= y <= cols - 0.5 - ROBOT_RADIUS)
        
    def plan(self, current_pose, current_vel, goal, obstacles):
        """
        Compute optimal velocity command.

        Args:
            current_pose (tuple): (x, y, theta) current coordinates and heading
            current_vel (tuple): (v, omega) current linear and angular velocities
            goal (tuple): (x, y) goal position
            obstacles (List): obstacle points
        """
        if current_pose is None:
            logger.warning("Current pose is None. Cannot plan.")
            return None
        elif current_vel is None:
            logger.warning("Current velocity is None. Cannot plan.")
            return None
        elif goal is None:
            logger.warning("Goal is None. Cannot plan.")
            return None
        elif self.is_pose_inbounds(current_pose[0], current_pose[1]) is False:
            logger.warning("Invalid current pose coordinates")
            logger.warning(f"Current pose coords are X: {current_pose[0]}, Y: {current_pose[1]}")
            return None
        elif self.is_coord_valid(goal[0], goal[1]) is False:
            logger.warning("Invalid goal coordinates")
            logger.warning(f"Goal coords are X: {goal[0]}, Y: {goal[1]}")
            return None
        
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
                if score is None:
                    continue
                kinematics.update({vel: score})
        if not kinematics:
            return (0.0, 0.0)
        best_v, best_omega = max(kinematics, key=kinematics.get)
        
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
            return None
        elif goal is None:
            return None
        elif pos is None:
            return None
        
        contact = ROBOT_RADIUS + CELL_RADIUS
        
        min_clearance = float('inf')
        for p in pos:
            x, y, theta = p
            
            if not self.is_pose_inbounds(x, y):
                return None
            
            if obstacles.size == 0:
                min_clearance = min(min_clearance, 10.0)
            else:
                for obstacle in obstacles:
                    clear = heuristic_forward((x, y), obstacle)
                    if clear < contact:
                        return None
                    min_clearance = min(min_clearance, clear)
        
        braking_room = max(0.0, min_clearance - contact)
        if abs(vel[0]) > np.sqrt(2.0 * braking_room * self.max_accel_x):
            return None
        
        _, _, final_theta = pos[-1]
        heading = abs(normalize_angle(angle_between_points((pos[-1][0], pos[-1][1]), goal) - final_theta))
        
        return (-(self.heading_weight * heading) - self.obstacle_cost_gain * (1.0 / min_clearance)) + (self.speed_cost_gain * vel[0])

def heuristic_forward(pos, goal):
    """Euclidian distance heuristic."""
    return np.sqrt((pos[0] - goal[0])**2 + (pos[1] - goal[1])**2)