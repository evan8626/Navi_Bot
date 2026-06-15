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
from collections import deque
import math
import numpy as np
import time
import logging
import heapq

logger = logging.getLogger(__name__)

ROBOT_RADIUS = 0.25 # meters
CELL_RADIUS = 0.5 # meters
GRID_RESOLUTION = 0.05 # meters
CLEARANCE_CAP = 1.5
YIELD_RANGE = 2.5              # m — react to a perceived mover only within this forward range
YIELD_STANDOFF = 1.05          # m — ramp the top speed down to a halt this far short of it
YIELD_CONE = math.radians(80)  # half-angle of the forward arc that counts as "ahead"

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
        self.obstacle_cost_gain = 1.0
        self.goal_weight = 2.0     # reward trajectories that end CLOSER to the goal
        self.turn_weight = 0.25 # 0.25
        
        self.grid_resolution = GRID_RESOLUTION
        self.occupancy_grid = None
        
        self.position_history = deque(maxlen=5)
        
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
    
    def norm(self, x, hi, lo):
        return (x - lo) / (hi - lo) if hi > lo else 0.0

    def is_dynamic_obstacle(self, obs):
        """True if `obs` is NOT in the known static map — a perceived mover.

        The static occupancy grid is the global planner's responsibility (it
        routes around it); the local yield rule is only for obstacles that
        aren't on that map (a crossing hazard fed in by perception). With no
        grid set, every obstacle is treated as unknown/dynamic.
        """
        if self.occupancy_grid is None:
            return True
        r, c = int(round(obs[0])), int(round(obs[1]))
        if 0 <= r < len(self.occupancy_grid) and 0 <= c < len(self.occupancy_grid[0]):
            return self.occupancy_grid[r][c] == 0
        return True
    
    def track_movers(self, movers, dt):
        """
        Estimate each mover's velocity by differencing over the WHOLE window
        (newest - oldest), not frame-to-frame, so the hazard's stop-then-jump
        motion still reads as a steady velocity. Returns [(pos, vel), ...] with
        pos/vel as length-2 arrays, in cells and cells/second.
        """
        movers = [np.asarray(m, dtype=float) for m in movers]
        tracked = []
        if self.position_history:
            oldest = self.position_history[0]
            span = len(self.position_history) * dt
            for m in movers:
                vel = np.zeros(2)
                if len(oldest):
                    prev = min(oldest, key=lambda p: np.hypot(*(m - p)))
                    disp = m - prev
                    if np.hypot(*disp) <= self.max_vel_x * span + 1.0:
                        vel = disp / span
                tracked.append((m, vel))
        else:
            tracked = [(m, np.zeros(2)) for m in movers]
        self.position_history.append(movers)
        return tracked
        
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
        
        static_obstacle = [o for o in obstacles if not self.is_dynamic_obstacle(o)]
        movers = self.track_movers([o for o in obstacles if self.is_dynamic_obstacle(o)], control_period)

        # Yield to a perceived MOVER ahead (one not in the static map): ramp the
        # window's top speed down to a halt as the mover closes inside YIELD_RANGE,
        # stopping a standoff short of it. Capping v_max (rather than returning
        # (0, 0)) lets the dynamic-window accel limit smooth the slowdown and keeps
        # the robot from freezing, so it resumes the moment the mover leaves the
        # forward arc. Static obstacles are filtered out — steering around those is
        # the global path's job, and braking for them is what froze the robot.
        contact = ROBOT_RADIUS + CELL_RADIUS * 1.25 
        nearest_ahead = float('inf')
        # for obs in obstacles:
        #     if not self.is_dynamic_obstacle(obs):
        for (mpos, mvel) in movers:
            dx, dy = mpos[0] - current_pose[0], mpos[1] - current_pose[1]
            if dx * mvel[0] + dy * mvel[1] > 0:
                continue
            d = math.hypot(dx, dy)
            if d < contact or abs(normalize_angle(math.atan2(dy, dx) - current_pose[2])) <= YIELD_CONE:
                nearest_ahead = min(nearest_ahead, d)
        if nearest_ahead < YIELD_RANGE:
            frac = max(0.0, (nearest_ahead - YIELD_STANDOFF) / (YIELD_RANGE - YIELD_STANDOFF))
            v_max = max(v_min, min(v_max, frac * self.max_vel_x))

        candidates = []

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
                    
                #score = self.score_trajectory(vel, goal, obstacles, positional_info)
                score = self.score_trajectory(vel, goal, static_obstacle, movers, positional_info, step_time)
                if score is None:
                    continue
                candidates.append((v, omega, score[0], score[1], score[2], score[3]))  # v, omega, heading, clearance, speed, goal_dist
        if not candidates:
            return (0.0, 0.0)

        # Pull each term into its own column, then normalize across candidates.
        om = [c[1] for c in candidates]
        h  = [c[2] for c in candidates]
        cl = [c[3] for c in candidates]
        sp = [c[4] for c in candidates]
        gd = [c[5] for c in candidates]
        omax, omin = max(om), min(om)
        hmax, hmin = max(h), min(h)
        clmax, clmin = max(cl), min(cl)
        spmax, spmin = max(sp), min(sp)
        gdmax, gdmin = max(gd), min(gd)

        # Score every surviving candidate and keep the single best. Heading error
        # and goal distance are costs (lower = better -> reward 1 - norm);
        # clearance and speed are rewards (higher = better -> norm directly).
        best_cmd = (0.0, 0.0)
        best_score = float('-inf')
        for v_c, omega_c, heading_c, clearance_c, speed_c, goaldist_c in candidates:
            s = (self.heading_weight    * (1 - self.norm(heading_c, hmax, hmin))
               + self.obstacle_cost_gain * self.norm(clearance_c, clmax, clmin)
               + self.speed_cost_gain    * self.norm(speed_c, spmax, spmin)
               + self.turn_weight        * (1 - self.norm(abs(omega_c), omax, omin))
               + self.goal_weight       * (1 - self.norm(goaldist_c, gdmax, gdmin)))
            if s > best_score:
                best_score = s
                best_cmd = (v_c, omega_c)
        return best_cmd
    
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
    
    def score_trajectory(self, vel, goal, obstacles, movers, pos, step_time):
        """Score a velocity command"""
        if vel is None or goal is None or pos is None:
            return None
        
        contact = ROBOT_RADIUS + CELL_RADIUS * 1.25
        min_clearance = float('inf')
        
        for i, p in enumerate(pos):
            x, y, theta = p
            if not self.is_pose_inbounds(x, y):
                return None
            t = min(i, 17.0) * step_time
            
            for obstacle in obstacles:
                clear = heuristic_forward((x, y), obstacle)
                if clear < contact:
                    return None
                min_clearance = min(min_clearance, clear)
            
            for (mx, my), (vx, vy) in movers:
                px, py = mx + vx * t, my + vy * t
                clear = math.hypot(x - px, y - py)
                if clear < contact:
                    return None
                min_clearance = min(min_clearance, clear)
                
        min_clearance = min(min_clearance, CLEARANCE_CAP)
        braking_room = max(0.0, min_clearance - contact)
        if abs(vel[0]) > np.sqrt(2.0 * braking_room * self.max_accel_x):
            return None
        
        _, _, final_theta = pos[-1]
        heading = abs(normalize_angle(angle_between_points((pos[-1][0], pos[-1][1]), goal) - final_theta))
        goal_dist = distance((pos[-1][0], pos[-1][1]), goal)

        return (heading, min_clearance, vel[0], goal_dist)

def heuristic_forward(pos, goal):
    """Euclidian distance heuristic."""
    return np.sqrt((pos[0] - goal[0])**2 + (pos[1] - goal[1])**2)