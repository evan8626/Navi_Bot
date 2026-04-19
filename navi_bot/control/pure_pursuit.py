#!/usr/bin/env python3
"""
Pure Pursuit control

Implements pure pursuit control algorithm which uses a lookbehind
and an arc for determining path for robot to follow and how fast
it needs to execute.
"""

from navi_bot.utils.geometry import distance, angle_between_points, normalize_angle

import numpy as np

MAX_LOOKAHEAD = 8.0 # meters
MAX_LINEAR_VELOCITY = 1.0 # m/s

class PurePursuitController:

    def __init__(self, lookahead = 2.0, linear_velocity = 1.0):
        self.lookahead_distance = 0.0 # in meters
        self.linear_velocity = 0.0 # m/s
        self.waypoint = 0 # current waypoint

        if lookahead > MAX_LOOKAHEAD:
            self.lookahead_distance = MAX_LOOKAHEAD
        else:
            self.lookahead_distance = lookahead
        
        if linear_velocity > MAX_LINEAR_VELOCITY:
            self.linear_velocity = MAX_LINEAR_VELOCITY
        else:
            self.linear_velocity = linear_velocity
    
    def find_next_lookahead(self, path, current_pos):
        #===========================================
        #          DYNAMIC LOOKAHEAD
        #            ld = kdd * Vf
        #
        # ld = dynamic lookahead (faster = father, slower = closer)
        # kdd = gain
        # Vf = forward velocity
        # for later use. implementing simple lookahead now.
        #
        #===========================================

        path_len = len(path)
        if self.waypoint >= path_len:
            return path[-1] # Returns last coords in path list
        else:
            for i, coord in enumerate(path[self.waypoint:], start=self.waypoint):
                if(distance(current_pos, coord) >= self.lookahead_distance):
                    next_lookahead = coord
                    self.waypoint = i
                    return next_lookahead

    def pure_pursuit(self, path, current_pose):
        x, y, theta = current_pose
        current_position = (x, y)
        next_lookahead = self.find_next_lookahead(path, current_position)
        if next_lookahead is None:
            print(f"Next lookahead point is {next_lookahead} either path is exhausted, or there is an issue with lookahead.")
            return (0, 0)
        else:
            a = angle_between_points(current_position, next_lookahead)
            alpha = a - theta
            alpha = normalize_angle(alpha)
            k = (2 * np.sin(alpha)) / self.lookahead_distance
            omega = self.linear_velocity * k
            return (self.linear_velocity, omega)