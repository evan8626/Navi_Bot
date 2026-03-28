#!/usr/bin/env pthon3
"""
Motion Controller

Implements PID control for trajectory tracking and waypoint navigation.
"""

import numpy as np
from collections import deque

class PIDController:
    """
    Standard PID controller with anti-windup
    """

    def __init__(self, kp=1.0, ki=0.0, kd=0.0, output_limits=None):
        """
        Initialize PID controller.

        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            output_limits: (min, max) tuple for output saturation
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits

        # State
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def update(self, error, dt):
        """
        Compute control output.

        Args:
            error: Current error (setpoint - measurment)
            dt: Time since last update

        Returns:
            Control output
        """
        # Proportional term
        p_term = self.kp * error

        # Integral term with anti-windup
        self.integral += error * dt
        i_term = self.ki * self.integral

        # Derivative term
        if dt > 0:
            derivative = (error - self.prev_error) / dt
        else:
            derivative = 0.0
        d_term = self.kd * derivative

        # Compute output
        output = p_term + i_term + d_term

        # Apply limits
        if self.output_limits:
            output = np.clip(output, self.output_limits[0], self.output_limits[1])

            # Anti-windup: prevent integral accumulation if saturated
            if (output == self.output_limits[0] or output == self.output_limits[1]):
                self.integral -= error * dt
        
        self.prev_error = error

        return output

    def reset(self):
        """Reset controller state."""
        self.integral = 0.0
        self.prev_error = 0.0

class MotionController:
    """
    High-level motion controller for trajectory tracking.

    Uses two PID controllers:
    - Linear velocity control (distance error)
    - Angualr velocity control (heading error)
    """

    def __init__(self):
        # PID controllers
        self.linear_pid = PIDController(kp=0.5, ki=0.01, kd=0.1, output_limits=(-1.0, 1.0)) # m/s
        self.angular_pid = PIDController(kp=2.0, ki=0.05, kd=0.2, output_limits=(-2.0, 2.0)) # rads/s

        # Goal tracking
        self.current_goal = None
        self.goal_tolerance = 0.1 # meters
        self.angle_tolerance = 0.1 # radians

    def set_goal(self, x, y, theta=None):
        """Set navigational goal."""
        self.current_goal = (x, y, theta)

    def compute_control(self, current_pose, dt):
        """
        Compute velocity commands to reach goal.

        Args:
            current_pose: (x, y, theta) current robot pose
            dt: Time step

        Returns:
            (v, omega) velocity commands, or None if no goal
        """
        if self.current_goal is None:
            return 0.0, 0.0

        goal_x, goal_y, goal_theta = self.current_goal
        x, y, theta = current_pose

        # Compute errors
        dx = goal_x - x
        dy = goal_y - y
        distance_error = np.sqrt(dx**2 + dy**2)

        # Heading to goal
        desired_heading = np.arctan2(dy, dx)
        heading_error = self.normalize_angle(desired_heading - theta)

        # Check if goal reached
        if distance_error < self.goal_tolerance:
            # If goal has orientation requirement, rotate to it
            if goal_theta is not None:
                angle_error = self.normalize_angle(goal_theta - theta)
                if abs(angle_error) > self.angle_tolerance:
                    omega = self.angular_pid.update(angle_error, dt)
                    return 0.0, omega
            # Goal fully reached
            return 0.0, 0.0

        # Compute velocity commands
        v = self.linear_pid.update(distance_error, dt)
        omega = self.angular_pid.update(heading_error, dt)

        # Slow down when turning sharply
        if abs(heading_error) > np.pi / 4:
            v *= 0.5
        return v, omega
    
    def is_goal_reached(self, current_pose):
        """Check if current goal is reached"""
        if self.current_goal is None:
            return True
        
        goal_x, goal_y, goal_theta = self.current_goal
        x, y, theta = current_pose

        dx = goal_x - x
        dy = goal_y - y
        distance = np.sqrt(dx**2 + dy**2)

        if ditance > self.goal_tolerance:
            return False
        
        if goal_theta is not None:
            angle_error = abs(self.normalize_angle(goal_theta - theta))
            return angle_error < self.angle_tolerance

        return True

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

    def reset(self):
        """Reset controller state"""
        self.linear_pid.reset()
        self.angular_pid.reset()
        self.current_goal = None
