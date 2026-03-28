#!/usr/bin/env python3
"""
Wheel Odometry

Computes robot pose from wheel encoder data.
Handles differential drive kinematics.
"""

from navi_bot.utils.geometry import normalize_angle
import numpy as np

class WheelOdometry:
    """
    Dead reckoning odometry for differential drive robot.

    Note: Wheel odometry accumulates error over time due to:
    - Wheel slip
    - Uneven surfaces
    - Encoder resolution
    Must be fused with other sensors for accurate localization
    """
    def __init__(self, wheel_base=0.4, wheel_radius=0.1, ticks_per_rev=1000):
        """
        Initialize odometry.

        Args:
            wheel_base: Distance between wheels (meters)
            wheel_radius: Radius of wheels (meters)
            ticks_per_rev: Encoder ticks per wheel revolution
        """
        self.wheel_base = wheel_base
        self.wheel_radius = wheel_radius
        self.ticks_per_rev = ticks_per_rev

        # Pose state [x, y, theta]
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Previous encoder values
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0

        # Velocity estimates
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

    def update(self, left_ticks, right_ticks):
        """
        Update pose estimate from encoder readings.

        Args:
            left_ticks: Current left wheel encoder value
            right_ticks: Current right wheel encoder value
            dt: Time since last update (seconds)

        Returns:
            (x, y, theta, v, omega) - pose and velocity
        """
        # Compute wheel movements
        delta_left = left_ticks - self.prev_left_ticks
        delta_right = right_ticks - self.prev_right_ticks

        # Convert ticks to distance
        dist_left = self.ticks_to_distance(delta_left)
        dist_right = self.ticks_to_distance(delta_right)

        # Compute robot displacement
        delta_s = (dist_left + dist_right) / 2.0 # Forward distance
        delta_theta = (dist_right - dist_left) / self.wheel_base # Rotation

        # Update pose
        self.theta += delta_theta
        self.theta = normalize_angle(self.theta)

        self.x += delta_s * np.cos(self.theta)
        self.y += delta_s * np.sin(self.theta)

        # Compute velocities
        if dt > 0:
            self.linear_velocity = delta_s / dt
            self.angular_velocity = delta_theta / dt

        # Update previous values
        self.prev_left_ticks = left_ticks
        self.prev_right_ticks = right_ticks

        return self.x, self.y, self.theta, self.linear_velocity, self.angular_velocity

    def ticks_to_distance(self, ticks):
        """ Convert encoder ticks to linear distance. """
        revolutions = ticks / self.ticks_per_rev
        distance = revolutions * 2 * np.pi * self.wheel_radius
        return distance

    def reset(self, x=0.0, y=0.0, theta=0.0):
        """Reset pose to specified values."""
        self.x = x
        self.y = y
        self.theta = theta

    def get_pose(self):
        """Get current pose estimate"""
        return self.x, self.y, self.theta

    def get_velocity(self):
        """Get current velocity estimate."""
        return self.linear_velocity, self.angular_velocity
