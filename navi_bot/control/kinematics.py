#!/usr/bin/env python3
"""
Differential Drive Kinematics

Forward and inverse kinematics for differential drive mobile robot.
"""

import numpy as np

class DifferentialDriveKinematics:
    """
    Kinematics model for differential drive robot.

    Robot has two independently driven wheels.
    Control inputs: (v, omega) - linear and angular velocity
    Outputs: (v_left, v_right) - individual wheel velocities
    """

    def __init__(self, wheel_base=0.4, wheel_radius=0.1, max_wheel_speed=10.0):
        """
        Initialize kinematics model.

        Args:
            wheel_base: Distance between wheels (meters)
            wheel_radius: Radius of wheels (meters)
            max_wheel_speed: Maximum wheel angular velocity (rad/s)
        """
        self.L = wheel_base
        self.r = wheel_radius
        self.max_wheel_speed = max_wheel_speed

    def inverse_kinematics(self, v, omega):
        """
        Convert robot velocity to wheel velocities.

        Args:
            v: Linear velocity (m/s)
            omega: Angular velocity (rad/s)
        
        Returns:
            (v_left, v_right): Wheel velocities (m/s)
        """
        v_left = v - (omega * self.L / 2.0)
        v_right = v + (omega * self.L / 2.0)

        # Apply wheel speed limits
        v_left = self.limit_wheel_velocity(v_left)
        v_right = self.limit_wheel_velocity(v_right)

        return v_left, v_right

    def forward_kinematics(self, v_left, v_right):
        """
        Convert wheel velocities to robot velocity.

        Args:
            v_left: Left wheel velocity (m/s)
            v_right: Right wheel velocity (m/s)

        Returns:
            (v, omega): Linear and angular velocity
        """
        v = (v_left, v_right) / 2.0
        omega = (v_right - v_left) / self.L

        return v, omega

    def limit_wheel_velocity(self, v_wheel):
        """Enforce wheel velocity limits."""
        max_v = self.max_wheel_speed * self.r
        return np.clip(v_wheel, -max_v, max_v)

    def compute_pose_update(self, v, omega, theta, dt):
        """
        Compute post change given velocities.

        Args:
            v: Linear velocity
            omega: Angular velocity
            theta: Current orientation
            dt: Time step

        Returns:
            (dx, dy, dtheta): Change in pose
        """
        if abs(omega) < 1e-6:
            # Straight line motion
            dx = v * np.cos(theta) * dt
            dy = v * np.sin(theta) * dt
            dtheta = 0
        else:
            # Arc motion
            R = v / omega # Radius of curvature
            dtheta = omega * dt
            dx = R * (np.sin(theta + dtheta) - np.sin(theta))
            dy = R * (-np.sin(theta + dtheta) + np.cos(theta))

        return dx, dy, theta

    def validate_velocities(self, v, omega):
        """
        Check if velocity command is feasible.

        Returns: True if valid, False otherwise
        """
        v_left, v_right = self.inverse_kinematics(v, omega)
        max_v = self.max_wheel_speed * self.r
        
        return abs(v_left) <= max_v and abs(v_right) <= max_v