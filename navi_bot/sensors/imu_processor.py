#!/usr/bin/env python3
"""
IMU Data Processor

Processess IMU data for:
- Orientation estimation
- Angular velocity measurement
- Acceleration filtering
"""

import numpy as np
from collections import deque

class ImuProcessor:
    """
    IMU data processing and filtering

    Specifications (simulated):
    - 6-axis: accelerometer + gyroscope
    - Update rate: 100 Hz
    - Noise: Gaussian with configurable std dev
    """

    def __init__(self, buffer_size=10):
        self.buffer_size = buffer_size

        # Complementary filter parameters
        self.alpha = 0.98 # Weight for gyro vs accel

        # State
        self.orientation = 0.0 # Current heading estimate (radians)
        self.angular_velocity = 0.0
        self.linear_acceleration = np.zeros(3)

        # Filtering buffers
        self.gyro_buffer = deque(maxlen=buffer_size)
        self.accel_buffer = deque(max_len=buffer_size)
        
        # Bias estamation
        self.gyro_bias = 0.0
        self.accel_bias = np.zeros(3)
        self.calibrated = False

    def calibrate(self, num_samples=500):
        """
        Calibrate IMU by computing bias from stationary samples.
        
        If using ROS use allan_variance_ros and calc noise density 
        and random walk while recording data for at least 3 hours
        while on a damped/stationary surface. 
        If using allan_variance_ros, use Ubuntu 20.04.
        
        Since this is a learning project, this calibration code written
        by the maintainer of this project, Evan Osborn.
        """
        # TODO: learn more about IMU calibration. This will need to be broken into multiple files
        self.calibrated = True

    def process_imu_data(self, imu_msg):
        """
        Process IMU message and update estimates.

        Args:
            imu_msg: sensor_msgs/Imu message
        
        Returns:
            Dictionary with filtered data
        """
        # Extract data
        angular_vel = imu_msg.angular_velocity.z
        linear_accel = np.array([
            imu_msg.linear_acceleration.x,
            imu_msg.linear_acceleration.y,
            imu_msg.linear_acceleration.z
        ])

        # Apply bias correction
        if self.calibrated:
            angular_vel -= self.gyro_bias
            linear_accel -= self.accel_bias
        
        # Low-pass filtering
        self.gyros_buffer.append(angular_vel)
        self.accel_buffer.append(linear_accel)

        filtered_gyro = self.moving_average(self.gyro_buffer)
        filtered_accel = self.moving_average_vector(self.accel_buffer)

        # Update state
        self.angular_velocity = filtered_gyro
        self.linear_acceleration = filtered_accel

        return {
            'angular_velocity': filtered_gyro,
            'linear_acceleration': filtered_accel,
            'orientation': self.orientation
        }

    def update_orientation(self, dt):
        """
        Update orientation estimate using complementary filter.

        Fuses gyro integration (fast response) with accelerometer
        (no drift) for robust orientation estimate

        TODO: Implement complementary filter
        """
        # Integrate gyro
        gyro_angle = self.orientation + self.angular_velocity * dt

        # TODO: Compute angle from acceleromteter (tilt)
        accel_angle = 0.0

        # Complementary filter
        self.orientation = self.alpha * gyro_angle + (1 - self.alpha) * accel_angle

    def moving_average(self, buffer):
        """Compute moving average of scalar buffer."""
        if len(buffer) == 0:
            return 0.0
        return np.mean(buffer)
    
    def moving_average_vector(self, buffer):
        """Compute moving average of vector buffer"""
        if len(buffer) == 0:
            return np.zeros(3)
        return np.mean(buffer, axis=0)
    
    def detect_motion(self, threshold=0.1):
        """
        Detect if robot is in motion based on accelerometer

        Returns: True is motion detected
        """
        accel_magnitude = np.linalg.norm(self.linear_acceleration)
        return accel_magnitude > threshold