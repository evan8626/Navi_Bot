#!/usr/bin/env python3
"""
Wheel Odometry

Computes robot pose from wheel encoder data.
Handles differential drive kinematics.
"""

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
    # TODO: Implement odometry code