#!/usr/bin/env python3
"""
LIDAR Data Processor

Processes raw LIDAR scans for:
- Obstacle detection
- Local costmap generation
- Scan matching for localization
"""

import numpy as np
from navi_bot.mock_ros2 import LaserScan
from sklearn.cluster import DBSCAN

ROBOT_RADIUS = 0.25 # meters
CLUSTER_EPS = 0.3 # meters
CLUSTER_MIN_SAMPLES = 10

class LidarProcessor:
    """
    LIDAR data processing and obstacle detection.

    Specifications (simulated):
    - Range: 0.1m to 10m
    - FOV: 360 degrees
    - Resolution: 1 degree (360 points)
    - Update rate: 10 Hz
    """

    def __init__(self):
        self.min_range = 0.1
        self.max_range = 10.0
        self.angle_min = -np.pi
        self.angle_max = np.pi
        self.angle_increment = np.pi / 180.0 # 1 degree

        # Obstacle detection parameters
        self.obstacle_threshold = 0.5 # meters
        self.min_obstacle_points = 3

    def process_scan(self, scan_msg):
        """
        Process a LIDAR scan message.

        Args:
            scan_msg: sensor_msgs/LaserScan

        Returns:
            Dictionary with processed data:
            - obstacles: List of (x, y) obstacle points in robot frame
            - closest_obstacle: Distance to nearest obstacle
            - clear_directions: Boolean array of safe directions
        """
        ranges = np.array(scan_msg.ranges)

        # Filter invalid readings
        valid_mask = (ranges >= self.min_range) & (ranges <= self.max_range)
        valid_ranges = np.where(valid_mask, ranges, np.inf)

        # Detect closest obstacle
        closest = np.min(valid_ranges)

        # Compute clear directions
        clear_dirs = self.compute_clear_directions(valid_ranges)
        
        # Get obstacles
        obstacles = self.detect_obstacles(valid_ranges, scan_msg.angle_min, scan_msg.angle_increment)

        return {
            'obstacles': obstacles,
            'closest_obstacle': closest,
            'clear_directions': clear_dirs,
            'num_valid_points': np.sum(valid_mask)
        }

    def detect_obstacles(self, ranges, angle_min, angle_increment):
        """
        Convert range readings to obstacle points in Cartesian coordinates.
        """
        obstacles = self.__polar_to_cartesian(ranges, angle_min, angle_increment)
        
        db = DBSCAN(eps=CLUSTER_EPS, min_samples=CLUSTER_MIN_SAMPLES).fit(obstacles)
        labels = db.labels_
        unique_labels = set(labels)
        clustered_obstacles = []
        
        for label in unique_labels:
            if label == -1:
                continue # noise
            cluster_points = np.array(obstacles)[labels == label]
            n_clusters = len(set(labels)) - (1 if -1 in labels else 0)
            n_noise_ = list(labels).count(-1)
            if len(cluster_points) >= self.min_obstacle_points:
                clustered_obstacles.append({
                    'centroid': cluster_points.mean(axis=0),
                    'size': len(cluster_points)
                })
                
        return clustered_obstacles

    def compute_clear_directions(self, ranges):
        """
        Determine which directions are safe to move.

        Returns boolean array: True if direction is clear
        """
        clear_dirs = np.zeros_like(ranges, dtype=bool)
        window_half = 15 # ±15 degrees
        for direction in range(len(ranges)):
            #angle_window = np.arctan((ranges[direction] - ROBOT_RADIUS) / (ROBOT_RADIUS * np.tan(self.angle_increment / 2)))
            window = ranges[max(0, direction - window_half) : direction + window_half + 1]
            clear_dirs[direction] = np.all(window > self.obstacle_threshold)

        return clear_dirs

    def update_costmap(self, obstacles, costmap):
        """
        Update local costmap with obstacle info

        """
        
        grid = np.array(costmap.data).reshape(costmap.height, costmap.width)
        
        for obstacle in obstacles:
            x, y = obstacle['centroid']
            row = int(x / costmap.resolution)
            col = int(y / costmap.resolution)
            for i in range(-int(ROBOT_RADIUS / costmap.resolution), int(ROBOT_RADIUS / costmap.resolution) + 1):
                for j in range(-int(ROBOT_RADIUS / costmap.resolution), int(ROBOT_RADIUS / costmap.resolution) + 1):
                    r = np.sqrt(i**2 + j**2) * costmap.resolution
                    if r <= ROBOT_RADIUS:
                        if 0 <= row + i < costmap.height and 0 <= col + j < costmap.width:
                            grid[row + i, col + j] = 100 # Mark as occupied
        
        costmap.data = grid.ravel().tolist()
        

    # def scan_match(self, current_scan, reference_scan):
    #     """
    #     Perform scan matching for localization

    #     TODO: Implement ICP or other scan matching algo
    #     Returns: (dx, dy, dtheta) transformation
    #     """
    #     return (0.0, 0.0, 0.0)
    
    def __polar_to_cartesian(self, ranges, angle_min, angle_increment):
        """
        Convert polar coordinates to Cartesian (x, y) points.

        Args:
            ranges: Array of range readings
            angle_min: Starting angle of the scan
            angle_increment: Angle between measurements

        Returns:
            List of (x, y) points in robot frame
        """
        points = []
        for i, r in enumerate(ranges):
            if r < self.max_range:
                angle = angle_min + i * angle_increment
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                points.append((x, y))
        return points