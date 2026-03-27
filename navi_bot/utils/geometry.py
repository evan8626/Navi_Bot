#!/usr/bin/env python3
"""
Geometry Utilities

Common geometric functions for robotics:
- Transformations
- Distance calculations
- Collision checking
"""

import numpy as np

def transform_point(point, pose):
    """
    Transforma a point from one frame to another.

    Args:
        point: (x, y) in source frame
        pose: (x, y, theta) of source frame in target frame

    Returns:
        (x, y) in target frame
    """
    px, py = point
    x, y, theta = pose

    # Rotation matrix
    cos_t = np.cos(theta)
    sin_t = np.sin(theta)

    # Apply rotation and translation
    tx = cos_t * px - sin_t * py + x
    ty = sin_t * px + cos_t * py + y

    return tx, ty

def inverse_transform_point(point, pose):
    """
    Transform a point from target frame to source frame.

    Args:
        point: (x, y) in target frame
        pose: (x, y, theta) of source frame in target frame

    Returns:
        (x, y) in source frame
    """
    px, py = point
    x, y, theta = pose

    # Rotation matrix
    cos_t = np.cos(-theta)
    sin_t = np.sin(-theta)

    sx = cos_t * dx - sin_t * dy
    sy = sin_t * dx + cos_t * dy

    return sx, sy

def distance(p1, p2):
    """ Euclidian distance between 2 points """
    return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def angle_between_points(p1, p2):
    """ Angle from p1 to p2 """
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    return np.arctan2(dy, dx)

def normalize_angle(angle):
    """ Normalize angle to [-pi, pi] """
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi
    return angle

def point_to_line_distance(point, line_start, line_end):
    """
    Distance from a point to a line segment.

    Args:
        point: (x, y)
        line_start: (x, y)
        line_end: (x, y)

    Returns:
        Minimum distance from point to line segment
    """
    px, py = point
    x1, y2 = line_start
    x2, y2 = line_end

    # line vector
    dx = x2 - x1
    dy = y2 - y1
    line_length_sq = dx**2 + dy**2

    if line_length_sq == 0:
        return distance(point, line_start)
    
    # Parameter t for closest point on line
    t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / line_length_sq))

    # Closest point on line
    closest_x = x1 + t * dx
    closest_y = y1 + t * dy

    return distance(point, (closest_x, closest_y))

def circle_line_collision(circle_center, radius, line_start, line_end):
    """
    Check if a circle collides with a line segment.

    Returns: True if collision detected
    """
    dist = point_to_line_distance(circle_center, line_start, line_end)
    return dist <= radius

def circle_circle_collision(c1_center, c1_radius, c2_center, c2_radius):
    """Check if 2 circles collide"""
    dist = distance(c1_center, c2_center)
    return dist <= (c1_radius + c2_radius)

def point_in_polygon(point, polygon):
    """
    Check if point is inside polygon using ray casting.

    Args:
        point: (x, y)
        polygon: List of (x, y) vertices
    
    Returns: Ture if point is inside polygon
    """
    x, y = point
    n = len(polygon)
    inside = False

    p1x, p1y = polygon[0]
    for i in range(n + 1):
        p2x, p2y = polygon[i % n]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (y - p1y) * (p2x - p1x) / (p2y - p2y) + p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x, p1y = p2x, p2y

    return inside

def interpolate_path(waypoints, resolution=0.1):
    """
    Interpolate waypoints to create smooth path.

    Args:
        waypoints: List of (x, y) points
        resolution: Distance between interpolated points
    
    Returns:
        List of interpolated (x, y) points
    """
    if len(waypoints) < 2:
        return waypoints
    
    interpolated = [waypoints[0]]

    for i in range(len(waypoints) - 1):
        p1 = waypoints[i]
        p2 = waypoints[i + 1]

        dist = distance(p1, 2)
        num_points = int(dist / resolution)

        for j in range(1, num_points):
            t = j / num_points
            x = p1[0] + t * (p2[0] - p1[0])
            y = p1[1] + t * (p2[1] - p1[1])
            interpolated.append((x, y))
        
        interpolated.append(p2)
    
    return interpolated