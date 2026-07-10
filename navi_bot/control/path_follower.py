#!/usr/bin/env python3
"""
Path Follower

Waypoint bookkeeping for driving a global path with a local planner:
advances past reached waypoints and yields a LOOKAHEAD target — a point far
enough along the path that the local planner's rollout doesn't overshoot it
and stall at direction-change corners (the lesson from the A*+DWA suites).

Pure logic, no ROS: the robot controller wires it to topics, and the
integration tests can drive the very same implementation, so the follower
that ships is the follower that gets tested.

Convention: waypoints and poses share one coordinate frame (the planners'
(row, col) grid cells today; world metres once the world<->grid boundary
helpers land). The follower never converts frames.
"""

import math


class PathFollower:
    """
    Tracks progress along a waypoint path.

    Args:
        wp_tol: distance at which a waypoint counts as reached (advances past it)
        goal_tol: distance at which the FINAL waypoint counts as arrival
        lookahead: minimum distance of the returned target point — aim ahead,
                   not at the nearest waypoint
    """

    def __init__(self, wp_tol=0.6, goal_tol=0.7, lookahead=1.0):
        self.wp_tol = wp_tol
        self.goal_tol = goal_tol
        self.lookahead = lookahead
        self.path = None
        self.wp_idx = 0

    def set_path(self, path):
        """Adopt a new global path (list of (x, y)); resets progress.
        An empty/None path clears the follower."""
        self.path = [(float(p[0]), float(p[1])) for p in path] if path else None
        self.wp_idx = 0

    def clear(self):
        """Drop the current path (e.g. after arrival)."""
        self.path = None
        self.wp_idx = 0

    def has_path(self):
        return bool(self.path)

    def goal_reached(self, x, y):
        """True when the pose is within goal_tol of the path's final waypoint."""
        if not self.path:
            return False
        gx, gy = self.path[-1]
        return math.hypot(gx - x, gy - y) <= self.goal_tol

    def target(self, x, y):
        """
        The current steering target for the local planner: first advance past
        any waypoints within wp_tol, then aim at the first waypoint at least
        `lookahead` away (or the final waypoint). None if no path is set.
        """
        if not self.path:
            return None
        last = len(self.path) - 1
        while self.wp_idx < last and math.hypot(self.path[self.wp_idx][0] - x,
                                                self.path[self.wp_idx][1] - y) <= self.wp_tol:
            self.wp_idx += 1
        target_idx = self.wp_idx
        while target_idx < last and math.hypot(self.path[target_idx][0] - x,
                                               self.path[target_idx][1] - y) < self.lookahead:
            target_idx += 1
        return self.path[target_idx]
