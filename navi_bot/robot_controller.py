#!/usr/bin/env python3
"""
Main Robot Controller Node

This node orchestrates all robot subsystems and enforces real-time constraints.
It coordinates sensor processing, path planning, motion control, and state management.

Two drive modes, checked in this order each control cycle:

* PATH mode — a global path arrived on /planned_path (or via set_path). The
  PathFollower picks a lookahead target along it and DWA computes the local
  command, avoiding obstacles perceived by LIDAR. On arrival the node
  publishes 'goal_reached' on /nav_status (closing the state machine's
  feedback loop) and falls back to idle.
* POINT-GOAL mode — no path, but a goal was set via set_goal(): the PID
  MotionController drives straight at it (small adjustments, docking).

A hard safety governor backstops both modes: full stop when the closest
LIDAR reading is < 0.5 m, half speed when < 1.0 m. DWA already slows for
obstacles on its own — the governor is the last resort, not the avoidance.

Frame convention: pose, path waypoints, and obstacles all share one frame
(the planners' grid cells today; world metres once the world<->grid boundary
helpers land). LIDAR readings are robot-relative and get transformed into
that frame using the current pose.
"""

# ROS 2 types via the compat shim: real rclpy when a ROS 2 env is active,
# otherwise the in-process mock_ros2 (so the tests run without ROS 2 installed).
from navi_bot.ros_compat import (
    Node, Twist, Pose2D, LaserScan, Float32, String, Path, OccupancyGrid,
    rclpy, init, shutdown, spin, occupancy_to_grid, path_to_waypoints,
    latched_map_qos,
)

import math
import time

from navi_bot.control.motion_controller import MotionController
from navi_bot.control.kinematics import DifferentialDriveKinematics
from navi_bot.control.path_follower import PathFollower
from navi_bot.planners.dwa import DWAPlanner
from navi_bot.utils.geometry import transform_point


class RobotController(Node):
    """
    Main controller that integrates all robot subsystems.

    Responsibilities:
    - Coordinate sensor data flow
    - Execute control loop at fixed frequency (50 Hz target)
    - Follow the planned global path with the DWA local planner
    - Monitor real-time performance (deadline misses)
    - Report navigation status to the state machine
    """

    def __init__(self):
        super().__init__('robot_controller')

        # Timing constraints
        self.control_period = 0.020 # 20ms = 50Hz
        self.deadline_misses = 0

        # Robot state
        self.current_pose = Pose2D()
        self.current_velocity = Twist()
        self.battery_level = 100.0
        self.last_cmd = (0.0, 0.0)   # (v, omega) actually published last cycle

        # Obstacle detection state
        self.closest_obstacle_dist = float('inf')
        self.closest_obstacle_angle = 0.0
        self.obstacle_threshold = 1.5 # meters - consider obstacles closer than this
        self.valid_obstacles = [] # List of (distance, angle) tuples

        # Motion controllers
        self.motion_controller = MotionController()          # point-goal PID
        self.kinematics = DifferentialDriveKinematics(wheel_base=0.4, wheel_radius=0.1, max_wheel_speed=10.0)
        self.follower = PathFollower()                       # global-path bookkeeping
        self.dwa = DWAPlanner()                              # local planner

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav_status_pub = self.create_publisher(String, '/nav_status', 10)

        # Subscribers
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.pose_sub = self.create_subscription(Pose2D, '/pose', self.pose_callback, 10)
        self.path_sub = self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, latched_map_qos())

        # Control loop timer
        self.control_timer = self.create_timer(self.control_period, self.control_loop)

        self.get_logger().info('Robot Controller initialized')

    def lidar_callback(self, msg):
        """Process incoming LIDAR data."""
        # Reset obstacle tracking for this scan
        self.valid_obstacles = []
        self.closest_obstacle_dist = float('inf')
        self.closest_obstacle_angle = 0.0

        for index, distance in enumerate(msg.ranges):
            if msg.range_min <= distance and distance <= msg.range_max:
                angle = msg.angle_min + (index * msg.angle_increment)
                if distance <= self.obstacle_threshold:
                    self.valid_obstacles.append((distance, angle))
                    if distance < self.closest_obstacle_dist:
                        self.closest_obstacle_dist = distance
                        self.closest_obstacle_angle = angle

        # After filtering, log results
        if self.valid_obstacles:
            self.get_logger().info(
                f'Detected {len(self.valid_obstacles)} obstacles, '
                f'closest at {self.closest_obstacle_dist:.2f}m, '
                f'angle {self.closest_obstacle_angle:.2f}rad')

    def pose_callback(self, msg):
        """Update robot pose estimate"""
        self.current_pose = msg

    def path_callback(self, msg):
        """Adopt a new global path from the path planner: the follower resets
        its progress and DWA gets the path for its path-following score term
        (the anti-limit-cycle term — refreshed on every replan)."""
        waypoints = path_to_waypoints(msg)
        if not waypoints:
            self.follower.clear()
            self.dwa.set_global_path(None)
            return
        self.follower.set_path(waypoints)
        self.dwa.set_global_path(waypoints)
        self.get_logger().info(f'Following new path with {len(waypoints)} waypoints')

    def map_callback(self, msg):
        """Give DWA the static map so LIDAR returns from known walls are
        classified as static (steer-around) rather than movers (yield)."""
        self.dwa.set_occupancy_grid(occupancy_to_grid(msg))

    def set_goal(self, x, y, theta=None):
        """
        Set a navigation goal for the robot.

        Args:
            x: Goal x position (meters)
            y: Goal y position (meters)
            theta: Goal orientation (radians), optional
        """
        self.motion_controller.set_goal(x, y, theta)

    def set_path(self, waypoints):
        """Follow a path handed over directly (bypassing /planned_path)."""
        self.path_callback(waypoints)

    def world_obstacles(self):
        """LIDAR readings (robot-relative distance/angle) transformed into the
        shared pose frame for the local planner."""
        pose = (self.current_pose.x, self.current_pose.y, self.current_pose.theta)
        return [transform_point((d * math.cos(a), d * math.sin(a)), pose)
                for d, a in self.valid_obstacles]

    def control_loop(self):
        """
        Main control loop - executes at a fixed frequncy.

        This must complete withing the deadline (20ms).
        Steps:
        1. Read sensor data
        2. Update localization
        3. Check for obstacles.
        4. Compute control commands (path mode, else point-goal mode)
        5. Publish commands
        6. Monitor timing
        """
        loop_start = time.perf_counter()

        pose = (self.current_pose.x, self.current_pose.y, self.current_pose.theta)

        if self.follower.has_path():
            # PATH mode: follower picks the lookahead target, DWA plans locally.
            if self.follower.goal_reached(pose[0], pose[1]):
                self.follower.clear()
                self.dwa.set_global_path(None)
                self.nav_status_pub.publish(String(data='goal_reached'))
                self.get_logger().info('Goal reached — path complete')
                velocity, omega = 0.0, 0.0
            else:
                target = self.follower.target(pose[0], pose[1])
                cmd = self.dwa.plan(pose, self.last_cmd, target, self.world_obstacles())
                if cmd is None:
                    self.get_logger().warning('DWA returned no command — holding')
                    velocity, omega = 0.0, 0.0
                else:
                    velocity, omega = cmd
        else:
            # POINT-GOAL mode: PID straight at the motion controller's goal.
            velocity, omega = self.motion_controller.compute_control(pose, self.control_period)

        # Safety governor (last-resort backstop; DWA slows on its own).
        if self.closest_obstacle_dist < 0.5:
            # STOP
            velocity = 0.0
            omega = 0.0
        elif self.closest_obstacle_dist < 1.0:
            # SLOW DOWN BY 50%
            velocity *= 0.5
            omega *= 0.5

        # creating cmd_vel to get ready for publishing
        cmd_vel = Twist()
        cmd_vel.linear.x = float(velocity)
        cmd_vel.angular.z = float(omega)

        # cmd_vel published.
        self.cmd_vel_pub.publish(cmd_vel)
        self.last_cmd = (float(velocity), float(omega))

        # Check deadline
        loop_time = time.perf_counter() - loop_start
        if loop_time > self.control_period:
            self.deadline_misses += 1
            self.get_logger().warning(
                f'Deadline miss! Loop took {loop_time*1000:.2f}ms'
                f'Total misses: {self.deadline_misses}')


    def shutdown(self):
        """Clean shutdown"""
        self.get_logger().info('Shutting down robot controller')
        # Stop robot
        self.cmd_vel_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.shutdown()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
