#!/usr/bin/env pthon3
"""
Main Robot Controller Node

This node orchestrates all robot subsystems and enforces real-time constraints.
It coordinates sensor processing, path planning, motion control, and state management.
"""

# FOR USE WITH ACTUAL ROS2 INSTALL
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist, Pose2D
# from sensor_msgs.msg import LaserSCan
# from std_msgs.msg import Float32, String

# FOR USE WITH MOCK ROS2
from navi_bot.mock_ros2 import Node, Twist, Pose2D, LaserScan, Float32, String
import navi_bot.mock_ros2 as rclpy
from navi_bot.mock_ros2 import init, shutdown, spin

import time

from navi_bot.control.motion_controller import MotionController
from navi_bot.control.kinematics import DifferentialDriveKinematics


class RobotController(Node):
    """
    Main controller that integrates all robot subsystems.

    Responsibilities:
    - Coordinate sensor data flow
    - Execute control loop at fixed frequency (50 Hz target)
    - Monitor real-time performance (deadline misses)
    - Integrate with state machine for high-level behavior
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

        # Obstacle detection state
        self.closest_obstacle_dist = float('inf')
        self.closest_obstacle_angle = 0.0
        self.closest_threshold = 1.5 # meters - consider obstacles closer than this
        self.valid_obstacles = [] # List of (distance, angle) tuples

        # Motion controllers
        self.motion_controller = MotionController()
        self.kinematics = DifferentialDriveKinematics(wheel_base=0.4, wheel_radius=0.1, max_wheel_speed=10.0)

        # Subscribers
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.pose_sub = self.create_subscription(Pose2D, '/pose', self.pose_callback, 10)

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

        # TODO: Update local costmap

    def pose_callback(self, msg):
        """Update robot pose estimate"""
        self.current_pose = msg

    def set_goal(self, x, y, theta=None):
        """
        Set a navigation goal for the robot.

        Args:
            x: Goal x position (meters)
            y: Goal y position (meters)
            theta: Goal orientation (radians), optional
        """
        self.motion_controller.set_goal(x, y, theta)

    def control_loop(self):
        """
        Main control loop - executes at a fixed frequncy.

        This must complete withing the deadline (26ms).
        Steps:
        1. Read sensor data
        2. Update localization
        3. Check for obstacles.
        4. Compute control commands
        5. Publish commands
        6. Monitor timing
        """
        loop_start = time.perf_counter()

        # linear velocity and angular velocity gotten from current_pose
        current_pose = (self.current_pose.x, self.current_pose.y, self.current_pose.theta)
        velocity, omega = self.motion_controller.compute_control(current_pose, self.control_period)

        # safety scaling.
        if self.closest_obstacle_dist < 0.5:
            # STOP
            velocity = 0
            omega = 0
        elif self.closest_obstacle_dist < 1.0:
            # SLOW DOWN BY 50%
            velocity *= 0.5
            omega *= 0.5

        # creating cmd_vel to get read for publishing
        cmd_vel = Twist()
        cmd_vel.linear.x = velocity
        cmd_vel.angular.z = omega

        # cmd_vel published.
        self.cmd_vel_pub.publish(cmd_vel)

        # Check deadline
        loop_time = time.perf_counter() - loop_start
        if loop_time > self.control_period:
            self.deadline_misses += 1
            self.get_logger().warn(
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