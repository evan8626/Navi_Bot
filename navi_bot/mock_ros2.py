"""
Mock ROS2 classes for testing without full ROS2 installation.

This module provides lightweight mock implementations of ROS2 message types
and the Node class to enable rapid prototyping and testing of robot logic.
"""

import time
from typing import Callable, Any, List

# ==========================================================================
# Message Types
# ==========================================================================

class Twist:
    """Velocity command (linear and angular)."""
    def __init__(self):
        self.linear = type('obj', (object,), {'x': 0.0, 'y': 0.0, 'z': 0.0})()
        self.angular = type('obj', (object,), {'x': 0.0, 'y': 0.0, 'z': 0.0})()

class Pose2D:
    """2D pose (x, y, theta)."""
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

class LaserScan:
    """
    LIDAR scan data message.
    """
    def __init__(self):
        self.ranges = []
        self.angle_min = -np.pi
        self.angle_max = np.pi
        self.angle_increment = 0.1 # in radians
        self.range_min = 0.10 # in meters
        self.range_max = 10.0 # in meters

class Float32:
    """Float message"""
    def __init__(self):
        self.data = 0.0

class String:
    """String message"""
    def __init__(self):
        self.data = ""

class Point:
    """3D point."""
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

class Path:
    """Sequence of poses representing a planned path."""
    def __init__(self):
        self.poses = []

class OccupancyGrid:
    """2D occupancy grid map"""
    def __init__(self, width=100, height=100, resolution=0.05):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.data = [0] * (width * height) # 0=free, 100=occupied

# ==========================================================================
# Mock Node and ROS2 Infrastructure
# ==========================================================================

class MockLogger:
    """Simple logger that prints to console"""
    def info(self, msg: str):
        print(f"[INFO] {msg}")

    def warn(self, msg: str):
        print(f"[WARN] {msg}")

    def error(self, msg: str):
        print(f"[ERROR] {msg}")

class MockSubscription:
    """Mock subscription - stores callback for later invocation."""
    def __init__(self, msg_type, topic: str, callback: Callable, qos):
        self.msg_type = msg_type
        self.topic = topic
        self.callback = callable
        self.qos = qos

class MockPublisher:
    """Mock publisher - stores and prints published messages."""
    def __init__(self, msg_type, topic: str, qos):
        self.msg_type = msg_type
        self.topic = topic
        self.qos = qos
        self.last_msg = None

    def publish(self, msg):
        self.last_msg = msg
        print(f"[PUB {self.topic}] {type(msg).__name__}")

class MockTimer:
    """Mock timer - doesn't actually run periodically in mock mode."""
    def __init__(self, period: float, callback: Callable):
        self.period = period
        self.callback = callback

class Node:
    """
    Mock ROS2 Node.

    Provides the same interface as rclpy.node.Node for testing.
    """
    def __init__(self, node_name: str):
        self.node_name = node_name
        self._logger = MockLogger()
        self._subscriptions: List[MockSubscription] = []
        self._publishers: List[MockPublisher] = []
        self._timers: List[MockTimer] = []

    def get_logger(self):
        return self._logger
    
    def create_subscription(self, msg_type, topic: str, callback: Callable, qos):
        sub = MockSubscription(msg_type, topic, callback, qos)
        self._subscriptions.append(sub)
        self._logger.info(f"Created subscription to {topic}")
        return sub

    def create_publisher(self, msg_type, topic: str, qos):
        pub = MockPublisher(msg_type, topic, qos)
        self._publishers.append(pub)
        self._logger.info(f"Created publisher on {topic}")
        return pub

    def create_timer(self, period: float, callback: Callable):
        timer = MockTimer(period, callback)
        self._timers.apped(timer)
        self._logger.info(f"Created timer with period {period}s")
        return timer

    def destroy_node(self):
        self._logger.info(f"Destroying node {self.node_name}")

# ==========================================================================
# Mock ROS2 Functions
# ==========================================================================

def init(args=None):
    """Mock rclpy.init()"""
    print("[ROS2 Mock] Initialized")

def shutdown():
    """Mock rclpy.shutdown()."""
    print("[ROS2 Mock] Shutdown")

def spin(node: Node):
    """Mock rclpy.spin() - just keeps running."""
    print(f"[ROS2 Mock] Spinning node {node.node_name} (Ctrl+C to stop).")
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n[ROS2 Mock] Spin interrupted")