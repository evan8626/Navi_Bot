#!/usr/bin/env pthon3
"""
Mission State MAchein

Manages high-level robot behavior and mission execution.
States: IDLE, NAVIGATION, PICKING_UP, DELIVERING, CHARGING, ERROR
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

class RobotState(Enum):
    """Robot operational states."""
    IDLE = auto()
    READY = auto()
    NAVIGATION = auto()
    PICKING_UP = auto()
    DELIVERING = auto()
    CHARGING = auto()
    ERROR = auto()

class Mission:
    """Represents a delivery mission."""

    def __init__(self, mission_id, pickup_location, delivery_location):
        self.mission_id = mission_id
        self.pickup_location = pickup_location
        self.delivery_location = delivery_location
        self.status = 'pending'

class StateMachine(Node):
    """
    State machien for mission planning and execution.

    Handles transitions between states based on:
    - Mission queue
    - Battery level
    - Navigational status
    - Error conditions
    """

    def __init__(self):
        super().__init__('state_machine')

        # State
        self.current_state = RobotState.IDLE
        self.previous_state = None

        # Mission queue
        self.mission_queue = []
        self.current_mission = None

        # Robot status
        self.battery_level = 100.0
        self.is_at_goal = False
        self.has_error = False

        # Parameters
        self.declare_parameter('max_charge_threshold', 98.0)
        self.declare_parameter('min_mission_begin_threshold', 30.0)
        self.declare_parameter('low_battery_threshold', 20.0)
        self.declare_parameter('critical_battery_threshold', 10.0)
        
        # Battery Thresholds
        self.max_threshold = self.get_parameter('max_charge_threshold').get_parameter_value().double_value
        self.min_mission_threshold = self.get_parameter('min_mission_begin_threshold').get_parameter_value().double_value
        self.low_threshold = self.get_parameter('low_battery_threshold').get_parameter_value().double_value
        self.critical_threshold = self.get_parameter('critical_battery_threshold').get_parameter_value().double_value

        # Publishers
        self.state_pub = self.create_publisher(String, '/robot_state', 10)
        self.goal_pub = self.create_publisher(Pose2D, '/goal_pose', 10)

        # Subscribers
        self.nav_status_sub = self.create_subscription(
            String, '/nav_status', self.nav_status_callback, 10)

        # State machine timer (runs at 5Hz)
        self.sm_timer = self.create_timer(0.2, self.update_state_machine)

        self.get_logger().info('State Machine initialized')

    def add_mission(self, mission):
        """Add a mission to the queue"""
        self.mission_queue.append(mission)
        self.get_logger().info(f'Added mission {mission.mission_id}')
    
    def nav_status_callback(self, msg):
        """Update navigation status."""
        if msg.data == 'goal_reached':
            self.is_at_goal = True
        elif msg.data == 'error':
            self.has_error = True

    def update_state_machine(self):
        """
        State machine update logic.

        TODO: Implement full state transition logic:
        - Check battery level
        - Process mission queue
        - Handle navigation completion
        - Detect and handle errors
        """
        self.previous_state = self.current_state

        # State transition logic
        if self.current_state == RobotState.IDLE:
            self.handle_idle_state()
        elif self.current_state == RobotState.NAVIGATION:
            self.handle_navigation_state()
        elif self.current_state == RobotState.PICKING_UP:
            self.handle_picking_up()
        elif self.current_state == RobotState.DELIVERING:
            self.handle_delivery()
        elif self.current_state == RobotState.CHARGING:
            self.handle_charging()
        elif self.current_state == RobotState.ERROR:
            self.handle_error_state()
        
        # Publish state if changed
        if self.current_state != self.previous_state:
            self.state_pub.publish(String(data=self.current_state.name))
            self.get_logger().info(
                f'State transition: {self.previous_state.name} -> {self.current_state.name}')

    def handle_idle_state(self):
        """Handle IDLE state logic"""
        
        if not self.mission_queue:
            print("Mission queue is empty. No missions to process.\n") 
            print(f"Robot is {self.current_state}.\n")
            self.previous_state = self.current_state
            self.current_state = RobotState.IDLE
            return self.current_state
        
        elif self.battery_level <= self.min_mission_threshold:
            print(f"Battery level too low to begin mission.\n")
            print(f"Battery level must be at or above {self.min_mission_thresholdd}. Current level is: {self.battery_level}.\n")
            self.previous_state = self.current_state
            self.current_state = RobotState.CHARGING
            return self.current_state
        
        self.current_mission = self.mission_queue.pop(0)
        print(f"Mission found and assigned, battery level is {self.battery_level}.\n")
        self.previous_state = self.current_state
        self.current_state = RobotState.NAVIGATION
        return self.current_state

    def handle_navigation_state(self):
        """Handle NAVIGATING state logic."""
        # TODO: Monitor navigation progress
        # TODO: Check for goal reached
        pass

    def handle_picking_up(self):
        """Handle PICKING_UP state logic"""
        print("At goal for item pickup.\n")
        self.previous_state = self.current_state
        self.current_state = RobotState.PICKING_UP
        print("Picking up...\n")
        time.sleep(2) # this is here to simulate the item getting picked up. Could simulate a prox sensor getting tripped, or a limit switch to behave like an indication the item was picked up instead.
        print("Item picked up.\n")
        

    def handle_delivery(self):
        """Handle DELIVERY state logic"""
        # TODO: Simulate delivery operation
        pass

    def handle_charging(self):
        """Handle CHARGING state logic"""
        if self.battery_level <= self.min_mission_threshold:
            # Charge 10% above battery low level regardless of if mission is assigned
            # Set this way to prevent immediate return to charging station upon completion of task
            print(f"Robot battery level is: {self.battery_level}.\n")
            print("Battery must be greater than 30% to accept mission.\n")
            self.previous_state = self.current_state
            self.current_state = RobotState.CHARGING
            return self.current_state
        
        elif self.battery_level > self.min_mission_threshold and self.current_mission is None:
            # Continue charging if above the 30% threshold, and no mission assigned
            print(f"No mission assigned. Battery level is: {self.battery_level}.\n")
            print("Robot will continue to charge until mission is assigned.\n")
            self.previous_state = self.current_state
            self.current_state = RobotState.CHARGING
            return self.current_state
        
        elif self.battery_level > self.max_threshold and self.current_mission is None:
            # Stop charging battery above 95% charge
            print(f"Robot battery level is: {self.battery_level}.\n")
            print("Setting robot to IDLE.\n")
            self.previous_state = self.current_state
            self.current_state = RobotState.IDLE
            return self.current_state
        
        elif self.battery_level > self.min_mission_threshold and self.current_mission:
            # Go from charging to mission/NAVIGATION state
            print(f"Mission assigned, battery level is {self.battery_level}. READY.\n")
            self.previous_state = self.current_state
            self.current_state = RobotState.NAVIGATION
            return self.current_state
        
        else:
            # Unknown state/Error
            self.previous_state = self.current_state
            self.current_state = RobotState.ERROR
            return self.current_state
        
    def handle_error_state(self):
        """Handle ERROR state logic"""
        # TODO: Implement error recovery
        # ERROR state can also handle UKNOWN state?
        pass

def main(args=None):
    rclpy.init(args=args)
    state_machine = StateMachine()

    try:
        rclpy.spin(state_machine)
    except KeyboardInterrupt:
        pass
    finally:
        state_machine.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()