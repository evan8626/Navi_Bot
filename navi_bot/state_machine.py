#!/usr/bin/env python3
"""
Mission State Machine

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

from enum import Enum, auto

class RobotState(Enum):
    """Robot operational states."""
    IDLE = auto()
    READY = auto()
    PICK_NAV = auto()
    DELIVERY_NAV = auto()
    PICKING_UP = auto()
    PICKED_UP = auto()
    DELIVERING = auto()
    DELIVERED = auto()
    CHARGING = auto()
    ERROR = auto()

class Mission:
    """
    Represents a delivery mission.
    
    Status Values:
    'pending'     - assigned, but nothing done yet
    'picking_up'  - robot is picking up item
    'picked_up'   - item is on the robot
    'delivering'  - item is being delivered
    'delivered'   - mission complete/item offloaded
    'navigating' - robot is going to a goal
    """
    
    def __init__(self, mission_id, pickup_location, delivery_location):
        self.mission_id = mission_id
        self.pickup_location = pickup_location
        self.delivery_location = delivery_location
        self.status = 'pending'

class StateMachine(Node):
    """
    State machine for mission planning and execution.

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
        
        # Mission Objectives
        self.pickup_complete = False
        self.delivery_complete = False
        
        # Operation Flags
        self._pickup_timer_started = False
        self._delivery_timer_started = False

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
        self.nav_status_sub = self.create_subscription(String, '/nav_status', self.nav_status_callback, 10)

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
        """
        self.previous_state = self.current_state

        # State transition logic
        if self.has_error:
            self.transition_state(RobotState.ERROR)
            self.handle_error_state()
        elif self.current_state == RobotState.IDLE:
            self.handle_idle_state()
        elif self.current_state == RobotState.PICK_NAV:
            self.handle_navigating_to_pickup()
        elif self.current_state == RobotState.PICKING_UP:
            self.handle_picking_up()
        elif self.current_state == RobotState.PICKED_UP:
            self.handle_navigating_to_delivery()
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
            self.get_logger().info("Mission queue is empty. No missions to process.\n") 
            self.get_logger().info(f"Robot is {self.current_state}.\n")
            self.transition_state(RobotState.IDLE)
            return
        
        elif self.battery_level <= self.min_mission_threshold:
            self.get_logger().info(f"Battery level too low to begin mission.\n")
            self.get_logger().info(f"Battery level must be at or above {self.min_mission_threshold}. Current level is: {self.battery_level}.\n")
            self.transition_state(RobotState.CHARGING)
            return
        
        self.current_mission = self.mission_queue.pop(0)
        self.get_logger().info(f"Mission found and assigned, battery level is {self.battery_level}.\n")
        if self.current_mission.status == 'pending' or self.current_mission.status == 'delivered':
            self.transition_state(RobotState.PICK_NAV)
        elif self.current_mission.status == 'picked_up':
            self.transition_state(RobotState.DELIVERY_NAV)

    def handle_navigating_to_pickup(self):
        """Handle PICK_NAV state logic"""
        self.current_mission.status = 'navigating'
        if self.is_at_goal:
            self.current_mission.status = 'picking_up'
            self.transition_state(RobotState.PICKING_UP)

    def handle_picking_up(self):
        """Handle PICKING_UP state logic"""
        if not self._pickup_timer_started:
            self.get_logger().info("At goal for item pickup.\n")
            self.get_logger().info("Picking up...\n")
            self._pickup_timer_started = True
            self.create_timer(2.0, self._pickup_done)
            
        if self.pickup_complete:
            self.pickup_complete = False
            self._pickup_timer_started = False
            self.current_mission.status = 'picked_up'
            self.is_at_goal = False
            self.transition_state(RobotState.PICKED_UP)
    
    def _pickup_done(self):
        self.pickup_complete = True
    
    def handle_navigating_to_delivery(self):
        """Handle DELIVERY_NAV state logic"""
        self.current_mission.status = 'navigating'
        if self.is_at_goal:
            self.current_mission.status = 'delivering'
            self.transition_state(RobotState.DELIVERING)
        
    def handle_delivery(self):
        """Handle DELIVERING state logic"""
        if not self._delivery_timer_started:
            self.get_logger().info("At goal for item drop-off.\n")
            self.get_logger().info("Delivering...\n")
            self._delivery_timer_started = True
            self.create_timer(2.0, self._delivery_done)
            
        if self.delivery_complete:
            self.delivery_complete = False
            self._delivery_timer_started = False
            self.current_mission.status = 'delivered'
            self.is_at_goal = False
            self.transition_state(RobotState.IDLE)
            
    def _delivery_done(self):
        self.delivery_complete = True

    def handle_charging(self):
        """Handle CHARGING state logic"""
        if self.battery_level <= self.min_mission_threshold:
            # Charge 10% above battery low level regardless of if mission is assigned
            # Set this way to prevent immediate return to charging station upon completion of task
            self.get_logger().info(f"Robot battery level is: {self.battery_level}.\n")
            self.get_logger().info("Battery must be greater than 30% to accept mission.\n")
            self.transition_state(RobotState.CHARGING)
        
        elif (self.battery_level > self.min_mission_threshold and self.battery_level < self.max_threshold) and self.current_mission is None:
            # Continue charging if above the 30% threshold, and no mission assigned
            self.get_logger().info(f"No mission assigned. Battery level is: {self.battery_level}.\n")
            self.get_logger().info("Robot will continue to charge until mission is assigned.\n")
            self.transition_state(RobotState.CHARGING)
        
        elif self.battery_level > self.max_threshold and self.current_mission is None:
            # Stop charging battery above 95% charge
            self.get_logger().info(f"Robot battery level is: {self.battery_level}.\n")
            self.get_logger().info("Setting robot to IDLE.\n")
            self.transition_state(RobotState.IDLE)
        
        elif self.battery_level > self.min_mission_threshold and self.current_mission:
            # Go from charging to mission/NAVIGATION state
            self.get_logger().info(f"Mission assigned, battery level is {self.battery_level}. READY.\n")
            if self.current_mission.status == 'pending' or self.current_mission.status == 'delivered':
                self.transition_state(RobotState.PICK_NAV)
            elif self.current_mission.status == 'picked_up':
                self.transition_state(RobotState.DELIVERY_NAV)
        
        else:
            # Unknown state/Error
            self.transition_state(RobotState.ERROR)
        
    def handle_error_state(self):
        """Handle ERROR state logic"""
        self.get_logger().error(
            f"ERROR state entered from {self.previous_state}."
            f"Mission: {self.current_mission.mission_id if self.current_mission else 'None'}."
            f"Battery: {self.battery_level}"
        )
        self.has_error = False
        self.transition_state(RobotState.IDLE)
    
    def transition_state(self, state):
        """Handles state transition"""
        self.previous_state = self.current_state
        self.current_state = state
        

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