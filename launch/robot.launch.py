#!/usr/bin/env python3
"""
Main launch file for Navi bot

Launches all necessary nodes:
- Robot controller
- State machine
- Path planner
- Sensor nodes (simulated)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    pck_dir = get_package_share_directory('navi_bot')

    # Configuration files
    robot_params = os.path.join(pck_dir, 'config', 'robot_params.yaml')
    control_params = os.path.join(pck_dir, 'config', 'control_params.yaml')
    mission_config = os.path.join(pck_dir, 'config', 'mission_config.yaml')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        # Declare Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        # Robot controller node
        Node(
            package='navi_bot',
            executable='robot_controller',
            name='robot_controller',
            parameter=[
                robot_params,
                control_params,
                {'use_sim_time' : use_sim_time}
            ],
            output='screen'
        ),

        # State machine node
        Node(
            package='navi_bot',
            executable='state_machine',
            name='state_machine',
            parameters=[
                mission_config,
                {'use_sim_time': use_sim_time}
            ],
            output='screen'
        ),

        # Path planner node
        Node(
            package='navi_bot',
            executable='path_planner',
            name='path_planner',
            parameters=[
                robot_params,
                control_params,
                {'use_sim_time': use_sim_time}
            ],
            output='screen'
        )

        # TODO: Add sensor simulation nodes
        # TODO: Add visualization nodes (rviz2)
    ])