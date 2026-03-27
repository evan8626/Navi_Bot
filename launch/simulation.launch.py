#!/usr/bin/env python3
"""
Launch file for simulation environment.

Includes Gazebo simulator (optional) and visualization tools.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('navi_bot')

    # Include main robot launch file
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_dir, 'launch', 'robot.launch.py')
        ]),
        launch_arguments={'use_sim_time' : 'true'}.items()
    )

    return LaunchDescription([
        robot_launch,

        # RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_dir, 'config', 'navi_bot.rviz')],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),

        # TODO: Add Gazebo launch
        # TODO: Add simulated sensor publishers
        # TODO: Add robot state publisher for TF
    ])