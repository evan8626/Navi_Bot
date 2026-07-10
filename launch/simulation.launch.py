#!/usr/bin/env python3
"""
Simulation launch file.

Wraps robot.launch.py with simulation time enabled and adds visualization
(RViz). The Gazebo physics, simulated sensor publishers, and the
TF/robot_state_publisher stack are scaffolded below but stay disabled until
their artifacts (a robot URDF and a world file) exist.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('navi_bot')

    use_rviz = LaunchConfiguration('use_rviz')

    # Bring up the core robot nodes with the sim clock.
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'robot.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    # Only pass `-d <config>` when the RViz config actually exists, so a missing
    # file doesn't abort the launch — RViz opens with its default layout instead.
    rviz_config = os.path.join(pkg_dir, 'config', 'navi_bot.rviz')
    rviz_args = ['-d', rviz_config] if os.path.exists(rviz_config) else []

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz for visualization',
        ),

        robot_launch,

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=rviz_args,
            parameters=[{'use_sim_time': True}],
            output='screen',
            condition=IfCondition(use_rviz),
        ),

        # --- Physics / sensors / TF (disabled — need a URDF + world) ----------
        # These require artifacts that don't exist yet:
        #   * a robot description (URDF/xacro) for robot_state_publisher + TF
        #   * a Gazebo world file
        #   * a spawn entity for the model
        # Scaffolding for when they land:
        #
        # robot_state_publisher (publishes TF from the URDF):
        #   Node(package='robot_state_publisher', executable='robot_state_publisher',
        #        parameters=[{'use_sim_time': True, 'robot_description': <urdf_xml>}],
        #        output='screen'),
        #
        # Gazebo:
        #   IncludeLaunchDescription(gazebo_ros/launch/gazebo.launch.py) with a world arg
        #
        # Spawn the model:
        #   Node(package='gazebo_ros', executable='spawn_entity.py',
        #        arguments=['-topic', 'robot_description', '-entity', 'navi_bot']),
    ])
