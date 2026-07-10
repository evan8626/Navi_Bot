#!/usr/bin/env python3
"""
Main launch file for Navi bot.

Brings up the core nodes that run on the robot (real or simulated):
- robot_controller : the 50 Hz control loop
- state_machine    : mission sequencing
- path_planner     : global + local planning

Sensor simulation and visualization live in simulation.launch.py; this file is
the minimal "real robot" bring-up.

Note: config/*.yaml (robot_params, control_params, mission_config) are the
project's own APPLICATION config, not ROS 2 parameter files -- they use a plain
`robot:`/`sensors:` layout, not the `<node>: ros__parameters:` schema that rcl
requires (feeding them as node params makes rcl fail to parse and the node dies
on startup). They're installed to share/navi_bot/config for nodes to load
directly. To promote a value to a real ROS 2 parameter, reformat it under
`<node_name>: ros__parameters:`, declare_parameter() it in the node, then add
the file back to that Node(parameters=[...]) below.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declared below; simulation.launch.py overrides this to 'true'.
    use_sim_time = LaunchConfiguration('use_sim_time')
    common = [{'use_sim_time': use_sim_time}]

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use the simulation (Gazebo) clock instead of wall time',
        ),

        # Static map, published once and latched (TRANSIENT_LOCAL) so the
        # planner/controller receive it whenever they come up.
        Node(
            package='navi_bot',
            executable='map_publisher',
            name='map_publisher',
            parameters=common,
            output='screen',
            emulate_tty=True,
        ),

        Node(
            package='navi_bot',
            executable='robot_controller',
            name='robot_controller',
            parameters=common,
            output='screen',
            emulate_tty=True,
        ),

        Node(
            package='navi_bot',
            executable='state_machine',
            name='state_machine',
            parameters=common,
            output='screen',
            emulate_tty=True,
        ),

        Node(
            package='navi_bot',
            executable='path_planner',
            name='path_planner',
            parameters=common,
            output='screen',
            emulate_tty=True,
        ),

        # --- Sensor simulation -------------------------------------------------
        # No sim-sensor executable exists yet. When one is added, register it in
        # setup.py console_scripts (e.g. 'sensor_sim = navi_bot.sensors.sim:main')
        # and enable it here:
        #
        # Node(
        #     package='navi_bot',
        #     executable='sensor_sim',
        #     name='sensor_sim',
        #     parameters=common,
        #     output='screen',
        # ),
    ])
