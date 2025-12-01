# Copyright (c) 2025 The Nerites Authors.
# All rights reserved.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # ------------------------
    # Declare launch arguments
    # ------------------------
    args = [
        DeclareLaunchArgument('starting_time', default_value='0.0'),
        DeclareLaunchArgument('end_time', default_value='-1'),
        DeclareLaunchArgument('current_vel', default_value='1'),
        DeclareLaunchArgument('horizontal_angle', default_value='0.0'),
        DeclareLaunchArgument('vertical_angle', default_value='0.0'),
    ]

    # ------------------------
    # Define the node
    # ------------------------
    timed_current_node = Node(
        package='uuv_control_utils',
        executable='set_timed_current_perturbation.py',
        name='set_timed_current_perturbation',
        output='screen',
        parameters=[{
            'starting_time': LaunchConfiguration('starting_time'),
            'end_time': LaunchConfiguration('end_time'),
            'current_velocity': LaunchConfiguration('current_vel'),
            'horizontal_angle': LaunchConfiguration('horizontal_angle'),
            'vertical_angle': LaunchConfiguration('vertical_angle')
        }]
    )

    # ------------------------
    # Return launch description
    # ------------------------
    return LaunchDescription(args + [timed_current_node])
