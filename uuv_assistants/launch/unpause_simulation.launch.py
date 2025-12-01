# Copyright (c) 2025 The Nerites Authors.
# All rights reserved.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare the timeout argument with a default value of 0
    timeout_arg = DeclareLaunchArgument('timeout', default_value='0')

    # Define the node
    unpause_simulation_node = Node(
        package='uuv_assistants',
        executable='unpause_simulation.py',
        name='unpause_simulation',
        parameters=[{'timeout': LaunchConfiguration('timeout')}],
        output='screen'
    )

    return LaunchDescription([
        timeout_arg,
        unpause_simulation_node
    ])
