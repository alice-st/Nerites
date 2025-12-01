# Copyright (c) 2025 The Nerites Authors.
# All rights reserved.

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uuv_assistants',
            executable='publish_footprints.py',
            name='publish_footprints',
            output='screen'
        )
    ])
