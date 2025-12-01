# Copyright (c) 2025 The Nerites Authors.
# All rights reserved.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    uuv_name = LaunchConfiguration('uuv_name')
    start_time = LaunchConfiguration('start_time')
    radius = LaunchConfiguration('radius')
    center_x = LaunchConfiguration('center_x')
    center_y = LaunchConfiguration('center_y')
    center_z = LaunchConfiguration('center_z')
    center = LaunchConfiguration('center')

    n_points = LaunchConfiguration('n_points')
    heading_offset = LaunchConfiguration('heading_offset')
    duration = LaunchConfiguration('duration')
    max_forward_speed = LaunchConfiguration('max_forward_speed')

    return LaunchDescription([
        DeclareLaunchArgument('uuv_name', default_value='rexrov'),
        DeclareLaunchArgument('start_time', default_value='-1'),
        DeclareLaunchArgument('radius', default_value='8'),
        DeclareLaunchArgument('center_x', default_value='4'),
        DeclareLaunchArgument('center_y', default_value='2'),
        DeclareLaunchArgument('center_z', default_value='-22'),
        DeclareLaunchArgument('center', default_value='4,2,-22'),
        DeclareLaunchArgument('n_points', default_value='50'),
        DeclareLaunchArgument('heading_offset', default_value='0'),
        DeclareLaunchArgument('duration', default_value='0'),
        DeclareLaunchArgument('max_forward_speed', default_value='0.3'),

        GroupAction([
            PushRosNamespace(namespace=uuv_name),
            Node(
                package='uuv_control_utils',
                executable='start_circular_trajectory.py',
                name='start_circular_trajectory',
                output='screen',
                parameters=[
                    {'start_time': start_time},
                    {'radius': radius},
                    {'center': center},
                    {'n_points': n_points},
                    {'heading_offset': heading_offset},
                    {'duration': duration},
                    {'max_forward_speed': max_forward_speed}
                ]
            )
        ])
    ])
