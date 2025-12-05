#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    # Default waypoint file
    default_waypoint_file = os.path.join(
        get_package_share_directory('uuv_control_utils'),
        'config',
        'example_waypoints.yaml'
    )

    # Launch arguments
    uuv_name = LaunchConfiguration('uuv_name')
    filename = LaunchConfiguration('filename')
    start_time = LaunchConfiguration('start_time')
    interpolator = LaunchConfiguration('interpolator')

    declare_uuv_name_arg = DeclareLaunchArgument(
        'uuv_name',
        description='Name of the UUV (namespace)'
    )

    declare_filename_arg = DeclareLaunchArgument(
        'filename',
        default_value=default_waypoint_file,
        description='Waypoint YAML file'
    )

    declare_start_time_arg = DeclareLaunchArgument(
        'start_time',
        default_value='-1',
        description='Start time for sending waypoints'
    )

    declare_interpolator_arg = DeclareLaunchArgument(
        'interpolator',
        default_value='lipb',
        description='Interpolator type'
    )

    send_waypoint_node = Node(
        package='uuv_control_utils',
        executable='send_waypoint_file.py',
        name='send_waypoint_file',
        output='screen',
        parameters=[{
            'filename': filename,
            'start_time': start_time,
            'interpolator': interpolator
        }]
    )

    grouped = GroupAction([
        PushRosNamespace(uuv_name),
        send_waypoint_node
    ])

    return LaunchDescription([
        declare_uuv_name_arg,
        declare_filename_arg,
        declare_start_time_arg,
        declare_interpolator_arg,
        grouped
    ])
