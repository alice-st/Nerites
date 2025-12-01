# Copyright (c) 2025 The Nerites Authors.
# All rights reserved.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch argument
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')

    # Node for static_transform_publisher
    world_ned_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_ned_frame_publisher',
        arguments=[
            '0', '0', '0',                     # x, y, z
            '1.5707963267948966', '0', '3.141592653589793',  # roll, pitch, yaw
            'world', 'world_ned'               # parent and child frames
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        world_ned_tf_node
    ])
