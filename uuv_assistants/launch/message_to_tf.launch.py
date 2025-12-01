# Copyright (c) 2025 The Nerites Authors.
# All rights reserved.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution

def generate_launch_description():
    # Launch arguments (equivalent to <arg> tags)
    namespace_arg = DeclareLaunchArgument('namespace', default_value='rexrov')
    world_frame_arg = DeclareLaunchArgument('world_frame', default_value='world')

    child_frame_id_arg = DeclareLaunchArgument(
        'child_frame_id',
        default_value=PathJoinSubstitution([
            TextSubstitution(text='/'),
            LaunchConfiguration('namespace'),
            TextSubstitution(text='base_link')
        ])
    )

    odometry_topic_arg = DeclareLaunchArgument(
        'odometry_topic',
        default_value=PathJoinSubstitution([
            TextSubstitution(text='/'),
            LaunchConfiguration('namespace'),
            TextSubstitution(text='pose_gt')
        ])
    )

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')

    # Node definition (equivalent to <node> in XML)
    ground_truth_node = Node(
        package='uuv_assistants',
        executable='uuv_message_to_tf',
        name=['ground_truth_to_tf_', LaunchConfiguration('namespace')],
        output='screen',
        parameters=[{
            'odometry_topic': LaunchConfiguration('odometry_topic'),
            'frame_id': LaunchConfiguration('world_frame'),
            'stabilized_frame_id': ['/', LaunchConfiguration('namespace'), '/base_stabilized'],
            'footprint_frame_id': ['/', LaunchConfiguration('namespace'), '/base_footprint'],
            'child_frame_id': LaunchConfiguration('child_frame_id'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # Namespace grouping (equivalent to <group> with <push-ros-namespace>)
    group_action = GroupAction([
        PushRosNamespace(LaunchConfiguration('namespace')),
        ground_truth_node
    ])

    # Build and return the launch description
    return LaunchDescription([
        namespace_arg,
        world_frame_arg,
        child_frame_id_arg,
        odometry_topic_arg,
        use_sim_time_arg,
        group_action
    ])
