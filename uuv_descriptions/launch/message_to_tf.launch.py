# Copyright (c) 2025 The Nerites Authors.
# All rights reserved.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    # ------------------------
    # Declare arguments
    # ------------------------
    args = [
        DeclareLaunchArgument('namespace', default_value='rexrov'),
    ]

    # ------------------------
    # Build the node name dynamically
    # ------------------------
    dynamic_name = PythonExpression([
        "'ground_truth_to_tf_' + str(", LaunchConfiguration('namespace'), ")"
    ])

    # ------------------------
    # Node definition
    # ------------------------
    ground_truth_node = Node(
        package='uuv_assistants',
        executable='uuv_message_to_tf',
        name=dynamic_name,
        output='screen',
        parameters=[{
            'odometry_topic': PythonExpression([
                "'/' + str(", LaunchConfiguration('namespace'), ") + '/pose_gt'"
            ]),
            'frame_id': '/world',
            'stabilized_frame_id': PythonExpression([
                "'/' + str(", LaunchConfiguration('namespace'), ") + '/base_stabilized'"
            ]),
            'footprint_frame_id': PythonExpression([
                "'/' + str(", LaunchConfiguration('namespace'), ") + '/base_footprint'"
            ]),
            'child_frame_id': PythonExpression([
                "'/' + str(", LaunchConfiguration('namespace'), ") + '/base_link'"
            ])
        }]
    )

    return LaunchDescription(args + [ground_truth_node])
