#!/usr/bin/env python3

# Copyright (c) 2025 The Nerites Authors.
# All rights reserved.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    uuv_name = LaunchConfiguration('uuv_name').perform(context)
    output_topic = LaunchConfiguration('output_topic').perform(context)
    message_type = LaunchConfiguration('message_type').perform(context)

    return [
        GroupAction([
            PushRosNamespace(uuv_name),

            Node(
                package='uuv_teleop',
                executable='vehicle_keyboard_teleop.py',
                name='keyboard_uuv_velocity_teleop',
                output='screen',
                remappings=[
                    ('output', f"/{uuv_name}/{output_topic}")
                ],
                parameters=[{
                    'type': message_type
                }]
            )
        ])
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('uuv_name'),
        DeclareLaunchArgument('output_topic', default_value='cmd_vel'),
        DeclareLaunchArgument('message_type', default_value='twist'),

        OpaqueFunction(function=launch_setup)
    ])
