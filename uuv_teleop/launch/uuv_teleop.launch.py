#!/usr/bin/env python3

# Copyright (c) 2025 The Nerites Authors.
# All rights reserved.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    uuv_name = LaunchConfiguration('uuv_name').perform(context)
    joy_id = LaunchConfiguration('joy_id').perform(context)
    deadman_button = LaunchConfiguration('deadman_button').perform(context)
    exclusion_buttons = LaunchConfiguration('exclusion_buttons').perform(context)
    axis_roll = LaunchConfiguration('axis_roll').perform(context)
    axis_pitch = LaunchConfiguration('axis_pitch').perform(context)
    axis_yaw = LaunchConfiguration('axis_yaw').perform(context)
    axis_x = LaunchConfiguration('axis_x').perform(context)
    axis_y = LaunchConfiguration('axis_y').perform(context)
    axis_z = LaunchConfiguration('axis_z').perform(context)
    gain_roll = LaunchConfiguration('gain_roll').perform(context)
    gain_pitch = LaunchConfiguration('gain_pitch').perform(context)
    gain_yaw = LaunchConfiguration('gain_yaw').perform(context)
    gain_x = LaunchConfiguration('gain_x').perform(context)
    gain_y = LaunchConfiguration('gain_y').perform(context)
    gain_z = LaunchConfiguration('gain_z').perform(context)
    output_topic = LaunchConfiguration('output_topic').perform(context)
    message_type = LaunchConfiguration('message_type').perform(context)

    return [
        GroupAction([
            PushRosNamespace(uuv_name),

            # Joystick Node
            Node(
                package='joy',
                executable='joy_node',
                name='joystick',
                parameters=[{
                    'autorepeat_rate': 10.0,
                    'dev': f"/dev/input/js{joy_id}"
                }]
            ),

            # Teleop Node
            Node(
                package='uuv_teleop',
                executable='vehicle_teleop.py',
                name='joy_uuv_velocity_teleop',
                output='screen',
                remappings=[
                    ('output', f"/{uuv_name}/{output_topic}"),
                    ('joy', f"/{uuv_name}/joy"),
                ],
                parameters=[{
                    'type': message_type,
                    'deadman_button': int(deadman_button),
                    'exclusion_buttons': exclusion_buttons,  # as string for now (parsed by teleop node)
                    'mapping': {
                        'x': {'axis': int(axis_x), 'gain': float(gain_x)},
                        'y': {'axis': int(axis_y), 'gain': float(gain_y)},
                        'z': {'axis': int(axis_z), 'gain': float(gain_z)},
                        'roll': {'axis': int(axis_roll), 'gain': float(gain_roll)},
                        'pitch': {'axis': int(axis_pitch), 'gain': float(gain_pitch)},
                        'yaw': {'axis': int(axis_yaw), 'gain': float(gain_yaw)},
                    }
                }]
            )
        ])
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('uuv_name'),
        DeclareLaunchArgument('joy_id', default_value='0'),
        DeclareLaunchArgument('deadman_button', default_value='-1'),
        DeclareLaunchArgument('exclusion_buttons', default_value='4,5'),
        DeclareLaunchArgument('axis_roll', default_value='-1'),
        DeclareLaunchArgument('axis_pitch', default_value='-1'),
        DeclareLaunchArgument('axis_yaw', default_value='0'),
        DeclareLaunchArgument('axis_x', default_value='4'),
        DeclareLaunchArgument('axis_y', default_value='3'),
        DeclareLaunchArgument('axis_z', default_value='1'),
        DeclareLaunchArgument('gain_roll', default_value='0.0'),
        DeclareLaunchArgument('gain_pitch', default_value='0.0'),
        DeclareLaunchArgument('gain_yaw', default_value='0.2'),
        DeclareLaunchArgument('gain_x', default_value='0.3'),
        DeclareLaunchArgument('gain_y', default_value='0.3'),
        DeclareLaunchArgument('gain_z', default_value='0.3'),
        DeclareLaunchArgument('output_topic', default_value='cmd_vel'),
        DeclareLaunchArgument('message_type', default_value='twist'),

        OpaqueFunction(function=launch_setup)
    ])
