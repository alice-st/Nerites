#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    # Launch arguments
    uuv_name = LaunchConfiguration('uuv_name')
    arm_name = LaunchConfiguration('arm_name')

    declare_uuv_name = DeclareLaunchArgument(
        'uuv_name',
        description='UUV name / namespace (e.g. rexrov)',
    )

    declare_arm_name = DeclareLaunchArgument(
        'arm_name',
        default_value='oberon7',
        description='Arm name (e.g. oberon7)',
    )

    # Controller spawner node (ROS 2 controller_manager spawner)
    controller_spawner = Node(
        package='controller_manager',
        executable='spawner.py',
        name=['controller_spawner_', arm_name],
        output='screen',
        # In ROS 2 the spawner takes controller names as separate args
        arguments=[
            ['/', uuv_name, '/', arm_name, '/azimuth/controller'],
            ['/', uuv_name, '/', arm_name, '/shoulder/controller'],
            ['/', uuv_name, '/', arm_name, '/elbow/controller'],
            ['/', uuv_name, '/', arm_name, '/roll/controller'],
            ['/', uuv_name, '/', arm_name, '/pitch/controller'],
            ['/', uuv_name, '/', arm_name, '/wrist/controller'],
        ],
        # Optional: carry the PID configuration as parameters
        parameters=[{
            'controller_config': {
                'azimuth': {
                    'controller': {
                        'type': 'effort_controllers/JointEffortController',
                        'joint': [arm_name, '/azimuth'],
                        'pid': {'p': 2000.0, 'i': 100.0, 'd': 50.0},
                    },
                },
                'shoulder': {
                    'controller': {
                        'type': 'effort_controllers/JointEffortController',
                        'joint': [arm_name, '/shoulder'],
                        'pid': {'p': 2000.0, 'i': 100.0, 'd': 50.0},
                    },
                },
                'elbow': {
                    'controller': {
                        'type': 'effort_controllers/JointEffortController',
                        'joint': [arm_name, '/elbow'],
                        'pid': {'p': 2000.0, 'i': 100.0, 'd': 50.0},
                    },
                },
                'roll': {
                    'controller': {
                        'type': 'effort_controllers/JointEffortController',
                        'joint': [arm_name, '/roll'],
                        'pid': {'p': 1000.0, 'i': 50.0, 'd': 10.0},
                    },
                },
                'pitch': {
                    'controller': {
                        'type': 'effort_controllers/JointEffortController',
                        'joint': [arm_name, '/pitch'],
                        'pid': {'p': 1000.0, 'i': 50.0, 'd': 10.0},
                    },
                },
                'wrist': {
                    'controller': {
                        'type': 'effort_controllers/JointEffortController',
                        'joint': [arm_name, '/wrist'],
                        'pid': {'p': 500.0, 'i': 30.0, 'd': 10.0},
                    },
                },
            },
        }],
    )

    group = GroupAction([
        PushRosNamespace(uuv_name),
        controller_spawner,
    ])

    return LaunchDescription([
        declare_uuv_name,
        declare_arm_name,
        group,
    ])
