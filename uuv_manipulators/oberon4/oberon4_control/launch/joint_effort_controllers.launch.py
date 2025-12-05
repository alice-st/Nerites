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
        default_value='oberon4',
        description='Arm name (e.g. oberon4)',
    )

    # Joints for which we want effort controllers
    joint_suffixes = [
        'azimuth',
        'shoulder',
        'wrist_joint',
        'jaw_1_joint',
        'jaw_2_joint',
    ]

    spawners = []

    for joint in joint_suffixes:
        # Controller name WITHOUT any slashes – valid as a ROS node name suffix
        controller_name = [arm_name, '_', joint, '_controller']

        spawners.append(
            Node(
                package='controller_manager',
                executable='spawner.py',
                name=['controller_spawner_', arm_name, '_', joint],
                namespace=uuv_name,
                output='screen',
                arguments=[controller_name,'-c', 'controller_manager'],
            )
        )

    group = GroupAction([
        PushRosNamespace(uuv_name),  # /rexrov
        *spawners,
    ])

    return LaunchDescription([
        declare_uuv_name,
        declare_arm_name,
        group,
    ])
