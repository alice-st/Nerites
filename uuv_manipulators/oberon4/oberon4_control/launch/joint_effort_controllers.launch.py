#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
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

    # Controllers as defined in oberon4_ros2_control.yaml
    controllers = [
        'joint_state_broadcaster',
        'oberon4_arm_controller',
    ]

    def ctrl_name(name: str):
        # YAML keys are exactly the controller names
        return name

    controller_spawners = [
        Node(
            package='controller_manager',
            executable='spawner.py',
            name=['controller_spawner_', name],
            output='screen',
            arguments=[
                ctrl_name(name),
                '--controller-manager', '/controller_manager',
            ],
        )
        for name in controllers
    ]

    group = GroupAction([
        # spawner nodes live under /<uuv_name>, controller_manager is global
        PushRosNamespace(uuv_name),
        *controller_spawners,
    ])

    return LaunchDescription([
        declare_uuv_name,
        declare_arm_name,
        group,
    ])
