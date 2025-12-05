#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Args
    uuv_name = LaunchConfiguration('uuv_name')
    arm_name = LaunchConfiguration('arm_name')
    deadman_button = LaunchConfiguration('deadman_button')
    exclusion_button = LaunchConfiguration('exclusion_button')
    open_button = LaunchConfiguration('open_button')
    close_button = LaunchConfiguration('close_button')

    declare_uuv_name = DeclareLaunchArgument(
        'uuv_name',
        description='UUV name / namespace (e.g. rexrov)',
    )
    declare_arm_name = DeclareLaunchArgument(
        'arm_name',
        default_value='oberon7',
        description='Arm name',
    )
    declare_deadman_button = DeclareLaunchArgument(
        'deadman_button',
        default_value='5',
        description='Deadman button index',
    )
    declare_exclusion_button = DeclareLaunchArgument(
        'exclusion_button',
        default_value='4',
        description='Exclusion button index',
    )
    declare_open_button = DeclareLaunchArgument(
        'open_button',
        default_value='1',
        description='Gripper open button index',
    )
    declare_close_button = DeclareLaunchArgument(
        'close_button',
        default_value='2',
        description='Gripper close button index',
    )

    # YAML config for gripper controller
    oberon7_desc_share = get_package_share_directory('oberon7_description')
    gripper_config_yaml = os.path.join(
        oberon7_desc_share,
        'params',
        'parallel_gripper_config.yaml',
    )

    # Controller spawner for finger joints
    controller_spawner = Node(
        package='controller_manager',
        executable='spawner.py',
        name=['controller_spawner_', arm_name, '_gripper'],
        output='screen',
        arguments=[
            ['/', uuv_name, '/', arm_name, '/finger_left_joint/controller'],
            ['/', uuv_name, '/', arm_name, '/finger_right_joint/controller'],
        ],
    )

    # Group under /<uuv_name>
    group_uuv = GroupAction([
        PushRosNamespace(uuv_name),
        controller_spawner,
    ])

    # Gripper controller node under /<uuv_name>/<arm_name>
    gripper_controller = Node(
        package='uuv_manipulators_control',
        executable='gripper_controller.py',
        name=['gripper_controller_', arm_name],
        output='screen',
        remappings=[
            ('joy', ['/', uuv_name, '/joy']),
        ],
        parameters=[
            gripper_config_yaml,
            {
                'exclusion_buttons': exclusion_button,
                'deadman_button': deadman_button,
                'open_button': open_button,
                'close_button': close_button,
                'kp': 100.0,
                'ki': 4.0,
                'kd': 1.0,
            },
        ],
    )

    group_arm = GroupAction([
        PushRosNamespace([uuv_name, '/', arm_name]),
        gripper_controller,
    ])

    return LaunchDescription([
        declare_uuv_name,
        declare_arm_name,
        declare_deadman_button,
        declare_exclusion_button,
        declare_open_button,
        declare_close_button,
        group_uuv,
        group_arm,
    ])
