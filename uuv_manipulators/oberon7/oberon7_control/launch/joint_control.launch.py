#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    # --- Launch arguments ---
    uuv_name = LaunchConfiguration('uuv_name')
    arm_name = LaunchConfiguration('arm_name')
    deadman_button = LaunchConfiguration('deadman_button')
    exclusion_button = LaunchConfiguration('exclusion_button')
    axis_azimuth = LaunchConfiguration('axis_azimuth')
    axis_shoulder = LaunchConfiguration('axis_shoulder')
    axis_elbow = LaunchConfiguration('axis_elbow')
    axis_roll = LaunchConfiguration('axis_roll')
    axis_pitch = LaunchConfiguration('axis_pitch')
    axis_yaw = LaunchConfiguration('axis_yaw')
    gripper_open_button = LaunchConfiguration('gripper_open_button')
    gripper_close_button = LaunchConfiguration('gripper_close_button')
    home_button = LaunchConfiguration('home_button')

    declare_uuv_name = DeclareLaunchArgument(
        'uuv_name',
        description='UUV name / namespace (e.g. rexrov)',
    )
    declare_arm_name = DeclareLaunchArgument(
        'arm_name',
        default_value='oberon7',
        description='Arm name (e.g. oberon7)',
    )
    declare_deadman_button = DeclareLaunchArgument(
        'deadman_button',
        default_value='5',
        description='Deadman button index (RB on Xbox 360)',
    )
    declare_exclusion_button = DeclareLaunchArgument(
        'exclusion_button',
        default_value='4',
        description='Exclusion button index (LB on Xbox 360)',
    )
    declare_axis_azimuth = DeclareLaunchArgument(
        'axis_azimuth', default_value='3', description='Joystick axis for azimuth'
    )
    declare_axis_shoulder = DeclareLaunchArgument(
        'axis_shoulder', default_value='4', description='Joystick axis for shoulder'
    )
    declare_axis_elbow = DeclareLaunchArgument(
        'axis_elbow', default_value='1', description='Joystick axis for elbow'
    )
    declare_axis_roll = DeclareLaunchArgument(
        'axis_roll', default_value='6', description='Joystick axis for roll'
    )
    declare_axis_pitch = DeclareLaunchArgument(
        'axis_pitch', default_value='7', description='Joystick axis for pitch'
    )
    declare_axis_yaw = DeclareLaunchArgument(
        'axis_yaw', default_value='0', description='Joystick axis for wrist yaw'
    )
    declare_gripper_open_button = DeclareLaunchArgument(
        'gripper_open_button', default_value='1', description='Gripper open button'
    )
    declare_gripper_close_button = DeclareLaunchArgument(
        'gripper_close_button', default_value='2', description='Gripper close button'
    )
    declare_home_button = DeclareLaunchArgument(
        'home_button', default_value='7', description='Home button index'
    )

    # --- Included launch files (ROS 2 .launch.py versions) ---
    oberon7_control_share = get_package_share_directory('oberon7_control')

    joint_effort_launch = os.path.join(
        oberon7_control_share, 'launch', 'joint_effort_controllers.launch.py'
    )
    gripper_controllers_launch = os.path.join(
        oberon7_control_share, 'launch', 'gripper_controllers.launch.py'
    )

    joint_effort_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(joint_effort_launch),
        launch_arguments={
            'uuv_name': uuv_name,
            'arm_name': arm_name,
        }.items(),
    )

    gripper_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gripper_controllers_launch),
        launch_arguments={
            'uuv_name': uuv_name,
            'arm_name': arm_name,
            'exclusion_button': exclusion_button,
            'deadman_button': deadman_button,
            'open_button': gripper_open_button,
            'close_button': gripper_close_button,
        }.items(),
    )

    # --- Nodes under /<uuv_name>/<arm_name> ---
    kinematics_service_node = Node(
        package='uuv_manipulators_kinematics',
        executable='kinematics_service.py',
        name='kinematics_service',
        output='screen',
    )

    joint_position_controller_node = Node(
        package='uuv_manipulators_control',
        executable='joint_position_controller.py',
        name=['joint_position_controller_', arm_name],
        output='screen',
        remappings=[
            ('joy', ['/', uuv_name, '/joy']),
        ],
        parameters=[{
            'exclusion_buttons': exclusion_button,
            'deadman_button': deadman_button,
            'home_button': home_button,
            'controller_config': {
                'azimuth': {
                    'joint_input_axis': axis_azimuth,
                    'axis_gain': 0.01,
                    'topic': ['/', uuv_name, '/', arm_name, '/azimuth/controller/command'],
                    'controller': {'p': 1000.0, 'i': 20.0, 'd': 50.0},
                },
                'shoulder': {
                    'joint_input_axis': axis_shoulder,
                    'axis_gain': 0.01,
                    'topic': ['/', uuv_name, '/', arm_name, '/shoulder/controller/command'],
                    'controller': {'p': 1000.0, 'i': 20.0, 'd': 50.0},
                },
                'elbow': {
                    'joint_input_axis': axis_elbow,
                    'axis_gain': 0.01,
                    'topic': ['/', uuv_name, '/', arm_name, '/elbow/controller/command'],
                    'controller': {'p': 1000.0, 'i': 20.0, 'd': 50.0},
                },
                'roll': {
                    'joint_input_axis': axis_roll,
                    'axis_gain': 0.01,
                    'topic': ['/', uuv_name, '/', arm_name, '/roll/controller/command'],
                    'controller': {'p': 300.0, 'i': 10.0, 'd': 20.0},
                },
                'pitch': {
                    'joint_input_axis': axis_pitch,
                    'axis_gain': 0.1,
                    'topic': ['/', uuv_name, '/', arm_name, '/pitch/controller/command'],
                    'controller': {'p': 300.0, 'i': 10.0, 'd': 0.0},
                },
                'wrist': {
                    'joint_input_axis': axis_yaw,
                    'axis_gain': 0.01,
                    'topic': ['/', uuv_name, '/', arm_name, '/wrist/controller/command'],
                    'controller': {'p': 200.0, 'i': 10.0, 'd': 0.0},
                },
            },
        }],
    )

    group_arm = GroupAction([
        PushRosNamespace([uuv_name, '/', arm_name]),
        kinematics_service_node,
        joint_position_controller_node,
    ])

    return LaunchDescription([
        declare_uuv_name,
        declare_arm_name,
        declare_deadman_button,
        declare_exclusion_button,
        declare_axis_azimuth,
        declare_axis_shoulder,
        declare_axis_elbow,
        declare_axis_roll,
        declare_axis_pitch,
        declare_axis_yaw,
        declare_gripper_open_button,
        declare_gripper_close_button,
        declare_home_button,
        joint_effort_controllers,
        gripper_controllers,
        group_arm,
    ])
