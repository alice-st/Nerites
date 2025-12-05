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
    exclusion_button = LaunchConfiguration('exclusion_button')
    deadman_button = LaunchConfiguration('deadman_button')
    home_button = LaunchConfiguration('home_button')

    axis_x = LaunchConfiguration('axis_x')
    axis_y = LaunchConfiguration('axis_y')
    axis_z = LaunchConfiguration('axis_z')
    axis_roll = LaunchConfiguration('axis_roll')
    axis_pitch = LaunchConfiguration('axis_pitch')
    axis_yaw = LaunchConfiguration('axis_yaw')

    gain_roll = LaunchConfiguration('gain_roll')
    gain_pitch = LaunchConfiguration('gain_pitch')
    gain_yaw = LaunchConfiguration('gain_yaw')
    gain_x = LaunchConfiguration('gain_x')
    gain_y = LaunchConfiguration('gain_y')
    gain_z = LaunchConfiguration('gain_z')

    gripper_open_button = LaunchConfiguration('gripper_open_button')
    gripper_close_button = LaunchConfiguration('gripper_close_button')

    declare_uuv_name = DeclareLaunchArgument(
        'uuv_name',
        default_value='',
        description='UUV name / namespace (e.g. rexrov)',
    )
    declare_arm_name = DeclareLaunchArgument(
        'arm_name',
        default_value='oberon7',
        description='Arm name (e.g. oberon7)',
    )
    declare_exclusion_button = DeclareLaunchArgument(
        'exclusion_button', default_value='4', description='Exclusion button'
    )
    declare_deadman_button = DeclareLaunchArgument(
        'deadman_button', default_value='5', description='Deadman button'
    )
    declare_home_button = DeclareLaunchArgument(
        'home_button', default_value='7', description='Home button'
    )

    declare_axis_x = DeclareLaunchArgument('axis_x', default_value='4', description='Axis X')
    declare_axis_y = DeclareLaunchArgument('axis_y', default_value='3', description='Axis Y')
    declare_axis_z = DeclareLaunchArgument('axis_z', default_value='1', description='Axis Z')
    declare_axis_roll = DeclareLaunchArgument('axis_roll', default_value='6', description='Axis roll')
    declare_axis_pitch = DeclareLaunchArgument('axis_pitch', default_value='7', description='Axis pitch')
    declare_axis_yaw = DeclareLaunchArgument('axis_yaw', default_value='0', description='Axis yaw')

    declare_gain_roll = DeclareLaunchArgument('gain_roll', default_value='0.05')
    declare_gain_pitch = DeclareLaunchArgument('gain_pitch', default_value='0.05')
    declare_gain_yaw = DeclareLaunchArgument('gain_yaw', default_value='0.05')
    declare_gain_x = DeclareLaunchArgument('gain_x', default_value='0.05')
    declare_gain_y = DeclareLaunchArgument('gain_y', default_value='0.05')
    declare_gain_z = DeclareLaunchArgument('gain_z', default_value='0.05')

    declare_gripper_open_button = DeclareLaunchArgument(
        'gripper_open_button', default_value='1', description='Gripper open button'
    )
    declare_gripper_close_button = DeclareLaunchArgument(
        'gripper_close_button', default_value='2', description='Gripper close button'
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
    # Teleop node that produces jt_vel_input
    joy_input_node = Node(
        package='uuv_teleop',
        executable='vehicle_teleop.py',
        name=['joy_', arm_name, '_input'],
        output='screen',
        remappings=[
            ('output', 'jt_vel_input'),
            ('joy', ['/', uuv_name, '/joy']),
        ],
        parameters=[{
            'type': 'twist',
            'deadman_button': deadman_button,
            'exclusion_buttons': [exclusion_button],
            'mapping': {
                'x': {
                    'axis': axis_x,
                    'gain': gain_x,
                },
                'y': {
                    'axis': axis_y,
                    'gain': gain_y,
                },
                'z': {
                    'axis': axis_z,
                    'gain': gain_z,
                },
                'roll': {
                    'axis': axis_roll,
                    'gain': gain_roll,
                },
                'pitch': {
                    'axis': axis_pitch,
                    'gain': gain_pitch,
                },
                'yaw': {
                    'axis': axis_yaw,
                    'gain': gain_yaw,
                },
            },
        }],
    )

    # Kinematics service
    kinematics_service_node = Node(
        package='uuv_manipulators_kinematics',
        executable='kinematics_service.py',
        name='kinematics_service',
        output='screen',
    )

    # Jacobian transpose Cartesian controller
    cartesian_controller_node = Node(
        package='uuv_manipulators_control',
        executable='jt_cartesian_controller.py',
        name='cartesian_controller',
        output='screen',
        remappings=[
            ('cmd_vel', 'jt_vel_input'),
        ],
        parameters=[{
            'publish_rate': 50,
            'Kp': [8000.0, 8000.0, 8000.0, 300.0, 300.0, 300.0],
            'Kd': [100.0, 100.0, 100.0, 2.0, 2.0, 2.0],
        }],
    )

    group = GroupAction([
        PushRosNamespace([uuv_name, '/', arm_name]),
        joy_input_node,
        kinematics_service_node,
        cartesian_controller_node,
    ])

    return LaunchDescription([
        declare_uuv_name,
        declare_arm_name,
        declare_exclusion_button,
        declare_deadman_button,
        declare_home_button,
        declare_axis_x,
        declare_axis_y,
        declare_axis_z,
        declare_axis_roll,
        declare_axis_pitch,
        declare_axis_yaw,
        declare_gain_roll,
        declare_gain_pitch,
        declare_gain_yaw,
        declare_gain_x,
        declare_gain_y,
        declare_gain_z,
        declare_gripper_open_button,
        declare_gripper_close_button,
        joint_effort_controllers,
        gripper_controllers,
        group,
    ])
