#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, Command
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch arguments
    uuv_name = LaunchConfiguration('uuv_name')
    arm_name = LaunchConfiguration('arm_name')
    deadman_button = LaunchConfiguration('deadman_button')
    exclusion_button = LaunchConfiguration('exclusion_button')
    gripper_open_button = LaunchConfiguration('gripper_open_button')
    gripper_close_button = LaunchConfiguration('gripper_close_button')
    home_button = LaunchConfiguration('home_button')
    axis_azimuth = LaunchConfiguration('axis_azimuth')
    axis_shoulder = LaunchConfiguration('axis_shoulder')
    axis_wrist = LaunchConfiguration('axis_wrist')

    declare_uuv_name = DeclareLaunchArgument(
        'uuv_name',
        description='UUV name / namespace (e.g. rexrov)',
    )
    declare_arm_name = DeclareLaunchArgument(
        'arm_name',
        default_value='oberon4',
        description='Arm name (e.g. oberon4)',
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
    declare_gripper_open_button = DeclareLaunchArgument(
        'gripper_open_button',
        default_value='1',
        description='Gripper open button index',
    )
    declare_gripper_close_button = DeclareLaunchArgument(
        'gripper_close_button',
        default_value='2',
        description='Gripper close button index',
    )
    declare_home_button = DeclareLaunchArgument(
        'home_button',
        default_value='7',
        description='Home button index',
    )
    declare_axis_azimuth = DeclareLaunchArgument(
        'axis_azimuth',
        default_value='0',
        description='Joystick axis for azimuth joint',
    )
    declare_axis_shoulder = DeclareLaunchArgument(
        'axis_shoulder',
        default_value='1',
        description='Joystick axis for shoulder joint',
    )
    declare_axis_wrist = DeclareLaunchArgument(
        'axis_wrist',
        default_value='3',
        description='Joystick axis for wrist joint',
    )

    # -------------------------------------------------------------------------
    # robot_description from xacro
    # -------------------------------------------------------------------------
    xacro_file = os.path.join(
        get_package_share_directory('uuv_descriptions'),
        'robots',
        'rexrov_oberon4.xacro'
    )

    # If your xacro needs more arguments, add them here (e.g. arm_name, etc.)
    robot_description_cmd = Command([
        'xacro ', xacro_file,
        ' ', 'uuv_name:=', uuv_name,
        ' ', 'arm_name:=', arm_name,
    ])

    # Tell ROS 2 this is a plain string, not YAML
    robot_description = ParameterValue(robot_description_cmd, value_type=str)

    # -------------------------------------------------------------------------
    # Include joint effort controllers launch
    # -------------------------------------------------------------------------
    oberon4_control_share = get_package_share_directory('oberon4_control')
    joint_effort_controllers_launch = os.path.join(
        oberon4_control_share,
        'launch',
        'joint_effort_controllers.launch.py',
    )

    joint_effort_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(joint_effort_controllers_launch),
        launch_arguments={
            'uuv_name': uuv_name,
            'arm_name': arm_name,
        }.items(),
    )

    # Namespace: /<uuv_name>/<arm_name>
    ns = [TextSubstitution(text=''), '/', uuv_name, '/', arm_name]

    # -------------------------------------------------------------------------
    # kinematics_service node
    # -------------------------------------------------------------------------
    # TODO: adjust base_link and tip_link to match your actual URDF link names.
    kinematics_service_node = Node(
        package='uuv_manipulators_kinematics',
        executable='kinematics_service.py',
        name='kinematics_service',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'arms.oberon4.base_link': 'rexrov/base_link',   # <-- change if needed
            'arms.oberon4.tip_link': 'jaw_1',               # <-- change if needed
        }],
    )

    # -------------------------------------------------------------------------
    # joint_position_controller node
    # -------------------------------------------------------------------------
    joint_position_controller_node = Node(
        package='uuv_manipulators_control',
        executable='joint_position_controller.py',
        name=['joint_position_controller_', arm_name],
        output='screen',
        remappings=[
            ('joy', ['/', uuv_name, '/joy']),
        ],
        parameters=[{
            'robot_description': robot_description,
            'exclusion_buttons': exclusion_button,
            'deadman_button': deadman_button,
            'home_button': home_button,
            # Arm kinematics parameters required by ArmInterface
            'arms.oberon4.base_link': 'rexrov/base_link',
            'arms.oberon4.tip_link': 'jaw_1',              
            'controller_config': {
                'azimuth': {
                    'joint_input_axis': axis_azimuth,
                    'axis_gain': 0.008,
                    'topic': ['/', uuv_name, '/', arm_name, '/azimuth/controller/command'],
                    'controller': {
                        'p': 800.0,
                        'i': 20.0,
                        'd': 50.0,
                    },
                },
                'shoulder': {
                    'joint_input_axis': axis_shoulder,
                    'axis_gain': 0.008,
                    'topic': ['/', uuv_name, '/', arm_name, '/shoulder/controller/command'],
                    'controller': {
                        'p': 800.0,
                        'i': 20.0,
                        'd': 50.0,
                    },
                },
                'wrist_joint': {
                    'joint_input_axis': axis_wrist,
                    'axis_gain': 0.05,
                    'topic': ['/', uuv_name, '/', arm_name, '/wrist_joint/controller/command'],
                    'controller': {
                        'p': 400.0,
                        'i': 2.0,
                        'd': 5.0,
                    },
                },
            },
        }],
    )

    # -------------------------------------------------------------------------
    # gripper_controller node
    # -------------------------------------------------------------------------
    # Inlines the same configuration you had in gripper_config.yaml
    gripper_controller_node = Node(
        package='uuv_manipulators_control',
        executable='gripper_controller.py',
        name=['gripper_controller_', arm_name],
        output='screen',
        remappings=[
            ('joy', ['/', uuv_name, '/joy']),
        ],
        parameters=[{
            'robot_description': robot_description,
            # Gripper config (flattened keys for ROS 2 params)
            'gripper.type': 'jaw',
            'gripper.base': 'gripper_base',
            'gripper.links': ['jaw_1', 'jaw_2'],
            'gripper.control_joint': 'jaw_1_joint',
            'gripper.mimic_joint': 'jaw_2_joint',
            'gripper.mimic_joint_gain': 1.0,
            'gripper.closed_limit': 'upper',
            'gripper.full_open_limit': 'lower',
            'gripper.groups.left_finger.links': ['jaw_1'],
            'gripper.groups.left_finger.ee': 'jaw_1',
            'gripper.groups.right_finger.links': ['jaw_2'],
            'gripper.groups.right_finger.ee': 'jaw_2',

            'exclusion_buttons': exclusion_button,
            'deadman_button': deadman_button,
            'open_button': gripper_open_button,
            'close_button': gripper_close_button,
            'kp': 40.0,
            'ki': 5.0,
            'kd': 1.0,
        }],
    )

    group = GroupAction([
        PushRosNamespace(ns),
        kinematics_service_node,
        joint_position_controller_node,
        gripper_controller_node,
    ])

    return LaunchDescription([
        declare_uuv_name,
        declare_arm_name,
        declare_deadman_button,
        declare_exclusion_button,
        declare_gripper_open_button,
        declare_gripper_close_button,
        declare_home_button,
        declare_axis_azimuth,
        declare_axis_shoulder,
        declare_axis_wrist,
        joint_effort_controllers,
        group,
    ])
