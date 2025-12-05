#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    # --- Launch arguments ---
    namespace = LaunchConfiguration('namespace')
    joy_id = LaunchConfiguration('joy_id')
    use_jt = LaunchConfiguration('use_jt')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')

    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='rexrov',
        description='Vehicle namespace',
    )

    declare_joy_id = DeclareLaunchArgument(
        'joy_id',
        default_value='0',
        description='Joystick device ID',
    )

    declare_use_jt = DeclareLaunchArgument(
        'use_jt',
        default_value='0',
        description='Enable Jacobian transpose control (1=yes, 0=no)',
    )

    declare_x = DeclareLaunchArgument(
        'x',
        default_value='0',
        description='Initial x position',
    )

    declare_y = DeclareLaunchArgument(
        'y',
        default_value='0',
        description='Initial y position',
    )

    declare_z = DeclareLaunchArgument(
        'z',
        default_value='-20',
        description='Initial z position',
    )

    declare_yaw = DeclareLaunchArgument(
        'yaw',
        default_value='0.0',
        description='Initial yaw (rad)',
    )

    # Path to the (ported) rexrov_oberon_demo ROS 2 launch file
    rexrov_oberon_demo_launch_path = os.path.join(
        get_package_share_directory('uuv_gazebo'),
        'launch',
        'rexrov_demos',
        'rexrov_oberon_demo.launch.py',  # make sure this exists in ROS 2 form
    )

    # Include rexrov_oberon_demo.launch.py with same args/mappings
    rexrov_oberon_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rexrov_oberon_demo_launch_path),
        launch_arguments={
            'namespace': namespace,
            'joy_id': joy_id,
            'use_jt': use_jt,

            # Starting position
            'x': x,
            'y': y,
            'z': z,
            'yaw': yaw,

            # Joystick mapping for vehicle control (Logitech Extreme 3D Pro)
            'axis_yaw': '2',
            'axis_x': '1',
            'axis_y': '0',
            'axis_z': '5',
            'deadman_button': '-1',
            'exclusion_buttons': '1',

            # Joystick mapping for the joint control node
            'axis_oberon_jc_azimuth': '2',
            'axis_oberon_jc_shoulder': '1',
            'axis_oberon_jc_elbow': '0',
            'axis_oberon_jc_roll': '3',
            'axis_oberon_jc_pitch': '4',
            'axis_oberon_jc_yaw': '5',

            # Joystick mapping for the Jacobian transpose control node
            'axis_oberon_jt_x': '1',
            'axis_oberon_jt_y': '0',
            'axis_oberon_jt_z': '5',
            'axis_oberon_jt_roll': '3',
            'axis_oberon_jt_pitch': '4',
            'axis_oberon_jt_yaw': '2',

            'oberon_exclusion_button': '0',
            'oberon_deadman_button': '1',
            'oberon_home_button': '6',

            'gripper_open_button': '11',
            'gripper_close_button': '10',
        }.items(),
    )

    return LaunchDescription([
        declare_namespace,
        declare_joy_id,
        declare_use_jt,
        declare_x,
        declare_y,
        declare_z,
        declare_yaw,
        rexrov_oberon_demo,
    ])
