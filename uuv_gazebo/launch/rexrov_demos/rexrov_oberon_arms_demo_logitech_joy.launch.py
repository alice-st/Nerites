#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    # --- Launch arguments ---
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    joy_id = LaunchConfiguration('joy_id')
    namespace = LaunchConfiguration('namespace')

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

    declare_joy_id = DeclareLaunchArgument(
        'joy_id',
        default_value='0',
        description='Joystick device ID',
    )

    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='rexrov',
        description='Vehicle namespace',
    )

    # Path to the (ported) rexrov_oberon_arms_demo ROS 2 launch file
    rexrov_oberon_arms_demo_launch_path = os.path.join(
        get_package_share_directory('uuv_gazebo'),
        'launch',
        'rexrov_demos',
        'rexrov_oberon_arms_demo.launch.py',   # assumes you've ported this
    )

    # Include rexrov_oberon_arms_demo.launch.py with the same arguments / mappings
    rexrov_oberon_arms_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rexrov_oberon_arms_demo_launch_path),
        launch_arguments={
            'joy_id': joy_id,
            'namespace': namespace,

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
            'vehicle_deadman_button': '-1',
            'vehicle_exclusion_buttons': '0,1',

            # Joystick mapping for Oberon7 joint control
            'axis_oberon_jc_azimuth': '2',
            'axis_oberon_jc_shoulder': '1',
            'axis_oberon_jc_elbow': '0',
            'axis_oberon_jc_roll': '3',
            'axis_oberon_jc_pitch': '4',
            'axis_oberon_jc_yaw': '5',

            # Joystick mapping for Oberon4 joint control
            'axis_oberon4_azimuth': '2',
            'axis_oberon4_shoulder': '1',
            'axis_oberon4_wrist': '0',

            'oberon_exclusion_button': '0',
            'oberon_deadman_button': '1',

            'oberon4_exclusion_button': '1',
            'oberon4_deadman_button': '0',

            'oberon_home_button': '6',

            'gripper_open_button': '11',
            'gripper_close_button': '10',
        }.items(),
    )

    return LaunchDescription([
        declare_x,
        declare_y,
        declare_z,
        declare_yaw,
        declare_joy_id,
        declare_namespace,
        rexrov_oberon_arms_demo,
    ])
