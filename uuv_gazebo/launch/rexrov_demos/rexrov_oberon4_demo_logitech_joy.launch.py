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

    # Path to the (ported) rexrov_oberon4_demo ROS 2 launch file
    rexrov_oberon4_demo_launch_path = os.path.join(
        get_package_share_directory('uuv_gazebo'),
        'launch',
        'rexrov_demos',
        'rexrov_oberon4_demo.launch.py',  # assuming you’ve ported this to ROS 2
    )

    # Include rexrov_oberon4_demo.launch.py with the same arguments/mappings
    rexrov_oberon4_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rexrov_oberon4_demo_launch_path),
        launch_arguments={
            'namespace': namespace,
            'joy_id': joy_id,
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
            # Joystick mapping for joint control
            'axis_oberon_azimuth': '2',
            'axis_oberon_shoulder': '1',
            'axis_oberon_wrist': '0',
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
        declare_x,
        declare_y,
        declare_z,
        declare_yaw,
        rexrov_oberon4_demo,
    ])
