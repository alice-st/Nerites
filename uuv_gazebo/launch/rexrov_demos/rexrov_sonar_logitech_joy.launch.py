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

    # Path to the (ported) rexrov_sonar ROS 2 launch file
    rexrov_sonar_launch_path = os.path.join(
        get_package_share_directory('uuv_gazebo'),
        'launch',
        'rexrov_demos',
        'rexrov_sonar.launch.py',   # make sure you've created/ported this
    )

    # Include rexrov_sonar.launch.py with same args/mappings
    rexrov_sonar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rexrov_sonar_launch_path),
        launch_arguments={
            'namespace': namespace,
            'joy_id': joy_id,
            'axis_yaw': '2',
            'axis_x': '1',
            'axis_y': '0',
            'axis_z': '5',
            # Starting position
            'x': x,
            'y': y,
            'z': z,
            'yaw': yaw,
        }.items(),
    )

    return LaunchDescription([
        declare_namespace,
        declare_joy_id,
        declare_x,
        declare_y,
        declare_z,
        declare_yaw,
        rexrov_sonar,
    ])
