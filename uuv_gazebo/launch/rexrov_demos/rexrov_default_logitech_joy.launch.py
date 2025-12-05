#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    # Launch configurations (arguments)
    namespace = LaunchConfiguration('namespace')
    joy_id = LaunchConfiguration('joy_id')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')

    # Declare arguments
    declare_namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='rexrov',
        description='Namespace of the vehicle'
    )

    declare_joy_id_arg = DeclareLaunchArgument(
        'joy_id',
        default_value='0',
        description='Joystick device ID'
    )

    declare_x_arg = DeclareLaunchArgument(
        'x',
        default_value='0',
        description='Initial x position'
    )

    declare_y_arg = DeclareLaunchArgument(
        'y',
        default_value='0',
        description='Initial y position'
    )

    declare_z_arg = DeclareLaunchArgument(
        'z',
        default_value='-70',
        description='Initial z position'
    )

    declare_yaw_arg = DeclareLaunchArgument(
        'yaw',
        default_value='0.0',
        description='Initial yaw (rad)'
    )

    # Path to the included launch file (ROS 2 version)
    rexrov_default_launch_path = os.path.join(
        get_package_share_directory('uuv_gazebo'),
        'launch',
        'rexrov_demos',
        'rexrov_default.launch.py',   
    )

    # Include rexrov_default.launch.py with arguments
    rexrov_default = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rexrov_default_launch_path),
        launch_arguments={
            'namespace': namespace,
            'joy_id': joy_id,
            'axis_yaw': '2',
            'axis_x': '1',
            'axis_y': '0',
            'axis_z': '5',
            'x': x,
            'y': y,
            'z': z,
            'yaw': yaw,
        }.items()
    )

    return LaunchDescription([
        declare_namespace_arg,
        declare_joy_id_arg,
        declare_x_arg,
        declare_y_arg,
        declare_z_arg,
        declare_yaw_arg,
        rexrov_default,
    ])
