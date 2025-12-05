#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Arguments
    joy_id = LaunchConfiguration('joy_id')

    declare_joy_id = DeclareLaunchArgument(
        'joy_id',
        default_value='0',
        description='Joystick device ID'
    )

    # Paths to the included launch files
    rov_bop_panel_launch_path = os.path.join(
        get_package_share_directory('uuv_gazebo_worlds'),
        'launch',
        'rov_bop_panel.launch.py', 
    )

    rexrov_oberon_arms_demo_launch_path = os.path.join(
        get_package_share_directory('uuv_gazebo'),
        'launch',
        'rexrov_demos',
        'rexrov_oberon_arms_demo.launch.py', 
    )

    # Include rov_bop_panel.launch.py
    rov_bop_panel = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rov_bop_panel_launch_path),
        launch_arguments={
            'x': '2.5',
            'y': '0',
            'z': '-4',
            'angle': '3.14',
        }.items()
    )

    # Include rexrov_oberon_arms_demo.launch.py
    rexrov_oberon_arms_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rexrov_oberon_arms_demo_launch_path),
        launch_arguments={
            'x': '0',
            'y': '0',
            'z': '-4',
            'joy_id': joy_id,
        }.items()
    )

    return LaunchDescription([
        declare_joy_id,
        rov_bop_panel,
        rexrov_oberon_arms_demo,
    ])
