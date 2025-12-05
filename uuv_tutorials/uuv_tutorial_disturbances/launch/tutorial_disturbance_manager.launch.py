# Copyright (c) 2025 The Nerites Authors.
# All rights reserved.

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
import os

def generate_launch_description():
    # Declare launch arguments
    set_timeout = LaunchConfiguration('set_timeout')
    timeout = LaunchConfiguration('timeout')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    K = LaunchConfiguration('K')
    Kd = LaunchConfiguration('Kd')
    Ki = LaunchConfiguration('Ki')
    slope = LaunchConfiguration('slope')

    return LaunchDescription([
        DeclareLaunchArgument('set_timeout', default_value='false'),
        DeclareLaunchArgument('timeout', default_value='0'),
        DeclareLaunchArgument('x', default_value='0'),
        DeclareLaunchArgument('y', default_value='0'),
        DeclareLaunchArgument('z', default_value='-20'),
        DeclareLaunchArgument('yaw', default_value='0'),
        DeclareLaunchArgument('K', default_value='5,5,5,5,5,5'),
        DeclareLaunchArgument('Kd', default_value='4118.98,4118.98,4118.98,8000.0,8000.0,8000.0'),
        DeclareLaunchArgument('Ki', default_value='0.06144,0.06144,0.06144,0.078,0.078,0.078'),
        DeclareLaunchArgument('slope', default_value='0.182,0.182,0.182,3.348,3.348,3.348'),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(get_package_share_directory('uuv_gazebo_worlds'), 
                    'launch', 
                    'empty_underwater_world.launch.py')
            ),
            launch_arguments={
                'set_timeout': 'false',
                'timeout': timeout
            }.items()
        ),

        TimerAction(
            period=5.0,
            actions=[
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(get_package_share_directory('uuv_descriptions'), 
                    'launch', 
                    'upload_rexrov.launch.py')
            )
        )
        ]
        ),

        TimerAction(
            period=10.0,
            actions=[
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(get_package_share_directory('uuv_trajectory_control'), 
                    'launch', 'rov_pid_controller.launch.py')
            ),
            launch_arguments={
                'uuv_name': 'rexrov',
                'model_name': 'rexrov',
                'K': K,
                'Kd': Kd,
                'Ki': Ki,
            }.items()
        )
        ]
        ),

        TimerAction(
            period=15.0,
            actions=[
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('uuv_control_utils'), 
                        'launch', 
                        'start_circular_trajectory.launch.py')
                ),
                launch_arguments={
                    'uuv_name': 'rexrov',
                    'start_time': '-1',
                    'radius': '8',
                    'center_x': '4',
                    'center_y': '2',
                    'center_z': '-22',
                    'center': '4,2,-22',
                    'n_points': '50',
                    'heading_offset': '0',
                    'duration': '0',
                    'max_forward_speed': '0.3'
                }.items()
            )
            ]
            ),
        
        TimerAction(
            period=20.0,
            actions=[
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(get_package_share_directory('uuv_control_utils'), 'launch', 'start_disturbance_manager.launch.py')
            ),
            launch_arguments={
                'uuv_name': 'rexrov',
                'use_file': 'true',
                'disturbance_file': os.path.join(
                    get_package_share_directory('uuv_tutorial_disturbances'),
                    'config',
                    'disturbances.yaml'
                )
            }.items()
        )
        ]
        )
    ])