# Copyright (c) 2025 The Nerites Authors.
# All rights reserved.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction

import os

def launch_setup(context, *args, **kwargs):
    # === Resolve LaunchConfigurations at runtime ===
    timeout = LaunchConfiguration('timeout').perform(context)
    starting_time = LaunchConfiguration('starting_time').perform(context)
    end_time = LaunchConfiguration('end_time').perform(context)
    current_vel = LaunchConfiguration('current_vel').perform(context)
    horizontal_angle = LaunchConfiguration('horizontal_angle').perform(context)
    vertical_angle = LaunchConfiguration('vertical_angle').perform(context)

    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    z = LaunchConfiguration('z').perform(context)
    yaw = LaunchConfiguration('yaw').perform(context)

    K = LaunchConfiguration('K').perform(context)
    Kd = LaunchConfiguration('Kd').perform(context)
    Ki = LaunchConfiguration('Ki').perform(context)
    slope = LaunchConfiguration('slope').perform(context)

    # === Include: empty_underwater_world.launch ===
    empty_world = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('uuv_gazebo_worlds'),
                'launch',
                'empty_underwater_world.launch.py'
            )
        ),
        launch_arguments={
            'set_timeout': 'true',
            'timeout': timeout
        }.items()
    )

    # === Include: upload_rexrov.launch ===
    upload_rexrov = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('uuv_descriptions'),
                'launch',
                'upload_rexrov.launch.py'
            )
        ),
        launch_arguments={
            'x': x,
            'y': y,
            'z': z,
            'yaw': yaw
        }.items()
    )

    # === Include: rov_nmb_sm_controller.launch ===
    controller = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('uuv_trajectory_control'),
                'launch',
                'rov_nmb_sm_controller.launch.py'
            )
        ),
        launch_arguments={
            'uuv_name': 'rexrov',
            'model_name': 'rexrov',
            'K': K,
            'Kd': Kd,
            'Ki': Ki,
            'slope': slope
        }.items()
    )

    # === Include: start_circular_trajectory.launch ===
    circular_traj = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('uuv_control_utils'),
                'launch',
                'start_circular_trajectory.launch.py'
            )
        ),
        launch_arguments={
            'uuv_name': 'rexrov',
            'radius': '5',
            'center_z': '-25',
            'max_forward_speed': '0.6'
        }.items()
    )

    # === Include: set_timed_current_perturbation.launch ===
    current_perturbation = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('uuv_control_utils'),
                'launch',
                'set_timed_current_perturbation.launch.py'
            )
        ),
        launch_arguments={
            'starting_time': starting_time,
            'end_time': end_time,
            'current_vel': current_vel,
            'horizontal_angle': horizontal_angle,
            'vertical_angle': vertical_angle
        }.items()
    )

    # === RViz Node ===
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=[
            '-d',
            os.path.join(
                get_package_share_directory('uuv_gazebo'),
                'rviz',
                'controller_demo.rviz'
            )
        ]
    )
    
    return [
        empty_world,
        TimerAction(period=10.0, actions=[upload_rexrov]),
        TimerAction(period=15.0, actions=[controller]),
        TimerAction(period=20.0, actions=[circular_traj]),
        TimerAction(period=22.0, actions=[current_perturbation]),
        TimerAction(period=24.0, actions=[rviz_node])]



def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('set_timeout', default_value='false'),
        DeclareLaunchArgument('timeout', default_value='105'),

        DeclareLaunchArgument('starting_time', default_value='10'),
        DeclareLaunchArgument('end_time', default_value='40'),
        DeclareLaunchArgument('current_vel', default_value='0.7'),
        DeclareLaunchArgument('horizontal_angle', default_value='50'),
        DeclareLaunchArgument('vertical_angle', default_value='20'),

        DeclareLaunchArgument('x', default_value='0'),
        DeclareLaunchArgument('y', default_value='0'),
        DeclareLaunchArgument('z', default_value='-20'),
        DeclareLaunchArgument('yaw', default_value='0'),

        DeclareLaunchArgument('K', default_value='5,5,5,5,5,5'),
        DeclareLaunchArgument('Kd', default_value='4118.98,4118.98,4118.98,8000.0,8000.0,8000.0'),
        DeclareLaunchArgument('Ki', default_value='0.06144,0.06144,0.06144,0.078,0.078,0.078'),
        DeclareLaunchArgument('slope', default_value='0.182,0.182,0.182,3.348,3.348,3.348'),

        OpaqueFunction(function=launch_setup)
    ])
