# Copyright (c) 2025 The Nerites Authors.
# All rights reserved.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction

import os

def launch_setup(context, *args, **kwargs):
    # Resolve all arguments at runtime
    uuv_name = LaunchConfiguration('uuv_name').perform(context)
    model_name = LaunchConfiguration('model_name').perform(context)
    saturation = LaunchConfiguration('saturation').perform(context)
    Kp = LaunchConfiguration('Kp').perform(context)
    Kd = LaunchConfiguration('Kd').perform(context)
    Ki = LaunchConfiguration('Ki').perform(context)

    output_dir = LaunchConfiguration('output_dir').perform(context)
    config_file = LaunchConfiguration('config_file').perform(context)
    tam_file = LaunchConfiguration('tam_file').perform(context)

    # === Include thruster_manager.launch ===
    thruster_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare('uuv_thruster_manager').perform(context),
                'launch',
                'thruster_manager.launch.py'
            )
        ),
        launch_arguments={
            'uuv_name': uuv_name,
            'model_name': model_name,
            'output_dir': output_dir,
            'config_file': config_file,
            'tam_file': tam_file
        }.items()
    )

    # === Group: vehicle namespace ===
    group = GroupAction([
        PushRosNamespace(uuv_name),

        # --- trajectory_marker_publisher node ---
        Node(
            package='uuv_control_utils',
            executable='trajectory_marker_publisher.py',
            name='trajectory_marker_publisher',
            output='screen',
            remappings=[
                ('trajectory', 'dp_controller/trajectory'),
                ('waypoints', 'dp_controller/waypoints')
            ]
        ),

        # --- tutorial_dp_controller node ---
        Node(
            package='uuv_tutorial_dp_controller',
            executable='tutorial_dp_controller.py',
            name='tutorial_dp_controller',
            output='screen',
            remappings=[
                ('odom', 'pose_gt'),
                ('trajectory', 'dp_controller/trajectory'),
                ('input_trajectory', 'dp_controller/input_trajectory'),
                ('waypoints', 'dp_controller/waypoints'),
                ('error', 'dp_controller/error'),
                ('reference', 'dp_controller/reference'),
                ('thruster_output', 'thruster_manager/input_stamped'),
            ],
            parameters=[{
                'saturation': float(saturation),
                # These are strings like "1,2,3" in ROS 1 style; split into floats for ROS 2 param
                'Kp': [float(x) for x in Kp.split(',')],
                'Kd': [float(x) for x in Kd.split(',')],
                'Ki': [float(x) for x in Ki.split(',')]
            }]
        )
    ])

    return [
            TimerAction(period=2.0, actions=[thruster_manager_launch]),
            TimerAction(period=5.0, actions=[group]),
        ]



def generate_launch_description():
    return LaunchDescription([
        # === Declare Arguments ===
        DeclareLaunchArgument('uuv_name'),
        DeclareLaunchArgument('model_name', default_value=LaunchConfiguration('uuv_name')),
        DeclareLaunchArgument('saturation', default_value='5000'),

        DeclareLaunchArgument('Kp', default_value='11993.888,11993.888,11993.888,19460.069,19460.069,19460.069'),
        DeclareLaunchArgument('Kd', default_value='9077.459,9077.459,9077.459,18880.925,18880.925,18880.925'),
        DeclareLaunchArgument('Ki', default_value='321.417,321.417,321.417,2096.951,2096.951,2096.951'),

        DeclareLaunchArgument('output_dir', default_value=[FindPackageShare('uuv_thruster_manager'), '/config/', LaunchConfiguration('model_name')]),
        DeclareLaunchArgument('config_file', default_value=[FindPackageShare('uuv_thruster_manager'), '/config/', LaunchConfiguration('model_name'), '/thruster_manager.yaml']),
        DeclareLaunchArgument('tam_file', default_value=[FindPackageShare('uuv_thruster_manager'), '/config/', LaunchConfiguration('model_name'), '/TAM.yaml']),

        OpaqueFunction(function=launch_setup)
    ])
