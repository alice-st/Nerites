# Copyright (c) 2025 The Nerites Authors.
# All rights reserved.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # -------------------------
    # Declare launch arguments
    # -------------------------
    args = [
        DeclareLaunchArgument('model_name'),
        DeclareLaunchArgument('uuv_name', default_value=LaunchConfiguration('model_name')),
        DeclareLaunchArgument('joy_id', default_value='0'),
    ]

    # -------------------------
    # Include thruster_manager.launch.py
    # -------------------------
    thruster_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('uuv_thruster_manager'),
                'launch',
                'thruster_manager.launch.py'   # assumes already converted to .py
            ])
        ),
        launch_arguments={
            'uuv_name': LaunchConfiguration('uuv_name'),
            'model_name': LaunchConfiguration('model_name')
        }.items()
    )

    # -------------------------
    # Main node: acceleration_control.py
    # -------------------------
    accel_control_node = Node(
        package='uuv_control_cascaded_pid',
        executable='acceleration_control.py',
        name='acceleration_control',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('uuv_control_cascaded_pid'),
                'config',
                LaunchConfiguration('model_name'),
                'inertial.yaml'
            ])
        ]
    )

    # Grouping with namespace
    group = GroupAction([
        PushRosNamespace(LaunchConfiguration('uuv_name')),
        accel_control_node
    ])

    # -------------------------
    # Include uuv_teleop.launch.py
    # -------------------------
    uuv_teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('uuv_teleop'),
                'launch',
                'uuv_teleop.launch.py'
            ])
        ),
        launch_arguments={
            'uuv_name': LaunchConfiguration('uuv_name'),
            'joy_id': LaunchConfiguration('joy_id'),
            'output_topic': TextSubstitution(text='cmd_accel'),
            'message_type': TextSubstitution(text='accel')
        }.items()
    )

    # -------------------------
    # Return everything
    # -------------------------
    return LaunchDescription(args + [
        thruster_manager_launch,
        group,
        uuv_teleop_launch
    ])
