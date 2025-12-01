# Copyright (c) 2025 The Nerites Authors.
# All rights reserved.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # ------------------------
    # Declare launch arguments
    # ------------------------
    args = [
        DeclareLaunchArgument('model_name'),
        DeclareLaunchArgument('uuv_name', default_value=LaunchConfiguration('model_name')),
    ]

    # ------------------------
    # Include thruster_manager.launch.py
    # ------------------------
    thruster_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('uuv_thruster_manager'),
                'launch',
                'thruster_manager.launch.py'  # must be converted to .py
            ])
        ),
        launch_arguments={
            'uuv_name': LaunchConfiguration('uuv_name'),
            'model_name': LaunchConfiguration('model_name')
        }.items()
    )

    # ------------------------
    # Acceleration control node
    # ------------------------
    accel_control_node = Node(
        package='uuv_control_cascaded_pid',
        executable='acceleration_control.py',
        name='acceleration_control',
        output='screen',
        parameters=[
            {'tf_prefix': PythonExpression(["'", LaunchConfiguration('uuv_name'), "' + '/'"])},
            PathJoinSubstitution([
                FindPackageShare('uuv_control_cascaded_pid'),
                'config',
                LaunchConfiguration('model_name'),
                'inertial.yaml'
            ])
        ]
    )

    # ------------------------
    # Velocity control node
    # ------------------------
    vel_control_node = Node(
        package='uuv_control_cascaded_pid',
        executable='velocity_control.py',
        name='velocity_control',
        output='screen',
        remappings=[
            ('odom', PythonExpression(["'/' + '", LaunchConfiguration('uuv_name'), "' + '/pose_gt'"])),
            ('cmd_accel', PythonExpression(["'/' + '", LaunchConfiguration('uuv_name'), "' + '/cmd_accel'"]))
        ],
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('uuv_control_cascaded_pid'),
                'config',
                LaunchConfiguration('model_name'),
                'vel_pid_control.yaml'
            ])
        ]
    )

    # ------------------------
    # Position control node
    # ------------------------
    pos_control_node = Node(
        package='uuv_control_cascaded_pid',
        executable='position_control.py',
        name='position_control',
        output='screen',
        remappings=[
            ('odom', PythonExpression(["'/' + '", LaunchConfiguration('uuv_name'), "' + '/pose_gt'"])),
        ],
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('uuv_control_cascaded_pid'),
                'config',
                LaunchConfiguration('model_name'),
                'pos_pid_control.yaml'
            ])
        ]
    )

    # ------------------------
    # Group namespace
    # ------------------------
    group = GroupAction([
        PushRosNamespace(LaunchConfiguration('uuv_name')),
        accel_control_node,
        vel_control_node,
        pos_control_node
    ])

    # ------------------------
    # Return everything
    # ------------------------
    return LaunchDescription(args + [
        thruster_manager_launch,
        group
    ])
