# Copyright (c) 2025 The Nerites Authors.
# All rights reserved.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    uuv_name = LaunchConfiguration('uuv_name')
    model_name = LaunchConfiguration('model_name')
    saturation = LaunchConfiguration('saturation')
    gui_on = LaunchConfiguration('gui_on')
    use_params_file = LaunchConfiguration('use_params_file')
    use_ned_frame = LaunchConfiguration('use_ned_frame')
    Kp = LaunchConfiguration('Kp')
    Kd = LaunchConfiguration('Kd')
    Ki = LaunchConfiguration('Ki')

    controller_config_file = LaunchConfiguration('controller_config_file')
    thruster_manager_output_dir = LaunchConfiguration('thruster_manager_output_dir')
    thruster_manager_config_file = LaunchConfiguration('thruster_manager_config_file')
    tam_file = LaunchConfiguration('tam_file')

    return LaunchDescription([
        DeclareLaunchArgument('uuv_name'),
        DeclareLaunchArgument('model_name', default_value=uuv_name),
        DeclareLaunchArgument('saturation', default_value='1200'),
        DeclareLaunchArgument('gui_on', default_value='true'),
        DeclareLaunchArgument('use_params_file', default_value='false'),
        DeclareLaunchArgument('use_ned_frame', default_value='false'),
        DeclareLaunchArgument('Kp', default_value='11993.888,11993.888,11993.888,19460.069,19460.069,19460.069'),
        DeclareLaunchArgument('Kd', default_value='9077.459,9077.459,9077.459,18880.925,18880.925,18880.925'),
        DeclareLaunchArgument('Ki', default_value='321.417,321.417,321.417,2096.951,2096.951,2096.951'),
        DeclareLaunchArgument('controller_config_file', default_value=os.path.join(
            get_package_share_directory('uuv_trajectory_control'), 'config', 'controllers', 'pid', 'rexrov', 'params.yaml')),
        DeclareLaunchArgument('thruster_manager_output_dir', default_value=os.path.join(
            get_package_share_directory('uuv_thruster_manager'), 'config', 'rexrov')),
        DeclareLaunchArgument('thruster_manager_config_file', default_value=os.path.join(
            get_package_share_directory('uuv_thruster_manager'), 'config', 'rexrov', 'thruster_manager.yaml')),
        DeclareLaunchArgument('tam_file', default_value=os.path.join(
            get_package_share_directory('uuv_thruster_manager'), 'config', 'rexrov', 'TAM.yaml')),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(os.path.join(
                get_package_share_directory('uuv_thruster_manager'), 'launch', 'thruster_manager.launch.py'
            )),
            launch_arguments={
                'uuv_name': uuv_name,
                'model_name': model_name,
                'output_dir': thruster_manager_output_dir,
                'config_file': thruster_manager_config_file,
                'tam_file': tam_file
            }.items()
        ),

        GroupAction([
            PushRosNamespace(namespace=uuv_name),
            Node(
                package='uuv_control_utils',
                executable='trajectory_marker_publisher.py',
                name='trajectory_marker_publisher',
                output='screen',
                condition=IfCondition(gui_on),
                remappings=[
                    ('trajectory', 'dp_controller/trajectory'),
                    ('waypoints', 'dp_controller/waypoints')
                ]
            )
        ]),

        GroupAction([
            GroupAction([
                PushRosNamespace(namespace=uuv_name),
                Node(
                    package='uuv_trajectory_control',
                    executable='rov_pid_controller.py',
                    name='rov_pid_controller',
                    output='screen',
                    remappings=[
                        ('odom', 'pose_gt_ned'),
                        ('trajectory', 'dp_controller/trajectory'),
                        ('input_trajectory', 'dp_controller/input_trajectory'),
                        ('waypoints', 'dp_controller/waypoints'),
                        ('error', 'dp_controller/error'),
                        ('reference', 'dp_controller/reference'),
                        ('thruster_output', 'thruster_manager/input_stamped'),
                        ('tf', '/tf'),
                        ('tf_static', '/tf_static')
                    ],
                    parameters=[
                        {'inertial_frame_id': 'world_ned'},
                        controller_config_file
                    ]
                )
            ], condition=IfCondition(use_ned_frame))
        ], condition=IfCondition(use_params_file)),

        GroupAction([
            GroupAction([
                PushRosNamespace(namespace=uuv_name),
                Node(
                    package='uuv_trajectory_control',
                    executable='rov_pid_controller.py',
                    name='rov_pid_controller',
                    output='screen',
                    remappings=[
                        ('odom', 'pose_gt'),
                        ('trajectory', 'dp_controller/trajectory'),
                        ('input_trajectory', 'dp_controller/input_trajectory'),
                        ('waypoints', 'dp_controller/waypoints'),
                        ('error', 'dp_controller/error'),
                        ('reference', 'dp_controller/reference'),
                        ('thruster_output', 'thruster_manager/input_stamped'),
                        ('tf', '/tf'),
                        ('tf_static', '/tf_static')
                    ],
                    parameters=[
                        {'inertial_frame_id': 'world'},
                        controller_config_file
                    ]
                )
            ], condition=UnlessCondition(use_ned_frame))
        ], condition=IfCondition(use_params_file)),

        GroupAction([
            GroupAction([
                PushRosNamespace(namespace=uuv_name),
                Node(
                    package='uuv_trajectory_control',
                    executable='rov_pid_controller.py',
                    name='rov_pid_controller',
                    output='screen',
                    remappings=[
                        ('odom', 'pose_gt_ned'),
                        ('trajectory', 'dp_controller/trajectory'),
                        ('input_trajectory', 'dp_controller/input_trajectory'),
                        ('waypoints', 'dp_controller/waypoints'),
                        ('error', 'dp_controller/error'),
                        ('reference', 'dp_controller/reference'),
                        ('thruster_output', 'thruster_manager/input_stamped'),
                        ('tf', '/tf'),
                        ('tf_static', '/tf_static')
                    ],
                    parameters=[
                        {
                            'saturation': saturation,
                            'Kp': [Kp],
                            'Kd': [Kd],
                            'Ki': [Ki],
                            'inertial_frame_id': 'world_ned'
                        }
                    ]
                )
            ], condition=IfCondition(use_ned_frame))
        ], condition=UnlessCondition(use_params_file)),

        GroupAction([
            GroupAction([
                PushRosNamespace(namespace=uuv_name),
                Node(
                    package='uuv_trajectory_control',
                    executable='rov_pid_controller.py',
                    name='rov_pid_controller',
                    output='screen',
                    remappings=[
                        ('odom', 'pose_gt'),
                        ('trajectory', 'dp_controller/trajectory'),
                        ('input_trajectory', 'dp_controller/input_trajectory'),
                        ('waypoints', 'dp_controller/waypoints'),
                        ('error', 'dp_controller/error'),
                        ('reference', 'dp_controller/reference'),
                        ('thruster_output', 'thruster_manager/input_stamped'),
                        ('tf', '/tf'),
                        ('tf_static', '/tf_static')
                    ],
                    parameters=[
                        {
                            'saturation': saturation,
                            'Kp': [Kp],
                            'Kd': [Kd],
                            'Ki': [Ki],
                            'inertial_frame_id': 'world'
                        }
                    ]
                )
            ], condition=UnlessCondition(use_ned_frame))
        ], condition=UnlessCondition(use_params_file))
    ])