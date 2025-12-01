# Copyright (c) 2025 The Nerites Authors.
# All rights reserved.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration 

import os

def launch_setup(context, *args, **kwargs):
    # === Resolve all runtime arguments ===
    uuv_name = context.launch_configurations['uuv_name']
    model_name = context.launch_configurations['model_name']
    saturation = context.launch_configurations['saturation']
    gui_on = context.launch_configurations['gui_on']
    use_params_file = context.launch_configurations['use_params_file']
    use_ned_frame = context.launch_configurations['use_ned_frame']

    lambda_vals = context.launch_configurations['lambda']
    rho_constant = context.launch_configurations['rho_constant']
    k = context.launch_configurations['k']
    c = context.launch_configurations['c']
    adapt_slope = context.launch_configurations['adapt_slope']
    rho_0 = context.launch_configurations['rho_0']
    drift_prevent = context.launch_configurations['drift_prevent']

    # === Resolve file paths ===
    controller_config_file = os.path.join(
        get_package_share_directory('uuv_trajectory_control'),
        'config', 'controllers', 'mb_sm', model_name, 'params.yaml'
    )
    model_params_file = os.path.join(
        get_package_share_directory('uuv_trajectory_control'),
        'config', 'models', model_name, 'params.yaml'
    )
    thruster_manager_output_dir = os.path.join(
        get_package_share_directory('uuv_thruster_manager'),
        'config', model_name
    )
    thruster_manager_config_file = os.path.join(
        get_package_share_directory('uuv_thruster_manager'),
        'config', model_name, 'thruster_manager.yaml'
    )
    tam_file = os.path.join(
        get_package_share_directory('uuv_thruster_manager'),
        'config', model_name, 'TAM.yaml'
    )

    # === Includes ===
    thruster_manager_launch = os.path.join(
        get_package_share_directory('uuv_thruster_manager'),
        'launch', 'thruster_manager.launch'
    )

    thruster_include = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(thruster_manager_launch),
        launch_arguments={
            'uuv_name': uuv_name,
            'model_name': model_name,
            'output_dir': thruster_manager_output_dir,
            'config_file': thruster_manager_config_file,
            'tam_file': tam_file
        }.items()
    )

    # === Actions list ===
    actions = [thruster_include]

    # === GUI marker node (conditional) ===
    if gui_on == 'true':
        actions.append(
            GroupAction([
                PushRosNamespace(namespace=uuv_name),
                Node(
                    package='uuv_control_utils',
                    executable='trajectory_marker_publisher.py',
                    name='trajectory_marker_publisher',
                    output='screen',
                    remappings=[
                        ('trajectory', 'dp_controller/trajectory'),
                        ('waypoints', 'dp_controller/waypoints')
                    ]
                )
            ])
        )

    # === Controller nodes when use_params_file == true ===
    if use_params_file == 'true':
        if use_ned_frame == 'true':
            actions.append(
                GroupAction([
                    PushRosNamespace(namespace=uuv_name),
                    Node(
                        package='uuv_trajectory_control',
                        executable='rov_mb_sm_controller.py',
                        name='rov_mb_smcontroller',
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
                            controller_config_file,
                            model_params_file,
                            {'saturation': saturation, 'inertial_frame_id': 'world_ned'}
                        ]
                    )
                ])
            )
        else:
            actions.append(
                GroupAction([
                    PushRosNamespace(namespace=uuv_name),
                    Node(
                        package='uuv_trajectory_control',
                        executable='rov_mb_sm_controller.py',
                        name='rov_mb_smcontroller',
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
                            controller_config_file,
                            model_params_file,
                            {'saturation': saturation, 'inertial_frame_id': 'world'}
                        ]
                    )
                ])
            )

    # === Controller nodes when use_params_file == false ===
    else:
        if use_ned_frame == 'true':
            actions.append(
                GroupAction([
                    PushRosNamespace(namespace=uuv_name),
                    Node(
                        package='uuv_trajectory_control',
                        executable='rov_mb_sm_controller.py',
                        name='rov_mb_smcontroller',
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
                            model_params_file,
                            {'saturation': saturation,
                             'lambda': [lambda_vals],
                             'rho_constant': [rho_constant],
                             'k': [k],
                             'c': [c],
                             'adapt_slope': [adapt_slope],
                             'rho_0': [rho_0],
                             'drift_prevent': drift_prevent,
                             'inertial_frame_id': 'world_ned'}
                        ]
                    )
                ])
            )
        else:
            actions.append(
                GroupAction([
                    PushRosNamespace(namespace=uuv_name),
                    Node(
                        package='uuv_trajectory_control',
                        executable='rov_mb_sm_controller.py',
                        name='rov_mb_smcontroller',
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
                            model_params_file,
                            {'saturation': saturation,
                             'lambda': [lambda_vals],
                             'rho_constant': [rho_constant],
                             'k': [k],
                             'c': [c],
                             'adapt_slope': [adapt_slope],
                             'rho_0': [rho_0],
                             'drift_prevent': drift_prevent,
                             'inertial_frame_id': 'world'}
                        ]
                    )
                ])
            )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('uuv_name'),
        DeclareLaunchArgument('model_name', default_value=LaunchConfiguration('uuv_name')),
        DeclareLaunchArgument('saturation', default_value='1200'),
        DeclareLaunchArgument('gui_on', default_value='true'),
        DeclareLaunchArgument('use_params_file', default_value='false'),
        DeclareLaunchArgument('use_ned_frame', default_value='false'),
        DeclareLaunchArgument('lambda', default_value='10,10,10,10,10,10'),
        DeclareLaunchArgument('rho_constant', default_value='10000,10000,10000,10000,10000,10000'),
        DeclareLaunchArgument('k', default_value='500,500,500,500,500,500'),
        DeclareLaunchArgument('c', default_value='50,50,50,1,1,1'),
        DeclareLaunchArgument('adapt_slope', default_value='100,10,10'),
        DeclareLaunchArgument('rho_0', default_value='3000,3000,8000,1500,1500,8000'),
        DeclareLaunchArgument('drift_prevent', default_value='0.03'),
        OpaqueFunction(function=launch_setup)
    ])
