# Copyright (c) 2025 The Nerites Authors.
# All rights reserved.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os

def launch_setup(context, *args, **kwargs):
    # === Resolve launch arguments ===
    uuv_name = context.launch_configurations['uuv_name']
    model_name = context.launch_configurations['model_name']
    saturation = context.launch_configurations['saturation']
    gui_on = context.launch_configurations['gui_on']
    use_params_file = context.launch_configurations['use_params_file']
    use_ned_frame = context.launch_configurations['use_ned_frame']

    Kp = context.launch_configurations['Kp']
    Kd = context.launch_configurations['Kd']
    Ki = context.launch_configurations['Ki']
    Hm = context.launch_configurations['Hm']

    # === Build file paths ===
    controller_config_file = os.path.join(
        get_package_share_directory('uuv_trajectory_control'),
        'config', 'controllers', 'nl_pid', model_name, 'params.yaml'
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

    # === Include thruster manager ===
    thruster_manager_launch = os.path.join(
        get_package_share_directory('uuv_thruster_manager'),
        'launch', 'thruster_manager.launch.py'
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

    actions = [thruster_include]

    # === GUI marker node (only if gui_on == true) ===
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
        # world_ned case
        if use_ned_frame == 'true':
            actions.append(
                GroupAction([
                    PushRosNamespace(namespace=uuv_name),
                    Node(
                        package='uuv_trajectory_control',
                        executable='rov_nl_pid_controller.py',
                        name='rov_nl_pid_controller',
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
                            controller_config_file,
                            model_params_file,
                            {'saturation': saturation,
                             'inertial_frame_id': 'world_ned'}
                        ]
                    )
                ])
            )
        # world case
        else:
            actions.append(
                GroupAction([
                    PushRosNamespace(namespace=uuv_name),
                    Node(
                        package='uuv_trajectory_control',
                        executable='rov_nl_pid_controller.py',
                        name='rov_nl_pid_controller',
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
                            {'saturation': saturation,
                             'inertial_frame_id': 'world'}
                        ]
                    )
                ])
            )

    # === Controller nodes when use_params_file == false ===
    else:
        # world_ned case
        if use_ned_frame == 'true':
            actions.append(
                GroupAction([
                    PushRosNamespace(namespace=uuv_name),
                    Node(
                        package='uuv_trajectory_control',
                        executable='rov_nl_pid_controller.py',
                        name='rov_nl_pid_controller',
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
                             'Kp': [Kp],
                             'Kd': [Kd],
                             'Ki': [Ki],
                             'Hm': [Hm],
                             'inertial_frame_id': 'world_ned'}
                        ]
                    )
                ])
            )
        # world case
        else:
            actions.append(
                GroupAction([
                    PushRosNamespace(namespace=uuv_name),
                    Node(
                        package='uuv_trajectory_control',
                        executable='rov_nl_pid_controller.py',
                        name='rov_nl_pid_controller',
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
                             'Kp': [Kp],
                             'Kd': [Kd],
                             'Ki': [Ki],
                             'Hm': [Hm],
                             'inertial_frame_id': 'world'}
                        ]
                    )
                ])
            )

    return actions


def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('uuv_name'),
        DeclareLaunchArgument('model_name'),
        DeclareLaunchArgument('saturation', default_value='6000'),
        DeclareLaunchArgument('gui_on', default_value='true'),
        DeclareLaunchArgument('use_params_file', default_value='false'),
        DeclareLaunchArgument('use_ned_frame', default_value='false'),
        DeclareLaunchArgument('Kp', default_value='6017.059667616178,6017.059667616178,6017.059667616178,9731.391095849109,9731.391095849109,9731.391095849109'),
        DeclareLaunchArgument('Kd', default_value='2682.9509286089447,2682.9509286089447,2682.9509286089447,9440.462338720527,9440.462338720527,9440.462338720527'),
        DeclareLaunchArgument('Ki', default_value='0,0,0,0,0,0'),
        DeclareLaunchArgument('Hm', default_value='1657.655912647713,1657.655912647713,1657.655912647713,4193.901741127024,4193.901741127024,4193.901741127024'),
        OpaqueFunction(function=launch_setup)
    ])
