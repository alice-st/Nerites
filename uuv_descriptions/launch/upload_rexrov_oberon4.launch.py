# Copyright (c) 2025 The Nerites Authors.
# All rights reserved.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, OpaqueFunction 
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace, SetParameter
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from nerites_utils.time import is_sim_time
import xacro
import os

def launch_setup(context, *args, **kwargs):
    # Retrieve LaunchConfiguration values
    debug = LaunchConfiguration('debug').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    z = LaunchConfiguration('z').perform(context)
    roll = LaunchConfiguration('roll').perform(context)
    pitch = LaunchConfiguration('pitch').perform(context)
    yaw = LaunchConfiguration('yaw').perform(context)
    use_ned_frame = LaunchConfiguration('use_ned_frame').perform(context)

    # Get simulation time
    res = is_sim_time(return_param=False, use_subprocess=True)

    # Locate xacro file
    xacro_file = os.path.join(
        get_package_share_directory('uuv_descriptions'),
        'robots',
        'rexrov_oberon4.xacro'
    )

    inertial_ref = 'world_ned' if use_ned_frame.lower() == 'true' else 'world'

    # Use xacro to generate URDF in memory
    mappings = {
        'debug': debug,
        'namespace': namespace,
        'inertial_reference_frame': inertial_ref
    }
    doc = xacro.process(xacro_file, mappings=mappings)

    # Nodes
    group_actions = GroupAction([
        PushRosNamespace(namespace),

        Node(
            package='uuv_descriptions',
            executable='spawn_model',
            name='urdf_spawner',
            output='screen',
            arguments=[
                '-urdf',
                '-x', x,
                '-y', y,
                '-z', z,
                '-R', roll,
                '-P', pitch,
                '-Y', yaw,
                '-model', namespace,
                '-param', f"/{namespace}/robot_description"
            ],
            parameters=[{'use_sim_time': res}]
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            respawn=True,
            parameters=[
                {'use_sim_time': res},
                {'robot_description': doc}
            ]
        )
    ])

    # Arm parameters (kept as SetParameter)
    arms_params = [
        SetParameter(name=f"{namespace}/arms/n_arms", value=1),
        SetParameter(name=f"{namespace}/arms/names", value=['oberon4'])
    ]

    # FIX: remove IncludeLaunchDescription for YAML (we can attach YAML to a node later)
    # Placeholder for arm YAML (needs to be attached to a specific node)
    arm_config_path = os.path.join(
        FindPackageShare('oberon4_description').perform(context),
        'params',
        'robot_config.yaml'
    )

    # message_to_tf include
    message_to_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare('uuv_assistants').perform(context),
                'launch',
                'message_to_tf.launch'
            )
        ),
        launch_arguments={
            'namespace': namespace,
            'world_frame': 'world',
            'child_frame_id': f"/{namespace}/base_link"
        }.items()
    )

    return [group_actions, *arms_params, message_to_tf]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('debug', default_value='0'),
        DeclareLaunchArgument('x', default_value='0'),
        DeclareLaunchArgument('y', default_value='0'),
        DeclareLaunchArgument('z', default_value='-20'),
        DeclareLaunchArgument('roll', default_value='0.0'),
        DeclareLaunchArgument('pitch', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('namespace', default_value='rexrov'),
        DeclareLaunchArgument('use_ned_frame', default_value='false'),

        OpaqueFunction(function=launch_setup)
    ])
