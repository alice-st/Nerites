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
    # Retrieve launch arguments
    debug = LaunchConfiguration('debug').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    z = LaunchConfiguration('z').perform(context)
    roll = LaunchConfiguration('roll').perform(context)
    pitch = LaunchConfiguration('pitch').perform(context)
    yaw = LaunchConfiguration('yaw').perform(context)
    use_ned_frame = LaunchConfiguration('use_ned_frame').perform(context)

    # Sim time
    sim_time = is_sim_time(return_param=False, use_subprocess=True)

    # Locate the xacro file
    xacro_file = os.path.join(
        get_package_share_directory('uuv_descriptions'),
        'robots',
        'rexrov_oberon7.xacro'
    )

    inertial_ref = 'world_ned' if use_ned_frame.lower() == 'true' else 'world'

    # Generate URDF string directly from xacro
    mappings = {
        'debug': debug,
        'namespace': namespace,
        'inertial_reference_frame': inertial_ref
    }
    doc = xacro.process(xacro_file, mappings=mappings)

    # URDF spawner
    args = (
        f"-gazebo_namespace /gazebo "
        f"-x {x} -y {y} -z {z} "
        f"-R {roll} -P {pitch} -Y {yaw} "
        f"-entity {namespace} -topic robot_description").split()


    # Build main group
    group_actions = GroupAction([
        PushRosNamespace(namespace),

        # Spawn URDF model into Gazebo
        Node(
            name='urdf_spawner',
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            respawn=False,
            arguments=args,
            parameters=[{'use_sim_time': sim_time}]
        ),

        # Robot state publisher (publishes TF tree)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            respawn=False,
            parameters=[
                {'use_sim_time': sim_time},
                {'robot_description': doc}
            ]
        )
    ])

    # Arm parameters
    arms_params = [
        SetParameter(name=f"{namespace}/arms/n_arms", value=1),
        SetParameter(name=f"{namespace}/arms/names", value=['oberon7'])
    ]

    oberon7_config_path = os.path.join(
        FindPackageShare('oberon7_description').perform(context),
        'params', 'robot_config.yaml'
    )

    # Include message_to_tf.launch
    message_to_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare('uuv_assistants').perform(context),
                'launch',
                'message_to_tf.launch.py'
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
