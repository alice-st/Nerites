# Copyright (c) 2025 The Nerites Authors.
# All rights reserved.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def launch_setup(context, *args, **kwargs):
    # Get values from the launch arguments
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    z = LaunchConfiguration('z').perform(context)
    angle = LaunchConfiguration('angle').perform(context)
    use_sim_time_str = LaunchConfiguration('use_sim_time').perform(context)
    use_sim_time = use_sim_time_str.lower() == 'true'

    # Include the empty underwater world launch
    empty_world_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare('uuv_gazebo_worlds').perform(context),
                'launch',
                'empty_underwater_world.launch.py'
            )
        )
    )

    # Spawn entity command arguments
    spawn_args = (
        f"-gazebo_namespace /gazebo "
        f"-x {x} -y {y} -z {z} "
        f"-P 1.57 -Y {angle} "
        f"-entity rov_bop_panel "
        f"-database rov_bop_panel"
    )

    actions = [
        # Node: plankton_global_sim_time
        Node(
            package='plankton_utils',
            executable='plankton_global_sim_time',
            name='plankton_global_sim_time',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        empty_world_include,

        # Node: spawn the rov_bop_panel SDF entity
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='sdf_spawner',
            output='screen',
            arguments=spawn_args.split(),
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ]

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('x', default_value='-1'),
        DeclareLaunchArgument('y', default_value='-1'),
        DeclareLaunchArgument('z', default_value='-4'),
        DeclareLaunchArgument('angle', default_value='0'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        OpaqueFunction(function=launch_setup)
    ])
