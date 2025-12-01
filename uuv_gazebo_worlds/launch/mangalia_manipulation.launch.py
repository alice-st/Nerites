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
    # Get argument values from context
    paused = LaunchConfiguration('paused').perform(context)
    gui = LaunchConfiguration('gui').perform(context)
    use_sim_time_str = LaunchConfiguration('use_sim_time').perform(context)
    use_sim_time = use_sim_time_str.lower() == 'true'

    # Gazebo world include
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare('gazebo_ros').perform(context),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'world': 'worlds/mangalia_manipulation.world',
            'pause': paused,
            'use_sim_time': use_sim_time_str,  # keep as string for Gazebo arg passing
            'gui': gui,
            'headless': 'false',
            'debug': 'false',
            'verbose': 'true',
            'extra_gazebo_args': '-s libgazebo_ros_force_system.so --ros-args -r gazebo:__ns:=/gazebo'
        }.items()
    )

    # Publish world NED frame include
    world_ned_frame = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare('uuv_assistants').perform(context),
                'launch',
                'publish_world_ned_frame.launch.py'
            )
        ),
        launch_arguments={'use_sim_time': use_sim_time_str}.items()
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
        gazebo_launch,
        world_ned_frame,
        # Node: publish_world_models
        Node(
            package='uuv_assistants',
            executable='publish_world_models.py',
            name='publish_world_models',
            output='screen',
            parameters=[
                os.path.join(
                    FindPackageShare('uuv_gazebo_worlds').perform(context),
                    'config',
                    'mangalia.yaml'  # same YAML as original XML
                ),
                {'use_sim_time': use_sim_time}
            ]
        )
    ]

    return actions

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('paused', default_value='false'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        OpaqueFunction(function=launch_setup)
    ])
