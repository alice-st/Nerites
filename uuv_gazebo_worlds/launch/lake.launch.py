# Copyright (c) 2025 The Nerites Authors.
# All rights reserved.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def launch_setup(context, *args, **kwargs):
    # Retrieve and convert LaunchConfiguration values
    gui = LaunchConfiguration('gui').perform(context)
    paused = LaunchConfiguration('paused').perform(context)
    set_timeout = LaunchConfiguration('set_timeout').perform(context)
    timeout = LaunchConfiguration('timeout').perform(context)
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
            'world': 'worlds/lake.world',
            'pause': paused,
            'use_sim_time': use_sim_time_str,  # pass raw string for Gazebo arg
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

    # Conditional timeout group
    timeout_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    FindPackageShare('uuv_assistants').perform(context),
                    'launch',
                    'set_simulation_timer.launch'
                )
            ),
            launch_arguments={'timeout': timeout}.items()
        )
    ])

    actions = [
        # Node: nerites_global_sim_time
        Node(
            package='nerites_utils',
            executable='nerites_global_sim_time',
            name='nerites_global_sim_time',
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
                    'lake.yaml'
                ),
                {'use_sim_time': use_sim_time}
            ]
        )
    ]

    if set_timeout.lower() == 'true':
        actions.append(timeout_group)

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('paused', default_value='false'),
        DeclareLaunchArgument('set_timeout', default_value='false'),
        DeclareLaunchArgument('timeout', default_value='0.0'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # NEW: pass-through envs from parent process
        DeclareLaunchArgument('gazebo_master_uri', default_value='http://127.0.0.1:11345'),
        DeclareLaunchArgument('ros_domain_id', default_value='50'),

        # Set env inside this launch (affects gzserver+gzclient it starts)
        SetEnvironmentVariable('GAZEBO_MASTER_URI', LaunchConfiguration('gazebo_master_uri')),
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('ros_domain_id')),

        OpaqueFunction(function=launch_setup)
    ])