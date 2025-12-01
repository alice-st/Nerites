# Copyright (c) 2025 The Nerites Authors.
# All rights reserved.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import TimerAction

def launch_setup(context, *args, **kwargs):
    # === Resolve LaunchConfiguration values at runtime ===
    timeout = LaunchConfiguration('timeout').perform(context)
    starting_time = LaunchConfiguration('starting_time').perform(context)
    duration = LaunchConfiguration('duration').perform(context)

    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    z = LaunchConfiguration('z').perform(context)
    yaw = LaunchConfiguration('yaw').perform(context)

    K = LaunchConfiguration('K').perform(context)
    Kd = LaunchConfiguration('Kd').perform(context)
    Ki = LaunchConfiguration('Ki').perform(context)
    slope = LaunchConfiguration('slope').perform(context)

    # === Include: empty_underwater_world.launch ===
    empty_world = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('uuv_gazebo_worlds'),
                'launch',
                'empty_underwater_world.launch.py'
            )
        ),
        launch_arguments={
            'set_timeout': 'true',
            'timeout': timeout
        }.items()
    )

    # === Include: upload_rexrov.launch ===
    upload_rexrov = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('uuv_descriptions'),
                'launch',
                'upload_rexrov.launch.py'
            )
        ),
        launch_arguments={
            'x': x,
            'y': y,
            'z': z,
            'yaw': yaw
        }.items()
    )

    # === Include: rov_nmb_sm_controller.launch ===
    controller = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('uuv_trajectory_control'),
                'launch',
                'rov_nmb_sm_controller.launch.py'
            )
        ),
        launch_arguments={
            'uuv_name': 'rexrov',
            'model_name': 'rexrov',
            'K': K,
            'Kd': Kd,
            'Ki': Ki,
            'slope': slope
        }.items()
    )

    # === Include: start_circular_trajectory.launch ===
    circular_traj = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('uuv_control_utils'),
                'launch',
                'start_circular_trajectory.launch.py'
            )
        ),
        launch_arguments={
            'uuv_name': 'rexrov',
            'radius': '5',
            'center_z': '-25',
            'max_forward_speed': '0.6'
        }.items()
    )

    # === Thruster state changes ===
    thruster2_state = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('uuv_control_utils'),
                'launch',
                'set_thruster_state.launch.py'
            )
        ),
        launch_arguments={
            'uuv_name': 'rexrov',
            'starting_time': starting_time,
            'duration': duration,
            'is_on': '0',
            'thruster_id': '2'
        }.items()
    )

    thruster6_state = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('uuv_control_utils'),
                'launch',
                'set_thruster_state.launch.py'
            )
        ),
        launch_arguments={
            'uuv_name': 'rexrov',
            'starting_time': starting_time,
            'duration': duration,
            'is_on': '0',
            'thruster_id': '6'
        }.items()
    )

    # === RViz2 node ===
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=[
            '-d',
            os.path.join(
                get_package_share_directory('uuv_gazebo'),
                'rviz',
                'controller_demo.rviz'
            )
        ]
    )

    return [
        empty_world,
        TimerAction(period=10.0, actions=[upload_rexrov]),
        TimerAction(period=15.0, actions=[controller]),
        TimerAction(period=20.0, actions=[circular_traj]),
        TimerAction(period=22.0, actions=[thruster2_state]),
        TimerAction(period=24.0, actions=[thruster6_state]),
        TimerAction(period=26.0, actions=[rviz_node])
    ]



def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('set_timeout', default_value='false'),
        DeclareLaunchArgument('timeout', default_value='105'),
        DeclareLaunchArgument('starting_time', default_value='10'),
        DeclareLaunchArgument('duration', default_value='30'),

        DeclareLaunchArgument('x', default_value='0'),
        DeclareLaunchArgument('y', default_value='0'),
        DeclareLaunchArgument('z', default_value='-20'),
        DeclareLaunchArgument('yaw', default_value='0'),

        DeclareLaunchArgument('K', default_value='5,5,5,5,5,5'),
        DeclareLaunchArgument('Kd', default_value='4118.98,4118.98,4118.98,8000.0,8000.0,8000.0'),
        DeclareLaunchArgument('Ki', default_value='0.06144,0.06144,0.06144,0.078,0.078,0.078'),
        DeclareLaunchArgument('slope', default_value='0.182,0.182,0.182,3.348,3.348,3.348'),

        OpaqueFunction(function=launch_setup)
    ])
