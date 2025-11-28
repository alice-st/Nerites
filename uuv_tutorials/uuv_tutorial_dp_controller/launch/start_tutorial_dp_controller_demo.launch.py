from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction

import os

def launch_setup(context, *args, **kwargs):
    record = LaunchConfiguration('record').perform(context)

    # === Include: ocean_waves.launch ===
    ocean_waves = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare('uuv_gazebo_worlds').perform(context),
                'launch',
                'ocean_waves.launch.py'
            )
        )
    )

    # === Include: upload_rexrov.launch ===
    upload_rexrov = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare('uuv_descriptions').perform(context),
                'launch',
                'upload_rexrov.launch.py'
            )
        ),
        launch_arguments={
            'x': '20',
            'y': '0',
            'z': '-20',
            'yaw': '0'
        }.items()
    )

    # === Include: start_tutorial_dp_controller.launch ===
    start_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare('uuv_tutorial_dp_controller').perform(context),
                'launch',
                'start_tutorial_dp_controller.launch.py'
            )
        ),
        launch_arguments={
            'uuv_name': 'rexrov',
            'model_name': 'rexrov'
        }.items()
    )

    # === Include: record_demo.launch ===
    record_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare('uuv_gazebo').perform(context),
                'launch',
                'controller_demos',
                'record_demo.launch.py'
            )
        ),
        launch_arguments={
            'record': record
        }.items()
    )

    # === RViz2 Node ===
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=[
            '-d',
            os.path.join(
                FindPackageShare('uuv_gazebo').perform(context),
                'rviz',
                'controller_demo.rviz'
            )
        ]
    )

    return [
        ocean_waves,
        TimerAction(period=10.0, actions=[upload_rexrov]),
        TimerAction(period=15.0, actions=[start_controller]),
        TimerAction(period=20.0, actions=[record_demo]),
        TimerAction(period=22.0, actions=[rviz_node]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('record', default_value='false'),
        OpaqueFunction(function=launch_setup)
    ])
