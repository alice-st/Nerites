from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction

import os

def launch_setup(context, *args, **kwargs):
    # === Resolve Launch Configurations at runtime ===
    set_timeout = LaunchConfiguration('set_timeout').perform(context)
    timeout = LaunchConfiguration('timeout').perform(context)

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
    start_circular = IncludeLaunchDescription(
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

    # === Three includes: apply_link_wrench.launch with different args ===
    wrench1 = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('uuv_control_utils'),
                'launch',
                'apply_link_wrench.launch.py'
            )
        ),
        launch_arguments={
            'uuv_name': 'rexrov',
            'starting_time': '5',
            'duration': '10',
            'force_x': '3000',
            'force_y': '3000'
        }.items()
    )

    wrench2 = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('uuv_control_utils'),
                'launch',
                'apply_link_wrench.launch.py'
            )
        ),
        launch_arguments={
            'uuv_name': 'rexrov',
            'starting_time': '20',
            'duration': '10',
            'force_y': '3000',
            'force_z': '3000'
        }.items()
    )

    wrench3 = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('uuv_control_utils'),
                'launch',
                'apply_link_wrench.launch.py'
            )
        ),
        launch_arguments={
            'uuv_name': 'rexrov',
            'starting_time': '35',
            'duration': '10',
            'torque_y': '3000',
            'torque_z': '3000'
        }.items()
    )

    # === Node: RViz2 ===
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=[
            '-d', os.path.join(
                get_package_share_directory('uuv_gazebo'),
                'rviz',
                'controller_demo.rviz'
            )
        ]
    )

    return [
        empty_world,
        TimerAction(period=10.0,actions=[upload_rexrov]),
        TimerAction(period=15.0,actions=[controller]),
        TimerAction(period=20.0,actions=[start_circular]),
        TimerAction(period=22.0,actions=[wrench1]),
        TimerAction(period=24.0,actions=[wrench2]),
        TimerAction(period=26.0,actions=[wrench3])
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('set_timeout', default_value='false'),
        DeclareLaunchArgument('timeout', default_value='105'),
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
