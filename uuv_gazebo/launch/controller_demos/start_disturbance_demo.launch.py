from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def launch_setup(context, *args, **kwargs):
    # Resolve runtime argument
    uuv_name = context.launch_configurations['uuv_name']

    # Resolve include file path
    disturbance_manager_launch = os.path.join(
        get_package_share_directory('uuv_control_utils'),
        'launch',
        'start_disturbance_manager.launch.py'
    )

    # Resolve disturbance file path
    disturbance_file = os.path.join(
        get_package_share_directory('uuv_gazebo'),
        'config',
        'disturbances',
        'test_disturbances.yaml'
    )

    # Build the include action
    disturbance_include = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(disturbance_manager_launch),
        launch_arguments={
            'uuv_name': uuv_name,
            'use_file': '1',
            'disturbance_file': disturbance_file
        }.items()
    )

    return [disturbance_include]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('uuv_name'),
        OpaqueFunction(function=launch_setup)
    ])
