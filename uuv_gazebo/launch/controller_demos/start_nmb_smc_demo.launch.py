from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
import os

def launch_setup(context, *args, **kwargs):
    # Resolve runtime args
    record = context.launch_configurations['record']
    use_ned_frame = context.launch_configurations['use_ned_frame']

    # Resolve all paths dynamically
    ocean_waves_launch = os.path.join(
        get_package_share_directory('uuv_gazebo_worlds'),
        'launch',
        'ocean_waves.launch.py'
    )

    upload_rexrov_launch = os.path.join(
        get_package_share_directory('uuv_descriptions'),
        'launch',
        'upload_rexrov.launch.py'
    )

    rov_nmb_sm_controller_launch = os.path.join(
        get_package_share_directory('uuv_trajectory_control'),
        'launch',
        'rov_nmb_sm_controller.launch.py'
    )

    record_demo_launch = os.path.join(
        get_package_share_directory('uuv_gazebo'),
        'launch',
        'controller_demos',
        'record_demo.launch.py'
    )

    rviz_config = os.path.join(
        get_package_share_directory('uuv_gazebo'),
        'rviz',
        'controller_demo.rviz'
    )

    # Include: ocean_waves
    ocean_waves = IncludeLaunchDescription(AnyLaunchDescriptionSource(ocean_waves_launch))

    # Include: upload_rexrov (passing x,y,z,yaw,use_ned_frame)
    upload_rexrov = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(upload_rexrov_launch),
        launch_arguments={
            'x': '20',
            'y': '0',
            'z': '-20',
            'yaw': '0',
            'use_ned_frame': use_ned_frame
        }.items()
    )

    # Include: rov_nmb_sm_controller
    rov_nmb_sm_controller = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(rov_nmb_sm_controller_launch),
        launch_arguments={
            'uuv_name': 'rexrov',
            'model_name': 'rexrov',
            'use_ned_frame': use_ned_frame
        }.items()
    )

    # Include: record_demo
    record_demo = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(record_demo_launch),
        launch_arguments={
            'record': record,
            'use_ned_frame': use_ned_frame
        }.items()
    )

    # RViz Node
    rviz_node = Node(
        name='rviz2',
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )
    # Return all launch entities
    return [ocean_waves, 
           TimerAction(period=5.0, actions=[upload_rexrov]), 
           TimerAction(period=10.0, actions=[rov_nmb_sm_controller]), 
           TimerAction(period=13.0, actions=[record_demo]), 
           TimerAction(period=15.0, actions=[rviz_node])]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('record', default_value='false'),
        DeclareLaunchArgument('use_ned_frame', default_value='false'),
        OpaqueFunction(function=launch_setup)
    ])
