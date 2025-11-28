from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
import os

def launch_setup(context, *args, **kwargs):
    # Resolve all launch arguments at runtime
    record = context.launch_configurations['record']
    x = context.launch_configurations['x']
    y = context.launch_configurations['y']
    z = context.launch_configurations['z']
    yaw = context.launch_configurations['yaw']
    use_ned_frame = context.launch_configurations['use_ned_frame']

    # Resolve file paths
    ocean_waves_launch = os.path.join(
        get_package_share_directory('uuv_gazebo_worlds'),
        'launch', 'ocean_waves.launch.py'
    )

    upload_rexrov_launch = os.path.join(
        get_package_share_directory('uuv_descriptions'),
        'launch', 'upload_rexrov.launch.py'
    )

    rov_pid_controller_launch = os.path.join(
        get_package_share_directory('uuv_trajectory_control'),
        'launch', 'rov_pid_controller.launch.py'
    )

    record_demo_launch = os.path.join(
        get_package_share_directory('uuv_gazebo'),
        'launch', 'controller_demos', 'record_demo.launch.py'
    )

    rviz_config = os.path.join(
        get_package_share_directory('uuv_gazebo'),
        'rviz', 'controller_demo.rviz'
    )

    # Includes

    ocean_waves = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(ocean_waves_launch)
    )

    upload_rexrov = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(upload_rexrov_launch),
        launch_arguments={
            'x': x,
            'y': y,
            'z': z,
            'yaw': yaw,
            'use_ned_frame': use_ned_frame
        }.items()
    )

    rov_pid_controller = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(rov_pid_controller_launch),
        launch_arguments={
            'uuv_name': 'rexrov',
            'model_name': 'rexrov',
            'use_ned_frame': use_ned_frame
        }.items()
    )

    record_demo = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(record_demo_launch),
        launch_arguments={
            'record': record,
            'use_ned_frame': use_ned_frame
        }.items()
    )

   # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )
    
    return [ocean_waves, 
           TimerAction(period=5.0, actions=[upload_rexrov]), 
           TimerAction(period=10.0, actions=[rov_pid_controller]), 
           TimerAction(period=13.0, actions=[record_demo]),
           TimerAction(period=15.0, actions=[rviz_node])]

def generate_launch_description():
    return LaunchDescription([
        # All <arg> from XML → DeclareLaunchArgument
        DeclareLaunchArgument('record', default_value='false'),
        DeclareLaunchArgument('x', default_value='0'),
        DeclareLaunchArgument('y', default_value='0'),
        DeclareLaunchArgument('z', default_value='-20'),
        DeclareLaunchArgument('yaw', default_value='0'),
        DeclareLaunchArgument('use_ned_frame', default_value='false'),

        # Runtime setup
        OpaqueFunction(function=launch_setup)
    ])
