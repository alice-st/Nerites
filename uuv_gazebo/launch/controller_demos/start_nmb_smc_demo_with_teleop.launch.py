from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import TimerAction

def launch_setup(context, *args, **kwargs):
    # Resolve all arguments at runtime
    record = context.launch_configurations['record']
    use_ned_frame = context.launch_configurations['use_ned_frame']
    joy_id = context.launch_configurations['joy_id']
    axis_yaw = context.launch_configurations['axis_yaw']
    axis_x = context.launch_configurations['axis_x']
    axis_y = context.launch_configurations['axis_y']
    axis_z = context.launch_configurations['axis_z']

    # Resolve file paths
    ocean_waves_launch = os.path.join(
        get_package_share_directory('uuv_gazebo_worlds'),
        'launch', 'ocean_waves.launch.py'
    )

    upload_rexrov_launch = os.path.join(
        get_package_share_directory('uuv_descriptions'),
        'launch', 'upload_rexrov.launch.py'
    )

    rov_nmb_sm_controller_launch = os.path.join(
        get_package_share_directory('uuv_trajectory_control'),
        'launch', 'rov_nmb_sm_controller.launch.py'
    )

    record_demo_launch = os.path.join(
        get_package_share_directory('uuv_gazebo'),
        'launch', 'controller_demos', 'record_demo.launch.py'
    )

    teleop_launch = os.path.join(
        get_package_share_directory('uuv_teleop'),
        'launch', 'uuv_teleop.launch.py'
    )

    rviz_config = os.path.join(
        get_package_share_directory('uuv_gazebo'),
        'rviz', 'controller_demo.rviz'
    )

    # Include: ocean_waves
    ocean_waves = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(ocean_waves_launch)
    )

    # Include: upload_rexrov (passing some default args + use_ned_frame)
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

    # Include: joystick teleop
    teleop = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(teleop_launch),
        launch_arguments={
            'uuv_name': 'rexrov',
            'joy_id': joy_id,
            'output_topic': 'cmd_vel',
            'message_type': 'twist',
            'axis_yaw': axis_yaw,
            'axis_x': axis_x,
            'axis_y': axis_y,
            'axis_z': axis_z,
            'gain_yaw': '0.2',
            'gain_x': '0.5',
            'gain_y': '0.5',
            'gain_z': '0.5'
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
           TimerAction(period=10.0, actions=[rov_nmb_sm_controller]), 
           TimerAction(period=13.0, actions=[record_demo]),
           TimerAction(period=13.0, actions=[teleop]) ,
           TimerAction(period=15.0, actions=[rviz_node])]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('record', default_value='false'),
        DeclareLaunchArgument('use_ned_frame', default_value='false'),
        DeclareLaunchArgument('joy_id', default_value='0'),
        DeclareLaunchArgument('axis_yaw', default_value='0'),
        DeclareLaunchArgument('axis_x', default_value='4'),
        DeclareLaunchArgument('axis_y', default_value='3'),
        DeclareLaunchArgument('axis_z', default_value='1'),
        OpaqueFunction(function=launch_setup)
    ])
