from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        #Declare arguments (from <arg> tags in XML)
        DeclareLaunchArgument('x', default_value='0'),
        DeclareLaunchArgument('y', default_value='0'),
        DeclareLaunchArgument('z', default_value='-5'),
        DeclareLaunchArgument('yaw', default_value='0'),

        DeclareLaunchArgument('record', default_value='false'),
        DeclareLaunchArgument('bag_filename', default_value='recording.bag'),

        DeclareLaunchArgument('use_ned_frame', default_value='false'),
        DeclareLaunchArgument('joy_id', default_value='0'),

        #Include upload_eca_a9.launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('eca_a9_description'),
                '/launch/upload_eca_a9.launch.py'
            ]),
            launch_arguments={
                'x': LaunchConfiguration('x'),
                'y': LaunchConfiguration('y'),
                'z': LaunchConfiguration('z'),
                'yaw': LaunchConfiguration('yaw'),
            }.items()
        ),

        #Include start_auv_teleop.launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('eca_a9_control'),
                '/launch/start_auv_teleop.launch.py'
            ]),
            launch_arguments={
                'joy_id': LaunchConfiguration('joy_id'),
            }.items()
        ),

        #Include record_demo.launch.py (handles optional recording)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('eca_a9_gazebo'),
                '/launch/record_demo.launch.py'
            ]),
            launch_arguments={
                'record': LaunchConfiguration('record'),
                'use_ned_frame': LaunchConfiguration('use_ned_frame'),
                'bag_filename': LaunchConfiguration('bag_filename'),
            }.items()
        )
    ])
